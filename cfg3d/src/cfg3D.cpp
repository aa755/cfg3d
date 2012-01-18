/*
 * File:   main.cpp
 * Author: abhishek
 *
 * Created on July 26, 2011, 7:41 PM
 */

#include "structures.cpp"
#include "CPU_generatedDataStructures.cpp"
template<typename SceneRHS_Type1, typename SceneRHS_Type2>
class RScene : public Rule {
    template<typename TypeExtracted, typename TypeCombinee>
    void combineAndPushGivenTypes(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */) {

        TypeExtracted * RHS_extracted = dynamic_cast<TypeExtracted *> (extractedSym);
        FindNTsToCombineWith finder(extractedSym, terminals, iterationNo);
        NonTerminal * nt = finder.nextEligibleNT();

        //int count=0;
        while (nt != NULL) {
            //                    nt->printData(); //checked that duplicates not extracted, and exhaustive
            //  count++;
            //                    if(typeid(*nt)==typeid(TypeCombinee) )
            if (typeid (*nt) == typeid (TypeCombinee) && nt->isMutuallyExhaustive(RHS_extracted)) {
                TypeCombinee * RHS_combinee = dynamic_cast<TypeCombinee *> (nt);
                addToPqueueIfNotDuplicate(applyRule<TypeExtracted,TypeCombinee>(RHS_extracted, RHS_combinee), pqueue);
            }
            nt = finder.nextEligibleNT();
        }

        //  cout<<"nnc: "<<count<<endl;
    }

    template<typename RHS_Type1, typename RHS_Type2>
    void combineAndPushGeneric(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals, long iterationNo /* = 0 */) {
        if(typeid(*extractedSym)==typeid(RHS_Type1))
        {
            combineAndPushGivenTypes<RHS_Type1,RHS_Type2>(extractedSym,pqueue,terminals,iterationNo);
        }
        else if(typeid(*extractedSym)==typeid(RHS_Type2))
        {
            combineAndPushGivenTypes<RHS_Type2,RHS_Type1>(extractedSym,pqueue,terminals,iterationNo);            
        }
    }

public:
    template<typename RHS_Type1, typename RHS_Type2>
    NonTerminal* applyRule(RHS_Type1 * RHS_unordered1, RHS_Type2 * RHS_unordered2)
    {
        Scene * LHS=new Scene();
        LHS->addChild(RHS_unordered1);
        LHS->addChild(RHS_unordered2);
        LHS->setAdditionalCost(0);
        LHS->computeSpannedTerminals();        
        //cout<<"S->fc\n";        
        cout<<"S->fc: cost "<<LHS->getCost()<<"\n";        
//        cerr<<RHS_plane1->set_membership<<"\n";        
//        cerr<<RHS_plane2->set_membership<<"\n";        
        return LHS;
    }
    
     void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals , long iterationNo /* = 0 */)
    {
         combineAndPushGeneric<SceneRHS_Type1,SceneRHS_Type2>(extractedSym,pqueue,terminals,iterationNo);
    }
};

class RPlaneSeg : public Rule {
public:
    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        if (typeid (*extractedSym) != typeid (Terminal))
            return;
        
        Plane * LHS = new Plane();
        LHS->addChild(extractedSym);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndSetCost();
        addToPqueueIfNotDuplicate(LHS,pqueue); //TODO: duplicate check is not required
    }
};

class RPlane_PlaneSeg : public Rule {
public:
    
    int get_Nof_RHS_symbols() {
        return 2;
    }

    void get_typenames(vector<string> & names) {
        names.push_back(typeid (Plane).name());
        names.push_back(typeid (Terminal).name());
    }
    
    NonTerminal* applyRule(Plane * RHS_plane, Terminal *RHS_seg, vector<Terminal*> & terminals) {
        if (!RHS_plane->isAllCloseEnough(RHS_seg)) {
            return NULL;
        }
        
        Plane * LHS = new Plane();
        LHS->addChild(RHS_plane);
        //TODO : FIRST COMPUTE PLANE PARAMS FROM JUST THIS AND THEN 
        LHS->addChild(RHS_seg);
        LHS->computeSpannedTerminals();
        LHS->computePlaneParamsAndSetCost();
        return LHS;
    }

    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
    {
        set<int>::iterator it;
        //all terminals have cost=0, all NT's have cost>0,
        //so when a terminal is extracted, no non-terminal(plane)
        //has been extracted yet
        //so, if sym is of type Terminal, it cannot be combined with a plane
        if (typeid (*extractedSym) == typeid (Plane))
        {
            extractedSym->resetNeighborIterator();
            int index;
            while (extractedSym->nextNeighborIndex(index))
            {
                Plane * plane=dynamic_cast<Plane*> (extractedSym);
                NonTerminal *newNT=applyRule(plane,terminals[index],terminals);
                addToPqueueIfNotDuplicate(newNT,pqueue);
            }

        }
    }
};

void solveLinearEquationPair(const Vector2f& v1, const Vector2f& v2, const Vector2f& b, Vector2f& solution) {
    Matrix2f A;
    float v10 = v1[0];
    float v11 = v1[1];
    float v20 = v2[0];
    float v21 = v2[1];
    A << v10,v11, v20,v21;
    solution = A.colPivHouseholderQr().solve(b);
}

Vector2f getDirection(pcl::PointXYZ& p1, pcl::PointXYZ& p2) {
    Vector2f v1(p1.x, p1.y);
    Vector2f v2(p2.x, p2.y);
    return v1 - v2;
}

pcl::PointXYZ getFarthestInDirection(Plane& plane, const Vector2f direction) {
    float farthestX;
    float farthestY;
    
    if (direction[0] < 0) {
        farthestX = plane.getMinX();
    } else {
        farthestX = plane.getMaxX();
    }
    
    if (direction[1] < 0) {
        farthestY = plane.getMinY();
    } else {
        farthestY = plane.getMaxY();
    }
    
    return pcl::PointXYZ(farthestX, farthestY, 0);
}

pcl::PointXYZ getPlanePlaneOcclusionPoint(Plane& plane1, Plane& plane2, pcl::PointXYZ& c1, pcl::PointXYZ& c2, Vector2f& d1, Vector2f& d2) {
    Vector4f p1Params = plane1.getPlaneParams();
    Vector4f p2Params = plane2.getPlaneParams();
    Vector2f intersectionPoint;
    Vector2f v1(p1Params[0],p1Params[1]);
    Vector2f v2(p2Params[0],p2Params[1]);
    Vector2f v4(-p1Params[3],-p2Params[3]);
    solveLinearEquationPair(v1, v2, v4, intersectionPoint);
        
    pcl::PointXYZ centroid1;
    plane1.getCentroid(centroid1);
    pcl::PointXYZ centroid2;
    plane2.getCentroid(centroid2);
    
    pcl::PointXYZ i(intersectionPoint[0], intersectionPoint[1], 0);
    
    // Where d1 and d2 are the directions away from the intersection point of the two planes.
    d2 = getDirection(centroid1, i);
    d1 = getDirection(centroid2, i);
 
    
    c1 = getFarthestInDirection(plane1, d2);
    c2 = getFarthestInDirection(plane2, d1);
    
    Vector2f b(c2.x - c1.x, c2.y - c1.y);
    Vector2f row1(plane1.getPlaneNormal()[0], -plane2.getPlaneNormal()[0]);
    Vector2f row2(plane1.getPlaneNormal()[1], -plane2.getPlaneNormal()[1]);
    
//    Vector2f row1(d1[0], -d2[0]);
//    Vector2f row2(d1[1], -d2[1]);
    Vector2f r;
    solveLinearEquationPair(row1, row2, b, r);
    
    float x_p = c2.x + r[1] * plane2.getPlaneNormal()[0];
    float y_p = c2.y + r[1] * plane2.getPlaneNormal()[1];
    cout<<"Making potential occlusion plane..."<<endl;
    
    cout<<"\tp1Params = "<<p1Params<<endl;
    cout<<"\tp2Params = "<<p2Params<<endl;
    cout<<"\tp1Centroid = "<<centroid1<<endl;
    cout<<"\tp2Centroid = "<<centroid2<<endl;
    cout<<"\tp1Max = "<<plane1.getMaxPoint()<<endl;
    cout<<"\tp1Min = "<<plane1.getMinPoint()<<endl;
    cout<<"\tp2Max = "<<plane2.getMaxPoint()<<endl;
    cout<<"\tp2Min = "<<plane2.getMinPoint()<<endl;
    cout<<"\ti = ("<<i.x<<","<<i.y<<")"<<endl;
    cout<<"\td1 = ("<<d1[0]<<","<<d1[1]<<")"<<endl;
    cout<<"\td2 = ("<<d2[0]<<","<<d2[1]<<")"<<endl;
    cout<<"\tc1 = ("<<c1.x<<","<<c1.y<<")"<<endl;
    cout<<"\tc2 = ("<<c2.x<<","<<c2.y<<")"<<endl;
    cout<<"\tocclusionPoint = ("<<x_p<<","<<y_p<<")"<<endl;
    return pcl::PointXYZ(x_p, y_p, 0);
}

vector<pcl::PointXYZ> getPointsToSample(pcl::PointXYZ& c1, pcl::PointXYZ& occlusionPoint, Plane& plane, float sampleFactor) {
    vector<pcl::PointXYZ> samplePoints;
    float xStep = (occlusionPoint.x - c1.x)/sampleFactor;
    float yStep = (occlusionPoint.y - c1.y)/sampleFactor;
//    float xStep = fabs(c1.x - occlusionPoint.x)/sampleFactor;
//    float yStep = fabs(c1.y - occlusionPoint.y)/sampleFactor;
    float zStep = fabs(plane.getMaxZ() - plane.getMinZ())/sampleFactor;
    float currentX = c1.x;
    float currentY = c1.y;
    float samplesTaken = 0;
    while(samplesTaken < sampleFactor) {
        for (float k = plane.getMinZ(); k < plane.getMaxZ(); k+=zStep) {
            pcl::PointXYZ samplePoint(currentX, currentY, k);
            samplePoints.push_back(samplePoint);
        }
        currentX = currentX + xStep;
        currentY = currentY + yStep;
        samplesTaken = samplesTaken + 1;
    }
    return samplePoints;
}

//bool isSamplePointsOccluded(vector<pcl::PointXYZ>& samplePoints, float occlusionThreshold, float sampleFactor) {
//    float numOccludedPoints = 0;
//    vector<pcl::PointXYZ>::iterator it;
//    for (it = samplePoints.begin(); it != samplePoints.end(); it++) {
//        bool isOccluded = occlusionChecker->isOccluded(*it);
//        printPoint(*it);
//        cout<<"^^isOccluded = "<<isOccluded<<endl;
//        if (isOccluded) {
//            numOccludedPoints = numOccludedPoints + 1;
//        }
//    }
//    return (numOccludedPoints / (sampleFactor * sampleFactor)) > occlusionThreshold;
//}

//bool canHallucinatePlane(Plane& plane1, Plane& plane2, vector<pcl::PointXYZ>& samplePoints) {
//    float sampleFactor = 5;
//    float occlusionThreshold = .5;
//    pcl::PointXYZ c1;
//    pcl::PointXYZ c2;
//    Vector2f d1;
//    Vector2f d2;
//    pcl::PointXYZ occlusionPoint = getPlanePlaneOcclusionPoint(plane1, plane2, c1, c2, d1, d2);
//    samplePoints = getPointsToSample(c1, occlusionPoint, plane1, sampleFactor);
//    if (isSamplePointsOccluded(samplePoints, occlusionThreshold, sampleFactor)) {
//        cout<<"PlaneTripletGenerated"<<endl;
//        return true;
//    } else {
//        samplePoints = getPointsToSample(c2, occlusionPoint, plane2, sampleFactor);
//        if (isSamplePointsOccluded(samplePoints, occlusionThreshold, sampleFactor)) {
//            cout<<"PlaneTripletGenerated"<<endl;
//            return true;
//        }
//    }
//    return false;
//}

//class RPlaneTriplet_PlanePairPlane : public Rule {
//public:
//    int get_Nof_RHS_symbols() {
//        return 2;
//    }
//
//    void get_typenames(vector<string> & names) {
//        names.push_back(typeid (PlanePair).name());
//        names.push_back(typeid (Plane).name());
//    }
//    
//    NonTerminal* applyRule(PlanePair* RHS_planePair , vector<Terminal*> & terminals) {
//        Plane* plane1 = dynamic_cast<Plane*>(RHS_planePair->getChild(0));
//        Plane* plane2 = dynamic_cast<Plane*>(RHS_planePair->getChild(1));
//        cout<<"attempting to hallucinate..."<<endl;
//        plane1->printData();
//        plane2->printData();
//        vector<pcl::PointXYZ> hallucinationPoints;
//        if (!canHallucinatePlane(*plane1, *plane2, hallucinationPoints)) {
//            return NULL;
//        }
//        else {
//            vector<pcl::PointXYZ>::iterator it;
//            
//            // Get hallucinated plane
//            
//            Terminal* hallucinatedSegment = new HallucinatedTerminal(hallucinationPoints);
//                    cout<<"hallucination\n---------"<<endl;
//        plane1->printData();
//        plane2->printData();
//        cout<<hallucinatedSegment->getIndex()<<endl;
//        cout<<"--------"<<endl;
//
//            SingleRule<Plane, Terminal> rule;
//            Plane* RHS_hallucinatedPlane = dynamic_cast<Plane*>(rule.applyRule(hallucinatedSegment, terminals));
//            RHS_hallucinatedPlane->declareOptimal();
//            
//            PlaneTriplet* LHS = new PlaneTriplet();
//            
//            LHS->addChild(RHS_planePair);
//            LHS->computeSpannedTerminals();
//            
//            LHS->addChild(RHS_hallucinatedPlane);
//            LHS->setAdditionalCost(1);
//            
//            return LHS;
//        }
//    }
//
//    void combineAndPush(Symbol * extractedSym, SymbolPriorityQueue & pqueue, vector<Terminal*> & terminals /* = 0 */, long iterationNo /* = 0 */)
//    {
//        if (typeid (*extractedSym) == typeid (PlanePair))
//        {
//            PlanePair* planePair = dynamic_cast<PlanePair*> (extractedSym);
//            // Try to hallucinate.
//            NonTerminal *newNT = applyRule(planePair, terminals);
//            addToPqueueIfNotDuplicate(newNT,pqueue);
//        }
//    }
//};

/// Templated type Marker TRM
template<>
    bool DoubleRule<PlanePair, Plane, Plane> :: setCost(PlanePair * output, Plane * input1, Plane * input2, vector<Terminal*> & terminals)
    {
        assert(output->children.size()==2);
        double parallelity = input1->dotProduct(input2);
        if (parallelity < .2 && isVerticalEnough(input1) && isVerticalEnough(input2) && isOnTop(input1, TABLE_HEIGHT) && isOnTop(input2, TABLE_HEIGHT)) {
            output->setAdditionalCost(parallelity);
            return true;
        }
        else {
            return false;
        }
    }

template<>
    bool DoubleRule<Corner, PlanePair, Plane> :: setCost(Corner * output, PlanePair * input1, Plane * input2, vector<Terminal*> & terminals)
    {
        Vector3d planePairCrossProduct = input1->getCrossProduct();
        Vector3d planeNormal(input2->getPlaneNormal());
        output->setAdditionalCost(1-fabs(planePairCrossProduct.dot(planeNormal)));
        return true;
    }

template<> 
    bool DoubleRule<Boundary, FloorSurface, Wall> :: setCost(Boundary * output, FloorSurface * input1, Wall * input2, vector<Terminal*> & terminals)
    {
 //       cerr<<"correct cost"; // needs specialization
        output->setAdditionalCost(0);
        return true;
    }

template<>
    bool DoubleRule<SceneGeneric, Floor, Wall> :: setCost(SceneGeneric * output, Floor * input1, Wall * input2, vector<Terminal*> & terminals)
    {
 //       cerr<<"correct cost"; // needs specialization
        int numSpannedPoints=input1->getNumPoints()+input2->getNumPoints();
        float coverage=((float)numSpannedPoints)/((float)NUMPointsToBeParsed);
        if(coverage<0.75)
            return false;
        else
        {
     //   cout<<"S->fc\n";        
                output->setAdditionalCost(5.0/coverage);
        cout<<"S->fc: cost "<<output->getCost()<<"\n";        
                return true;
        }
    }
    
template<>
    bool SingleRule<Plane, Terminal> :: setCost(Plane* output, Terminal* input, vector<Terminal*> & terminals)
    {
        output->computePlaneParamsAndSetCost();
        cout<<"plane rule called"<<endl;
        return true;
    }

template<>
    bool SingleRule<Wall, Plane> :: setCost(Wall* output, Plane* input, vector<Terminal*> & terminals)
    {
        Vector4f planeParams = input->getPlaneParams();
        double additionalCost=fabs(planeParams[2]);
        if(additionalCost>0.2 || !input->checkSize(output))
            return false;
        else 
        {
            
            output->setAdditionalCost(additionalCost);
            return true;
        }           
    }

template<>
    bool SingleRule<Leg, Plane> :: setCost(Leg* output, Plane* input, vector<Terminal*> & terminals)
    {
    //cerr<<"called"<<fabs(input->getMaxZ() - TABLE_HEIGHT)<<","<<(input->getMaxZ())<<endl;
        double maxZDiff=(TABLE_HEIGHT-input->getMaxZ());

        if(isVerticalEnough(input) &&  isOnTop(TABLE_HEIGHT,input) && maxZDiff <0.20 ) // makes 75 degrees or less with vertical
        {
        Vector4f planeParams = input->getPlaneParams();
        double normalZ=fabs(planeParams[2]);
            output->setAdditionalCost(normalZ + fabs(maxZDiff));
//            output->setAdditionalCost( maxZDiff);
            return true;
        }
        else 
            return false;
    }

template<>
    bool SingleRule<Legs, Leg> :: setCost(Legs* output, Leg* input, vector<Terminal*> & terminals)
    {
        output->appendLeg(input);
        output->setAdditionalCost(0);
        return true;
    }

template<>
    bool SingleRule<Floor, FloorSurface> :: setCost(Floor* output, FloorSurface* input, vector<Terminal*> & terminals)
    {
        output->SetEldestChild(input);
        output->setAdditionalCost(0);
        return true;
    }

template<>
    bool DoubleRule<Floor, Floor, Table> :: setCost(Floor* output, Floor* input1, Table* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input2, input1->GetEldestChild())) 
        {
            output->setAdditionalCost(0);
            output->SetEldestChild(input1->GetEldestChild());
            return true;
        } 
        else 
        {
            return false;
        }
    }

template<>
    bool SingleRule<TableTopSurface, Plane> :: setCost(TableTopSurface* output, Plane* input, vector<Terminal*> & terminals) {
        double additionalCost=input->computeZMinusCSquared(TABLE_HEIGHT);
        if(additionalCost>(0.3*0.3)*input->getNumPoints() || ! input->isHorizontalEnough()|| !input->checkSize(output) )
            return false;
        else 
        {
            output->setAdditionalCost(additionalCost);
            return true;
        }                   
    }

template<>
    bool SingleRule<KeyboardTray, Plane> :: setCost(KeyboardTray* output, Plane* input, vector<Terminal*> & terminals) {
        double additionalCost=input->computeZMinusCSquared(TABLE_HEIGHT-0.1);
        if(additionalCost>(0.3*0.3)*input->getNumPoints())
            return false;
        else 
        {
            output->setAdditionalCost(additionalCost/10); // need not be planar
            return true;
        }                   
    }

template<>
    bool SingleRule<FloorSurface, Plane> :: setCost(FloorSurface * output, Plane* input, vector<Terminal*> & terminals) {
        double additionalCost=input->getZSquaredSum();
        if(additionalCost>(0.2*0.2)*input->getNumPoints() || ! input->isHorizontalEnough() )
            return false;
        else 
        {
            output->setAdditionalCost(additionalCost);
            return true;
        }                   
}

bool isZCloseEnough(double value, double height) {
    return fabs(value - height) <= .3;
}

template<>
    bool DoubleRule<Table, TableTop, Legs> :: setCost(Table* output, TableTop* input1, Legs* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input1, input2)) 
        {
            output->setAdditionalCost(input1->computeCostOfAddingLegs(input2));
            return true;
        } 
        else 
        {
            return false;
        }
    }

template<>
    bool DoubleRule<Table, TableTopSurface, Legs> :: setCost(Table* output, TableTopSurface* input1, Legs* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input1, input2)) 
        {
            output->setAdditionalCost(input1->computeCostOfAddingLegs(input2));
          //  cerr<<"table id"<< output->getId() <<"created w/ cost"<<output->getCost()<<endl;
            return true;
        } 
        else 
        {
          //  cerr<<"table rejected w/ cost"<<output->getCost()<<endl;
            return false;
        }
    }

bool isCloseEnoughToTableHeight(Plane* input) {
    return isZCloseEnough(input->getMinZ(), TABLE_HEIGHT);
}

bool isCloseEnoughToCompMonTop(Plane* input) {
    return isZCloseEnough(input->getMaxZ(), 1.1);
}

// Assumes all computers are above table_height
//template<>
//    bool DoubleRule<Computer, Plane, Plane> :: setCost(Computer* output, Plane* input1, Plane* input2, vector<Terminal*> & terminals) {
//
//    if (!isVerticalEnough(input1) || !isVerticalEnough(input2)) {
//        return false;
//    } else {
//        double minZOfBothPlanes = min(input1->getMinZ(), input2->getMinZ());
//        
//        if (!isCloseEnoughToTableHeight(input1) || !isCloseEnoughToTableHeight(input2) ||
//            !isCloseEnoughToCompMonTop(input1) || !isCloseEnoughToCompMonTop(input2)) {
//            return false;
//        } else {
//            double distanceFromTable = fabs(minZOfBothPlanes - TABLE_HEIGHT);
//            double zNormal1 = input1->getZNormal();
//            double zNormal2 = input2->getZNormal();
//            output->setAdditionalCost(distanceFromTable + zNormal1 + zNormal2);
//            
//            //TODO: maybe add costs for maxZ?
//            return true;
//        }
//    }
//}

// Assumes all monitors are above table_height
template<>
    bool SingleRule<Monitor, Plane> :: setCost(Monitor* output, Plane* input, vector<Terminal*> & terminals) {
        if (!isVerticalEnough(input)) {
            return false;
        } else {
            if (!isCloseEnoughToTableHeight(input) || 
                !isCloseEnoughToCompMonTop(input)) {
                return false;
            } else {
                if( !input->checkSize(output))
                    return false;
                
                double distanceFromTable = fabs(input->getMinZ() - TABLE_HEIGHT);
                output->setAdditionalCost(distanceFromTable + input->getZNormal());
                return true;
            }
        }
    }

template<>
    bool SingleRule<Computer, PlaneTriplet> :: setCost(Computer* output, PlaneTriplet* input, vector<Terminal*> & terminals) {
        output->setAdditionalCost(0);
        return true;
    }

template<>
    bool DoubleRule<TableTopObjects, Computer, Monitor> :: setCost(TableTopObjects* output, Computer* input1, Monitor* input2, vector<Terminal*> & terminals) {
        output->setAdditionalCost(0);
        return true;
    }


template<>
    bool SingleRule<TableTop, TableTopSurface> :: setCost(TableTop* output, TableTopSurface* input, vector<Terminal*> & terminals)
    {
        output->setAdditionalCost(0);
        output->SetEldestChild(input);
        return true;
    }

template<>
    bool DoubleRule<TableTop, TableTop, Monitor> :: setCost(TableTop* output, TableTop* input1, Monitor* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input2, input1->GetEldestChild())) 
        {
            output->setAdditionalCost(0);
            output->SetEldestChild(input1->GetEldestChild());
            return true;
        } 
        else 
        {
            return false;
        }
    }

template<>
    bool DoubleRule<TableTop, TableTop, KeyboardTray> :: setCost(TableTop* output, TableTop* input1, KeyboardTray* input2, vector<Terminal*> & terminals) {
    float difference=input1->GetEldestChild()->getMinZ() - input2->getMinZ();
    
        if ( 0.03<difference && difference<0.18  ) 
        {
            output->setAdditionalCost(0);
            output->SetEldestChild(input1->GetEldestChild());
            return true;
        } 
        else 
        {
            return false;
        }
    }

template<>
    bool DoubleRule<TableTop, TableTop, PlanePair> :: setCost(TableTop* output, TableTop* input1, PlanePair* input2, vector<Terminal*> & terminals) {
        if (isOnTop(input2, input1->GetEldestChild())) 
        {
            output->setAdditionalCost(0);
            output->SetEldestChild(input1->GetEldestChild());
            return true;
        } 
        else 
        {
            return false;
        }
    }


template<>
    bool DoubleRule<Legs, Legs, Leg> :: setCost(Legs* output, Legs* input1, Leg* input2, vector<Terminal*> & terminals)
    {
        output->setLegs(input1->getLegs());
        output->appendLeg(input2);
        vector<Leg*> legs = input1->getLegs();
        vector<Leg*>::iterator it;
        double costCount = 0;
        for (it = legs.begin(); it != legs.end(); it++) {
                costCount = costCount + (*it)->computeLegLegCost(input2);
//            costCount = costCount + computeLegLegCost(*it, input2);
        }
        output->setAdditionalCost(costCount);
        return true;
    }

template<>
    bool DoubleRule<TableTop, TableTopSurface, TableTopObjects> :: setCost(TableTop* output, TableTopSurface* input1, TableTopObjects* input2, vector<Terminal*> & terminals) {

        if (isOnTop(input2, input1)) 
        {
            output->setAdditionalCost(0);
            return true;
        } 
        else 
        {
            return false;
        }
    }

void appendRuleInstancesForPrimitives(vector<RulePtr> & rules) {
    
    // planes
    rules.push_back(RulePtr(new RPlaneSeg()));
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));
    //rules.push_back(RulePtr(new RPlaneTriplet_PlanePairPlane()));
    
}

void appendRuleInstances(vector<RulePtr> & rules) {
    rules.push_back(RulePtr(new DoubleRule<PlanePair, Plane, Plane>()));
    //rules.push_back(RulePtr(new DoubleRule<Corner, PlanePair, Plane>()));
    
    // planes
    rules.push_back(RulePtr(new SingleRule<Plane, Terminal>()));
    rules.push_back(RulePtr(new RPlane_PlaneSeg()));
    //rules.push_back(RulePtr(new RPlaneTriplet_PlanePairPlane()));
    
    // boundary, wall, floor
    rules.push_back(RulePtr(new SingleRule<FloorSurface, Plane>()));
    rules.push_back(RulePtr(new SingleRule<Wall, Plane>()));
//    rules.push_back(RulePtr(new DoubleRule<Boundary,Floor,Wall>()));
    rules.push_back(RulePtr(new DoubleRule<SceneGeneric,Floor,Wall>()));
    rules.push_back(RulePtr(new SingleRule<Floor,FloorSurface>()));
    rules.push_back(RulePtr(new DoubleRule<Floor,Floor,Table>()));

    // table
    rules.push_back(RulePtr(new SingleRule<Leg, Plane>()));
    rules.push_back(RulePtr(new SingleRule<Legs,Leg>()));
    rules.push_back(RulePtr(new DoubleRule<Legs,Legs,Leg>()));
    
    // computer
  //  rules.push_back(RulePtr(new SingleRule<Computer, PlanePair>()));
  //  rules.push_back(RulePtr(new DoubleRule<Computer, Plane, Plane>()));
    
    // monitor
    rules.push_back(RulePtr(new SingleRule<Monitor, Plane>()));  
    
    // whole scene
    //rules.push_back(RulePtr(new RScene<Table,Boundary>()));
    
    // table
    rules.push_back(RulePtr(new SingleRule<TableTop, TableTopSurface>()));
    rules.push_back(RulePtr(new DoubleRule<TableTop, TableTop, PlanePair>()));
    rules.push_back(RulePtr(new DoubleRule<TableTop, TableTop, Monitor>()));
    rules.push_back(RulePtr(new SingleRule<TableTopSurface, Plane>()));
//    rules.push_back(RulePtr(new DoubleRule<TableTop, TableTop, KeyboardTray>()));
//    rules.push_back(RulePtr(new SingleRule<KeyboardTray, Plane>()));
    
    //need to fix this  all tables might not be neighbors
    rules.push_back(RulePtr(new DoubleRule<Table,TableTop,Legs>()));
    rules.push_back(RulePtr(new DoubleRule<Table,TableTopSurface,Legs>()));
}

void outputOnBothStreams(string str)
{
    cout<<str<<endl;
    cerr<<str<<endl;
}

void runParse(map<int, set<int> > & neighbors, int maxSegIndex) {
    vector<RulePtr> rules;
   // appendRuleInstances(rules);
    appendRuleInstancesForPrimitives(rules);
    CPUAppendLearningRules(rules);
    //    vector<set<NonTerminal*> > ancestors(numPoints,set<NonTerminal*>());

    SymbolPriorityQueue pq(maxSegIndex);

    vector<Terminal *> terminals;

    Terminal * temp;
    for (int i = 1; i <= maxSegIndex; i++) {
        temp = new Terminal(i-1); // index is segment Number -1 
        temp->setNeighbors( neighbors[i],maxSegIndex);
        terminals.push_back(temp);
        pq.pushTerminal(temp);
    }

    NUMPointsToBeParsed=0;
    for(unsigned int i=0;i<scene.size();i++)
    {
        if(rand()%10 != 1)
            continue;
        int segIndex=scene.points[i].segment;
  //      cout<<"seg "<<segIndex<<endl;
        if(segIndex>0 && segIndex<=maxSegIndex)
        {
            terminals.at(segIndex-1)->addPointIndex(i);
            NUMPointsToBeParsed++;
        }
    }
    
    for(unsigned int i=0;i<terminals.size();i++)
    {
      //  terminals.at(i)->computeMinDistanceBwNbrTerminals(terminals)
        terminals.at(i)->computeFeatures();
    }

    segMinDistances.setZero(terminals.size(),terminals.size());
    
    for(unsigned int i1=0;i1<terminals.size();i1++)
    {
            for(unsigned int i2=i1+1;i2<terminals.size();i2++)
            {
                float minDistance=getSmallestDistance(scene, terminals.at(i1)->getPointIndicesBoostPtr(), terminals.at(i2)->getPointIndicesBoostPtr());
                segMinDistances(i1,i2)=minDistance;
                segMinDistances(i2,i1)=minDistance;
            }
    }
    
    cout<<"minDistances computed\n"<<endl;
    cerr<<"minDistances computed\n"<<endl;

    for(unsigned int i=0;i<terminals.size();i++)
    {
       // cout<<"s "<<i<<terminals[i]->getPointIndices().size()<<endl;
        assert(terminals[i]->getPointIndices().size()>0); 
        // if this happens, delete this NT. NEED TO CHANGE SIZE OF NEIGHBOR VECTOR
    }
    
    Terminal::totalNumTerminals = terminals.size();

    Symbol *min;
    long count = 0;
    long rulecount = 0;
    bool alreadyExtracted=false;
    while (true) {
        min = pq.pop(alreadyExtracted);

        
        if(min==NULL)
        {
            outputOnBothStreams("parsing failed. goal is not derivable from the given rules ... fix the rules or PQ insertion threshold ... or rules' thershold\n");
            exit(-1);
        }
        
        if(alreadyExtracted)
        {
            delete min;
            cout << "dup" << endl;
            // since there are no parent links yet(not yet declared optimal),
            // and it was not a child of anyone (not yet combined)
            // and it was repoved from PQ
            // and it was not in NTSetsExtracted,
            // deleting does not cause dangling pointers
            continue;
        }

        cout << "\n\n\niter: " << count++ << " cost:" << min->getCost() <<" typ:"<<min->getName()<< endl;

        Scene *dummyTypeCheck=dynamic_cast<Scene*>(min);
        if (dummyTypeCheck!=NULL) // if min is of type Scene(Goal)
        {
            cout << "goal reached!!" << endl;
            cerr << "goal reached!! with cost:"<<min->getCost()<< endl;
            min->printData();
            return;
        }
        if (typeid (*min) == typeid (Terminal) || !alreadyExtracted) {
            min->declareOptimal();
            min->printData();
 //           cout<<"mz"<<min->getMaxZ()<<endl;
            
            for (size_t i = 0; i < rules.size(); i++) {
                rules.at(i)->combineAndPush(min, pq, terminals,rulecount++); // combine with the eligible NT's to form new NTs and add them to the priority queue
                //an eligible NT should not span any terminal already in min
                //an eligible NT should contain atleast 1 terminal in combneCandidates
            }

        }
        //pq.pop();

    }
}

void subsample(pcl::PointCloud<PointT> & inp, pcl::PointCloud<PointT> & out) {
    out.points.clear();
    out.header = inp.header;
    for (size_t i = 0; i < inp.size(); i++) {
        if (rand() % 5 == 1) {
            out.points.push_back(inp.points[i]);
        }
    }
}


void convertToXY(const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<pcl::PointXY> & cloudxy)
{
    cloudxy.points.resize(cloud.size());
    cloudxy.sensor_origin_=cloud.sensor_origin_;
    for (size_t i = 0; i < cloud.size(); i++)
    {
        cloudxy.points[i].x = cloud.points[i].x;
        cloudxy.points[i].y = cloud.points[i].y;
    }
}


int main(int argc, char** argv) {
    assert(isinf(infinity()));
    
    if(argc!=3)
    {
        cerr<<"usage: "<<argv[0]<<" <pcdFile> <nbrMap> "<<endl;
    }
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
    fileName=string(argv[1]);
    fileName=fileName.substr(0, fileName.length()-4);

#ifndef DISABLE_HALLUCINATION
    occlusionChecker = new OccupancyMap<PointT>(scene);
#endif
    //convertToXY(scene,scene2D);
  //  scene2DPtr=createStaticShared<pcl::PointCloud<pcl::PointXY> >(&scene2D);
    map<int, set<int> > neighbors;
    int maxSegIndex= parseNbrMap(argv[2],neighbors);
    cout<<"scene has "<<scene.size()<<" points"<<endl;
    runParse(neighbors,maxSegIndex);

    return 0;
    
}
