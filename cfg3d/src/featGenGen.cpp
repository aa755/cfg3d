/* 
 * File:   featGenGen.cpp
 * Author: aa755
 *
 * Created on February 15, 2012, 7:39 PM
 */
//#define USING_SVM_FOR_LEARNING_CFG

#include <cstdlib>
#include "utils.h"
#include <boost/weak_ptr.hpp>
using namespace std;
using namespace boost;

char * dotFile;
bool metaLearning=false;
string functionName;
string learning;

class TreeNode
{
    Node name;
    /** anything made up of just terminals
     */
    bool primitivePart;
    bool object;
    bool terminal;
    
    
public:
    string getFullName()
    {
        return name.fullName;
    }
    typedef pair<string,set<string> > RuleType;
    typedef  shared_ptr<TreeNode> Ptr;
    typedef  weak_ptr<TreeNode> WPtr;
    static map< RuleType , vector<string> > ruleOrdering;
    static map<string,Ptr> nameToTreeNode;
    static ofstream errFile;
//    static map<>

    
    vector<Ptr> children;
    WPtr parent;
    
    TreeNode(string name_,Ptr parent_ ):name(name_),parent(parent_)
    {
        primitivePart=false;
    }
    
    TreeNode(string name_):name(name_)
    {
        primitivePart=false;
    }
    
    
    Ptr getParent()
    {
        return parent.lock();
    }

    void addChild(Ptr child)
    {
        children.push_back(child);
    }
    
    
    void print()
    {
        string type="";
        if(terminal)
            type=type+"Terminal,";
        if(primitivePart)
            type=type+"PrimitivePart,";
        
        cout<<name.type<<"__"<<name.id<<":"<<type<<endl;
        for(vector<Ptr>::iterator it=children.begin();it!=children.end();it++)
        {
            (*it)->print();
        }
        
    }
    
    string replace(string inp, char s, char d)
    {
        char data[inp.size()];
        for(int i=0;i<(int)inp.size();i++)
        {
            if(data[i]==',')
                data[i]='_';
        }
        return string(data);
    }
//    RPlane_PlaneSeg rulePPG;
//    RPlaneSeg rulePG;
//#include "helper.cpp"    
    void featGenGen(ofstream & ofile)
    {
        if(children.size()==0)
        {
            assert(name.type=="Terminal");
            terminal=true;
            object=false;
            primitivePart=false;
        }
        else
        {
                terminal=false;
            bool allterminal=true;
            bool someTerminal=false;
            map<string,Ptr> name2child;
            RuleType rul;
            rul.first=name.type;
            //vector<string>  typeOrder;
            set<string> dupCheck;
            
            for(vector<Ptr>::iterator it=children.begin();it!=children.end();it++)
            {
                (*it)->featGenGen(ofile);
                allterminal=allterminal && (*it)->terminal;
                someTerminal=someTerminal || (*it)->terminal;
                name2child[(*it)->name.type]=(*it);
                rul.second.insert((*it)->name.type);
            }
          
            assert((!someTerminal) || allterminal); //someTerminal => allTerminal
            if(allterminal)
            {
                primitivePart=true;
                object=false;
                ofile<<"temp= rulePG->"+functionName+"(sceneInfo.getTerminalSafe("<< children.at(0)->name.id << "));\n";
                for(int i=1;i<(int)children.size();i++)
                {
                    ofile<<"temp=rulePPG->"+functionName+"(temp,sceneInfo.getTerminalSafe("<< children.at(i)->name.id<<"));\n";            
                }
                ofile<<"\t"<<name.getDeclSimple()<<";\n";
                ofile<<"{\n\tSingleRule<"<< name.type<<",Plane> tr("+learning+");\n";
                ofile<<"\t"<<name.fullName <<"= rules.lookupRuleOfSameType(tr)->"+functionName+"(temp, dummy);\n}\n";
            }
            else
            {
                if (!name.isComplexType())
                {
                    for (vector<Ptr>::iterator it = children.begin(); it != children.end(); it++)
                    {
                        if (!dupCheck.insert((*it)->name.type).second)
                        {
                            errFile << dotFile<< ":dup:" << name.fullName << "-" << (*it)->name.type << endl;
                        }
                        //              typeOrder.push_back((*it)->name.type);
                    }
                    
                    ofile << "\t" << name.getDeclSimple() << ";\n";
                    if (children.size() == 1)
                    {
                        ofile << "{\n\tSingleRuleNoFeature<" << name.type << "," << children.at(0)->name.type << "> tr("+learning+");\n";
                        ofile <<"\t"<< name.fullName << "= rules.lookupRuleOfSameType(tr)->"+functionName+"(" << children.at(0)->name.fullName << ", dummy);\n}\n";
                    }
                    else if (children.size() >= 2)
                    {
                        //string types="";
                        string names="";
                        string interType="";
                        const vector<string> & typeOrder=ruleOrdering[rul];
                        assert(typeOrder.size()!=0) ; // o/w this mapping was not present in order file
                        //assert(typeOrder.size()==children.size()) ; // o/w this mapping was not present in order file
                        //types.append(typeOrder.at(0));
                        interType.append(typeOrder.at(0));
                        names.append(name2child[typeOrder.at(0)]->name.fullName);
                        for(int i=1;i<(int)typeOrder.size();i++)
                        {
                            string oldInterType=interType;
                            interType.append("_");
                            interType.append(typeOrder.at(i));
                            string oldNames=names;
                            names.append("_");
                            names.append(name2child[typeOrder.at(i)]->name.fullName);
                            ofile << "boost::shared_ptr<"<< interType<<"> "<< names<<";\n";
                            
                            ofile << "{\n\tDoubleRule<" << interType << "," <<oldInterType<<","<< typeOrder.at(i) << "> tr("+learning+");\n";
                                                        
                            ofile <<"\t"<<  names<< "= rules.lookupRuleOfSameType(tr)->"+functionName+"(" << oldNames <<","<< name2child[typeOrder.at(i)]->name.fullName << ", dummy);\n}\n";
                        }
                        ofile << "{\n\tSingleRuleNoFeature<" << name.type << "," << interType << "> tr("+learning+");\n";
                        ofile <<"\t"<< name.fullName << "= rules.lookupRuleOfSameType(tr)->"+functionName+"(" << names << ", dummy);\n}\n";
                        
                    }
                }
                else
                {
                    string support=name.stripComplexFromType();
                    bool baseOccluded=false;
                    if(name.isOccludedComplexType())
                    {
                        baseOccluded=true;
                        errFile<<"FloorOccluded"<<dotFile<<endl;
                        support=name.stripOccludedComplexFromType();// remove occluded too
                    }
                    
                    // move the support to first
                    bool found=false;
                    vector<string> childTypes;
                    for (vector<Ptr>::iterator it = children.begin(); it != children.end(); it++)
                    {
                        if ((*it)->name.type == support)
                        {
                            found=true;
                            if(it!=children.begin())
                                iter_swap(it,children.begin());
                        }
                        else
                            childTypes.push_back((*it)->name.getCorrectedType());
                    }
                    
                    
                    assert(found||baseOccluded);
                    
                    ofile<<"\t"<<name.getDecl()<<";\n";
 
                    ofile<<"{\n\t tempTypeStrs.clear();\n"<<endl;
                    
                    for(vector<string>::iterator it=childTypes.begin();it!=childTypes.end();it++)
                    {
                        ofile<<"\ttempTypeStrs.push_back(typeid("<<*it<<").name());\n";
                    }
                    
                    if(baseOccluded)
                    {
                        ofile << "{\n\t DoubleRuleComplex <" << support << "," <<children.at(0)->name.getCorrectedType()<< "> tr(tempTypeStrs,"+learning+");\n";
                       // ofile <<"\t"<< name.fullName << "= rules.lookupRuleOfSameType(tr)->"+functionName+"(" << children.at(0)->name.fullName << ", dummy);\n}\n";                        

                        ofile <<"\t"<<  name.fullName << "= rules.lookupRuleOfSameType(tr)->"+functionName+"(boost::shared_ptr<SupportComplex<"+support+"> >(),"<< children.at(0)->name.fullName << ", dummy);\n}\n";
                    }
                    else
                    {
                        ofile << "{\n\tSingleRuleComplex<" << support << "> tr;\n";                        
                        ofile <<"\t"<< name.fullName << "= rules.lookupRuleOfSameType(tr)->"+functionName+"(" << children.at(0)->name.fullName << ", dummy);\n}\n";
                    }
                    
                        for(int i=1;i<(int)children.size();i++)
                        {
                            ofile << "\t{\n\t\tDoubleRuleComplex<" << support << "," <<children.at(i)->name.getCorrectedType()<< "> tr(tempTypeStrs,"+learning+");\n";
                            ofile <<"\t\t"<<  name.fullName << "= rules.lookupRuleOfSameType(tr)->"+functionName+"(" << name.fullName <<","<< children.at(i)->name.fullName << ", dummy);\n\t}\n";
                        }
       
                        ofile<<"}\n";
                        
                }
                primitivePart=false;
                object=true;                                
            }
        }
            
    }
    
    static Ptr readDotTree(char * treeFile)
    {
        std::ifstream fileI;
        std::string line;

        fileI.open(treeFile);
        getline(fileI, line);
        assert(line.substr(0, 7) == "digraph");

        Ptr rootNode;
        
        bool rootFound=false;
        if (fileI.is_open())
        {
            while (fileI.good())
            {
                getline(fileI, line);
                if (line.at(0) == '}')
                    break;

                

                vector<string> toks;
                getTokens(line, toks, "\t ->;");
                assert(toks.size()==1 || toks.size()==2);
                
                if (toks.size() == 1)
                {
                    if(rootFound)
                        continue;
                    
                    string name=toks.at(0);
                    Node nd(name);
                    if(nd.isRootType())
                    {
                        rootFound=true;
                        rootNode=Ptr(new TreeNode(name,Ptr()));
                        nameToTreeNode[name]=rootNode;
                    }
                }
                
                if (toks.size() == 2)
                {
                     if(nameToTreeNode.find(toks.at(0))==nameToTreeNode.end()) // nodes not reachable from root
                         continue;
                     
                    Ptr parent = nameToTreeNode[toks.at(0)];
                    assert(parent != NULL);

                    string childName = toks.at(1);
                    Ptr child(new TreeNode(childName,parent));
                    nameToTreeNode[childName] = child;
                    parent->addChild(child);

                }
            }

        }
        else
        {
            cout << "could not open the "<<treeFile<<" ..exiting\n";
            exit(-1);
        }
        
        return rootNode;

    }
    
    static void parseMapping(char * file)
    {
        std::ifstream fileI;
        std::string line;

        fileI.open(file);

        
        if (fileI.is_open())
        {
            while (fileI.good())
            {
                getline(fileI, line);
                
                if (line.size() == 0)
                    break;
                //cout<<line<<endl;
                RuleType rule;
                vector<string> toks;
                getTokens(line, toks, ";");
                assert( toks.size()==2);
                string ruleLHS=toks.at(0);
                string order =toks.at(1);
                getTokens(order, toks, "_");
                rule.first = ruleLHS;
                
                if(EndsWith(ruleLHS,"Complex"))
                    break;
                
                for(int i=0;i<(int)toks.size();i++)
                {
                    if(!rule.second.insert(toks.at(i)).second) // insertion should not fail => no duplicate type
                    cerr<<"WARN:"<<line<<endl;
                }
                
                ruleOrdering[rule]=toks;
                //cout<<rule<<endl;
            }
        }
    }
};

map< TreeNode::RuleType , vector<string> > TreeNode::ruleOrdering;
map<string,TreeNode::Ptr> TreeNode::nameToTreeNode;
ofstream TreeNode::errFile;

void createRunLearnFront(ofstream & outputLearnerCode) {
#ifdef USING_SVM_FOR_LEARNING_CFG
    outputLearnerCode << "#define USING_SVM_FOR_LEARNING_CFG\n";
#endif
    outputLearnerCode << "#define CONSIDER_ALL_SEGMENTS_TRAINING_NEW\n";
    outputLearnerCode << "#include\"structures.h\"\n";
    outputLearnerCode << "#include\"wallDistance.h\"\n";
    outputLearnerCode << "#include\"generatedDataStructures.cpp\"\n";
    outputLearnerCode<<"void runLearn(SceneInfo & sceneInfo, RulesDB & rules) {"<<endl;
    outputLearnerCode<<"RPlane_PlaneSeg rulePPGb; boost::shared_ptr<RPlane_PlaneSeg> rulePPG=rules.lookupRuleOfSameType(rulePPGb);\n";
    outputLearnerCode<<"RPlaneSeg rulePGb; boost::shared_ptr<RPlaneSeg> rulePG=rules.lookupRuleOfSameType(rulePGb);\n";
    outputLearnerCode<<"Plane::SPtr temp;\n";
    outputLearnerCode<<"vector<Terminal_SPtr> dummy; \n";
    outputLearnerCode<<"vector<string> tempTypeStrs;\n";
//    outputLearnerCode<<"    vector<Terminal_SPtr> temp;"<<endl<<endl;
}

void createRunLearnBack(ofstream & outputLearnerCode) {
    outputLearnerCode<<"}"<<endl<<endl;
    
    outputLearnerCode<<"int main(int argc, char** argv) {"<<endl;
    outputLearnerCode<<"if(argc!=3)\n{\ncerr<<\"usage:\"<<argv[0]<<\" <PCDFile> <origPCDFileWithNaNs>\"<<endl;\n exit(-1);\n}\n";        
    outputLearnerCode<<"    SceneInfo::SPtr sc(new SceneInfo());"<<endl;
    outputLearnerCode<<"    sc->init(argv[1]);RulesDB rules;sc->setRulesDB(createStaticShared<RulesDB>(&rules));\n";
    outputLearnerCode<<"    runLearn(*sc,rules);"<<endl;
    outputLearnerCode<<"}"<<endl;

}

/**
 * 
 */
int main(int argc, char** argv)
{

        functionName="applyRuleLearning";
        learning="true";
        if(metaLearning)
        {
            functionName="applyRuleInference";
            learning="false";
        }

    TreeNode::errFile.open("errTrees.txt",ios::app);
    
    dotFile=argv[1];
    TreeNode::Ptr root=TreeNode::readDotTree(dotFile);    
    assert(root!=NULL);
    ofstream ofile("featGen.cpp",ios::out);
    TreeNode::parseMapping(argv[2]);

    createRunLearnFront(ofile);
    
    ofile<<"\n Rule::META_LEARNING=!"<<learning<<";\n";
    ofile<<"\n //"<<argv[1]<<"\n";
    
    root->featGenGen(ofile);
    if(metaLearning)
    {
        ofile<<"\nScene::printOnlyScene("<<root->getFullName()<<");\n";
    }

#ifdef USING_SVM_FOR_LEARNING_CFG
        ofile<<"\n"<<root->getFullName()<<"->printPsi();\n";
        ofile<<"\n"<<root->getFullName()<<"->printLabelMap(\""<<string(argv[1])+".labelmap" <<"\");\n";
#endif
        
        ofile<<"ENTMAP entMap; "<<root->getFullName()<<"->mapEntities(entMap); "<<root->getFullName()<<"->printEntitiesMap(entMap);\n";
    createRunLearnBack(ofile);
    ofile.close();
    TreeNode::errFile.close();
//    root->setFlags();
//    root->print();

    return 0;
}

