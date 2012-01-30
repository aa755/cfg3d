/* 
 * File:   Rules_Wall.h
 * Author: aa755
 *
 * Created on January 25, 2012, 5:08 PM
 */

#ifndef RULES_WALL_H
#define	RULES_WALL_H
#include "structures.cpp"
class Wall : public VisualObject , PlanarPrimitive{};

template<>
void SingleRule<Wall, Plane> ::computeAdditionalFeats(Plane* input)
{
//    features.push_back(input->getZNormal());
    if(!learning)
    {
        features.push_back(input->getDistanceToBoundary());
    }
}

template<> 
    double SingleRule<Wall, Plane>::getCostScaleFactor()
    {
        return 100.0;
    }



#endif	/* RULES_WALL_H */

