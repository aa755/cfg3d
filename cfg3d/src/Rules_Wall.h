/* 
 * File:   Rules_Wall.h
 * Author: aa755
 *
 * Created on January 25, 2012, 5:08 PM
 */

#ifndef RULES_WALL_H
#define	RULES_WALL_H

class Wall : public Scene, PlanarPrimitive{};

template<>
void SingleRule<Wall, Plane> ::computeFeaturesSpecializable(Plane* input)
{
    features.push_back(input->getZNormal());
    features.push_back(input->getDistanceToBoundary());
    
}



#endif	/* RULES_WALL_H */

