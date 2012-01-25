/* 
 * File:   Rules_Floor.h
 * Author: aa755
 *
 * Created on January 25, 2012, 5:10 PM
 */

#ifndef RULES_FLOOR_H
#define	RULES_FLOOR_H

class Floor : public Scene, PlanarPrimitive{};
template<>
void SingleRule<Floor, Plane> ::computeFeaturesSpecializable(Plane* input)
{
    features.push_back(input->getZNormal());
    features.push_back(input->getCentroidZ());
}

template<> 
    double SingleRule<Floor, Plane>::getCostScaleFactor()
    {
        return 600.0;
    }

#endif	/* RULES_FLOOR_H */

