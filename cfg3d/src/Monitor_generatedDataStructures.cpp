#include "helper.cpp"
class monitor : public Scene, PlanarPrimitive{};

template<> 
    double SingleRule<monitor, Plane>::getCostScaleFactor()
    {
        return 1000.0;
    }

void MonitorAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<monitor, Plane>()));
}
