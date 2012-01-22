#include "helper.cpp"
class monitor : public Scene{};

void MonitorAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<monitor, Plane>()));
}
