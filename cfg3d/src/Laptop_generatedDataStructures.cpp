#include "helper.cpp"
class Laptop : public NonTerminal{};
class LaptopKeyboard : public NonTerminal{};
class LaptopScreen : public NonTerminal{};

void LaptopAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<LaptopScreen, Plane>()));
    learningRules.push_back(RulePtr(new SingleRule<LaptopKeyboard, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<Laptop, LaptopScreen, LaptopKeyboard>()));
}
