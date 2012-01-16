#include "helper.cpp"
class CPU : public Scene{};
class CPUTop_CPUFront : public NonTerminalIntermediate{};
class CPUTop_CPUFront_CPULSide : public Scene{};
class CPUTop_CPUFront_CPULSide_CPURSide : public NonTerminalIntermediate{};
class CPUBack : public Plane{};
class CPUFront : public Plane{};
class CPULSide : public Plane{};
class CPURSide : public Plane{};
class CPUTop : public Plane{};

void CPUAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<CPUTop, Plane>()));
    learningRules.push_back(RulePtr(new SingleRule<CPUFront, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPUFront, CPUTop, CPUFront>()));
    learningRules.push_back(RulePtr(new SingleRule<CPULSide, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPUFront_CPULSide, CPUTop_CPUFront, CPULSide>()));
    learningRules.push_back(RulePtr(new SingleRule<CPURSide, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPUFront_CPULSide_CPURSide, CPUTop_CPUFront_CPULSide, CPURSide>()));
    learningRules.push_back(RulePtr(new SingleRule<CPUBack, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPU, CPUTop_CPUFront_CPULSide_CPURSide, CPUBack>()));
}
