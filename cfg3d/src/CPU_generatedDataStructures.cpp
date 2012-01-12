#include "helper.cpp"
class CPUTop_CPURSide : public NonTerminalIntermediate{};
class CPUTop_CPURSide_CPUFront : public NonTerminalIntermediate{};
class CPUTop_CPURSide_CPUFront_CPUBack : public Scene{};
class CPU : public Scene{};
class CPUBack : public Plane{};
class CPUFront : public Plane{};
class CPULSide : public Plane{};
class CPURSide : public Plane{};
class CPUTop : public Plane{};

void CPUAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<CPUTop, Plane>()));
    learningRules.push_back(RulePtr(new SingleRule<CPURSide, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPURSide, CPUTop, CPURSide>()));
    learningRules.push_back(RulePtr(new SingleRule<CPUFront, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPURSide_CPUFront, CPUTop_CPURSide, CPUFront>()));
    learningRules.push_back(RulePtr(new SingleRule<CPUBack, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPURSide_CPUFront_CPUBack, CPUTop_CPURSide_CPUFront, CPUBack>()));
    learningRules.push_back(RulePtr(new SingleRule<CPULSide, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPU, CPUTop_CPURSide_CPUFront_CPUBack, CPULSide>()));
}
