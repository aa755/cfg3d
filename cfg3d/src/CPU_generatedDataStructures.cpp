#include "helper.cpp"
class CPUTop_CPURSide : public NonTerminalIntermediate{};
//class CPUTop_CPURSide : public Scene{};
class CPUTop_CPURSide_CPUFront : public NonTerminalIntermediate{};
//class CPUTop_CPURSide_CPUFront : public Scene{};
//class CPUTop_CPURSide_CPUFront_CPUBack : public Scene{};
class CPUTop_CPURSide_CPUFront_CPUBack : public NonTerminalIntermediate{};
class CPU : public Scene{};
class CPUBack : public PlanarPrimitive{};
class CPUFront : public PlanarPrimitive{};
class CPULSide : public PlanarPrimitive{};
class CPURSide : public PlanarPrimitive{};
class CPUTop : public PlanarPrimitive{};

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
