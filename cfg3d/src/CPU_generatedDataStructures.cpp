#include "helper.cpp"
class CPU : public VisualObject{};
class CPUFront_CPUTop : public NonTerminalIntermediate{};
class CPUFront_CPUTop_CPURSide : public VisualObject{};
class CPUFront_CPUTop_CPURSide_CPUBack : public NonTerminalIntermediate{};
class CPUFront_CPURSide : public NonTerminalIntermediate{};
class CPUFront_CPURSide_CPUTop : public VisualObject{};
class CPUFront_CPURSide_CPUTop_CPUBack : public NonTerminalIntermediate{};
class CPUBack : public PlanarPrimitive{};
class CPUFront : public PlanarPrimitive{};
//class CPULSide : public PlanarPrimitive{};
class CPUSide : public PlanarPrimitive{};
class CPUTop : public PlanarPrimitive{};

void CPUAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<CPUFront, Plane>()));
    learningRules.push_back(RulePtr(new SingleRule<CPUTop, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUFront_CPUTop, CPUFront, CPUTop>()));
    learningRules.push_back(RulePtr(new SingleRule<CPUSide, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUFront_CPUTop_CPURSide, CPUFront_CPUTop, CPUSide>()));
    //learningRules.push_back(RulePtr(new SingleRule<CPUBack, Plane>()));
  //  learningRules.push_back(RulePtr(new DoubleRule<CPUFront_CPUTop_CPURSide_CPUBack, CPUFront_CPUTop_CPURSide, CPUBack>()));
    //learningRules.push_back(RulePtr(new DoubleRule<CPU, CPUFront_CPUTop_CPURSide_CPUBack, CPULSide>()));

    learningRules.push_back(RulePtr(new DoubleRule<CPUFront_CPURSide, CPUFront, CPUSide>()));
    learningRules.push_back(RulePtr(new DoubleRule<CPUFront_CPURSide_CPUTop, CPUFront_CPURSide, CPUTop>()));
    //learningRules.push_back(RulePtr(new DoubleRule<CPUFront_CPURSide_CPUTop_CPUBack, CPUFront_CPURSide_CPUTop, CPUBack>()));
    //learningRules.push_back(RulePtr(new DoubleRule<CPU, CPUFront_CPURSide_CPUTop_CPUBack, CPULSide>()));
	
}
