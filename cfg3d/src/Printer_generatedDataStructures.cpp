#include "helper.cpp"
class printer : public VisualObject{};
class printerFront_printerTop : public NonTerminalIntermediate{};
class printerFront_printerTop_printerRSide : public VisualObject{};
class printerFront_printerTop_printerRSide_printerBack : public NonTerminalIntermediate{};
class printerFront_printerRSide : public NonTerminalIntermediate{};
class printerFront_printerRSide_printerTop : public VisualObject{};
class printerFront_printerRSide_printerTop_printerBack : public NonTerminalIntermediate{};
class printerBack : public PlanarPrimitive{};
class printerFront : public PlanarPrimitive{};
//class printerLSide : public PlanarPrimitive{};
class printerSide : public PlanarPrimitive{};
class printerTop : public PlanarPrimitive{};

void printerAppendLearningRules(vector<RulePtr>& learningRules) {
    learningRules.push_back(RulePtr(new SingleRule<printerFront, Plane>()));
    learningRules.push_back(RulePtr(new SingleRule<printerTop, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerTop, printerFront, printerTop>()));
    learningRules.push_back(RulePtr(new SingleRule<printerSide, Plane>()));
    learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerTop_printerRSide, printerFront_printerTop, printerSide>()));
    //learningRules.push_back(RulePtr(new SingleRule<printerBack, Plane>()));
  //  learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerTop_printerRSide_printerBack, printerFront_printerTop_printerRSide, printerBack>()));
    //learningRules.push_back(RulePtr(new DoubleRule<printer, printerFront_printerTop_printerRSide_printerBack, printerLSide>()));

    learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerRSide, printerFront, printerSide>()));
    learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerRSide_printerTop, printerFront_printerRSide, printerTop>()));
    //learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerRSide_printerTop_printerBack, printerFront_printerRSide_printerTop, printerBack>()));
    //learningRules.push_back(RulePtr(new DoubleRule<printer, printerFront_printerRSide_printerTop_printerBack, printerLSide>()));
	
}
