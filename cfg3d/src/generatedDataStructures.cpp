#include "structures.cpp"

class sofaArm : public PlanarPrimitive{};
class monitor : public PlanarPrimitive{};
class Chair : public NonTerminal{};
class tableTop : public PlanarPrimitive{};
class sofaBackRest : public PlanarPrimitive{};
class window : public PlanarPrimitive{};
class CPUSide : public PlanarPrimitive{};
class cupboardFront : public PlanarPrimitive{};
class chairLArmRest : public PlanarPrimitive{};
class CPUTop : public PlanarPrimitive{};
class printerFront : public PlanarPrimitive{};
class printerTray : public PlanarPrimitive{};
class paper : public PlanarPrimitive{};
class dustbinNarrow : public PlanarPrimitive{};
class ACVent : public PlanarPrimitive{};
class CPU : public NonTerminal{};
class Wall : public PlanarPrimitive{};
class chairArmRest : public PlanarPrimitive{};
class printerSide : public PlanarPrimitive{};
class dustbinWide : public PlanarPrimitive{};
class door : public PlanarPrimitive{};
class keyboard : public PlanarPrimitive{};
class FridgeSide : public PlanarPrimitive{};
class tableDrawer : public PlanarPrimitive{};
class cupboardSide : public PlanarPrimitive{};
class tableBack : public PlanarPrimitive{};
class dustbin : public NonTerminal{};
class Floor : public PlanarPrimitive{};
class WhiteBoardRack : public PlanarPrimitive{};
class pillarRight : public PlanarPrimitive{};
class pillar : public NonTerminal{};
class chairRArmRest : public PlanarPrimitive{};
class printerTop : public PlanarPrimitive{};
class sofaBase : public PlanarPrimitive{};
class FridgeTop : public PlanarPrimitive{};
class Table : public NonTerminal{};
class mug : public PlanarPrimitive{};
class chairBase : public PlanarPrimitive{};
class tableLeg : public PlanarPrimitive{};
class CPUFront : public PlanarPrimitive{};
class pillarMid : public PlanarPrimitive{};
class keyboardTray : public PlanarPrimitive{};
class printer : public NonTerminal{};
class pillarLeft : public PlanarPrimitive{};
class chairBack : public PlanarPrimitive{};
class chairBackRest : public PlanarPrimitive{};
class sofaSide : public PlanarPrimitive{};
class sofaFrontBelowSeat : public PlanarPrimitive{};
class FridgeDoor : public PlanarPrimitive{};
class CPUFront_CPUSide : public NonTerminalIntermediate{};
class CPUFront_CPUSide_CPUTop : public NonTerminalIntermediate{};
class CPUFront_CPUSide_chairBack : public NonTerminalIntermediate{};
class CPUFront_CPUTop : public NonTerminalIntermediate{};
class chairArmRest_chairBack : public NonTerminalIntermediate{};
class chairArmRest_chairBackRest : public NonTerminalIntermediate{};
class chairArmRest_chairBack_chairBackRest : public NonTerminalIntermediate{};
class chairArmRest_chairBack_chairBase : public NonTerminalIntermediate{};
class chairBackRest_chairBase : public NonTerminalIntermediate{};
class chairBackRest_chairBase_chairArmRest : public NonTerminalIntermediate{};
class chairBackRest_chairBase_chairArmRest_chairArmRest : public NonTerminalIntermediate{};
class chairBackRest_chairBase_chairBack : public NonTerminalIntermediate{};
class chairBack_chairBackRest : public NonTerminalIntermediate{};
class chairBack_chairLArmRest : public NonTerminalIntermediate{};
class chairBack_chairLArmRest_chairRArmRest : public NonTerminalIntermediate{};
class FridgeDoor_FridgeSide : public NonTerminalIntermediate{};
class FridgeDoor_FridgeSide_FridgeTop : public NonTerminalIntermediate{};
class Fridge : public NonTerminal{};
class sofaBackRest_sofaBase : public NonTerminalIntermediate{};
class Sofa : public NonTerminal{};
class keyboardTray_tableBack : public NonTerminalIntermediate{};
class keyboardTray_tableBack_tableDrawer : public NonTerminalIntermediate{};
class keyboardTray_tableTop : public NonTerminalIntermediate{};
class tableDrawer_tableLeg : public NonTerminalIntermediate{};
class tableDrawer_tableTop : public NonTerminalIntermediate{};
class tableDrawer_tableTop_keyboardTray : public NonTerminalIntermediate{};
class tableDrawer_tableTop_keyboardTray_tableBack : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableBack : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_keyboardTray : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_keyboardTray_tableBack : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_keyboardTray_tableBack_tableLeg : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_keyboardTray_tableLeg : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_tableBack : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_tableBack_tableDrawer : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_tableDrawer : public NonTerminalIntermediate{};
class tableDrawer_tableTop_tableLeg_tableLeg : public NonTerminalIntermediate{};
class tableLeg_tableLeg : public NonTerminalIntermediate{};
class tableLeg_tableTop : public NonTerminalIntermediate{};
class tableLeg_tableTop_keyboardTray : public NonTerminalIntermediate{};
class tableLeg_tableTop_keyboardTray_tableLeg : public NonTerminalIntermediate{};
class tableLeg_tableTop_tableBack : public NonTerminalIntermediate{};
class cupboardFront_cupboardSide : public NonTerminalIntermediate{};
class cupboard : public NonTerminal{};
class dustbinNarrow_dustbinWide : public NonTerminalIntermediate{};
class pillarLeft_pillarRight : public NonTerminalIntermediate{};
class pillarLeft_pillarRight_pillarMid : public NonTerminalIntermediate{};
class printerFront_printerSide : public NonTerminalIntermediate{};
class printerFront_printerTop : public NonTerminalIntermediate{};
class printerFront_printerTop_printerSide : public NonTerminalIntermediate{};
class printerFront_printerTop_printerSide_printerTray : public NonTerminalIntermediate{};
class chairBackRest_sofaBase : public NonTerminalIntermediate{};
class sofa : public NonTerminal{};
class sofaBackRest_sofaBase_sofaArm : public NonTerminalIntermediate{};
class sofaBackRest_sofaBase_sofaFrontBelowSeat : public NonTerminalIntermediate{};
class sofaBackRest_sofaBase_sofaFrontBelowSeat_sofaSide : public NonTerminalIntermediate{};
void appendLearningRules(vector<RulePtr>& learningRules) {
vector<string> temp;
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofaArm,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<monitor,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<tableTop,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofaBackRest,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRuleComplex<Table>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<window,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPUSide,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<cupboardFront,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<chairLArmRest,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPUTop,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printerFront,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printerTray,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<paper,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<dustbinNarrow,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<ACVent,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPU,CPUFront>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Wall,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<chairArmRest,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printerSide,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<dustbinWide,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<door,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<keyboard,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<FridgeSide,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<tableDrawer,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<cupboardSide,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<tableBack,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<dustbin,dustbinWide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Floor,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<WhiteBoardRack,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<pillarRight,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<pillar,pillarMid>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<chairRArmRest,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printerTop,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofaBase,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<FridgeTop,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<mug,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<chairBase,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<tableLeg,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPUFront,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBase>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<pillarMid,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<keyboardTray,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPU,CPUTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printer,printerFront>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<pillarLeft,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<chairBack,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<chairBackRest,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofaSide,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofaFrontBelowSeat,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<FridgeDoor,Plane>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<CPUFront_CPUSide,CPUFront,CPUSide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPU,CPUFront_CPUSide>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<CPUFront_CPUSide_CPUTop,CPUFront_CPUSide,CPUTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPU,CPUFront_CPUSide_CPUTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<CPUFront_CPUSide_chairBack,CPUFront_CPUSide,chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPU,CPUFront_CPUSide_chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<CPUFront_CPUTop,CPUFront,CPUTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<CPU,CPUFront_CPUTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRuleComplex<CPU>()));
	temp.clear();
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<CPU,paper>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairArmRest_chairBack,chairArmRest,chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairArmRest_chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairArmRest_chairBackRest,chairArmRest,chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairArmRest_chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairArmRest_chairBack_chairBackRest,chairArmRest_chairBack,chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairArmRest_chairBack_chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairArmRest_chairBack_chairBase,chairArmRest_chairBack,chairBase>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairArmRest_chairBack_chairBase>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBackRest_chairBase,chairBackRest,chairBase>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBackRest_chairBase>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBackRest_chairBase_chairArmRest,chairBackRest_chairBase,chairArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBackRest_chairBase_chairArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBackRest_chairBase_chairArmRest_chairArmRest,chairBackRest_chairBase_chairArmRest,chairArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBackRest_chairBase_chairArmRest_chairArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBackRest_chairBase_chairBack,chairBackRest_chairBase,chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBackRest_chairBase_chairBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBack_chairBackRest,chairBack,chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBack_chairBackRest>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBack_chairLArmRest,chairBack,chairLArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBack_chairLArmRest_chairRArmRest,chairBack_chairLArmRest,chairRArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Chair,chairBack_chairLArmRest_chairRArmRest>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRuleComplex<Floor>()));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,Wall>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,ACVent>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,pillar>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,sofa>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,Chair>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,window>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,tableTop>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,Sofa>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,Table>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,SupportComplex<Table> >(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,CPU>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,door>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,printer>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(SupportComplex<Floor>).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Chair).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,cupboard>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Floor,dustbin>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<FridgeDoor_FridgeSide,FridgeDoor,FridgeSide>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<FridgeDoor_FridgeSide_FridgeTop,FridgeDoor_FridgeSide,FridgeTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Fridge,FridgeDoor_FridgeSide_FridgeTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<sofaBackRest_sofaBase,sofaBackRest,sofaBase>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Sofa,sofaBackRest_sofaBase>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<keyboardTray_tableBack,keyboardTray,tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<keyboardTray_tableBack_tableDrawer,keyboardTray_tableBack,tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,keyboardTray_tableBack_tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<keyboardTray_tableTop,keyboardTray,tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,keyboardTray_tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableLeg,tableDrawer,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop,tableDrawer,tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_keyboardTray,tableDrawer_tableTop,keyboardTray>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_keyboardTray>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_keyboardTray_tableBack,tableDrawer_tableTop_keyboardTray,tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_keyboardTray_tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableBack,tableDrawer_tableTop,tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg,tableDrawer_tableTop,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_keyboardTray,tableDrawer_tableTop_tableLeg,keyboardTray>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_keyboardTray>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_keyboardTray_tableBack,tableDrawer_tableTop_tableLeg_keyboardTray,tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_keyboardTray_tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_keyboardTray_tableBack_tableLeg,tableDrawer_tableTop_tableLeg_keyboardTray_tableBack,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_keyboardTray_tableBack_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_keyboardTray_tableLeg,tableDrawer_tableTop_tableLeg_keyboardTray,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_keyboardTray_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_tableBack,tableDrawer_tableTop_tableLeg,tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_tableBack_tableDrawer,tableDrawer_tableTop_tableLeg_tableBack,tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_tableBack_tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_tableDrawer,tableDrawer_tableTop_tableLeg,tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_tableDrawer>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableDrawer_tableTop_tableLeg_tableLeg,tableDrawer_tableTop_tableLeg,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableDrawer_tableTop_tableLeg_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableLeg_tableLeg,tableLeg,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableLeg_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableLeg_tableTop,tableLeg,tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableLeg_tableTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableLeg_tableTop_keyboardTray,tableLeg_tableTop,keyboardTray>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableLeg_tableTop_keyboardTray>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableLeg_tableTop_keyboardTray_tableLeg,tableLeg_tableTop_keyboardTray,tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableLeg_tableTop_keyboardTray_tableLeg>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<tableLeg_tableTop_tableBack,tableLeg_tableTop,tableBack>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<Table,tableLeg_tableTop_tableBack>()));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,CPU>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,printer>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,keyboard>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,keyboardTray>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,monitor>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,SupportComplex<CPU> >(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,paper>(temp)));
	temp.clear();
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(CPU).name());
	appendRuleInstance(learningRules,RulePtr(new DoubleRuleComplex<Table,mug>(temp)));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<cupboardFront_cupboardSide,cupboardFront,cupboardSide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<cupboard,cupboardFront_cupboardSide>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<dustbinNarrow_dustbinWide,dustbinNarrow,dustbinWide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<dustbin,dustbinNarrow_dustbinWide>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<pillarLeft_pillarRight,pillarLeft,pillarRight>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<pillar,pillarLeft_pillarRight>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<pillarLeft_pillarRight_pillarMid,pillarLeft_pillarRight,pillarMid>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<pillar,pillarLeft_pillarRight_pillarMid>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<printerFront_printerSide,printerFront,printerSide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printer,printerFront_printerSide>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<printerFront_printerTop,printerFront,printerTop>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printer,printerFront_printerTop>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<printerFront_printerTop_printerSide,printerFront_printerTop,printerSide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printer,printerFront_printerTop_printerSide>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<printerFront_printerTop_printerSide_printerTray,printerFront_printerTop_printerSide,printerTray>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<printer,printerFront_printerTop_printerSide_printerTray>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<chairBackRest_sofaBase,chairBackRest,sofaBase>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofa,chairBackRest_sofaBase>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofa,sofaBackRest_sofaBase>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<sofaBackRest_sofaBase_sofaArm,sofaBackRest_sofaBase,sofaArm>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofa,sofaBackRest_sofaBase_sofaArm>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<sofaBackRest_sofaBase_sofaFrontBelowSeat,sofaBackRest_sofaBase,sofaFrontBelowSeat>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofa,sofaBackRest_sofaBase_sofaFrontBelowSeat>()));
	appendRuleInstance(learningRules,RulePtr(new DoubleRule<sofaBackRest_sofaBase_sofaFrontBelowSeat_sofaSide,sofaBackRest_sofaBase_sofaFrontBelowSeat,sofaSide>()));
	appendRuleInstance(learningRules,RulePtr(new SingleRule<sofa,sofaBackRest_sofaBase_sofaFrontBelowSeat_sofaSide>()));
}
