#include "helper.cpp"

class sofaArm : public PlanarPrimitive{};
class monitor : public PlanarPrimitive{};
class Chair : public NonTerminal{};
class tableTop : public PlanarPrimitive{};
class sofaBackRest : public PlanarPrimitive{};
class window : public PlanarPrimitive{};
class CPUSide : public PlanarPrimitive{};
class cupboardFront : public PlanarPrimitive{};
class CPU : public NonTerminal{};
class cupboardSide : public PlanarPrimitive{};
class CPUTop : public PlanarPrimitive{};
class pillar : public NonTerminal{};
class printerFront : public PlanarPrimitive{};
class printerTray : public PlanarPrimitive{};
class paper : public PlanarPrimitive{};
class dustbinNarrow : public PlanarPrimitive{};
class Wall : public PlanarPrimitive{};
class chairArmRest : public PlanarPrimitive{};
class printerSide : public PlanarPrimitive{};
class dustbinWide : public PlanarPrimitive{};
class keyboard : public PlanarPrimitive{};
class FridgeSide : public PlanarPrimitive{};
class tableDrawer : public PlanarPrimitive{};
class tableBack : public PlanarPrimitive{};
class dustbin : public NonTerminal{};
class Floor : public PlanarPrimitive{};
class WhiteBoardRack : public PlanarPrimitive{};
class pillarRight : public PlanarPrimitive{};
class ACVent : public PlanarPrimitive{};
class printer : public NonTerminal{};
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
class pillarLeft : public PlanarPrimitive{};
class chairBack : public PlanarPrimitive{};
class chairBackRest : public PlanarPrimitive{};
class sofaSide : public PlanarPrimitive{};
class sofaFrontBelowSeat : public PlanarPrimitive{};
class door : public PlanarPrimitive{};
class FridgeDoor : public PlanarPrimitive{};
class CPUSide_CPUFront : public NonTerminalIntermediate{};
class CPUSide_CPUFront_CPUTop : public NonTerminalIntermediate{};
class CPUSide_CPUFront_chairBack : public NonTerminalIntermediate{};
class CPUTop_CPUFront : public NonTerminalIntermediate{};
class chairArmRest_chairBack : public NonTerminalIntermediate{};
class chairArmRest_chairBackRest : public NonTerminalIntermediate{};
class chairArmRest_chairBack_chairBackRest : public NonTerminalIntermediate{};
class chairArmRest_chairBack_chairBase : public NonTerminalIntermediate{};
class chairBack_chairBackRest : public NonTerminalIntermediate{};
class chairBase_chairBackRest : public NonTerminalIntermediate{};
class chairBase_chairBackRest_chairArmRest : public NonTerminalIntermediate{};
class chairBase_chairBackRest_chairBack : public NonTerminalIntermediate{};
class FridgeTop_FridgeSide : public NonTerminalIntermediate{};
class FridgeTop_FridgeSide_FridgeDoor : public NonTerminalIntermediate{};
class Fridge : public NonTerminal{};
class sofaBackRest_sofaBase : public NonTerminalIntermediate{};
class Sofa : public NonTerminal{};
class keyboardTray_tableBack : public NonTerminalIntermediate{};
class keyboardTray_tableBack_tableDrawer : public NonTerminalIntermediate{};
class keyboardTray_tableTop : public NonTerminalIntermediate{};
class tableDrawer_tableLeg : public NonTerminalIntermediate{};
class tableLeg_tableLeg : public NonTerminalIntermediate{};
class tableLeg_tableTop : public NonTerminalIntermediate{};
class tableLeg_tableTop_keyboardTray : public NonTerminalIntermediate{};
class tableLeg_tableTop_keyboardTray_tableLeg : public NonTerminalIntermediate{};
class tableLeg_tableTop_tableBack : public NonTerminalIntermediate{};
class tableTop_tableDrawer : public NonTerminalIntermediate{};
class tableTop_tableDrawer_keyboardTray : public NonTerminalIntermediate{};
class tableTop_tableDrawer_keyboardTray_tableBack : public NonTerminalIntermediate{};
class tableTop_tableDrawer_tableBack : public NonTerminalIntermediate{};
class tableTop_tableDrawer_tableLeg : public NonTerminalIntermediate{};
class tableTop_tableDrawer_tableLeg_keyboardTray : public NonTerminalIntermediate{};
class tableTop_tableDrawer_tableLeg_keyboardTray_tableBack : public NonTerminalIntermediate{};
class tableTop_tableDrawer_tableLeg_tableBack : public NonTerminalIntermediate{};
class tableTop_tableDrawer_tableLeg_tableLeg : public NonTerminalIntermediate{};
class cupboardSide_cupboardFront : public NonTerminalIntermediate{};
class cupboard : public NonTerminal{};
class dustbinNarrow_dustbinWide : public NonTerminalIntermediate{};
class pillarRight_pillarLeft : public NonTerminalIntermediate{};
class pillarRight_pillarLeft_pillarMid : public NonTerminalIntermediate{};
class printerFront_printerTop : public NonTerminalIntermediate{};
class printerFront_printerTop_printerSide : public NonTerminalIntermediate{};
class printerFront_printerTop_printerSide_printerTray : public NonTerminalIntermediate{};
class printerSide_printerFront : public NonTerminalIntermediate{};
class chairBackRest_sofaBase : public NonTerminalIntermediate{};
class sofa : public NonTerminal{};
class sofaBackRest_sofaBase_sofaArm : public NonTerminalIntermediate{};
class sofaBackRest_sofaBase_sofaFrontBelowSeat : public NonTerminalIntermediate{};
class sofaBackRest_sofaBase_sofaFrontBelowSeat_sofaSide : public NonTerminalIntermediate{};
void appendLearningRules(vector<RulePtr>& learningRules) {
vector<string> temp;
	learningRules.push_back(RulePtr(new SingleRule<sofaArm,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<monitor,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<Chair,chairBackRest>()));
	learningRules.push_back(RulePtr(new SingleRule<tableTop,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<sofaBackRest,Plane>()));
	learningRules.push_back(RulePtr(new SingleRuleComplex<Table>()));
	temp.clear();
	temp.push_back(typeid(Table).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,Table>(temp)));
	learningRules.push_back(RulePtr(new SingleRule<window,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<CPUSide,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<cupboardFront,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<CPU,CPUFront>()));
	learningRules.push_back(RulePtr(new SingleRule<cupboardSide,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<CPUTop,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<pillar,pillarMid>()));
	learningRules.push_back(RulePtr(new SingleRule<printerFront,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<printerTray,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<paper,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<dustbinNarrow,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<Wall,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<chairArmRest,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<printerSide,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<dustbinWide,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<keyboard,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<FridgeSide,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<tableDrawer,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<tableBack,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<dustbin,dustbinWide>()));
	learningRules.push_back(RulePtr(new SingleRule<Floor,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<WhiteBoardRack,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<pillarRight,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<ACVent,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<printer,printerFront>()));
	learningRules.push_back(RulePtr(new SingleRule<printerTop,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<sofaBase,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<FridgeTop,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<Table,tableDrawer>()));
	learningRules.push_back(RulePtr(new SingleRule<mug,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<chairBase,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<tableLeg,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<Chair,chairBack>()));
	learningRules.push_back(RulePtr(new SingleRule<CPUFront,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<Chair,chairBase>()));
	learningRules.push_back(RulePtr(new SingleRule<pillarMid,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<keyboardTray,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<CPU,CPUTop>()));
	learningRules.push_back(RulePtr(new SingleRule<pillarLeft,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<chairBack,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<chairBackRest,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<sofaSide,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<sofaFrontBelowSeat,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<door,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<FridgeDoor,Plane>()));
	learningRules.push_back(RulePtr(new SingleRule<Table,tableTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<CPUSide_CPUFront,CPUSide,CPUFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<CPU,CPUSide,CPUFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<CPUSide_CPUFront_CPUTop,CPUSide_CPUFront,CPUTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<CPUSide_CPUFront_chairBack,CPUSide_CPUFront,chairBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<CPUTop_CPUFront,CPUTop,CPUFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<CPU,CPUTop,CPUFront>()));
	learningRules.push_back(RulePtr(new SingleRuleComplex<CPU>()));
	temp.clear();
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(paper).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<CPU,CPU>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<CPU,paper>(temp)));
	learningRules.push_back(RulePtr(new DoubleRule<chairArmRest_chairBack,chairArmRest,chairBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<Chair,chairArmRest,chairBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairArmRest_chairBackRest,chairArmRest,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<Chair,chairArmRest,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairArmRest_chairBack_chairBackRest,chairArmRest_chairBack,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairArmRest_chairBack_chairBase,chairArmRest_chairBack,chairBase>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairBack_chairBackRest,chairBack,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<Chair,chairBack,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairBase_chairBackRest,chairBase,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<Chair,chairBase,chairBackRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairBase_chairBackRest_chairArmRest,chairBase_chairBackRest,chairArmRest>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairBase_chairBackRest_chairBack,chairBase_chairBackRest,chairBack>()));
	learningRules.push_back(RulePtr(new SingleRuleComplex<Floor>()));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Floor).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,Wall>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,Chair>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,tableTop>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,Floor>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Sofa).name());
	temp.push_back(typeid(Floor).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,Sofa>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(Floor).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,SupportComplex<Table> >(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(CPU).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,ACVent>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,CPU>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(SupportComplex<WallH
>).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,dustbin>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,SupportComplex<WallH> >(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(pillar).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,pillar>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(ACVent).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(Wall).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(window).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,window>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(window).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(sofa).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,sofa>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(CPU).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(SupportComplex<Table
>).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(sofa).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Table).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,Table>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(SupportComplex<Table
>).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Wall).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(door).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,door>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(window).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(sofa).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(Chair).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Wall).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Chair).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(pillar).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(door).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(sofa).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Floor).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(Floor).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(tableTop).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(Floor).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,printer>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(dustbin).name());
	temp.push_back(typeid(cupboard).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Floor,cupboard>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(pillar).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(Chair).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Floor).name());
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(pillar).name());
	temp.clear();
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(ACVent).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(Floor).name());
	temp.clear();
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(Floor).name());
	temp.clear();
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Floor).name());
	learningRules.push_back(RulePtr(new SingleRuleComplex<FloorOccluded>()));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(Table).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,Wall>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,Table>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table
>).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,SupportComplex<Table> >(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Chair).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,Chair>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Chair).name());
	temp.push_back(typeid(CPU).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,CPU>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(SupportComplex<Table
>).name());
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(door).name());
	temp.push_back(typeid(cupboard).name());
	temp.push_back(typeid(tableTop).name());
	temp.push_back(typeid(Fridge).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,door>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,cupboard>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,tableTop>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,Fridge>(temp)));
	temp.clear();
	temp.push_back(typeid(Wall).name());
	temp.push_back(typeid(sofa).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,sofa>(temp)));
	temp.clear();
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(SupportComplex<Table
>).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,window>(temp)));
	temp.clear();
	temp.push_back(typeid(window).name());
	temp.push_back(typeid(SupportComplex<Table>).name());
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(pillar).name());
	temp.push_back(typeid(Chair).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<FloorOccluded,pillar>(temp)));
	learningRules.push_back(RulePtr(new DoubleRule<FridgeTop_FridgeSide,FridgeTop,FridgeSide>()));
	learningRules.push_back(RulePtr(new DoubleRule<FridgeTop_FridgeSide_FridgeDoor,FridgeTop_FridgeSide,FridgeDoor>()));
	learningRules.push_back(RulePtr(new DoubleRule<Fridge,FridgeTop,FridgeSide>()));
	learningRules.push_back(RulePtr(new DoubleRule<sofaBackRest_sofaBase,sofaBackRest,sofaBase>()));
	learningRules.push_back(RulePtr(new DoubleRule<Sofa,sofaBackRest,sofaBase>()));
	learningRules.push_back(RulePtr(new DoubleRule<keyboardTray_tableBack,keyboardTray,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<keyboardTray_tableBack_tableDrawer,keyboardTray_tableBack,tableDrawer>()));
	learningRules.push_back(RulePtr(new DoubleRule<Table,keyboardTray,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<keyboardTray_tableTop,keyboardTray,tableTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<Table,keyboardTray,tableTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableDrawer_tableLeg,tableDrawer,tableLeg>()));
	learningRules.push_back(RulePtr(new DoubleRule<Table,tableDrawer,tableLeg>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableLeg_tableLeg,tableLeg,tableLeg>()));
	learningRules.push_back(RulePtr(new DoubleRule<Table,tableLeg,tableLeg>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableLeg_tableTop,tableLeg,tableTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<Table,tableLeg,tableTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableLeg_tableTop_keyboardTray,tableLeg_tableTop,keyboardTray>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableLeg_tableTop_keyboardTray_tableLeg,tableLeg_tableTop_keyboardTray,tableLeg>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableLeg_tableTop_tableBack,tableLeg_tableTop,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer,tableTop,tableDrawer>()));
	learningRules.push_back(RulePtr(new DoubleRule<Table,tableTop,tableDrawer>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_keyboardTray,tableTop_tableDrawer,keyboardTray>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_keyboardTray_tableBack,tableTop_tableDrawer_keyboardTray,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_tableBack,tableTop_tableDrawer,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_tableLeg,tableTop_tableDrawer,tableLeg>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_tableLeg_keyboardTray,tableTop_tableDrawer_tableLeg,keyboardTray>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_tableLeg_keyboardTray_tableBack,tableTop_tableDrawer_tableLeg_keyboardTray,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_tableLeg_tableBack,tableTop_tableDrawer_tableLeg,tableBack>()));
	learningRules.push_back(RulePtr(new DoubleRule<tableTop_tableDrawer_tableLeg_tableLeg,tableTop_tableDrawer_tableLeg,tableLeg>()));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(CPU).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,CPU>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(printer).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,printer>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(keyboard).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,keyboard>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(keyboardTray).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,keyboardTray>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,monitor>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(CPU).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(SupportComplex<CPU
>).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,SupportComplex<CPU> >(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(SupportComplex<CPU>).name());
	temp.push_back(typeid(paper).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,paper>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(keyboard).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(monitor).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(monitor).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(CPU).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(paper).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboard).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboardTray).name());
	temp.push_back(typeid(paper).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(SupportComplex<CPU
>).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(keyboard).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(monitor).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(paper).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(keyboard).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(SupportComplex<CPU
>).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(mug).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(paper).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,mug>(temp)));
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(paper).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(CPU).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(monitor).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(paper).name());
	temp.push_back(typeid(paper).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(printer).name());
	temp.clear();
	temp.push_back(typeid(Table).name());
	temp.push_back(typeid(printer).name());
	temp.push_back(typeid(printer).name());
	temp.clear();
	temp.push_back(typeid(monitor).name());
	temp.push_back(typeid(tableTop).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<Table,tableTop>(temp)));
	learningRules.push_back(RulePtr(new SingleRuleComplex<WallH>()));
	temp.clear();
	temp.push_back(typeid(WhiteBoardRack).name());
	temp.push_back(typeid(Wall).name());
	learningRules.push_back(RulePtr(new DoubleRuleComplex<WallH,WhiteBoardRack>(temp)));
	learningRules.push_back(RulePtr(new DoubleRuleComplex<WallH,Wall>(temp)));
	learningRules.push_back(RulePtr(new DoubleRule<cupboardSide_cupboardFront,cupboardSide,cupboardFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<cupboard,cupboardSide,cupboardFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<dustbinNarrow_dustbinWide,dustbinNarrow,dustbinWide>()));
	learningRules.push_back(RulePtr(new DoubleRule<dustbin,dustbinNarrow,dustbinWide>()));
	learningRules.push_back(RulePtr(new DoubleRule<pillarRight_pillarLeft,pillarRight,pillarLeft>()));
	learningRules.push_back(RulePtr(new DoubleRule<pillar,pillarRight,pillarLeft>()));
	learningRules.push_back(RulePtr(new DoubleRule<pillarRight_pillarLeft_pillarMid,pillarRight_pillarLeft,pillarMid>()));
	learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerTop,printerFront,printerTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<printer,printerFront,printerTop>()));
	learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerTop_printerSide,printerFront_printerTop,printerSide>()));
	learningRules.push_back(RulePtr(new DoubleRule<printerFront_printerTop_printerSide_printerTray,printerFront_printerTop_printerSide,printerTray>()));
	learningRules.push_back(RulePtr(new DoubleRule<printerSide_printerFront,printerSide,printerFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<printer,printerSide,printerFront>()));
	learningRules.push_back(RulePtr(new DoubleRule<chairBackRest_sofaBase,chairBackRest,sofaBase>()));
	learningRules.push_back(RulePtr(new DoubleRule<sofa,chairBackRest,sofaBase>()));
	learningRules.push_back(RulePtr(new DoubleRule<sofa,sofaBackRest,sofaBase>()));
	learningRules.push_back(RulePtr(new DoubleRule<sofaBackRest_sofaBase_sofaArm,sofaBackRest_sofaBase,sofaArm>()));
	learningRules.push_back(RulePtr(new DoubleRule<sofaBackRest_sofaBase_sofaFrontBelowSeat,sofaBackRest_sofaBase,sofaFrontBelowSeat>()));
	learningRules.push_back(RulePtr(new DoubleRule<sofaBackRest_sofaBase_sofaFrontBelowSeat_sofaSide,sofaBackRest_sofaBase_sofaFrontBelowSeat,sofaSide>()));
}
