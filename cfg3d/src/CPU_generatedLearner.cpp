#include <queue>
#include "CPU_generatedDataStructures.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;

    SingleRule<CPUFront, Plane> ruleCPUFront(true);
    nodesCreatedSoFar.push(ruleCPUFront.applyRuleLearning(labelToPlanes.at("CPUFront"), temp));

    SingleRule<CPUTop, Plane> ruleCPUTop(true);
    nodesCreatedSoFar.push(ruleCPUTop.applyRuleLearning(labelToPlanes.at("CPUTop"), temp));

    DoubleRule<CPUFront_CPUTop, CPUFront, CPUTop> ruleCPUFront_CPUTop(true);;
    CPUFront* secondToLastCPUFront = dynamic_cast<CPUFront*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPUTop* lastCPUTop = dynamic_cast<CPUTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUFront_CPUTop.applyRuleLearning(secondToLastCPUFront, lastCPUTop, temp));

    SingleRule<CPUSide, Plane> ruleCPURSide(true);
    nodesCreatedSoFar.push(ruleCPURSide.applyRuleLearning(labelToPlanes.at("CPURSide"), temp));

    DoubleRule<CPUFront_CPUTop_CPURSide, CPUFront_CPUTop, CPUSide> ruleCPUFront_CPUTop_CPURSide(true);;
    CPUFront_CPUTop* secondToLastCPUFront_CPUTop = dynamic_cast<CPUFront_CPUTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPUSide* lastCPURSide = dynamic_cast<CPUSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUFront_CPUTop_CPURSide.applyRuleLearning(secondToLastCPUFront_CPUTop, lastCPURSide, temp));

    DoubleRule<CPUFront_CPURSide, CPUFront, CPUSide> ruleCPUFront_CPURSide(true);
    CPUFront_CPURSide * fs = ruleCPUFront_CPURSide.applyRuleLearning(secondToLastCPUFront, lastCPURSide, temp);
    DoubleRule<CPUFront_CPURSide_CPUTop, CPUFront_CPURSide, CPUTop> ruleCPUFront_CPURSide_CPUTop(true);
    ruleCPUFront_CPURSide_CPUTop.applyRuleLearning(fs, lastCPUTop, temp);
//    SingleRule<CPUBack, Plane> ruleCPUBack(true);
//    nodesCreatedSoFar.push(ruleCPUBack.applyRuleLearning(labelToPlanes.at("CPUBack"), temp));
//
//    DoubleRule<CPUFront_CPUTop_CPURSide_CPUBack, CPUFront_CPUTop_CPURSide, CPUBack> ruleCPUFront_CPUTop_CPURSide_CPUBack(true);;
//    CPUFront_CPUTop_CPURSide* secondToLastCPUFront_CPUTop_CPURSide = dynamic_cast<CPUFront_CPUTop_CPURSide*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    CPUBack* lastCPUBack = dynamic_cast<CPUBack*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    nodesCreatedSoFar.push(ruleCPUFront_CPUTop_CPURSide_CPUBack.applyRuleLearning(secondToLastCPUFront_CPUTop_CPURSide, lastCPUBack, temp));
//
    SingleRule<CPUSide, Plane> ruleCPULSide(true);
    ruleCPULSide.applyRuleLearning(labelToPlanes.at("CPULSide"), temp);
//    nodesCreatedSoFar.push(ruleCPULSide.applyRuleLearning(labelToPlanes.at("CPULSide"), temp));
//
//    DoubleRule<CPU, CPUFront_CPUTop_CPURSide_CPUBack, CPULSide> ruleCPU(true);;
//    CPUFront_CPUTop_CPURSide_CPUBack* secondToLastCPUFront_CPUTop_CPURSide_CPUBack = dynamic_cast<CPUFront_CPUTop_CPURSide_CPUBack*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    CPULSide* lastCPULSide = dynamic_cast<CPULSide*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    nodesCreatedSoFar.push(ruleCPU.applyRuleLearning(secondToLastCPUFront_CPUTop_CPURSide_CPUBack, lastCPULSide, temp));

}

int main(int argc, char** argv) {
if(argc!=3)
{
cerr<<"usage:"<<argv[0]<<" <PCDFile> <seg2labelFile> "<<endl;
 exit(-1);
}
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
    runLearn(scene, argv[2]);
}
