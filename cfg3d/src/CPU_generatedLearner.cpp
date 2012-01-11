#include <queue>
#include "CPU_generatedDataStructures.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;

    SingleRule<CPUTop, Plane> ruleCPUTop(true);
    nodesCreatedSoFar.push(ruleCPUTop.applyRuleLearning(labelToPlanes.at("CPUTop"), temp));

    SingleRule<CPURSide, Plane> ruleCPURSide(true);
    nodesCreatedSoFar.push(ruleCPURSide.applyRuleLearning(labelToPlanes.at("CPURSide"), temp));

    DoubleRule<CPUTop_CPURSide, CPUTop, CPURSide> ruleCPUTop_CPURSide(true);;
    CPUTop* secondToLastCPUTop = dynamic_cast<CPUTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPURSide* lastCPURSide = dynamic_cast<CPURSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUTop_CPURSide.applyRuleLearning(secondToLastCPUTop, lastCPURSide, temp));

    SingleRule<CPUFront, Plane> ruleCPUFront(true);
    nodesCreatedSoFar.push(ruleCPUFront.applyRuleLearning(labelToPlanes.at("CPUFront"), temp));

    DoubleRule<CPUTop_CPURSide_CPUFront, CPUTop_CPURSide, CPUFront> ruleCPUTop_CPURSide_CPUFront(true);;
    CPUTop_CPURSide* secondToLastCPUTop_CPURSide = dynamic_cast<CPUTop_CPURSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPUFront* lastCPUFront = dynamic_cast<CPUFront*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUTop_CPURSide_CPUFront.applyRuleLearning(secondToLastCPUTop_CPURSide, lastCPUFront, temp));

    SingleRule<CPUBack, Plane> ruleCPUBack(true);
    nodesCreatedSoFar.push(ruleCPUBack.applyRuleLearning(labelToPlanes.at("CPUBack"), temp));

    DoubleRule<CPUTop_CPURSide_CPUFront_CPUBack, CPUTop_CPURSide_CPUFront, CPUBack> ruleCPUTop_CPURSide_CPUFront_CPUBack(true);;
    CPUTop_CPURSide_CPUFront* secondToLastCPUTop_CPURSide_CPUFront = dynamic_cast<CPUTop_CPURSide_CPUFront*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPUBack* lastCPUBack = dynamic_cast<CPUBack*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUTop_CPURSide_CPUFront_CPUBack.applyRuleLearning(secondToLastCPUTop_CPURSide_CPUFront, lastCPUBack, temp));

    SingleRule<CPULSide, Plane> ruleCPULSide(true);
    nodesCreatedSoFar.push(ruleCPULSide.applyRuleLearning(labelToPlanes.at("CPULSide"), temp));

    DoubleRule<CPU, CPUTop_CPURSide_CPUFront_CPUBack, CPULSide> ruleCPU(true);;
    CPUTop_CPURSide_CPUFront_CPUBack* secondToLastCPUTop_CPURSide_CPUFront_CPUBack = dynamic_cast<CPUTop_CPURSide_CPUFront_CPUBack*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPULSide* lastCPULSide = dynamic_cast<CPULSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPU.applyRuleLearning(secondToLastCPUTop_CPURSide_CPUFront_CPUBack, lastCPULSide, temp));

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
