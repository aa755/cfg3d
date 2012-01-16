#include <queue>
#include "CPU_generatedDataStructures.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;

    SingleRule<CPUTop, Plane> ruleCPUTop(true);
    nodesCreatedSoFar.push(ruleCPUTop.applyRuleLearning(labelToPlanes.at("CPUTop"), temp));

    SingleRule<CPUFront, Plane> ruleCPUFront(true);
    nodesCreatedSoFar.push(ruleCPUFront.applyRuleLearning(labelToPlanes.at("CPUFront"), temp));

    DoubleRule<CPUTop_CPUFront, CPUTop, CPUFront> ruleCPUTop_CPUFront(true);;
    CPUTop* secondToLastCPUTop = dynamic_cast<CPUTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPUFront* lastCPUFront = dynamic_cast<CPUFront*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUTop_CPUFront.applyRuleLearning(secondToLastCPUTop, lastCPUFront, temp));

    SingleRule<CPULSide, Plane> ruleCPULSide(true);
    nodesCreatedSoFar.push(ruleCPULSide.applyRuleLearning(labelToPlanes.at("CPULSide"), temp));

    DoubleRule<CPUTop_CPUFront_CPULSide, CPUTop_CPUFront, CPULSide> ruleCPUTop_CPUFront_CPULSide(true);;
    CPUTop_CPUFront* secondToLastCPUTop_CPUFront = dynamic_cast<CPUTop_CPUFront*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPULSide* lastCPULSide = dynamic_cast<CPULSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUTop_CPUFront_CPULSide.applyRuleLearning(secondToLastCPUTop_CPUFront, lastCPULSide, temp));

    SingleRule<CPURSide, Plane> ruleCPURSide(true);
    nodesCreatedSoFar.push(ruleCPURSide.applyRuleLearning(labelToPlanes.at("CPURSide"), temp));

    DoubleRule<CPUTop_CPUFront_CPULSide_CPURSide, CPUTop_CPUFront_CPULSide, CPURSide> ruleCPUTop_CPUFront_CPULSide_CPURSide(true);;
    CPUTop_CPUFront_CPULSide* secondToLastCPUTop_CPUFront_CPULSide = dynamic_cast<CPUTop_CPUFront_CPULSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPURSide* lastCPURSide = dynamic_cast<CPURSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPUTop_CPUFront_CPULSide_CPURSide.applyRuleLearning(secondToLastCPUTop_CPUFront_CPULSide, lastCPURSide, temp));

    SingleRule<CPUBack, Plane> ruleCPUBack(true);
    nodesCreatedSoFar.push(ruleCPUBack.applyRuleLearning(labelToPlanes.at("CPUBack"), temp));

    DoubleRule<CPU, CPUTop_CPUFront_CPULSide_CPURSide, CPUBack> ruleCPU(true);;
    CPUTop_CPUFront_CPULSide_CPURSide* secondToLastCPUTop_CPUFront_CPULSide_CPURSide = dynamic_cast<CPUTop_CPUFront_CPULSide_CPURSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    CPUBack* lastCPUBack = dynamic_cast<CPUBack*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleCPU.applyRuleLearning(secondToLastCPUTop_CPUFront_CPULSide_CPURSide, lastCPUBack, temp));

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
