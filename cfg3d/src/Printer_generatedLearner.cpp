#include <queue>
#include "Printer_generatedDataStructures.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;

    SingleRule<printerFront, Plane> ruleprinterFront(true);
    nodesCreatedSoFar.push(ruleprinterFront.applyRuleLearning(labelToPlanes.at("printerFront"), temp));

    SingleRule<printerTop, Plane> ruleprinterTop(true);
    nodesCreatedSoFar.push(ruleprinterTop.applyRuleLearning(labelToPlanes.at("printerTop"), temp));

    DoubleRule<printerFront_printerTop, printerFront, printerTop> ruleprinterFront_printerTop(true);;
    printerFront* secondToLastprinterFront = dynamic_cast<printerFront*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    printerTop* lastprinterTop = dynamic_cast<printerTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleprinterFront_printerTop.applyRuleLearning(secondToLastprinterFront, lastprinterTop, temp));

    SingleRule<printerSide, Plane> ruleprinterRSide(true);
    nodesCreatedSoFar.push(ruleprinterRSide.applyRuleLearning(labelToPlanes.at("printerSide_right"), temp));

    DoubleRule<printerFront_printerTop_printerRSide, printerFront_printerTop, printerSide> ruleprinterFront_printerTop_printerRSide(true);;
    printerFront_printerTop* secondToLastprinterFront_printerTop = dynamic_cast<printerFront_printerTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    printerSide* lastprinterRSide = dynamic_cast<printerSide*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleprinterFront_printerTop_printerRSide.applyRuleLearning(secondToLastprinterFront_printerTop, lastprinterRSide, temp));

    DoubleRule<printerFront_printerRSide, printerFront, printerSide> ruleprinterFront_printerRSide(true);
    printerFront_printerRSide * fs = ruleprinterFront_printerRSide.applyRuleLearning(secondToLastprinterFront, lastprinterRSide, temp);
    DoubleRule<printerFront_printerRSide_printerTop, printerFront_printerRSide, printerTop> ruleprinterFront_printerRSide_printerTop(true);
    ruleprinterFront_printerRSide_printerTop.applyRuleLearning(fs, lastprinterTop, temp);
//    SingleRule<printerBack, Plane> ruleprinterBack(true);
//    nodesCreatedSoFar.push(ruleprinterBack.applyRuleLearning(labelToPlanes.at("printerBack"), temp));
//
//    DoubleRule<printerFront_printerTop_printerRSide_printerBack, printerFront_printerTop_printerRSide, printerBack> ruleprinterFront_printerTop_printerRSide_printerBack(true);;
//    printerFront_printerTop_printerRSide* secondToLastprinterFront_printerTop_printerRSide = dynamic_cast<printerFront_printerTop_printerRSide*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    printerBack* lastprinterBack = dynamic_cast<printerBack*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    nodesCreatedSoFar.push(ruleprinterFront_printerTop_printerRSide_printerBack.applyRuleLearning(secondToLastprinterFront_printerTop_printerRSide, lastprinterBack, temp));
//
    SingleRule<printerSide, Plane> ruleprinterLSide(true);
    ruleprinterLSide.applyRuleLearning(labelToPlanes.at("printerSide_left"), temp);
//    nodesCreatedSoFar.push(ruleprinterLSide.applyRuleLearning(labelToPlanes.at("printerLSide"), temp));
//
//    DoubleRule<printer, printerFront_printerTop_printerRSide_printerBack, printerLSide> ruleprinter(true);;
//    printerFront_printerTop_printerRSide_printerBack* secondToLastprinterFront_printerTop_printerRSide_printerBack = dynamic_cast<printerFront_printerTop_printerRSide_printerBack*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    printerLSide* lastprinterLSide = dynamic_cast<printerLSide*>(nodesCreatedSoFar.front());
//    nodesCreatedSoFar.pop();
//    nodesCreatedSoFar.push(ruleprinter.applyRuleLearning(secondToLastprinterFront_printerTop_printerRSide_printerBack, lastprinterLSide, temp));

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
