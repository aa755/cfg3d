#include <queue>
#include "Laptop_generatedDataStructures.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;

    SingleRule<LaptopScreen, Plane> ruleLaptopScreen(true);
    nodesCreatedSoFar.push(ruleLaptopScreen.applyRuleLearning(labelToPlanes.at("LaptopScreen"), temp));

    SingleRule<LaptopKeyboard, Plane> ruleLaptopKeyboard(true);
    nodesCreatedSoFar.push(ruleLaptopKeyboard.applyRuleLearning(labelToPlanes.at("LaptopKeyboard"), temp));

    DoubleRule<Laptop, LaptopScreen, LaptopKeyboard> ruleLaptop(true);;
    LaptopScreen* secondToLastLaptopScreen = dynamic_cast<LaptopScreen*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    LaptopKeyboard* lastLaptopKeyboard = dynamic_cast<LaptopKeyboard*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    nodesCreatedSoFar.push(ruleLaptop.applyRuleLearning(secondToLastLaptopScreen, lastLaptopKeyboard, temp));

}

int main(int argc, char** argv) {
    pcl::PointCloud<PointT> scene;
    pcl::io::loadPCDFile<PointT>(argv[1], scene);
    runLearn(scene, argv[2]);
}
