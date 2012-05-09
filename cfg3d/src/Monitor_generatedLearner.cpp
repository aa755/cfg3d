#include <queue>
#include "Monitor_generatedDataStructures.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol::SPtr> nodesCreatedSoFar;
    vector<Terminal_SPtr> temp;

    SingleRule<monitor, Plane> rulemonitor(true);
    nodesCreatedSoFar.push(rulemonitor.applyRuleLearning(labelToPlanes.at("monitor"), temp));

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
