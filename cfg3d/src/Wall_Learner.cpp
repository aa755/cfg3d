/* 
 * File:   Wall_Learner.cpp
 * Author: aa755
 *
 * Created on January 26, 2012, 9:37 PM
 */
#include <queue>
#include "Rules_Wall.h"
#include "helper.cpp"
void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {
    initialize(sceneToLearn, segmentToLabelFile);
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;

    cerr<<"map-size:"<<labelToPlanes.size()<<endl;
    SingleRule<Wall, Plane> ruleCPUFront(true);
    for(map<string, Plane*>::iterator it=labelToPlanes.begin();it!=labelToPlanes.end();it++)
    {
        cout<<"writing 1 rule\n";
        ruleCPUFront.applyRuleLearning(it->second, temp);
    }
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
