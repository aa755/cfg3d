
/* 
 * File:   mcmcParse.cpp
 * Author: aa755
 *
 * Created on April 12, 2012, 8:43 PM
 */
#define USING_SVM_FOR_LEARNING_CFG
#include "mcmcParsh.h"

int main(int argc, char** argv)
{

    SceneInfo::SPtr sceneInfo=initParsing(argc,argv);
    RulesDB::SPtr rules=RulesDB::SPtr(new RulesDB());
#ifdef USING_SVM_FOR_LEARNING_CFG
    rules->readModel("modelf");
    sceneInfo->setPsiSize(rules->getTotalNumParams());
#endif
    Forest forest(sceneInfo, rules );
    forest.runMCMC();
    return 0;
}

