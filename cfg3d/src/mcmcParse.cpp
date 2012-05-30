
/* 
 * File:   mcmcParse.cpp
 * Author: aa755
 *
 * Created on April 12, 2012, 8:43 PM
 */
//#define USING_SVM_FOR_LEARNING_CFG
#define USING_BEAM_SEARCH_FOR_INFERENCE
#include "mcmcParsh.h"
int main(int argc, char** argv)
{

    SceneInfo::SPtr sceneInfo=initParsing(argc,argv);
    RulesDB::SPtr rules=RulesDB::SPtr(new RulesDB());
#ifdef USING_SVM_FOR_LEARNING_CFG
    rules->readModel("modelf");
#endif
    
    Forest::SPtr forest(new Forest(sceneInfo, rules ));
#ifdef USING_BEAM_SEARCH_FOR_INFERENCE
  BeamSearch beamS(forest,50);
  beamS.runBeamSearch();
  beamS.printParsingResult();  
#else
    
    forest->runMCMC();
#endif
    
    return 0;
}

