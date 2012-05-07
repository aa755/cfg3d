
/* 
 * File:   mcmcParse.cpp
 * Author: aa755
 *
 * Created on April 12, 2012, 8:43 PM
 */

#include "mcmcParsh.h"

int main(int argc, char** argv)
{

    vector<Terminal *>  terminals;
    SceneInfo::SPtr sceneInfo=initParsing(argc,argv);
    
    Forest forest(sceneInfo, RulesDB::SPtr(new RulesDB()));
    forest.runMCMC();
    return 0;
}

