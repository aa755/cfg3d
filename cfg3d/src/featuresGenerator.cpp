#include "structures.cpp"
#include "rulesGenerator.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace std;
using namespace boost;

ofstream outputDataStructuresCode;
ofstream outputLearnerCode;

void createDataStructuresImports() {
    string importString = "#include ";

    string import = "\"helper.cpp\"";
    outputDataStructuresCode<<importString.append(import)<<endl;
}

void createLearnerImports(string objectName) {
    string importString = "#include ";

    string import = "<queue>";
    outputLearnerCode<<importString.append(import)<<endl;
    
    importString = "#include ";
    import = "generatedDataStructures.cpp";
    outputLearnerCode<<importString<<"\""<<objectName<<"_"<<import<<"\""<<endl;
}

// Data Structures
//void createClassAndWrite(string dataStructure) {
//    string front = "class ";
//    string className = dataStructure;
//    if (className != "Plane") {
//        string back = " : public NonTerminalIntermediate{};";
//        outputDataStructuresCode<<front.append(className).append(back)<<endl;
//    }
//}

void createNonIntermediateClassAndWrite(string dataStructure) {
    string front = "class ";
    string className = dataStructure;
    string back = " : public PlanarPrimitive{};";
    outputDataStructuresCode<<front.append(className).append(back)<<endl;
}

void createIntermediateClassAndWrite(string dataStructure) {
    string front = "class ";
    string className = dataStructure;
    string back = " : public NonTerminalIntermediate{};";
    outputDataStructuresCode<<front.append(className).append(back)<<endl;
}

void createGoal(string dataStructure) {
    string front = "class ";
    string className = dataStructure;
    string back = " : public NonTerminal{};";
    outputDataStructuresCode<<front.append(className).append(back)<<endl;
}

void writeIntermediateDataStructuresToFile(set<string> dataStructuresSet) {
    BOOST_FOREACH(string dataStructure, dataStructuresSet) {
        createIntermediateClassAndWrite(dataStructure);
    }
}

void writeNonIntermediateDataStructuresToFile(set<string> dataStructuresSet) {
    BOOST_FOREACH(string dataStructure, dataStructuresSet) {
        createNonIntermediateClassAndWrite(dataStructure);
    }
}

void createDataStructureFromString(string line, set<string>& intermediateDataStructuresSet, set<string>& nonIntermediateDataStructuresSet, string objectName) {
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    vector<string> tokenList;
    BOOST_FOREACH(string t, tokens)
    {   
        tokenList.push_back(t);
    }
    
    if (tokenList.at(0) == objectName) {
        createGoal(objectName);
    }
    else if (tokenList.size() == 2) {        
        nonIntermediateDataStructuresSet.insert(tokenList.at(0));
    } else if (tokenList.size() == 3) {
        intermediateDataStructuresSet.insert(tokenList.at(0));
    }
}

void createDataStructures(const char* file, string objectName) {
    ifstream rulesFile;
    rulesFile.open(file);

    string line;
    set<string> intermediateDataStructuresSet;
    set<string> nonIntermediateDataStructuresSet;
    if (rulesFile.is_open())
    {
        while (rulesFile.good()) {
            getline(rulesFile, line);
            if (line.size() == 0) 
            {
                break;
            }
            createDataStructureFromString(line, intermediateDataStructuresSet, nonIntermediateDataStructuresSet, objectName);
        }
    }
    writeIntermediateDataStructuresToFile(intermediateDataStructuresSet);
    writeNonIntermediateDataStructuresToFile(nonIntermediateDataStructuresSet);
    
    outputDataStructuresCode<<endl;
    outputDataStructuresCode<<"void "<<objectName<<"AppendLearningRules(vector<RulePtr>& learningRules) {"<<endl;    
}


// Learner      // Front
void createRunLearnFront() {
    outputLearnerCode<<"void runLearn(pcl::PointCloud<PointT> sceneToLearn, char* segmentToLabelFile) {"<<endl;
    outputLearnerCode<<"    initialize(sceneToLearn, segmentToLabelFile);"<<endl;
    outputLearnerCode<<"    queue<Symbol*> nodesCreatedSoFar;"<<endl;
    outputLearnerCode<<"    vector<Terminal*> temp;"<<endl<<endl;
}

                // Mid
void applySingleRule(string ruleName, string terminalLabel) {
    string front = "    nodesCreatedSoFar.push(";
    outputLearnerCode<<front.append(ruleName).append(".applyRuleLearning(labelToPlanes.at(\"").append(terminalLabel).append("\"), temp));")<<endl<<endl;
}

void applyDoubleRule(string ruleName, string LHS, string RHS1, string RHS2) {
    string tab1 = "    ";
    string temp1 = "";
    string secondToLast = temp1.append("secondToLast").append(RHS1);
    outputLearnerCode<<tab1.append(RHS1).append("* ").append(secondToLast).append(" = dynamic_cast<").append(RHS1).append("*>(nodesCreatedSoFar.front());")<<endl;
    outputLearnerCode<<"    nodesCreatedSoFar.pop();"<<endl;
    string tab2 = "    ";
    string temp2 = "";
    string last = temp2.append("last").append(RHS2);
    outputLearnerCode<<tab2.append(RHS2).append("* ").append(last).append(" = dynamic_cast<").append(RHS2).append("*>(nodesCreatedSoFar.front());")<<endl;
    outputLearnerCode<<"    nodesCreatedSoFar.pop();"<<endl;
    string front = "    nodesCreatedSoFar.push(";
    outputLearnerCode<<front.append(ruleName).append(".applyRuleLearning(").append(secondToLast).append(", ").append(last).append(", temp));")<<endl<<endl;
}

void createApplyRule(vector<string> rulesVector) {
    int count =  rulesVector.size();
    string LHS = rulesVector.at(0);
    string RHS;
    string ruleType;
    if (count == 2) {
        RHS = rulesVector.at(1);
        string tab = "    ";
        string ruleType = "SingleRule";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        string learningRule = ruleType.append("<").append(LHS).append(", ").append(RHS).append("> ").append(ruleName).append("(true)");
        ruleType = "SingleRule";
        string inferenceRule = ruleType.append("<").append(LHS).append(", ").append(RHS).append(">").append("()");
        outputLearnerCode<<tab<<learningRule<<";"<<endl;
        
        string newString = "    learningRules.push_back(RulePtr(new ";
        outputDataStructuresCode<<newString.append(inferenceRule).append("));")<<endl;
        
        applySingleRule(ruleName, LHS);
    }
    else if (count == 3) {
        string RHS1 = rulesVector.at(1);
        string RHS2 = rulesVector.at(2);
        string tab = "    ";
        ruleType = "DoubleRule";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        string learningRule = ruleType.append("<").append(LHS).append(", ").append(RHS1).append(", ").append(RHS2).append("> ").append(ruleName).append("(true);");
        ruleType = "DoubleRule";
        string inferenceRule = ruleType.append("<").append(LHS).append(", ").append(RHS1).append(", ").append(RHS2).append(">").append("()");
        outputLearnerCode<<tab<<learningRule<<";"<<endl;
        
        string newString = "    learningRules.push_back(RulePtr(new ";
        outputDataStructuresCode<<newString.append(inferenceRule).append("));")<<endl;
        
        applyDoubleRule(ruleName, LHS, RHS1, RHS2);
    }    
}

void parseApplyRule(string line) {
    vector<string> rulesVector;
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    BOOST_FOREACH(string t, tokens)
    {
        rulesVector.push_back(t);
    }
    
    createApplyRule(rulesVector);
}

void createRunLearnMid(const char* rulesFile) {
    ifstream rulesFileStream;
    rulesFileStream.open(rulesFile);

    string line;
    if (rulesFileStream.is_open())
    {
        while (rulesFileStream.good()) {
            getline(rulesFileStream, line);
            if (line.size() == 0) 
            {
                break;
            }
            parseApplyRule(line);
        }
    }
}

                // Back
void createRunLearnBack() {
    outputLearnerCode<<"}"<<endl<<endl;
    
    outputLearnerCode<<"int main(int argc, char** argv) {"<<endl;
    outputLearnerCode<<"if(argc!=3)\n{\ncerr<<\"usage:\"<<argv[0]<<\" <PCDFile> <seg2labelFile> \"<<endl;\n exit(-1);\n}\n";        

    outputLearnerCode<<"    pcl::io::loadPCDFile<PointT>(argv[1], scene);"<<endl;
    outputLearnerCode<<"    runLearn(scene, argv[2]);"<<endl;
    outputLearnerCode<<"}"<<endl;

}

void createLearner(const char* rulesFile) {
        createRunLearnFront();
        createRunLearnMid(rulesFile);
        createRunLearnBack();
}

int main(int argc, char** argv) {
    
    ///////////////////// Generating Rules ////////////////////
    
    if(argc!=4 && argc!=5)
    {
        cerr<<"usage:"<<argv[0]<<" <neighborMapFile> <seg2labelFile> <ObjectName> [<Order>]"<<endl;
        exit(-1);        
    }
    
    map<int, set<int> > nodeToNeighbors;
    parseGraph(argv[1], nodeToNeighbors);
    
    map<int, string> nodeToLabel;
    map<string, int> labelToNode;
    parseNodeToLabel(argv[2], nodeToLabel, labelToNode);
    
    string fileFront = argv[3];
    fileFront.append("Rules.txt");
    rulesFile.open(fileFront.c_str());
    
    if (argc==4) {
        generateRules(nodeToNeighbors, nodeToLabel, argv[3]);
    } else if (argc==5) {
        generateRulesInOrder(nodeToNeighbors, labelToNode, argv[4], argv[3]);
    }
    
    string dataStructuresFileFront = "";
    dataStructuresFileFront.append(argv[3]).append("_generatedDataStructures.cpp");
    outputDataStructuresCode.open(dataStructuresFileFront.c_str());
    const char* rulesFile = fileFront.c_str();
    createDataStructuresImports();
    createDataStructures(rulesFile, argv[3]);
    
    ///////////////////// Generating Learner ////////////////////
    string learnerFileFront = "";
    learnerFileFront.append(argv[3]).append("_generatedLearner.cpp");
    outputLearnerCode.open(learnerFileFront.c_str());
    createLearnerImports(argv[3]);
    createLearner(rulesFile);
    outputDataStructuresCode<<"}"<<endl;
    
    return 0;
}
