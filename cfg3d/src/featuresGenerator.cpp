#include "structures.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace std;
using namespace boost;

ofstream outputDataStructuresCode;
ofstream outputLearnerCode;

// Imports
//void createImport(string import, ofstream outputStream) {
//    string importString = "#include ";
//    outputStream<<importString.append(import)<<endl;
//}

void createDataStructuresImports() {
    string importString = "#include ";

    string import = "\"../src/helper.cpp\"";
    outputDataStructuresCode<<importString.append(import)<<endl;
}

void createLearnerImports() {
    string importString = "#include ";

    string import = "<queue>";
    outputLearnerCode<<importString.append(import)<<endl;
    
    importString = "#include ";
    import = "\"generatedDataStructures.cpp\"";
    outputLearnerCode<<importString.append(import)<<endl;
}

// Data Structures
void createClassAndWrite(string dataStructure) {
    string front = "class ";
    string className = dataStructure;
    if (className != "Plane") {
        string back = " : public NonTerminalIntermediate{};";
        outputDataStructuresCode<<front.append(className).append(back)<<endl;
    }
}

void writeDataStructuresToFile(set<string> dataStructuresSet) {
    BOOST_FOREACH(string dataStructure, dataStructuresSet) {
        createClassAndWrite(dataStructure);
    }
}

void createDataStructureFromString(string line, set<string>& dataStructuresSet) {
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    BOOST_FOREACH(string t, tokens)
    {
        dataStructuresSet.insert(t);
    }
}

void createDataStructures(char* file) {
    ifstream rulesFile;
    rulesFile.open(file);

    string line;
    set<string> dataStructuresSet;
    if (rulesFile.is_open())
    {
        while (rulesFile.good()) {
            getline(rulesFile, line);
            if (line.size() == 0) 
            {
                break;
            }
            createDataStructureFromString(line, dataStructuresSet);
        }
    }
    
    writeDataStructuresToFile(dataStructuresSet);
}

// Learner
        // Front
void createRunLearnFront() {
    outputLearnerCode<<"void runLearn(pcl::PointCloud<PointT> sceneToLearn) {"<<endl;
    outputLearnerCode<<"    initialize(sceneToLearn, \"segmentToLabel.txt\");"<<endl;
    outputLearnerCode<<"    queue<Symbol*> nodesCreatedSoFar;"<<endl;
    outputLearnerCode<<"    vector<Terminal*> temp;"<<endl;
}

        // Mid
void applySingleRule(string ruleName, string terminalLabel) {
    string front = "    nodesCreatedSoFar.push(";
    outputLearnerCode<<front.append(ruleName).append(".applyRuleLearning(labelToPlanes.at(\"").append(terminalLabel).append("\"), temp));")<<endl;
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
    outputLearnerCode<<front.append(ruleName).append(".applyRuleLearning(").append(secondToLast).append(", ").append(last).append(", temp));")<<endl;
}

void createApplyRule(vector<string> rulesVector) {
    int count =  rulesVector.size();
    string LHS = rulesVector.at(0);
    string RHS;
    string rule;
    string ruleType;
    if (count == 2) {
        RHS = rulesVector.at(1);
        string ruleType = "    SingleRule";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        rule = ruleType.append("<").append(LHS).append(", ").append(RHS).append("> ").append(ruleName).append("(true);");
        outputLearnerCode<<rule<<endl;
        applySingleRule(ruleName, LHS);
        outputLearnerCode<<endl;
    }
    else if (count == 3) {
        string RHS1 = rulesVector.at(1);
        string RHS2 = rulesVector.at(2);
        ruleType = "    DoubleRule";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        rule = ruleType.append("<").append(LHS).append(", ").append(RHS1).append(", ").append(RHS2).append("> ").append(ruleName).append("(true);");
        outputLearnerCode<<rule<<endl;
        applyDoubleRule(ruleName, LHS, RHS1, RHS2);
        outputLearnerCode<<endl;
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

void createRunLearnMid(char* rulesFile) {
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
    outputLearnerCode<<"    pcl::PointCloud<PointT> scene;"<<endl;
    outputLearnerCode<<"    pcl::io::loadPCDFile<PointT>(argv[1], scene);"<<endl;
    outputLearnerCode<<"    runLearn(scene);"<<endl;
    outputLearnerCode<<"}"<<endl;

}

void createLearner(char* rulesFile) {
        createRunLearnFront();
        createRunLearnMid(rulesFile);
        createRunLearnBack();
}

int main(int argc, char** argv) {
    outputDataStructuresCode.open("generatedDataStructures.cpp");
    char* rulesFile = "rules.txt";
    createDataStructuresImports();
    createDataStructures(rulesFile);
    
    outputLearnerCode.open("generatedLearner.cpp");
    createLearnerImports();
    createLearner(rulesFile);
    
    return 0;
}
