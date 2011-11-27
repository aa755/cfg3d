#include "structures.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
using namespace std;
using namespace boost;

ofstream outputCode;

// Imports
void createImport(string import) {
    string importString = "#include ";
    outputCode<<importString.append(import)<<endl;
}

void createImports() {
    createImport("\"../src/structures.cpp\"");
}

// Data Structures
void createClassAndWrite(string dataStructure) {
    string front = "class ";
    string className = dataStructure;
    if (className != "Terminal") {
        string back = " : public NonTerminalIntermediate{};";
        outputCode<<front.append(className).append(back)<<endl;
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

// Terminals
void appendTerminalToFile(vector<string> rulesVector) {
    int count =  rulesVector.size();
    string front;
    string nonTerminal;
    string back;
    if (count == 2) {
        front = "    labelToTerminal[\"";
        nonTerminal =  rulesVector.at(0);
        back = "\"] = new Terminal();";
        outputCode<<front.append(nonTerminal).append(back)<<endl;
    }
}

void createTerminalFromString(string line) {
    vector<string> ruleVector;
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    BOOST_FOREACH(string t, tokens)
    {
        ruleVector.push_back(t);
    }
    
    appendTerminalToFile(ruleVector);
}

void createTerminals(char* file) {
    ifstream rulesFile;
    outputCode<<"map<string, Terminal*> labelToTerminal;"<<endl;
    rulesFile.open(file);

    outputCode<<"void initializeTerminals() {"<<endl;
    string line;
    if (rulesFile.is_open())
    {
        while (rulesFile.good()) {
            getline(rulesFile, line);
            if (line.size() == 0) 
            {
                break;
            }
            
            createTerminalFromString(line);
        }
    }
    outputCode<<"}"<<endl;
}

int main(int argc, char** argv) {
    outputCode.open("parseStructure.cpp");
    char* fileName = "featuresInput.txt";
    createImports();
    createDataStructures(fileName);
    createTerminals(fileName);
    
    return 0;
}

//Intermediate NonTerminal