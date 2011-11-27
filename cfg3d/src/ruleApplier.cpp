//#include "../bin/generatedLearner.cpp"
#include "../bin/parseStructure.cpp"
//#include "helper.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;

ofstream outputDataStructuresCode;

// What generated code would look like
void runLearn() {
    queue<Symbol*> nodesCreatedSoFar;
    vector<Terminal*> temp;
    
    SingleRule<LTop, Terminal> ruleLTop;
    nodesCreatedSoFar.push(ruleLTop.applyRuleLearning(labelToTerminal.at("LTop"), temp));
    
    SingleRule<RTop, Terminal> ruleRTop;
    nodesCreatedSoFar.push(ruleRTop.applyRuleLearning(labelToTerminal.at("RTop"), temp));
    
    DoubleRule<LTop_RTop, LTop, RTop> ruleLTop_RTop;
    LTop* secondToLast = dynamic_cast<LTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop();
    RTop* last = dynamic_cast<RTop*>(nodesCreatedSoFar.front());
    nodesCreatedSoFar.pop(); 
//    nodesCreatedSoFar.push(ruleLTop_RTop.applyRuleLearning(secondToLast, last, temp));
}

void applySingleRule(string ruleName, string terminalLabel) {
    string front = "nodesCreatedSoFar.push(";
    outputDataStructuresCode<<front.append(ruleName).append(".applyRuleLearning(labelToTerminal.at(\"").append(terminalLabel).append("\"), temp));")<<endl;
}

void applyDoubleRule(string ruleName, string LHS, string RHS1, string RHS2) {
    outputDataStructuresCode<<RHS1.append("* secondToLast = dynamic_cast<").append(RHS1).append("*>(nodesCreatedSoFar.front());")<<endl;
    outputDataStructuresCode<<"nodesCreatedSoFar.pop();"<<endl;
    outputDataStructuresCode<<RHS2.append("* last = dynamic_cast<").append(RHS2).append("*>(nodesCreatedSoFar.front());")<<endl;
    outputDataStructuresCode<<"nodesCreatedSoFar.pop();"<<endl;
    string front = "nodesCreatedSoFar.push(";
    outputDataStructuresCode<<front.append(ruleName).append(".applyRuleLearning(secondToLast, last, temp));")<<endl;

}

// Rules
void createApplyRule(vector<string> rulesVector) {
    
    int count =  rulesVector.size();
    string LHS = rulesVector.at(0);
    string RHS;
    string rule;
    string ruleType;
    if (count == 2) {
        RHS = rulesVector.at(1);
        string ruleType = "    SingleRule(true)";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        rule = ruleType.append("<").append(LHS).append(", ").append(RHS).append("> ").append(ruleName).append(";");
        cout<<rule<<endl;
        applySingleRule(ruleName, RHS);
    }
    else if (count == 3) {
        string RHS1 = rulesVector.at(1);
        string RHS2 = rulesVector.at(2);
        ruleType = "    DoubleRule(true)";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        rule = ruleType.append("<").append(LHS).append(", ").append(RHS1).append(", ").append(RHS2).append("> ").append(ruleName).append(";");
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

void createRules(char* file) {
    ifstream rulesFile;
    outputDataStructuresCode<<"void runLearn() {"<<endl;
    outputDataStructuresCode<<"    initializeTerminals();"<<endl;
    outputDataStructuresCode<<"    queue<Symbol*> nodesCreatedSoFar;"<<endl;
    outputDataStructuresCode<<"    vector<Terminal*> temp;"<<endl;
    rulesFile.open(file);

    string line;
    if (rulesFile.is_open())
    {
        while (rulesFile.good()) {
            getline(rulesFile, line);
            if (line.size() == 0) 
            {
                break;
            }
            
            parseApplyRule(line);
        }
    }
    outputDataStructuresCode<<"}"<<endl;
}

//void loopOnRules(vector<RulePtr>& rules) {
//    vector<Symbol*> nodesCreatedSoFar;
//    while(!rules.empty()) {
//        
//    }
//}
//
int main(int argc, char** argv) {
    vector<RulePtr> rules; 
//    appendRuleInstances(rules);
    createRules("featuresInput.txt");
    return 0;
}
