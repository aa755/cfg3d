#include "../bin/parseStructure.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;

ofstream outputCode;

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
//    ruleLTop_RTop.applyRuleLearning(secondToLast, last, temp);
}

void applySingleRule(string ruleName, string terminalLabel) {
    string front = "nodesCreatedSoFar.push(";
    outputCode<<front.append(ruleName).append(".applyRuleLearning(labelToTerminal.at(\"").append(terminalLabel).append("\"), temp));")<<endl;
}

// Rules
void applyRuleToFile(vector<string> rulesVector) {
    
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
        rule = ruleType.append("<").append(LHS).append(", ").append(RHS).append("> ").append(ruleName).append(";");
        cout<<rule<<endl;
        applySingleRule(ruleName, RHS);
    }
    else if (count == 3) {
        string RHS1 =  rulesVector.at(1);
        string RHS2 =  rulesVector.at(2);
        ruleType = "    DoubleRule";
        string temp = "";
        string ruleName = temp.append("rule").append(LHS);
        rule = ruleType.append("<").append(LHS).append(", ").append(RHS1).append(", ").append(RHS2).append("> ").append(ruleName).append(";");
    }
    
}

void applyRuleFromString(string line) {
    vector<string> rulesVector;
    char_separator<char> sep(", ");
    tokenizer<char_separator<char> > tokens(line, sep);
    BOOST_FOREACH(string t, tokens)
    {
        rulesVector.push_back(t);
    }
    
    applyRuleToFile(rulesVector);
}

void createRules(char* file) {
    ifstream rulesFile;
    outputCode<<"void runLearn() {"<<endl;
    outputCode<<"    queue<Symbol*> nodesCreatedSoFar;"<<endl;
    outputCode<<"    vector<Terminal*> temp;"<<endl;
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
            
            applyRuleFromString(line);
        }
    }
    outputCode<<"}"<<endl;
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
    createRules("learner.cpp");
    return 0;
}
