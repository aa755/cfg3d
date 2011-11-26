#include "../bin/parseStructure.cpp"
#include <iostream>
#include <string>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

using namespace std;
using namespace boost;

void loopOnRules(vector<RulePtr>& rules) {
    while(!rules.empty()) {
    }
}

int main(int argc, char** argv) {
    vector<RulePtr> rules; 
    appendRuleInstances(rules);
    return 0;
}
