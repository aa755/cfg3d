/* 
 * File:   rules_generator.cpp
 * Author: lisherwin
 *
 * Created on November 12, 2011, 11:59 PM
 */

#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <fstream>
#include "utils.h"

#define MAX_SEG_INDEX 30

ofstream rulesFile;
using namespace std;

void printFileError() {
    cout<<"Could not open file"<<endl;
}

int parseGraph(char* file, map<int, set<int> > &nodeToNeighbors)
{
    ifstream labelFile;
    string line;
    labelFile.open(file);

    vector<int> nbrs;

    int max = 0;
    if (labelFile.is_open())
    {
        while (labelFile.good())
        {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0) 
            {
                break;
            }

            getTokens(line, nbrs);
            int segIndex = nbrs.at(0);

            if (segIndex > MAX_SEG_INDEX)
                continue;

            set<int> temp;
            nodeToNeighbors[segIndex] = temp;
            if (max < segIndex)
                max = segIndex;
            for (size_t i = 1; i < nbrs.size(); i++)
            {

                if (nbrs.at(i) > MAX_SEG_INDEX)
                    continue;

                nodeToNeighbors[segIndex].insert(nbrs.at(i));
            }
        }
    }
    else
    {
        printFileError();
        exit(-1);
    }
//    cout<<endl;
    return max;
}

int parseLabels(char* file, map<int, int> &nodeToLabel)
{
    ifstream labelFile;
    string line;
    labelFile.open(file);

    vector<string> nbrs;

    int max = 0;
    if (labelFile.is_open())
    {
        while (labelFile.good())
        {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0) 
            {
                break;
            }

            getTokens(line, nbrs);
            int nodeIndex = boost::lexical_cast<int>(nbrs.at(0));
            int labelIndex = boost::lexical_cast<int>(nbrs.at(1));
            nodeToLabel[nodeIndex] = labelIndex;
        }
    }
    else
    {
        printFileError();
        exit(-1);
    }
    return max;
}

int parseLabelIndexToText(char* file, map<int, string> &labelsToText)
{
    ifstream labelFile;
    string line;
    labelFile.open(file);

    vector<string> nbrs;

    int max = 0;
    if (labelFile.is_open())
    {
        while (labelFile.good())
        {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0) 
            {
                break;
            }

            getTokens(line, nbrs);
            int label = boost::lexical_cast<int>(nbrs.at(0));
            string labelText = nbrs.at(1);
            labelsToText[label] = labelText;
        }
    }
    else
    {
        printFileError();
        exit(-1);
    }
    cout<<endl;
    return max;
}


int parseNodeToLabelText(char* file, map<int, string> &nodeToLabelText)
{
    ifstream labelFile;
    string line;
    labelFile.open(file);

    vector<string> nbrs;

    int max = 0;
    if (labelFile.is_open())
    {
        while (labelFile.good())
        {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0) 
            {
                break;
            }

            getTokens(line, nbrs);
            int nodeIndex = boost::lexical_cast<int>(nbrs.at(0));
            string labelText = nbrs.at(1);
            nodeToLabelText[nodeIndex] = labelText;
        }
    }
    else
    {
        printFileError();
        exit(-1);
    }
    return max;
}

void printSet(set<int>& intSet) {
    set<int>::iterator it;
    for (it = intSet.begin(); it != intSet.end(); it++)
    {
        cout<<"\t"<<*it<<endl;
    }
}


void conquer(int conquering, set<int>& conquered, set<int>& toConquer, map<int, set<int> >& nodeToNeighbors, map<int, string>& nodeToLabelText) {
    set<int>::iterator it;
    conquered.insert(conquering);
    toConquer.erase(conquering);
    string conqueringName = nodeToLabelText[conquering];
    rulesFile<<conqueringName<<","<<"Plane"<<endl;
    set<int> neighbors = nodeToNeighbors[conquering];
     
    for (it = neighbors.begin(); it != neighbors.end(); it++) {
        if (conquered.find(*it) == conquered.end()) {
            toConquer.insert(*it);
        }
    }
}

void printRule(string leftNonTerminal, string rNT1, string rNT2) {
    rulesFile<<leftNonTerminal<<","<<rNT1<<","<<rNT2<<endl;
}

void generateRules(map<int, set<int> > &nodeToNeighbors, map<int, string> &nodeToLabelText, string goal) {
    set<int> conquered;
    set<int> toConquer;
    conquer(1, conquered, toConquer, nodeToNeighbors, nodeToLabelText);
    string internalNonTerminal = nodeToLabelText[1];
    while(!toConquer.empty()) {
        int conquering = *toConquer.begin();
        string conqueringName = nodeToLabelText[conquering];
        conquer(conquering, conquered, toConquer, nodeToNeighbors, nodeToLabelText);
        if (toConquer.empty()) {
            printRule(goal, internalNonTerminal, conqueringName);
        } else {
            string previousInternalNonTerminal = internalNonTerminal;
            internalNonTerminal.append("_").append(conqueringName);
            printRule(internalNonTerminal, previousInternalNonTerminal, conqueringName);
        }
    }
}