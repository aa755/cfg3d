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

using namespace std;

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
                break;

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
                cout << "adding " << nbrs.at(i) << " as a neighbos of " << segIndex << endl;
            }
        }
    }
    else
    {
        cout << "could not open label file...exiting\n";
        exit(-1);
    }
    return max;

}

int main(int argc, char** argv)
{
    cout << "Hello World!" << endl;
    map<int, set<int> > nodeToNeighbors;
    //    parseGraph("graph.txt", nodeToNeighbors);
    return 0;
}