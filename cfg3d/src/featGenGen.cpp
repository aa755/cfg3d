/* 
 * File:   featGenGen.cpp
 * Author: aa755
 *
 * Created on February 15, 2012, 7:39 PM
 */

#include <cstdlib>
#include "utils.h"
#include <boost/weak_ptr.hpp>
using namespace std;
using namespace boost;


class TreeNode
{
    Node name;
    /** anything made up of just terminals
     */
    bool primitivePart;
    bool object;
    bool terminal;
    
    
public:
    typedef pair<string,set<string> > RuleType;
    typedef  shared_ptr<TreeNode> Ptr;
    typedef  weak_ptr<TreeNode> WPtr;
    
    static map< RuleType , vector<string> > ruleOrdering;
    static map<string,Ptr> nameToTreeNode;

    
    vector<Ptr> children;
    WPtr parent;
    
    TreeNode(string name_,Ptr parent_ ):name(name_),parent(parent_)
    {
        primitivePart=false;
    }
    
    TreeNode(string name_):name(name_)
    {
        primitivePart=false;
    }
    
    
    Ptr getParent()
    {
        return parent.lock();
    }

    void addChild(Ptr child)
    {
        children.push_back(child);
    }
    
    
    void print()
    {
        string type="";
        if(terminal)
            type=type+"Terminal,";
        if(primitivePart)
            type=type+"PrimitivePart,";
        
        cout<<name.type<<"__"<<name.id<<":"<<type<<endl;
        for(vector<Ptr>::iterator it=children.begin();it!=children.end();it++)
        {
            (*it)->print();
        }
        
    }
    
    void setFlags()
    {
        if(children.size()==0)
        {
            terminal=true;
            object=false;
            primitivePart=false;
        }
        else
        {
            bool allterminal=true;
//            bool noTerminal=false;
            for(vector<Ptr>::iterator it=children.begin();it!=children.end();it++)
            {
                (*it)->setFlags();
                allterminal=allterminal && (*it)->terminal;
                //noTerminal= noTerminal || it->terminal;
            }
            
            if(allterminal)
            {
                terminal=false;
                primitivePart=true;
                object=false;
            }
            
        }
            
    }
    
    static Ptr readDotTree(char * treeFile)
    {
        std::ifstream fileI;
        std::string line;

        fileI.open(treeFile);
        getline(fileI, line);
        assert(line.substr(0, 7) == "digraph");

        Ptr rootNode;
        
        bool rootFound=false;
        if (fileI.is_open())
        {
            while (fileI.good())
            {
                getline(fileI, line);
                if (line.at(0) == '}')
                    break;

                

                vector<string> toks;
                getTokens(line, toks, "\t ->;");
                assert(toks.size()==1 || toks.size()==2);
                
                if (toks.size() == 1)
                {
                    if(rootFound)
                        continue;
                    
                    string name=toks.at(0);
                    Node nd(name);
                    if(nd.isRootType())
                    {
                        rootFound=true;
                        rootNode=Ptr(new TreeNode(name,Ptr()));
                        nameToTreeNode[name]=rootNode;
                    }
                }
                
                if (toks.size() == 2)
                {
                     if(nameToTreeNode.find(toks.at(0))==nameToTreeNode.end()) // nodes not reachable from root
                         continue;
                     
                    Ptr parent = nameToTreeNode[toks.at(0)];
                    assert(parent != NULL);

                    string childName = toks.at(1);
                    Ptr child(new TreeNode(childName,parent));
                    nameToTreeNode[childName] = child;
                    parent->addChild(child);

                }
            }

        }
        else
        {
            cout << "could not open the "<<treeFile<<" ..exiting\n";
            exit(-1);
        }
        
        return rootNode;

    }
    
    static void parseMapping(char * file)
    {
        std::ifstream fileI;
        std::string line;

        fileI.open(file);

        
        if (fileI.is_open())
        {
            while (fileI.good())
            {
                getline(fileI, line);
                
                if (line.size() == 0)
                    break;
                
                RuleType rule;
                vector<string> toks;
                getTokens(line, toks, ";");
                assert( toks.size()==2);
                string ruleStr=toks.at(0);
                string order =toks.at(1);
                getTokens(ruleStr, toks, " ,");
                rule.first = toks.at(0);
                for(int i=1;i<(int)toks.size();i++)
                {
                    rule.second.insert(toks.at(i));
                }

                getTokens(order, toks, " ,");
                
                // check that all members of the ordering are actually
                // present at the end of rule
                
                assert(rule.second.size()==toks.size());
                for(int i=0;i<(int)toks.size();i++)
                {
                    assert(rule.second.find(toks.at(i))!=rule.second.end());
                }
                
                ruleOrdering[rule]=toks;
                //cout<<rule<<endl;
            }
        }
    }
};

map< TreeNode::RuleType , vector<string> > TreeNode::ruleOrdering;
map<string,TreeNode::Ptr> TreeNode::nameToTreeNode;

/*
 * 
 */
int main(int argc, char** argv)
{

    
 //   TreeNode::Ptr root=TreeNode::readDotTree(argv[1]);    
//    root->setFlags();
//    root->print();
    
    TreeNode::parseMapping(argv[2]);
    return 0;
}

