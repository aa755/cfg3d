/* 
 * File:   utils.h
 * Author: aa755
 *
 * Created on August 20, 2011, 10:31 PM
 */

#ifndef UTILS_H
#define	UTILS_H
#define BOOST_DYNAMIC_BITSET_DONT_USE_FRIENDS
#include <boost/dynamic_bitset.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>
#include <vector>
#include <boost//lexical_cast.hpp>
#include <iostream>
#include <stdio.h>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/algorithms/transform.hpp>
#include <boost/lexical_cast.hpp>

using namespace std;

struct null_deleter
{
    void operator()(void const *) const
    {
    }
};

double sqr(double d) {
    return d*d;
}

template<typename X>
boost::shared_ptr<X> createStaticShared(X * x)
{
    boost::shared_ptr<X> px(x, null_deleter());
    return px;
}

template<typename X>
boost::shared_ptr<X> createConstStaticShared(const X * x)
{
    boost::shared_ptr<X> px(x, null_deleter());
    return px;
}

class AdvancedDynamicBitset :public boost::dynamic_bitset<>
{
    
    int position;
public:
    void iteratorReset()
    {
        position=-1;
    }
    
    bool nextOnBit(int & index)
    {
        if(position==-1)
            position=find_first();
        else
            position=find_next(position);
        
        index=position;
        return position!=(int)npos;
    }
    
    void print1BasedIndices()
    {
        iteratorReset();
        int index;
        while(nextOnBit(index))
        {
            std::cout<<index+1<<",";
        }
        std::cout<<endl;
    }
};

void getTokens(string str, vector<int>& out) {
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(string t, tokens) 
        {
		out.push_back(boost::lexical_cast<int>(t));
        }
}

vector<int> splitLineAsIntVector(string str) {
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        vector<int> out;
        BOOST_FOREACH(string t, tokens) 
        {
                out.push_back(boost::lexical_cast<int>(t));
        }
        return out;
}

vector<string> splitLineAsStringVector(string str) {
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        vector<string> out;
        BOOST_FOREACH(string t, tokens) 
        {
                out.push_back(t);
        }
        return out;
}

void getTokens(string str, vector<string>& out,string delim=",") {
        boost::char_separator<char> sep(delim.data());
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(string t, tokens) 
        {
		out.push_back(t);
        }
}

/**
 * 
 * @param file
 * @param neighbors
 * @return : max segment index
 */
int parseNbrMap(char * file,map<int, set<int> > & neighbors, int maxSegIndex) {
        std::ifstream labelFile;
    std::string line;
    labelFile.open(file);

    vector<int> nbrs;
    
    int max=0;
    if (labelFile.is_open()) {
        while (labelFile.good()) {
            getline(labelFile, line); //each line is a label
            if (line.size() == 0)
                break;
            
            getTokens(line, nbrs);
            int segIndex=nbrs.at(0);
            
            if(segIndex>maxSegIndex)
                continue;
            
            set<int> temp;
            neighbors[segIndex]=temp;
            if(max<segIndex)
                max=segIndex;
            for(size_t i=1;i<nbrs.size();i++)
            {

                if(nbrs.at(i)>maxSegIndex)
                        continue;
                
                neighbors[segIndex].insert(nbrs.at(i));
                cout<<"Adding "<<nbrs.at(i)<<" as a neighbors of "<<segIndex<<endl;
            }
        }
    } else {
        cout << "Could not open label file ... exiting\n";
        exit(-1);
    }

    return max;

}


void getTokens(string str, vector<float>& out,string delim=",") {
        boost::char_separator<char> sep(delim.data());
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(string t, tokens) 
        {
            if(t.compare("Inf")==0)
                out.push_back(numeric_limits<double>::infinity());
            else if(t.compare("NaN")==0)
                out.push_back(std::numeric_limits<double>::quiet_NaN());
            else
		out.push_back(boost::lexical_cast<float>(t));
        }
}

/**
 * executes a command and returns it's stdout output
 * copied from : http://stackoverflow.com/questions/478898/how-to-execute-a-command-and-get-output-of-command-within-c
 * @param cmd
 * @return 
 */
std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if (!pipe) return "ERROR";
    char buffer[128];
    std::string result = "";
    while(!feof(pipe)) {
        if(fgets(buffer, 128, pipe) != NULL)
                result += buffer;
    }
    pclose(pipe);
    return result;
}

Eigen::Vector3d getDirection(double azimuth, double elev) {
    namespace bg = boost::geometry;
    typedef bg::point<double, 3, bg::cs::cartesian> cartesian;
    typedef bg::point<double, 2, bg::cs::spherical<bg::degree> > spherical;

    cartesian pc;
    spherical ps(azimuth, elev);

    bg::transform<spherical, cartesian > (ps, pc);
    return Eigen::Vector3d(pc.get < 0 > (), pc.get < 1 > (), pc.get < 2 > ());
}
class Node
{
public:
    string type;
    int id;
    string memo;
    
    Node(string fullname)
    {
        int typeEnd=fullname.find("__");
        type=fullname.substr(0,typeEnd);
        int idEnd=fullname.find("__",typeEnd+1);
        
        if(idEnd==(int)string::npos)
            idEnd=fullname.size();
        string ids=fullname.substr(typeEnd+2,idEnd-typeEnd-2);
        //cout<<"++"<<ids<<"++"<<endl;
        id=boost::lexical_cast<int>(ids);
        if(idEnd==(int)fullname.size())
        {
            memo="";
        }
        else
        {
          memo=fullname.substr(idEnd+2);
        }
        //cout<<type<<"-"<<id<<"-"<<memo<<endl;
    }
    
    bool updateTypeCounts(map<string,int>& typeMaxId)
    {
        if(typeMaxId.find(type)==typeMaxId.end())
        {
            typeMaxId[type]=id;
            return true; // new type
        }
        else if(typeMaxId[type]<id)
            typeMaxId[type]=id;
        
        return false;
        
    }
};

//        
//    template<typename PT>
//    PT pointFromVector(const Eigen::Vector3d & d)
//    {
//        PT p;
//        for(int i=0;i<3;i++)
//            p.data[i]=d(i);
//        p.data[3]=1.0f;
//        return p;
//    }
//
#endif	/* UTILS_H */

