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
};

void getTokens(std::string str,std::vector<int> & out)
{
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(std::string t, tokens) 
        {
		out.push_back(boost::lexical_cast<int>(t));
        }
}

void getTokens(std::string str, std::vector<std::string> & out)
{
        boost::char_separator<char> sep(",");
        boost::tokenizer<boost::char_separator<char> > tokens(str, sep);
        
        out.clear();

        BOOST_FOREACH(std::string t, tokens) 
        {
		out.push_back(t);
        }
}

#endif	/* UTILS_H */

