/* 
 * File:   utils.h
 * Author: aa755
 *
 * Created on August 20, 2011, 10:31 PM
 */

#ifndef UTILS_H
#define	UTILS_H

struct null_deleter
{
    void operator()(void const *) const
    {
    }
};


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

#endif	/* UTILS_H */

