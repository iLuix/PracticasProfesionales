//
//  CustomPriorityQueue.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/19/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef CustomPriorityQueue_hpp
#define CustomPriorityQueue_hpp

#include <stdio.h>
#include <queue>
#include <typeinfo>
#include <set>


template<typename T>
class mycomparison
{
    bool reverse;
public:
    mycomparison(const bool& revparam=false)
    {reverse=revparam;}
    bool operator() (const T& lhs, const T&rhs) const
    {
        if (reverse) return (lhs>rhs);
        else return (lhs<rhs);
    }
};

#endif /* CustomPriorityQueue_hpp */
