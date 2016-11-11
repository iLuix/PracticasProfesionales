//
//  Obstacle.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/23/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef Obstacle_hpp
#define Obstacle_hpp

#include <stdio.h>
#include <vector>
#include <set>
#include <map>
#include "PrimitiveSegment.hpp"

class Obstacle{
    
    
protected:
    
    std::vector<PrimitiveSegment> primitives;
    std::map<std::pair<double,double>,std::vector<PrimitiveSegment> > mapVertexSegment;
    
public:
    
    Obstacle();
    
    void addSegment(double x1, double y1, double x2, double y2);
    
    //void calculateCycles();
    
    //void calculateCycle(std::pair<double,double> &p)
    
    //bool isPointInSegment(double x, double y);
    
    inline std::vector<PrimitiveSegment> &getPrimitives() {return primitives;}
};

#endif /* Obstacle_hpp */
