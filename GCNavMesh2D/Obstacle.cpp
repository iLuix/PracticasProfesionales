//
//  Obstacle.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/23/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "Obstacle.hpp"

Obstacle::Obstacle(){
    
}

void Obstacle::addSegment(double x1, double y1, double x2, double y2){
    
    PrimitiveSegment p(x1,y1,x2,y2);
    primitives.push_back(p);
    
    //mapVertexSegment[std::pair<double,double>(x1,y1)].push_back(p);
    //mapVertexSegment[std::pair<double,double>(x2,y2)].push_back(p);
    
}

/*void Obstacle::calculateCycles(){
    
    std::map<std::pair<double,double>, std::vector<PrimitiveSegment> >::iterator it;
    for(it=mapVertexSegment.begin(); it!=mapVertexSegment.end(); ++it){
        
    }
    
}

bool Obstacle::isPointInSegment(double x, double y){
    
    return false;
        
}*/
