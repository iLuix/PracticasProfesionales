//
//  PrimitiveSegment.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/20/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef PrimitiveSegment_hpp
#define PrimitiveSegment_hpp

#include <stdio.h>
#include <cmath>

class PrimitiveSegment{
    
protected:
    
    static int idCount;
    
    double x1;
    double y1;
    double x2;
    double y2;
    int id;
    bool obstacleEdge1;
    bool obstacleEdge2;
    double length;
    
public:
    
    PrimitiveSegment(double x1, double y1, double x2, double y2);
    PrimitiveSegment(double x1, double y1, bool o1, double x2, double y2, bool o2);
    void updateValues(double x1, double y1, bool o1, double x2, double y2, bool o2);
    void updateLength(){length = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));};
    PrimitiveSegment(const PrimitiveSegment &o);
    double distanceToPoint(double x, double y, double &t);
    inline void setId(int id){this->id = id;}
    inline double &getX1(){return x1;}
    inline double &getY1(){return y1;}
    inline double &getX2(){return x2;}
    inline double &getY2(){return y2;}
    inline int getId(){return id;}
    inline void setObstacleEdge1(bool isObstacleEdge){this->obstacleEdge1 = isObstacleEdge;}
    inline void setObstacleEdge2(bool isObstacleEdge){this->obstacleEdge2 = isObstacleEdge;}
    inline bool isObstacleEdge(){return obstacleEdge1 && obstacleEdge2;}
    inline bool isObstacleEdge1(){return obstacleEdge1;}
    inline bool isObstacleEdge2(){return obstacleEdge2;}
    inline double getLength(){return length;}
    void minDistancePositionToPoint(double x, double y, double &t, double &px, double &py, bool limit=true);
    void intersectionPointToSegment(PrimitiveSegment *p, double &t, double &px, double &py);
    void intersectionWithSegment(PrimitiveSegment *p, double &t, double &px, double &py);
    void middlePoint(double &px, double &py);
    void applyResolution(double resolution);
};

#endif /* PrimitiveSegment_hpp */
