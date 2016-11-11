//
//  PrimitiveSegment.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/20/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "PrimitiveSegment.hpp"

int PrimitiveSegment::idCount = 0;

PrimitiveSegment::PrimitiveSegment(double x1, double y1, double x2, double y2){
    
    //id = idCount++;
    updateValues(x1, y1, false, x2, y2, false);
}

PrimitiveSegment::PrimitiveSegment(double x1, double y1, bool o1, double x2, double y2, bool o2){
    
    //id = idCount++;
    updateValues(x1, y1, o1, x2, y2, o2);
}


PrimitiveSegment::PrimitiveSegment(const PrimitiveSegment &o){
    obstacleEdge1 = o.obstacleEdge1;
    obstacleEdge2 = o.obstacleEdge2;
    x1 = o.x1;
    x2 = o.x2;
    y1 = o.y1;
    y2 = o.y2;
    id = o.id;
}

void PrimitiveSegment::updateValues(double x1, double y1, bool o1, double x2, double y2, bool o2){
    if(x1==x2){
        if(y1 < y2){
            this->x1 = x1;
            this->y1 = y1;
            this->x2 = x2;
            this->y2 = y2;
            obstacleEdge1 = o1;
            obstacleEdge2 = o2;
        }else{
            this->x1 = x2;
            this->y1 = y2;
            this->x2 = x1;
            this->y2 = y1;
            obstacleEdge1 = o2;
            obstacleEdge2 = o1;
        }
    }else if(x1 < x2){
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        obstacleEdge1 = o1;
        obstacleEdge2 = o2;
    }else{
        this->x1 = x2;
        this->y1 = y2;
        this->x2 = x1;
        this->y2 = y1;
        obstacleEdge1 = o2;
        obstacleEdge2 = o1;
    }

    length = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    
}

double PrimitiveSegment::distanceToPoint(double x, double y, double &t){
    
    double px, py;
    minDistancePositionToPoint(x, y, t, px, py);
    return sqrt((px-x)*(px-x) + (py-y)*(py-y));
    
}


void PrimitiveSegment::minDistancePositionToPoint(double x, double y, double &t, double &px, double &py, bool limit){
    
    double x12 = x1-x2;
    double y12 = y1-y2;
    t = (x*(x12) - x2*(x12) + y*(y12) - y2*(y12))/((x12)*(x12) + (y12)*(y12));
    if(limit && t<0) t = 0;
    if(limit && t>1) t = 1;
    px = t*x1 + (1-t)*x2;
    py = t*y1 + (1-t)*y2;
    
}

void PrimitiveSegment::intersectionPointToSegment(PrimitiveSegment *p, double &t, double &px, double &py){
    
    double p42y = y2 - p->getY2();
    double p42x = x2 - p->getX2();
    double p12x = p->getX1() - p->getX2();
    double p12y = p->getY1() - p->getY2();
    double p34x = x1 - x2;
    double p34y = y1-y2;
    t =  ((p42y)*(p12x) - (p42x)*(p12y))/((p34x)*(p12y) - (p34y)*(p12x));
    px = t*x1 + (1-t)*x2;
    py = t*y1 + (1-t)*y2;
    
}

void PrimitiveSegment::intersectionWithSegment(PrimitiveSegment *p, double &t, double &px, double &py){
    
    double p42y = y2 - p->getY2();
    double p42x = x2 - p->getX2();
    
    
    double p12x = p->getX1() - p->getX2();
    double p12y = p->getY1() - p->getY2();
    double p34x = x1 - x2;
    double p34y = y1-y2;
    t =  ((p42y)*(p12x) - (p42x)*(p12y))/((p34x)*(p12y) - (p34y)*(p12x));
    px = t*x1 + (1-t)*x2;
    py = t*y1 + (1-t)*y2;
    
    if(t>-1e-6 && t<(1+1e-6)){
        if(px < (x1-1e-6) || px > (x2+1e-6) || py < (fmin(y1,y2)-1e-6)|| py > (fmax(y1,y2)+1e-6)){
            t = -10-t;
        }
    }
    
}

void PrimitiveSegment::middlePoint(double &px, double &py){
    double t = 0.5;
    px = t*x1 + (1-t)*x2;
    py = t*y1 + (1-t)*y2;
}

void PrimitiveSegment::applyResolution(double resolution){
    double invres = 1.0/resolution;
    x1 *= invres;
    x2 *= invres;
    y1 *= invres;
    y2 *= invres;
}