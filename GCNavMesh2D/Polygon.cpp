//
//  Polygon.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/5/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "Polygon.hpp"


Polygon::Polygon(){
    xmin = 1e100;
    xmax = -1e100;
    ymin = 1e100;
    ymax = -1e100;
}

Polygon::~Polygon(){
    /*for(int i=0; i<edges.size(); i++){
        PrimitiveSegment *p = edges.at(i);
        if(p!=0) delete p;
    }*/
}

void Polygon::addVertex(double x, double y, bool obstacleEdge){
    
    vertices.push_back(std::pair<double,double>(x,y));
    obstacleEdges.push_back(obstacleEdge);
    
    if(vertices.size() == 2){
        int i = (int)vertices.size()-1;
        PrimitiveSegment *p1 = new PrimitiveSegment(vertices.at(i-1).first, vertices.at(i-1).second, obstacleEdges.at(i-1), x, y, obstacleEdge);
        edges.push_back(p1);
        PrimitiveSegment *p2 = new PrimitiveSegment(x, y,obstacleEdge, vertices.at(0).first, vertices.at(0).second, obstacleEdges.at(0));
        edges.push_back(p2);
    }if(vertices.size() > 2){
        int i = (int)vertices.size()-1;
        int j = (int)edges.size()-1;
        
        /*PrimitiveSegment *p = edges.at(j);
        p->getX1() = vertices.at(i-1).first;
        p->getY1() = vertices.at(i-1).second;
        p->setObstacleEdge1(obstacleEdges.at(i-1));
        p->getX2() = x;
        p->getY2() = y;
        p->setObstacleEdge2(obstacleEdge);
        p->updateLength();*/
        
        
        edges.at(j)->updateValues(vertices.at(i-1).first, vertices.at(i-1).second, obstacleEdges.at(i-1), x, y, obstacleEdge);
        
        PrimitiveSegment *p1 = new PrimitiveSegment(x,y,obstacleEdge,vertices.at(0).first, vertices.at(0).second, obstacleEdges.at(0));
        edges.push_back(p1);
        
    }
    
    if(x < xmin) xmin = x;
    if(x > xmax) xmax = x;
    if(y < ymin) ymin = y;
    if(y > ymax) ymax = y;
    
}

bool Polygon::isPoinInPolygon(double x, double y){
    
    bool oddNodes = false;
    int N = (int) vertices.size();
    for(int i=0, j=(N-1); i<N; i++){
        std::pair<double, double> &vi = vertices.at(i);
        std::pair<double, double> &vj = vertices.at(j);
        if (( (vi.second< y && vj.second>=y)  ||   (vj.second< y && vi.second>=y)) &&  (vi.first<=x || vj.first<=x)) {
            oddNodes ^= (vi.first+(y-vi.second)/(vj.second-vi.second)*(vj.first-vi.first)<x);
        }
        j=i;
    }
    return oddNodes;
}

void Polygon::applyResolution(double resolution){
    
    double invres = 1.0/resolution;
    xmin = 1e100;
    xmax = -1e100;
    ymin = 1e100;
    ymax = -1e100;
    
    for(int i=0; i<vertices.size(); i++){
        vertices.at(i).first *= invres;
        vertices.at(i).second *= invres;
        if(vertices.at(i).first < xmin) xmin = vertices.at(i).first;
        if(vertices.at(i).first > xmax) xmax = vertices.at(i).first;
        if(vertices.at(i).second < ymin) ymin = vertices.at(i).second;
        if(vertices.at(i).second > ymax) ymax = vertices.at(i).second;
    }
    
    for(int i=0; i<edges.size(); i++){
        edges.at(i)->applyResolution(resolution);
    }
    
}


void Polygon::drawPolygon(cv::Mat &img, const cv::Vec3b &color){
    
    int yfirst = floor(ymin);
    int ylast = ceil(ymax);
    int xfirst = floor(xmin);
    int xlast = ceil(xmax);
    
    for(int y=yfirst; y<ylast; y++){
        for(int x=xfirst; x<xlast; x++){
            if(isPoinInPolygon(x, y)){
                img.at<cv::Vec3b>(y,x) = color;
            }
        }
    }
    
}


void Polygon::drawPolygon(cv::Mat &img, const cv::Vec3b &color, const cv::Vec3b &edgeColor, double thickness){
    drawPolygon(img, color);
    
    for(int i=0; i<edges.size(); i++){
        cv::line(img, cv::Point(edges.at(i)->getX1(), edges.at(i)->getY1()), cv::Point(edges.at(i)->getX2(), edges.at(i)->getY2()), edgeColor,thickness);
    }
    
}

void Polygon::drawPolygon(cv::Mat &img, const cv::Vec3b &color, const cv::Vec3b &edgeColor, const cv::Vec3b &limitColor, double thickness, double minDistObstacles){
    drawPolygon(img, color, edgeColor, thickness);
    for(int i=0; i<edges.size(); i++){
        if(edges.at(i)->isObstacleEdge()){
            cv::line(img, cv::Point(edges.at(i)->getX1(), edges.at(i)->getY1()), cv::Point(edges.at(i)->getX2(), edges.at(i)->getY2()), limitColor, thickness);
        }else{
            cv::line(img, cv::Point(edges.at(i)->getX1(), edges.at(i)->getY1()), cv::Point(edges.at(i)->getX2(), edges.at(i)->getY2()), edgeColor,thickness);
            
            double threshold = minDistObstacles / edges.at(i)->getLength();
            if(threshold > 1.0) threshold = 1.0;
            
            if(edges.at(i)->isObstacleEdge1()){
                double x = (1-threshold)*edges.at(i)->getX1() + (threshold)*edges.at(i)->getX2();
                double y = (1-threshold)*edges.at(i)->getY1() + (threshold)*edges.at(i)->getY2();
                cv::line(img, cv::Point(x,y), cv::Point(edges.at(i)->getX1(),edges.at(i)->getY1()), limitColor,thickness);
            }
            if(edges.at(i)->isObstacleEdge2()){
                double x = (1-threshold)*edges.at(i)->getX2() + (threshold)*edges.at(i)->getX1();
                double y = (1-threshold)*edges.at(i)->getY2() + (threshold)*edges.at(i)->getY1();
                cv::line(img, cv::Point(x,y), cv::Point(edges.at(i)->getX2(),edges.at(i)->getY2()), limitColor,thickness);
            }
        }
    }

}

bool Polygon::intersection(double x1, double y1, double x2, double y2, double &x, double &y, double minDistObstacles/*, cv::Mat &img, bool show*/){
    
    PrimitiveSegment ps(x1,y1,x2,y2);
    bool intersects = false;
    double t,a,px,py,dx=0,dy=0;
    double minDistance = 1e100;
    int pp = 0;
     for(int i=0; i<edges.size(); i++){
        
        PrimitiveSegment *p = edges.at(i);
      
        p->intersectionWithSegment(&ps, a, px, py);
        
        bool intersection = (a>=-1e-6 && a<=(1+1e-6));
         
        if(intersection){
          
            double distance = sqrt((px-x2)*(px-x2) + (py-y2)*(py-y2));
            
            if(distance < minDistance){
                double length = p->getLength();
                double threshold = minDistObstacles / length;
                if(threshold > 1.0) threshold = 1.0;
                threshold = 1.0 - threshold;
                
                x = px;
                y = py;
                dx = x;
                dy = y;
                

                if(p->isObstacleEdge1() && a > threshold){
                    dx = (threshold)*edges.at(i)->getX1() + (1-threshold)*edges.at(i)->getX2();
                    dy = (threshold)*edges.at(i)->getY1() + (1-threshold)*edges.at(i)->getY2();
                    pp = 1;
                }
                
                if(p->isObstacleEdge2() && (a) < (1-threshold)){
                    dx = (threshold)*edges.at(i)->getX2() + (1-threshold)*edges.at(i)->getX1();
                    dy = (threshold)*edges.at(i)->getY2() + (1-threshold)*edges.at(i)->getY1();
                    pp = 1;
                }
                
                minDistance = distance;
            }
        }
        
        
    }
    
    if(pp){
        x = dx;
        y = dy;
        //cv::circle(img, cv::Point(x,y), 1, cv::Scalar(0,0,255), 4);
        //cv::circle(img, cv::Point(dx,dy), 1, cv::Scalar(255,125,0), 4);
    }else{
        //cv::circle(img, cv::Point(x,y), 1, cv::Scalar(255,0,255), 4);
    }
    
    //cv::imshow("a",img);
    //cv::waitKey();
    return  intersects;
}


bool Polygon::checkRayIntersection(PrimitiveSegment &ray, double x1, double y1, double &x, double &y, PrimitiveSegment *&intseg, double &minDistance){
    
    double a,t,px=0,py=0;
    minDistance = 1e100;
    bool intersects = false;
    
    for(int i=0; i<edges.size(); i++){
        
        PrimitiveSegment *&p = edges.at(i);
        
        
        p->intersectionWithSegment(&ray, t, px, py);
        ray.intersectionWithSegment(p, a, px, py);
        
        bool intersection = (a>=-1e-6 && a<=(1+1e-6)) && (t>=-1e-6 && t<=(1+1e-6)) ;
        
        if(intersection){
            double distance = sqrt((px-x1)*(px-x1) + (py-y1)*(py-y1));
            if(distance < minDistance){
                minDistance = distance;
                intseg = p;
                x = px;
                y = py;
                intersects = intersection;
            }
            
        }
    }

    return intersects;
}

bool Polygon::intersectionClosestSegment(double x1, double y1, double x2, double y2, double &x, double &y, PrimitiveSegment *&pout, cv::Mat &img){
    
    PrimitiveSegment ps(x1,y1,x2,y2);
    cv::line(img, cv::Point(ps.getX1(),ps.getY1()), cv::Point(ps.getX2(),ps.getY2()),cv::Scalar(0,255,255),2);
    
    bool intersects = false;
    double a,px,py;
    double minDistance = 1e100;
    for(int i=0; i<edges.size(); i++){
        
        PrimitiveSegment *p = edges.at(i);
        
        ps.intersectionWithSegment(p, a, px, py);
        
        cv::line(img, cv::Point(p->getX1(),p->getY1()), cv::Point(p->getX2(),p->getY2()),cv::Scalar(255,255,0),2);
        
        
        bool intersection = (a>=-1e-6 && a<=(1+1e-6));
        
        if(intersection){
            cv::circle(img, cv::Point(px,py), 2, cv::Scalar(0,255,255),4);
            double distance = sqrt((px-x1)*(px-x1) + (py-y1)*(py-y1));
            if(distance < minDistance){
                cv::circle(img, cv::Point(x,y), 2, cv::Scalar(0,255,255),4);
                minDistance = distance;
                pout = p;
                x = px;
                y = py;
                intersects = intersection;
                cv::circle(img, cv::Point(x,y), 2, cv::Scalar(255,0,255),4);
            }
            
        }
        
        cv::imshow("a", img);
        cv::waitKey();
        
        
    }
    
    return  intersects;

}
