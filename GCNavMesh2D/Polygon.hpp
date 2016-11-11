//
//  Polygon.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/5/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef Polygon_hpp
#define Polygon_hpp

#include <stdio.h>
#include <vector>
#include <iostream>

#include "PrimitiveSegment.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


class Polygon{
    
protected:
    
    std::vector<std::pair<double,double> > vertices;
    std::vector<PrimitiveSegment*> edges;
    std::vector<bool> obstacleEdges;
    double resolution;
    double xmin,xmax,ymin,ymax;
    
public:
    
    Polygon();
    ~Polygon();
    
    void addVertex(double x, double y, bool obstacleEdge=false);
    
    bool isPoinInPolygon(double x, double y);
    
    inline std::vector<std::pair<double,double> > &getVertices(){return vertices;}
    inline std::vector<PrimitiveSegment*> &getEdges(){return edges;}
    inline std::vector<bool> &getObstacleEdges(){return obstacleEdges;}
    
    void applyResolution(double resolution);
    
    void drawPolygon(cv::Mat &img, const cv::Vec3b &color);
    
    void drawPolygon(cv::Mat &img, const cv::Vec3b &color, const cv::Vec3b &edgeColor, double thickness);
    
    void drawPolygon(cv::Mat &img, const cv::Vec3b &color, const cv::Vec3b &edgeColor, const cv::Vec3b &limitColor, double thickness, double minDistObstacles);
    
    bool intersection(double x1, double y1, double x2, double y2, double &x, double &y, double minDistObstacles/*, cv::Mat &img, bool show*/);
    bool intersectionClosestSegment(double x1, double y1, double x2, double y2, double &x, double &y, PrimitiveSegment *&pout, cv::Mat &img);

    bool checkRayIntersection(PrimitiveSegment &ray, double x1, double y1, double &x, double &y, PrimitiveSegment *&intseg, double &minDistance);
    
};

#endif /* Polygon_hpp */
