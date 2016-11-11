//
//  Scene.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/23/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef Scene_hpp
#define Scene_hpp

#include <stdio.h>
#include <vector>

#include "Polygon.hpp"
#include "PrimitiveSegment.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class NavMeshScene{
    
protected:
    
    std::vector<Polygon> obstacles;
    Polygon floor;
    std::vector<PrimitiveSegment*> primitives;
    
    int width, height;
    int resWidth, resHeight;
    double resolution;
    
    cv::Mat scene;
    
public:
   
    NavMeshScene(){}
    NavMeshScene(int width, int height, double resolution);
    ~NavMeshScene();
    
    void addPolygonObstacle(Polygon &obstacle, bool applyResolution = true);
    void addPolygonFloor(Polygon &floor, bool applyResolution = true);
    
    void drawScene(const cv::Vec3b &floorColor, const cv::Vec3b &obstaclesColor);
    void drawScene(cv::Mat &image, const cv::Vec3b &floorColor, const cv::Vec3b &obstaclesColor);

    void showScene();
    void saveScene(const std::string &file);
    
    bool isPointFreeInScene(double x, double y);
    bool isPointFreeInScene(double x, double y, Polygon &polygon);
    
    PrimitiveSegment *predictCollisionPoint(double x, double y, double xnew, double ynew, double &xint, double &yint);
    
    inline int getWidth(){return width;}
    inline int getHeight(){return height;}
    inline int getResWidth(){return resWidth;}
    inline int getResHeight(){return resHeight;}
    inline double getResolution(){return resolution;}
    inline std::vector<PrimitiveSegment*> &getPrimitives(){return primitives;}
    inline std::vector<Polygon> &getObstacles(){return obstacles;}
    inline Polygon &getFloor(){return floor;}
    cv::Mat &getScene(){return scene;}
    
};

#endif /* Scene_hpp */
