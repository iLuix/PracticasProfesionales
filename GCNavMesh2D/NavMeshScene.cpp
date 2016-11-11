//
//  Scene.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/23/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "NavMeshScene.hpp"

NavMeshScene::NavMeshScene(int width, int height, double resolution)
:width(width),height(height),resolution(resolution){
    
    double resinv = 1.0/resolution;
    resWidth = ceil(width*resinv)+1;
    resHeight = ceil(height*resinv)+1;
    scene = cv::Mat::zeros(resHeight, resWidth, CV_8UC3);
    
}

NavMeshScene::~NavMeshScene(){
    
}

void NavMeshScene::addPolygonObstacle(Polygon &obstacle, bool applyResolution){
    
    if(applyResolution){
        obstacle.applyResolution(resolution);
    }
    
    obstacles.push_back(obstacle);
    for(int i=0; i<obstacle.getEdges().size(); i++){
        PrimitiveSegment *p = obstacle.getEdges().at(i);
        p->setId((int)primitives.size());
        primitives.push_back(p);
    }
    
}

void NavMeshScene::addPolygonFloor(Polygon &floor, bool applyResolution){
    
    if(applyResolution){
        floor.applyResolution(resolution);
    }
    this->floor = floor;
    for(int i=0; i<floor.getEdges().size(); i++){
        PrimitiveSegment *p = floor.getEdges().at(i);
        p->setId((int)primitives.size());
        primitives.push_back(p);
    }
    
}

void NavMeshScene::drawScene(const cv::Vec3b &floorColor, const cv::Vec3b &obstaclesColor){
    drawScene(scene, floorColor, obstaclesColor);
}

void NavMeshScene::drawScene(cv::Mat &image, const cv::Vec3b &floorColor, const cv::Vec3b &obstaclesColor){
    
    if(image.rows != resHeight && image.cols != resWidth){
        image = cv::Mat(resHeight,resWidth,CV_8UC3);
    }
    
    floor.drawPolygon(image, floorColor);
    
    for(int i=0; i<obstacles.size(); i++){
        obstacles.at(i).drawPolygon(image, obstaclesColor);
    }
    

    
    /*image = cv::Mat(resHeight, resWidth, CV_8UC3);
    for(int i=0; i<resHeight; i++){
        for(int j=0; j<resWidth; j++){
            if(!isPointFreeInScene(j, i)){
                image.at<cv::Vec3b>(i,j) = obstaclesColor;
            }else{
                image.at<cv::Vec3b>(i,j) = floorColor;
            }
        }
    }
    
    for(int i=0; i<floor.getEdges().size(); i++){
        PrimitiveSegment *p = floor.getEdges().at(i);
        cv::line(image, cv::Point(p->getX1(),p->getY1()), cv::Point(p->getX2(),p->getY2()), obstaclesColor, 1.0/resolution);
    }*/
    
}

void NavMeshScene::showScene(){
    
    cv::imshow("Scene", scene);
    cv::waitKey();
    
}

void NavMeshScene::saveScene(const std::string &file){
    cv::imwrite(file, scene);
}

bool NavMeshScene::isPointFreeInScene(double x, double y){
    
    Polygon p;
    return isPointFreeInScene(x, y, p);
    
}


bool NavMeshScene::isPointFreeInScene(double x, double y,  Polygon &polygon){
    if(!floor.isPoinInPolygon(x, y)){
        polygon = floor;
        return false;
    }
    
    for(int i=0; i<obstacles.size(); i++){
        if(obstacles.at(i).isPoinInPolygon(x, y)){
            polygon = obstacles.at(i);
          return false;
        }
    }
    
    return true;
}

PrimitiveSegment *NavMeshScene::predictCollisionPoint(double x, double y, double xnew, double ynew, double &xint, double &yint){
    
    Polygon p;
    PrimitiveSegment ray(x,y,xnew,ynew);
    PrimitiveSegment *intseg = 0;
    double minDistance = 1e100;
    double xint_i=0, yint_i=0, distance=0;
    PrimitiveSegment *intset_i = 0;
    
    /*cv::Mat img;
    scene.copyTo(img);
    cv::circle(img, cv::Point(x,y), 3, cv::Scalar(0,255,0),5);
    cv::circle(img, cv::Point(xnew,ynew), 3, cv::Scalar(255,255,0),5);
    cv::line(img,cv::Point(ray.getX1(),ray.getY1()), cv::Point(ray.getX2(),ray.getY2()),cv::Scalar(255,255,0),2);*/

    
    for(int i=0; i<obstacles.size(); i++){
        
        if(obstacles.at(i).checkRayIntersection(ray, x, y, xint_i, yint_i, intset_i, distance)){
            
               if(distance < minDistance){
                xint = xint_i;
                yint = yint_i;
                intseg = intset_i;
                minDistance = distance;

            }

        }
    }
    
    if(floor.checkRayIntersection(ray, x, y, xint_i, yint_i, intset_i, distance)){
        if(distance < minDistance){
            xint = xint_i;
            yint = yint_i;
            intseg = intset_i;
        }
    }
    
    /*if(intseg!=0){
        cv::circle(img, cv::Point(xint,yint), 3, cv::Scalar(255,0,255),5);
        cv::line(img,cv::Point(intseg->getX1(),intseg->getY1()), cv::Point(intseg->getX2(),intseg->getY2()),cv::Scalar(255,0,255),3);
    }*/
    
    /*cv::imshow("a", img);
    std::stringstream ss;
    ss<<"p_"<<(int)x<<" "<<(int)y<<".png";
    cv::imwrite(ss.str(), img);
    cv::waitKey();*/
    
    return intseg;
}