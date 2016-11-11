//
//  NavMesh2DTrajectory.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/6/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef NavMesh2DTrajectory_hpp
#define NavMesh2DTrajectory_hpp

#include <stdio.h>
#include <vector>
#include "NavMesh2D.hpp"
#include <set>
#include "CustomPriorityQueue.hpp"

class NavMesh2DTrajectory{
    
protected:
    
    std::vector<std::pair<double,double> >trajectory;
    std::vector<std::pair<double,double> >connectionPoints;
    std::vector<NavMesh2DConnection*> connections;
    std::vector<std::pair<double,double> >minimumTrajectory;
    double xstart, ystart;
    double xend, yend;
    double xorigin, yorigin, xdestiny, ydestiny;
    double minDistObstacles;
    
public:
    
    inline std::vector<NavMesh2DConnection*>& getConnections(){return connections;}
    inline std::vector<std::pair<double,double> >& getTrajectory(){return trajectory;}
    inline std::vector<std::pair<double,double> >& getMinTrajectory(){return minimumTrajectory;}
    inline std::vector<std::pair<double,double> >& getConnectionPoints(){return connectionPoints;}
    inline double &getXstart(){return xstart;}
    inline double &getYstart(){return ystart;}
    inline double &getXend(){return xend;}
    inline double &getYend(){return yend;}
    inline double &getXorigin(){return xorigin;}
    inline double &getYorigin(){return yorigin;}
    inline double &getXdestiny(){return xdestiny;}
    inline double &getYdestiny(){return ydestiny;}
    inline double &getMinDistObstacles(){return minDistObstacles;}
};

class NavMesh2DTrajectoryBuilder{
    
protected:
    
    NavMesh2D navmesh;
    
public:
    
    double timesearch;
    int countsearchs;
    double maxtime;
    
    NavMesh2DTrajectoryBuilder(const std::string &fileName);
    NavMesh2DTrajectoryBuilder(NavMesh2D &navmesh);
    
    bool searchTrajectory(double x1, double y1, double x2, double y2, NavMesh2DTrajectory &outTrajectory, double t, double iterations, double minDistanceToObstacles, bool &finished);
    
    void drawTrajectory(NavMesh2DTrajectory &trajectory, cv::Mat &image);
    
    
protected:
    
    void dijkstraSearch(double x1, double y1, double x2, double y2, NavMesh2DConnection *con1, NavMesh2DConnection *con2, NavMesh2DTrajectory &outTrajectory);
    
    bool searchNearestNode(double x, double y, NavMesh2DConnection *&connection);
    
    void generateTrajectoryPoints(NavMesh2DTrajectory &outTrajectory, double x1, double y1, double x2, double y2, double t);
    
    void generateNewPoint(std::vector<std::pair<double,double> >&points, double t, double &ti, double x1, double y1, double x2, double y2, double distance=-1);
    
    void generateMinimumTrajectoryPoints(NavMesh2DTrajectory &outTrajectory, double t);
    
    void narrowTrajectory(/*cv::Mat &img,*/ NavMesh2DTrajectory &trajectory, int iterations, double minDistanceToObstacles);

    
};

#endif /* NavMesh2DTrajectory_hpp */
