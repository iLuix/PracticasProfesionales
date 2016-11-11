//
//  NavMesh2D.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/20/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef NavMesh2D_hpp
#define NavMesh2D_hpp

#include <stdio.h>
#include <cmath>
#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <ctime>
#include <fstream>
#include <sstream>
#include <queue>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "PrimitiveSegment.hpp"
#include "NavMeshScene.hpp"

/**
 * @brief Class GovernorGroup
 */
class GovernorGroup{
    
protected:
    
    static std::map<GovernorGroup,int> idsMap;
    
    int id;
    PrimitiveSegment *p1;
    PrimitiveSegment *p2;
    int t1;
    int t2;
    int idp1;
    int idp2;
    
public:
    
    GovernorGroup(PrimitiveSegment *p1, PrimitiveSegment *p2, double &dt1, double &dt2);
    GovernorGroup(GovernorGroup &gg);
    GovernorGroup(const GovernorGroup &gg);
    GovernorGroup(){}

    inline PrimitiveSegment *getP1(){return p1;}
    inline PrimitiveSegment *getP2(){return p2;}
    inline int getT1(){return t1;}
    inline int getT2(){return t2;}
    inline int getID(){return id;}
    
    bool operator<(const GovernorGroup &gg) const;
    bool operator==(const GovernorGroup &gg) const;
    
};

/**
 *@brief Class NavMesh2DNode
 */
class NavMesh2DConnection;

class NavMesh2DNode{
    
protected:
    
    int id;
    double x,y;
    std::vector<PrimitiveSegment*> governors;
    std::vector<std::pair<double,double> > closestPoints;
    std::map<int, std::pair<double,double> > mapClosestPoints;
    std::set<NavMesh2DConnection*> connections;
    
    double cost;
    bool visited;

public:
    
    NavMesh2DNode(double x, double y);
    NavMesh2DNode(NavMesh2DNode &other);
    NavMesh2DNode(const NavMesh2DNode &other);
    
    NavMesh2DNode *trajCon;
    inline void setVisited(bool visited){this->visited = visited;}
    inline bool isVisited() {return visited;}
    inline void setCost(double cost){this->cost = cost;}
    void setCost(double x, double y);
    inline double getCost(){return cost;}
    inline void setID(int id) {this->id = id;}
    inline int getID() {return id;}
    inline double getX() {return x;}
    inline double getY() {return y;}
    inline std::vector<PrimitiveSegment*> &getGovernors(){return governors;}
    inline std::vector<std::pair<double,double> > &getClosestPoints(){return closestPoints;}
    inline std::set<NavMesh2DConnection*> &getConnections(){return connections;}
    inline std::map<int, std::pair<double,double> > &getMapClosestPoints(){return mapClosestPoints;}
   
    
    bool operator<(const NavMesh2DNode &o) const;
    bool operator>(const NavMesh2DNode &o) const;
    //bool operator==(const NavMesh2DNode &o) const;
    bool operator>=(const NavMesh2DNode &o) const;
    bool operator<=(const NavMesh2DNode &o) const;

    
};

/**
 *@brief NavMeshConnection
 */
class NavMesh2DConnection{
    
protected:
    
    NavMesh2DNode* node1;
    NavMesh2DNode* node2;
    std::vector<std::pair<double,double> > connectionPoints;
    int connectionType;
    GovernorGroup gg;
    Polygon polygon;
    double distance;

    
public:
    
    double xstart,xend,ystart,yend;
    
    NavMesh2DConnection(NavMesh2DNode* node1, NavMesh2DNode* node2, GovernorGroup &gg);
    
    NavMesh2DConnection(){}
    
    NavMesh2DConnection(NavMesh2DNode* node1, NavMesh2DNode* node2, int connectionType, double distance);

    inline NavMesh2DNode* &getNode1(){return node1;}
    inline NavMesh2DNode* &getNode2(){return node2;}
    inline int getConnectionType(){return connectionType;}
    inline GovernorGroup &getGovernorGroup(){return gg;}
    inline std::vector<std::pair<double,double> > &getConnectionPoints(){return connectionPoints;}
    inline Polygon &getPolygon(){return polygon;}
    inline void setPolygon(Polygon &polygon){this->polygon = polygon;}
    inline double getDistance(){return distance;}
    
    bool operator<(const NavMesh2DConnection &o) const;
    
    bool operator==(const NavMesh2DConnection &o) const;
    
protected:
    
    void generateConnectionPoints();
    
    void calculateDistance();
    
    void generatePolygon();
};


class NavMesh2DNodeWrapper{
    
public:
    NavMesh2DNode* node;
    
    NavMesh2DNodeWrapper(NavMesh2DNode* node):node(node){}
    
    bool operator<(const NavMesh2DNodeWrapper &o) const{
        return node->getCost() < o.node->getCost();
    }
    
    bool operator>(const NavMesh2DNodeWrapper &o) const{
        return node->getCost() > o.node->getCost();
    }
    
    bool operator==(const NavMesh2DNodeWrapper &o) const{
        //return node->getCost() == o.node->getCost();
        return node == o.node;
    }
    
    bool operator>=(const NavMesh2DNodeWrapper &o) const{
        return node->getCost() >= o.node->getCost();
    }
    
    bool operator<=(const NavMesh2DNodeWrapper &o) const{
        return node->getCost() <= o.node->getCost();
    }

};

 
/**
*@brief Class NavMesh2D
*/
class NavMesh2D{
    
protected:
    
    double resolution;
    double width;
    double height;
    int resWidth;
    int resHeight;
    
    NavMeshScene scene;
    int **voronoi;
    int **medialAxis;
    int **medialAxisPts;
    
    std::vector<NavMesh2DNode*> graphNodes;
    
    std::map<GovernorGroup, std::vector<std::pair<int,int> > > medialAxisSegments;
    std::map<GovernorGroup, std::vector<std::pair<int,int> > >::iterator itmas;
    
    std::map<std::pair<int,int>, std::set<GovernorGroup> > medialAxisPoints;
    std::map<std::pair<int,int>, std::set<GovernorGroup> >::iterator itmap;
    
    std::set<std::pair<int,int> > eventPoints;
    std::set<std::pair<int,int> >::iterator itep;
    
    std::map<GovernorGroup, std::vector<NavMesh2DNode*> > connectionMap;
    std::map<GovernorGroup, std::vector<NavMesh2DNode*> >::iterator itcm;
    
public:
    
    inline double getResolution(){return resolution;}
    inline int getResHeight(){return resHeight;}
    inline int getResWidth(){return resWidth;}
    inline int getHeight(){return height;}
    inline int getWidth(){return width;}
    inline std::vector<NavMesh2DNode*>& getGraphNodes(){return graphNodes;}
    
    NavMesh2D(NavMeshScene &scene);
    
    NavMesh2D(const std::string &file);
    
    ~NavMesh2D();
    
    void drawScene();
    
    void drawScene(cv::Mat &image);
    
    void drawVoronoi();
    
    void drawVoronoi(cv::Mat &image);
    
    void drawMedialAxis();
    
    void drawMedialAxis(cv::Mat &image);
    
    void drawEventPoints();
    
    void drawEventPoints(cv::Mat &image);
    
    void drawNavMesh();
    
    void drawNavMesh(cv::Mat &image);
    
    void drawClosestPoints();
    
    void drawClosestPoints(cv::Mat &image);
    
    void drawConnections();
    
    void drawConnections(cv::Mat &image);
    
    void drawConnection(cv::Mat &image, NavMesh2DConnection &connection, const cv::Scalar &color);
    
    void showScene();
    
    void saveScene(const std::string &path);
    
    void build();
    
    void saveNavigationMesh(const std::string &filename);
    
    
    
protected:
    
    void drawPrimitiveSegment(PrimitiveSegment &p, cv::Mat &img, const cv::Vec3b &color, int thickness);
    
    void generateVoronoi();
    
    void generateMedialAxis();
    
    void generateEventPoints();
    
    void generateNodeConnections();
    
    void addVoronoiGovernor(int d, int x, int y, std::set<int> &governors);
    
    void filterConvexGovernors(int x, int y, std::set<int> &governors, std::vector<int> &noConvexGovernors);
    
    bool checkConvexity(PrimitiveSegment *p1, PrimitiveSegment *p2, double x, double y);
    
    void addConcaveNode(double px, double py);
    
    bool addEventPoint(int x, int y, int p);
    
    void mergeEventPoints();
    
    void mergeEventPoints(int x, int y, double &sx, double &sy, int &count, std::set<GovernorGroup> &governors);
    
    void loadNavigationMesh(const std::string &filename);
    
    
    

};

#endif /* NavMesh2D_hpp */