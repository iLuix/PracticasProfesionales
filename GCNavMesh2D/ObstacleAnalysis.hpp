//
//  ObstacleAnalysis.hpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/5/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#ifndef ObstacleAnalysis_hpp
#define ObstacleAnalysis_hpp

#include <stdio.h>
// A C++ Program to detect cycle in an undirected graph
#include<iostream>
#include <list>
#include <limits.h>
#include <vector>
#include <map>
#include <set>

using namespace std;

// Class for an undirected graph
class Point2D;
class Graph
{
    //  int V;    // No. of vertices
    map<int,set<int>> adj;    // Pointer to an array containing adjacency lists
    vector<vector<int>> edgesCloser;
    vector<vector<Point2D>> renctangleCloserPoligons;
    vector<vector<Point2D>> pointsPoligons;
    
    map<Point2D,int> mapa;
    
    
    bool isCyclicUtil(int v, bool visited[], int parent);
    int findFirstCyclic(int v, bool visited[], vector<int> &camino, int parent, int children, bool start=false);
    bool isInVector(int value, vector<int> &vectora);
    void addSetCyclic(vector<int> cycle);
    
public:
    Graph();   // Constructor
    void addEdge(int v, int w);   // to add an edge to graph
    void addEdge(double x1, double y1, double x2, double y2);
    bool isCyclic();   // returns true if there is a cycle
    void calculateCyclics();
    bool isGoodPoint(int x, int y);
    
    
    
    vector<vector<Point2D>> getEdgesCloserPoints() ;
    vector<vector<Point2D> > *getPointsPoligons();
};

class Point2D
{
    double x;
    double y;
public:
    Point2D();
    Point2D(double x_, double y_);
    
    double getX() const;
    void setX(double value);
    double getY() const;
    void setY(double value);
    
    friend bool operator<(Point2D const& a, Point2D const& b)
    {
        return (a.x < b.x) || (a.x == b.x && a.y < b.y) ;
    }
    friend bool operator==(Point2D const& a, Point2D const& b)
    {
        return (a.x == b.x  && a.y == b.y);
    }
    
    
};

#endif /* ObstacleAnalysis_hpp */
