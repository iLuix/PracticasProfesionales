//
//  NavMesh2DTrajectory.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/6/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "NavMesh2DTrajectory.hpp"

NavMesh2DTrajectoryBuilder::NavMesh2DTrajectoryBuilder(const std::string &fileName):
    navmesh(NavMesh2D(fileName)){
}

NavMesh2DTrajectoryBuilder::NavMesh2DTrajectoryBuilder(NavMesh2D &navmesh)
: navmesh(navmesh){
    
}

bool NavMesh2DTrajectoryBuilder::searchTrajectory(double x1, double y1, double x2, double y2, NavMesh2DTrajectory &outTrajectory, double t, double iterations, double minDistanceToObstacles, bool &finished){
    
    clock_t begin = clock();
    std::cout<<std::endl<<"NavMesh2D::searchTrajectory - searching trajectory...";
    
    double resolution = navmesh.getResolution();
    x1 = x1/resolution;
    x2 = x2/resolution;
    y1 = y1/resolution;
    y2 = y2/resolution;
    
    outTrajectory.getXorigin() = x1;
    outTrajectory.getYorigin() = y1;
    outTrajectory.getXdestiny() = x2;
    outTrajectory.getYdestiny() = y2;
    outTrajectory.getXstart() = x1;
    outTrajectory.getXend() = x2;
    outTrajectory.getYstart() = y1;
    outTrajectory.getYend() = y2;
    
    NavMesh2DConnection *con1=0,*con2=0;
    bool found1 = searchNearestNode(x1, y1, con1);
    bool found2 = searchNearestNode(x2, y2, con2);

    if(!found1 || !found2){
        std::cout<<std::endl<<"NavMesh2D::searchTrajectory - trajectory not found.";
        return false;
    }
    
    
    dijkstraSearch(x1,y1,x2,y2,con1, con2, outTrajectory);
    
    
    generateTrajectoryPoints(outTrajectory, x1,y1,outTrajectory.getXend(),outTrajectory.getYend(),t);
    
    //cv::Mat img;
    //navmesh.drawNavMesh(img);
    outTrajectory.getMinDistObstacles() = minDistanceToObstacles;
    narrowTrajectory(/*img,*/ outTrajectory, iterations, minDistanceToObstacles);
    generateMinimumTrajectoryPoints(outTrajectory, t);
    
    finished = (outTrajectory.getXdestiny() == outTrajectory.getXend()) && (outTrajectory.getYdestiny() == outTrajectory.getYend());
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    timesearch += elapsed;
    countsearchs++;
    if(elapsed > maxtime) maxtime = elapsed;
    std::cout<<std::endl<<"NavMesh2D::searching trajectory - trajectory found: "<<1000*elapsed<<"ms";
    return true;
}

void NavMesh2DTrajectoryBuilder::dijkstraSearch(double x1, double y1, double x2, double y2, NavMesh2DConnection *con1, NavMesh2DConnection *con2, NavMesh2DTrajectory &outTrajectory){
    
    
    
    //cv::Mat img;
    //navmesh.drawNavMesh(img);
    //cv::circle(img, cv::Point(x1,y1), 5, cv::Scalar(0,0,255),10);
    //cv::circle(img, cv::Point(x2,y2), 5, cv::Scalar(0,0,255),10);

    
    int N = (int)navmesh.getGraphNodes().size();
    for(int i=0; i<N; i++){
        NavMesh2DNode *n = navmesh.getGraphNodes().at(i);
        n->setVisited(false);
        n->setCost(0);
    }
    
    
    NavMesh2DNode *n1 = con1->getNode1();
    NavMesh2DNode *n2 = con1->getNode2();
    NavMesh2DNode *n3 = con2->getNode1();
    NavMesh2DNode *n4 = con2->getNode2();
    
    n1->setCost(x1, y1);
    n2->setCost(x1, y1);
    
    typedef std::priority_queue<NavMesh2DNodeWrapper,std::vector<NavMesh2DNodeWrapper>, mycomparison<NavMesh2DNodeWrapper> > mypq;
    mypq Q(mycomparison<NavMesh2DNodeWrapper>(true));
    //CustomPriorityQueue<NavMesh2DNodeWrapper> Q;
    //std::multiset<NavMesh2DNodeWrapper, std::less<NavMesh2DNodeWrapper> > Q;
    
    Q.push(n1);
    Q.push(n2);
    
    bool fn3 = false, fn4 = false;
    con2->xend = x2;
    con2->yend = y2;
    bool cont = true;
    int count = 0;
    while(!Q.empty() && cont){
        
        //NavMesh2DNodeWrapper b = Q.bottom();
        //if(b.node == n3 || b.node == n4) {
        //    cont = false;
        //}
        
        NavMesh2DNodeWrapper s = Q.top();
        //Q.erase(Q.begin());
        Q.pop();
        
        /*cv::circle(img, cv::Point(s.node->getX(),s.node->getY()), 5, cv::Scalar(255,0,255),10);
        cv::imshow("a", img);
        cv::waitKey();
        cv::circle(img, cv::Point(s.node->getX(),s.node->getY()), 5, cv::Scalar(255,200,255),10);*/
        
        /*if(s.node == n3 || s.node == n4){
            cont = false;
            int a = 0;
        }*/
        if(s.node == n3) fn3 = true;
        if(s.node == n4) fn4 = true;
        cont = !fn3 || !fn4;
        count++;
        if(count > 1e100){
            cont = false;
            n3 = s.node;
            n4 = s.node;
            outTrajectory.getXend() = s.node->getX();
            outTrajectory.getYend() = s.node->getY();
            con2->xend = s.node->getX();
            con2->yend = s.node->getY();

        }
        
        std::set<NavMesh2DConnection*> &connections = s.node->getConnections();
        std::set<NavMesh2DConnection*>::iterator it;
        
        
        for(it = connections.begin(); it!= connections.end(); ++it){
            
            NavMesh2DConnection *con = *it;
            NavMesh2DNode *n = con->getNode2();
            
            
            /*if( (*(--Q.end())).node == n3 || (*(--Q.end())).node == n4){
                cont = false;
            }*/
            
            double gn = n->getCost();
            double gs = s.node->getCost();
            
            double dsn = con->getDistance();
            double gns = gs + dsn;
            
            if((!n->isVisited()) || (gn > gns)){
                n->setCost(gns);
                n->setVisited(true);
                n->trajCon = s.node;
                Q.push(n);
                /*if(n == n3 || n == n4){
                    int aaa = 0;
                }*/
                

            }
            
            
            
        }
        
    }
    
    outTrajectory.getConnections().push_back(con2);
    
    
    NavMesh2DNode *node = (n3->getCost() < n4->getCost())?n3:n4;
    
    con2->xstart = node->getX();
    con2->ystart = node->getY();

    
    while((node !=0) && (node->getID() != con1->getNode1()->getID()) && (node->getID() != con1->getNode2()->getID())){
    //while((node !=0) && (node != con1->getNode1()) && (node != con1->getNode2())){
        if(node->trajCon != 0){
            std::set<NavMesh2DConnection*> &connections = node->getConnections();
            std::set<NavMesh2DConnection*>::iterator it;
            for(it=connections.begin(); it != connections.end(); it++){
                NavMesh2DConnection *con = *it;
                if(con->getNode2()->getID() == node->trajCon->getID()){
                    con->xstart = con->getNode2()->getX();
                    con->ystart = con->getNode2()->getY();
                    con->xend = con->getNode1()->getX();
                    con->yend = con->getNode1()->getY();
                    outTrajectory.getConnections().push_back(con);
                //if(con->getNode2() == node->trajCon){
                    break;
                }
            }
        }
        node = node->trajCon;
        
    }
    
    con1->xend = node->getX();
    con1->yend = node->getY();
    con1->xstart = x1;
    con1->ystart = y1;
    outTrajectory.getConnections().push_back(con1);
}


bool NavMesh2DTrajectoryBuilder::searchNearestNode(double x, double y, NavMesh2DConnection *&connection){
    
    int N = (int)navmesh.getGraphNodes().size();
    std::vector<NavMesh2DNode*>& nodes = navmesh.getGraphNodes();
    for(int i=0; i<N; i++){
        NavMesh2DNode* node = nodes.at(i);
        std::set<NavMesh2DConnection*>::iterator it;
        for(it=node->getConnections().begin(); it!= node->getConnections().end(); ++it){
            connection = *it;
            if(connection->getPolygon().isPoinInPolygon(x, y)){
                return true;
            }
        }
    }
    return false;
}

void NavMesh2DTrajectoryBuilder::generateTrajectoryPoints(NavMesh2DTrajectory &trajectory,  double gx1, double gy1, double gx2, double gy2, double t){
    
    std::vector<NavMesh2DConnection*> &connections = trajectory.getConnections();
    double ti = 0;
    double x1,y1,x2,y2;
    
    //trajectory.getXstart() = gx1;
    //trajectory.getXend() = gx2;
    //trajectory.getYstart() = gy1;
    //trajectory.getYend() = gy2;
    
    trajectory.getTrajectory().push_back(std::pair<double,double>(gx1,gy1));
    
    int N = (int)connections.size()-2;
    NavMesh2DConnection *con = connections.at(N);
    
    x2 = con->getNode2()->getX();
    y2 = con->getNode2()->getY();
    generateNewPoint(trajectory.getTrajectory(), t, ti, gx1, gy1, x2, y2);
    
    
    for(int i=N; i>0; i--){
        con = connections.at(i);

        int M = (int)con->getConnectionPoints().size() - 1;
        if(con->getConnectionType()<=2){

             x1 = con->getNode2()->getX();
             y1 = con->getNode2()->getY();
             generateNewPoint(trajectory.getTrajectory(), t, ti, x2, y2, x1,y1);
             x2 = con->getNode1()->getX();
             y2 = con->getNode1()->getY();
             generateNewPoint(trajectory.getTrajectory(), t, ti, x1, y1, x2, y2, con->getDistance());
            
        }else{
            std::vector<std::pair<double,double> > &points = con->getConnectionPoints();
            
            generateNewPoint(trajectory.getTrajectory(), t, ti, x2, y2, points.at(M).first, points.at(M).second);
            
            for(int j=M; j>0; j--){
                x1 = points.at(j).first;
                y1 = points.at(j).second;
                x2 = points.at(j-1).first;
                y2 = points.at(j-1).second;
                generateNewPoint(trajectory.getTrajectory(), t, ti, x1, y1, x2, y2);
            }
        }
    }

    con = connections.at(1);
    generateNewPoint(trajectory.getTrajectory(), t, ti,con->getNode1()->getX(), con->getNode1()->getY(), gx2, gy2);
    trajectory.getTrajectory().push_back(std::pair<double,double>(gx2,gy2));
}

void NavMesh2DTrajectoryBuilder::generateMinimumTrajectoryPoints(NavMesh2DTrajectory &trajectory, double t){
    
    std::vector<NavMesh2DConnection*> &connections = trajectory.getConnections();
    double ti = 0;
    double x1,y1,x2,y2;

    int N = (int)connections.size()-1;
    NavMesh2DConnection *con  = connections.at(N);
    trajectory.getMinTrajectory().push_back(std::pair<double,double>(con->xstart,con->ystart));

    
    for(int i=N; i>=0; i--){
        NavMesh2DConnection *con = connections.at(i);

        x1 = con->xstart;
        y1 = con->ystart;
        x2 = con->xend;
        y2 = con->yend;
        generateNewPoint(trajectory.getMinTrajectory(), t, ti, x1, y1, x2, y2);
    }
}

void NavMesh2DTrajectoryBuilder::generateNewPoint(std::vector<std::pair<double,double> >&points, double t, double &ti, double x1, double y1, double x2, double y2, double distance){
    if(distance < 0){
        distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }
    double div = t/distance;
    double tt = (t-ti)/distance;
    double x=x2,y=y2;
    tt = (tt>=0)?tt:0;
    ti = distance+ti;
    
    while(ti >= t){
        x = (1-tt)*x1 + (tt)*x2;
        y = (1-tt)*y1 + (tt)*y2;
        points.push_back(std::pair<double,double>(x,y));
        ti = (1-tt)*distance;
        tt += div;
        
    }

}

void NavMesh2DTrajectoryBuilder::drawTrajectory(NavMesh2DTrajectory &trajectory, cv::Mat &image){
    
    double resolution = navmesh.getResolution();
    navmesh.drawNavMesh(image);
    
    
    for(int i=0; i<trajectory.getConnections().size(); i++){
        NavMesh2DConnection *con = trajectory.getConnections().at(i);
        //con->getPolygon().drawPolygon(image, cv::Vec3b(255,235,255), cv::Vec3b(255,200,255),1.0/resolution);
        con->getPolygon().drawPolygon(image, cv::Vec3b(255,235,255), cv::Vec3b(255,235,255), cv::Vec3b(255,200,255), 1.0/resolution, trajectory.getMinDistObstacles()   );
    }
    
    //navmesh.drawClosestPoints(image);
    //navmesh.drawConnections(image);
    cv::circle(image, cv::Point(trajectory.getXstart(), trajectory.getYstart()), 2.0/resolution, cv::Scalar(0,0,255), 7.0/resolution);
    cv::circle(image, cv::Point(trajectory.getXend(), trajectory.getYend()), 2.0/resolution, cv::Scalar(0,255,0), 7.0/resolution);
    cv::circle(image, cv::Point(trajectory.getXdestiny(), trajectory.getYdestiny()), 2.0/resolution, cv::Scalar(0,255,0), 7.0/resolution);


    for(int i=0; i<trajectory.getTrajectory().size(); i++){
        std::pair<double,double> &p = trajectory.getTrajectory().at(i);
        cv::circle(image, cv::Point(p.first,p.second), 1.0/resolution, cv::Scalar(255,0,255), 1.0/resolution);
    }
    
    for(int i=0; i<trajectory.getMinTrajectory().size(); i++){
        std::pair<double,double> &p = trajectory.getMinTrajectory().at(i);
        cv::circle(image, cv::Point(p.first,p.second), 1.0/resolution, cv::Scalar(10,180,0), 1.0/resolution);
    }
    
    /*int M = (int)trajectory.getConnections().size() - 1;
    for(int i=M; i>=0; i--){
        NavMesh2DConnection *con = trajectory.getConnections().at(i);
        cv::line(image,cv::Point(con->xstart,con->ystart),cv::Point(con->xend,con->yend),cv::Scalar(0,255,0),1.0/navmesh.getResolution());
    }*/
    
}

void NavMesh2DTrajectoryBuilder::narrowTrajectory(/*cv::Mat &img,*/ NavMesh2DTrajectory &trajectory, int iterations, double minDistanceToObstacles){
    
    double dif = 0;
    //clock_t begin = clock();
    //std::cout<<std::endl<<"NavMesh2D::narrowTrajectory - narrowing trajectory...";
    
    //drawTrajectory(trajectory, img);
    for(int kk=0; kk<iterations; kk++){
        dif = 0;
        //std::cout<<std::endl<<"Iteracion "<<kk;
        //cv::Mat img2;
        //img.copyTo(img2);
        double x1 = trajectory.getXstart();
        double y1 = trajectory.getYstart();
        double x2,y2;
        int M = (int)trajectory.getConnections().size() - 2;
        NavMesh2DConnection *lcon = trajectory.getConnections().at(M+1);
        lcon->xstart = x1;
        lcon->ystart = y1;
        double x,y;
        for(int i=M; i>0; i--){
            //std::cout<<std::endl<<kk<<" "<<i;
            NavMesh2DConnection *con = trajectory.getConnections().at(i);
            //lcon->getPolygon().drawPolygon(img, cv::Vec3b(255,235,255), cv::Vec3b(255,235,255), cv::Vec3b(255,200,255), 2, minDistanceToObstacles);
            x2 = con->xend;
            y2 = con->yend;
            lcon->getPolygon().intersection(x1, y1, x2, y2, x, y,minDistanceToObstacles/*,img,kk>=3*/);
            
            dif += sqrt((lcon->xend-x)*(lcon->xend-x) + (lcon->yend-y)*(lcon->yend-y));
            dif += sqrt((con->xstart-x)*(con->xstart-x) + (con->ystart-y)*(con->ystart-y));
            
            lcon->xend = x;
            lcon->yend = y;
            con->xstart = x;
            con->ystart = y;
            
            /*if(kk>=1200 && i<6){
                lcon->getPolygon().drawPolygon(img2, cv::Vec3b(125,255,255), cv::Vec3b(0,200,200), 1/navmesh.getResolution());
                cv::line(img2,cv::Point(x1,y1),cv::Point(x2,y2),cv::Scalar(0,255,0),1.0/navmesh.getResolution());
                cv::circle(img2, cv::Point(x,y), 1, cv::Scalar(0,125,0),2.0/navmesh.getResolution());
                cv::circle(img2, cv::Point(x2,y2), 1, cv::Scalar(0,125,255),2.0/navmesh.getResolution());
                cv::imwrite("x.png", img2);
                cv::imshow("x", img2);
                cv::waitKey();
            }*/
            
            x1 = x;
            y1 = y;
            //x1 = con->xstart;
            //y1 = con->ystart;
            lcon = con;
            //break;
            
        }
        x2 = trajectory.getXend();
        y2 = trajectory.getYend();
        NavMesh2DConnection *con = trajectory.getConnections().at(0);
        lcon->getPolygon().intersection(x1, y1, x2, y2, x, y,minDistanceToObstacles/*,img,true*/);
        dif += sqrt((lcon->xend-x)*(lcon->xend-x) + (lcon->yend-y)*(lcon->yend-y));
        dif += sqrt((con->xstart-x)*(con->xstart-x) + (con->ystart-y)*(con->ystart-y));
        lcon->xend = x;
        lcon->yend = y;
        con->xstart = x;
        con->ystart = y;
        //std::cout<<std::endl<<dif;
        if(dif < 5) {
            
            //std::cout<<std::endl<<kk;
            kk = iterations;
        }
  
    }
    
    //clock_t end = clock();
    //double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    //std::cout<<std::endl<<"NavMesh2D::narrowTrajectory  - trajectory narrowed: "<<1000*elapsed<<"ms";
}
