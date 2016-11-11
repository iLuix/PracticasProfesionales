//
//  NavMesh2D.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 9/20/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "NavMesh2D.hpp"

//Class GovernorGroup -----------------------------------------------------------

std::map<GovernorGroup, int> GovernorGroup::idsMap;

//Constructor
GovernorGroup::GovernorGroup(PrimitiveSegment *p1, PrimitiveSegment *p2, double &dt1, double &dt2){
    
    //Initialization of members
    this->p1 = p1;
    this->p2 = p2;
    idp1 = p1->getId();
    idp2 = p2->getId();
    t1 = (dt1<0)?0:((dt1>1)?2:1);
    t2 = (dt2<0)?0:((dt2>1)?2:1);
    
    //Get unique id for governor group
    if(idsMap.count(*this)>0){
        id = idsMap[*this];
    }else{
        id = (int)idsMap.size() + 1;
        idsMap[*this] = id;
    }

}

//Constructor
GovernorGroup::GovernorGroup(GovernorGroup &o){
    p1 = o.p1;
    p2 = o.p2;
    t1 = o.t1;
    t2 = o.t2;
    id = o.id;
    idp1 = o.idp1;
    idp2 = o.idp2;
}

//Constructor
GovernorGroup::GovernorGroup(const GovernorGroup &o){
    p1 = o.p1;
    p2 = o.p2;
    t1 = o.t1;
    t2 = o.t2;
    id = o.id;
    idp1 = o.idp1;
    idp2 = o.idp2;
}

//Operator <
bool GovernorGroup::operator<(const GovernorGroup &o) const{
    
    return (idp1==o.idp1)?((idp2==o.idp2)?((t1==o.t1)?t2<o.t2:t1<o.t1):idp2<o.idp2):idp1<o.idp1;
}

//Operator ==
bool GovernorGroup::operator==(const GovernorGroup &o) const{
    return (idp1==o.idp1) && (idp2==o.idp2) && (t1 == o.t1) && (t2 == o.t2);
}

//Class NavMesh2DNode -------------------------------------------------------

NavMesh2DNode::NavMesh2DNode(double x, double y) : x(x), y(y), visited(false), cost(0), trajCon(0){
}

NavMesh2DNode::NavMesh2DNode(NavMesh2DNode &other){
    id = other.id;
    x = other.x;
    y = other.y;
    governors = other.governors;
    closestPoints = other.closestPoints;
    connections = other.connections;
    cost = other.cost;
    visited = other.visited;
    trajCon = other.trajCon;
}

NavMesh2DNode::NavMesh2DNode(const NavMesh2DNode &other){
    id = other.id;
    x = other.x;
    y = other.y;
    governors = other.governors;
    closestPoints = other.closestPoints;
    connections = other.connections;
    cost = other.cost;
    visited = other.visited;
    trajCon = other.trajCon;
}

void NavMesh2DNode::setCost(double x, double y){
    double d1 = this->x - x;
    double d2 = this->y - y;
    this->cost = sqrt(d1*d1 + d2*d2);
}

bool NavMesh2DNode::operator<(const NavMesh2DNode &o) const{
    return cost < o.cost;
}

bool NavMesh2DNode::operator>(const NavMesh2DNode &o) const{
    return cost > o.cost;
}

//bool NavMesh2DNode::operator==(const NavMesh2DNode &o) const{
    //return cost == o.cost;
//}

bool NavMesh2DNode::operator>=(const NavMesh2DNode &o) const{
    return cost >= o.cost;
}

bool NavMesh2DNode::operator<=(const NavMesh2DNode &o) const{
    return cost <= o.cost;
}


//Class NavNode2DConnection --------------------------------------------------


NavMesh2DConnection::NavMesh2DConnection(NavMesh2DNode* node1, NavMesh2DNode* node2, GovernorGroup &gg)
: node1(node1), node2(node2), gg(gg){
    
    if(gg.getT1() == 1 && gg.getT2() == 1){
        connectionType = 0;
    }else if(gg.getT1() == 0 && gg.getT2() == 0){
        connectionType = 1;
    }else if(gg.getT1() == 2 && gg.getT2() == 2){
        connectionType = 2;
    }else if(gg.getT1() == 0 && gg.getT2() == 2){
        connectionType = 2;
    }else if(gg.getT1() == 2 && gg.getT2() == 0){
        connectionType = 2;
    }else{
        connectionType = 3;
    }
    
    generateConnectionPoints();
    generatePolygon();
    calculateDistance();
}

NavMesh2DConnection::NavMesh2DConnection(NavMesh2DNode* node1, NavMesh2DNode* node2, int connectionType, double distance)
: node1(node1), node2(node2), connectionType(connectionType), distance(distance){
    
}

bool NavMesh2DConnection::operator<(const NavMesh2DConnection &o) const{
    if(node1->getID() == o.node1->getID()){
        return node2->getID() < o.node2->getID();
    }else{
        return node1->getID() < o.node1->getID();
    }
}

bool NavMesh2DConnection::operator==(const NavMesh2DConnection &o) const{
    return (node1->getID() == o.node1->getID()) && (node1->getID() == o.node2->getID());
}

void NavMesh2DConnection::generateConnectionPoints(){
    
    if(connectionType <= 2){
        
        double dt = 1.0/19.0;
        double t = 0;
        for(; t<=1.0; t=t+dt){
            double x = (1-t)*node1->getX() + (t)*node2->getX();
            double y = (1-t)*node1->getY() + (t)*node2->getY();
            connectionPoints.push_back(std::pair<double,double>(x,y));
        }
        
    }else{
        
        PrimitiveSegment *prim1 = 0;
        PrimitiveSegment *prim2 = 0;
        int t2 = gg.getT2();
        
        if(t2 == 1){
            prim1 = gg.getP2();
            prim2 = gg.getP1();
        }else{
            prim1 = gg.getP1();
            prim2 = gg.getP2();
        }
        
        double tt1,px1,py1;
        double tt2,px2,py2;
        double tt3,px3,py3;
        prim1->minDistancePositionToPoint(node1->getX(), node1->getY(), tt1, px1, py1);
        prim1->minDistancePositionToPoint(node2->getX(), node2->getY(), tt2, px2, py2);
        prim2->minDistancePositionToPoint(node1->getX(), node1->getY(), tt3, px3, py3);
        
        //double ttt1 = fmin(tt1,tt2);
        //double ttt2 = fmax(tt1,tt2);
        
        double dt = (tt2-tt1)/(19.0);
        
        double dx = (prim1->getX2()-prim1->getX1());
        double dy = (prim1->getY2()-prim1->getY1());
        
        
        double r = px3;
        double s = py3;
        double rrss = r*r + s*s;
        
        double t = tt1;
        while(((tt2>tt1)?(t<=tt2):(t>=tt2))){
            double u = t*prim1->getX1()  + (1-t)*prim1->getX2();
            double v = t*prim1->getY1()  + (1-t)*prim1->getY2();
            double uu = u*u;
            double vv = v*v;
            
            double h = 0, k=0;
            
            if(fabs(dx) > fabs(dy)){
                double dydx = dy/dx;
                double num = 0.5*(rrss - uu - vv) - v*(r-u)*dydx - u*(r-u);
                double den = s - v - (r-u)*dydx;
                k = num/den;
                h = v*dydx - k*dydx + u;
            }else{
                double dxdy = dx/dy;
                double num = 0.5*(rrss - u*u - v*v) - u*(s-v)*dxdy - v*(s-v);
                double den = r - u - (s-v)*dxdy;
                h = num/den;
                k = u*dxdy - h*dxdy + v;
            }
            
            //std::cout<<std::endl<<"b "<<h<<" "<<k;
            connectionPoints.push_back(std::pair<double,double>(h,k));
            
            t += dt;
        }
        
        //int a = 0;
        //a++;
        /*for(double t = ttt1; t<=ttt2; t=t+dt){
            
            
        }*/
    }

}


void NavMesh2DConnection::calculateDistance(){
    
    distance = 0;
    for(int i=0; i<(connectionPoints.size()-1); i++){
        std::pair<double,double> &point1 = connectionPoints.at(i);
        std::pair<double,double> &point2 = connectionPoints.at(i+1);
        distance += sqrt((point1.first-point2.first)*(point1.first-point2.first) + (point1.second-point2.second)*(point1.second-point2.second));
    }
}

void NavMesh2DConnection::generatePolygon(){
    
    polygon = Polygon();
    polygon.addVertex(node1->getX(), node1->getY(), false);
    std::pair<double,double> p1 = node1->getMapClosestPoints()[gg.getP1()->getId()];
    polygon.addVertex(p1.first, p1.second, true);
    std::pair<double,double> p2 = node2->getMapClosestPoints()[gg.getP1()->getId()];
    polygon.addVertex(p2.first, p2.second, true);
    polygon.addVertex(node2->getX(), node2->getY(), false);
    std::pair<double,double> p3 = node2->getMapClosestPoints()[gg.getP2()->getId()];
    polygon.addVertex(p3.first, p3.second, true);
    std::pair<double,double> p4 = node1->getMapClosestPoints()[gg.getP2()->getId()];
    polygon.addVertex(p4.first, p4.second,true);
    
}

//Class NavMesh2D  -----------------------------------------------------------

//Constructor
NavMesh2D::NavMesh2D(NavMeshScene &scene)
    : scene(scene){
    
        width = scene.getWidth();
        height = scene.getHeight();
        resWidth = scene.getResWidth();
        resHeight = scene.getResHeight();
        resolution = scene.getResolution();
        
        
        //Reserve memory
        voronoi = new int*[resHeight];
        medialAxis = new int*[resHeight];
        medialAxisPts = new int*[resHeight];
        for(int i=0; i<resHeight; i++){
            voronoi[i] = new int[resWidth];
            medialAxis[i] = new int[resWidth];
            medialAxisPts[i] = new int[resWidth];
        }
        
}

//Constructor from file
NavMesh2D::NavMesh2D(const std::string &file){
    
    loadNavigationMesh(file);
    
    double resinv = 1.0/resolution;
    resWidth = ceil(width*resinv)+1;
    resHeight = ceil(height*resinv)+1;
    
    //Reserve memory
    voronoi = new int*[resHeight];
    medialAxis = new int*[resHeight];
    medialAxisPts = new int*[resHeight];
    for(int i=0; i<resHeight; i++){
        voronoi[i] = new int[resWidth];
        medialAxis[i] = new int[resWidth];
        medialAxisPts[i] = new int[resWidth];
    }

}

//Destructor
NavMesh2D::~NavMesh2D(){
    
    if(voronoi != 0){
        for(int i=0; i<resHeight; i++){
            delete voronoi[i];
        }
        delete voronoi;
    }
    
    if(medialAxis != 0){
        for(int i=0; i<resHeight; i++){
            delete medialAxis[i];
        }
        delete medialAxis;
    }
    
    if(medialAxisPts != 0){
        for(int i=0; i<resHeight; i++){
            delete medialAxisPts[i];
        }
        delete medialAxisPts;
    }
    
    for(int i=0; i<graphNodes.size(); i++){
        delete graphNodes.at(i);
    }
    
    std::cout<<std::endl<<"NavMesh2D destroyed\n";
}


//drawScene
void NavMesh2D::drawScene(){
    scene.drawScene(cv::Vec3b(255,255,255), cv::Vec3b(255,0,0));
}

void NavMesh2D::drawScene(cv::Mat &image){
    scene.drawScene(image, cv::Vec3b(255,255,255), cv::Vec3b(255,0,0));
}

//drawVoronoi
void NavMesh2D::drawVoronoi(){
    drawVoronoi(scene.getScene());
}

void NavMesh2D::drawVoronoi(cv::Mat &image){
    
    if(image.rows != resHeight && image.cols != resWidth){
        image = cv::Mat::zeros(resHeight, resWidth, CV_8UC3);
    }
    
    for(int i=0; i<resHeight; i++){
        for(int j=0; j<resWidth; j++){
            int primitive = voronoi[i][j];
            if(primitive<0) continue;
            int r = (int)(255*0.333*primitive) % 255;
            int g = (int)(255*0.1*primitive) % 255;
            int b = (int)((1-255)*0.5*primitive) % 255;
            image.at<cv::Vec3b>(i,j) = cv::Vec3b(r,g,b);
        }
    }
    
}

//drawMedialAxis
void NavMesh2D::drawMedialAxis(){
    drawMedialAxis(scene.getScene());
}

void NavMesh2D::drawMedialAxis(cv::Mat &image){
    
    if(image.rows != resHeight && image.cols != resWidth){
        image = cv::Mat::zeros(resHeight, resWidth, CV_8UC3);
    }
    
    for(itmap=medialAxisPoints.begin(); itmap != medialAxisPoints.end(); ++itmap){
        std::pair<int,int> point = itmap->first;
        image.at<cv::Vec3b>(point.second,point.first) = cv::Vec3b(255,255,0);
    }
    
}

//drawEventPoints
void NavMesh2D::drawEventPoints(){
    drawEventPoints(scene.getScene());
}

void NavMesh2D::drawEventPoints(cv::Mat &image){
    
    if(image.rows != resHeight && image.cols != resWidth){
        image = cv::Mat::zeros(resHeight, resWidth, CV_8UC3);
    }
    
    for(int i=0; i<graphNodes.size(); i++){
        NavMesh2DNode *node = graphNodes.at(i);
        cv::circle(image, cv::Point(node->getX(), node->getY()), 2.0/resolution, cv::Scalar(255,255,0), 5.0/resolution);
    }
    
}

//drawNavMesh
void NavMesh2D::drawNavMesh(){
    drawNavMesh(scene.getScene());
}

void NavMesh2D::drawNavMesh(cv::Mat &image){
    
    drawScene(image);
    //drawMedialAxis();
    drawClosestPoints(image);
    drawConnections(image);
    drawEventPoints(image);
    
    
}

//drawClosestPoints
void NavMesh2D::drawClosestPoints(){
    drawClosestPoints(scene.getScene());
}

void NavMesh2D::drawClosestPoints(cv::Mat &image){
    
    if(image.rows != resHeight && image.cols != resWidth){
        image = cv::Mat::zeros(resHeight, resWidth, CV_8UC3);
    }
    
    for(int i=0; i<graphNodes.size(); i++){
        NavMesh2DNode *node = graphNodes.at(i);
        std::vector<std::pair<double, double> > closestPonts = node->getClosestPoints();
        for(int j=0; j<closestPonts.size(); j++){
            std::pair<double, double> point = closestPonts.at(j);
            cv::line(image, cv::Point(node->getX(), node->getY()), cv::Point(point.first, point.second), cv::Scalar(220,220,220), 1/resolution);
        }
    }
    
}

//drawConnections
void NavMesh2D::drawConnections(){
    drawConnections(scene.getScene());
}

void NavMesh2D::drawConnections(cv::Mat &image){
    
    if(image.rows != resHeight && image.cols != resWidth){
        image = cv::Mat::zeros(resHeight, resWidth, CV_8UC3);
    }
    
    for(int i=0; i<graphNodes.size(); i++){
        NavMesh2DNode *node = graphNodes.at(i);
        std::set<NavMesh2DConnection*> connections = node->getConnections();
        std::set<NavMesh2DConnection*>::iterator it;
        for(it=connections.begin(); it!=connections.end(); ++it){
            NavMesh2DConnection *connection = *it;
            drawConnection(image, *connection, cv::Scalar(255,255,0));
        }

    }

}


//drawConnection
void NavMesh2D::drawConnection(cv::Mat &image, NavMesh2DConnection &connection, const cv::Scalar &color){
    
    std::vector<std::pair<double,double> > &points = connection.getConnectionPoints();
    if(points.size() == 0){
        int a = 0;
        a++;
    }
    for(int i=0; i<(points.size()-1); i++){
        cv::Point p1(points.at(i).first, points.at(i).second);
        cv::Point p2(points.at(i+1).first, points.at(i+1).second);
        cv::line(image, p1, p2, color, 2/resolution);
    }
    
}

//drawPrimitiveSegment
void NavMesh2D::drawPrimitiveSegment(PrimitiveSegment &p, cv::Mat &img, const cv::Vec3b &color, int thickness){
    cv::line(img, cv::Point(p.getX1(),p.getY1()), cv::Point(p.getX2(), p.getY2()),color,thickness);
}

//showScene
void NavMesh2D::showScene(){
    
    cv::imshow("Scene", scene.getScene());
    cv::waitKey();
}

//saveScene
void NavMesh2D::saveScene(const std::string &path){
    cv::imwrite(path, scene.getScene());
}

//build
void NavMesh2D::build(){
    
    clock_t begin = clock();
    std::cout<<std::endl<<"NavMesh2D::build - Building navigation mesh...";
    
    generateVoronoi();
    generateMedialAxis();
    generateEventPoints();
    generateNodeConnections();
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    std::cout<<std::endl<<"NavMesh2D::build - Navigation mesh build : "<<elapsed<<"s";
    std::cout<<std::endl<<"NavMesh2D::build - Navigation mesh build : "<<graphNodes.size()<<" nodes";
}

//saveGraph
void NavMesh2D::saveNavigationMesh(const std::string &filename){
    
    std::ofstream out;
    out.open(filename);
    
    
    out<<"width "<<width<<"\n";
    out<<"height "<<height<<"\n";
    out<<"resolution "<<resolution<<"\n";
    
    //Write scene
    out<<"scene \n";
    
    //Floor
    std::vector<std::pair<double,double> > &vertices = scene.getFloor().getVertices();
    out<<"floor "<<vertices.size();
    for(int i=0; i<vertices.size(); i++){
        std::pair<double,double> &p = vertices.at(i);
        out<<" "<<p.first<<" "<<p.second;
    }
    out<<"\n";
    
    //Obstacles
    std::vector<Polygon> &obstacles = scene.getObstacles();
    for(int i=0; i<obstacles.size(); i++){
        vertices = obstacles.at(i).getVertices();
        out<<"obstacle "<<vertices.size();
        for(int j=0; j<vertices.size(); j++){
            std::pair<double,double> &p = vertices.at(j);
            out<<" "<<p.first<<" "<<p.second;
        }
        out<<"\n";
    }
    
    //Write nodes
    out<<"nodes "<<graphNodes.size()<<"\n";
    for(int i=0; i<graphNodes.size(); i++){
        NavMesh2DNode *node = graphNodes.at(i);
        out<<"node "<<node->getID()<<" "<<node->getX()<<" "<<node->getY()<<"\n";
        
        for(int j=0; j<node->getGovernors().size(); j++){
            std::pair<double,double> p = node->getClosestPoints().at(j);
            out<<"governor "<<node->getGovernors().at(j)->getId()<<" ";
            out<<p.first<<" "<<p.second<<"\n";
        }
        
    }
    
    for(int i=0; i<graphNodes.size(); i++){
        NavMesh2DNode *node = graphNodes.at(i);
        
        std::set<NavMesh2DConnection*>::iterator it;
        std::set<NavMesh2DConnection*> &connections = node->getConnections();
        for(it=connections.begin(); it!= connections.end(); ++it){
            NavMesh2DConnection *con = *it;
            out<<"connection "<<con->getNode1()->getID()<<" "<<con->getNode2()->getID()<<" ";
            out<<con->getConnectionType()<<" ";
            out<<con->getDistance();
            
            std::vector<std::pair<double,double> > &vertices = con->getPolygon().getVertices();
            std::vector<bool> &oedges = con->getPolygon().getObstacleEdges();
            //out<<"polygon "<<vertices.size();
            for(int j=0; j<vertices.size(); j++){
                std::pair<double,double> &p = vertices.at(j);
                out<<" "<<p.first<<" "<<p.second<<" "<<oedges.at(j);
            }
            //out<<"\n";
            
            //out<<"medialaxis "<<con.getConnectionPoints().size();
            for(int k=0; k<con->getConnectionPoints().size(); k++){
                out<<" "<<con->getConnectionPoints().at(k).first<<" "<<con->getConnectionPoints().at(k).second;
            }
            out<<"\n"<<std::flush;
        }
    }
    
    out.flush();
    out.close();
    
}

//loadNavigationMesh
void NavMesh2D::loadNavigationMesh(const std::string &filename){
    
    std::cout<<std::endl<<"NavMesh2D::loadingNavigationMesh - loading from file : "<<filename;
    
    std::ifstream in;
    in.open(filename);
    
    int state = 0;
    NavMesh2DNode *node = 0;
    
    if(in.is_open()){
        
        graphNodes.clear();
        
        std::string line;
        while(getline(in,line)){
            
            std::stringstream ss;
            
            ss<<line;
            std::string type;
            ss >> type;
            
            if(state == 0){
                
                if(type == "width"){
                    ss >> width;
                    continue;
                }
                
                if(type == "height"){
                    ss >> height;
                    continue;
                }
                
                if(type == "resolution"){
                    ss >> resolution;
                    continue;
                }
                
                
                if(type == "scene"){
                    scene = NavMeshScene(width,height,resolution);
                    state = 1;
                    continue;
                }
            }
            
            if(state == 1){
                
                if(type == "floor"){
                    int s;
                    double x,y;
                    ss >> s;
                    Polygon floor;
                    for(int i=0; i<s; i++){
                        ss >> x >> y;
                        floor.addVertex(x, y);
                    }
                    scene.addPolygonFloor(floor, false);
                    continue;
                }
                
                if(type == "obstacle"){
                    int s;
                    double x,y;
                    ss >> s;
                    Polygon obstacle;
                    for(int i=0; i<s; i++){
                        ss >> x >> y;
                        obstacle.addVertex(x, y);
                    }
                    scene.addPolygonObstacle(obstacle, false);
                    continue;
                }
                
                if(type == "nodes"){
                    state = 2;
                    continue;
                }
                
            }
            
            if(state == 2){
                
                if(type == "node"){
                    int id;
                    double x, y;
                    ss >> id >> x >> y;
                    node = new NavMesh2DNode(x,y);
                    node->setID(id);
                    graphNodes.push_back(node);
                }
                
                if(type == "governor"){
                    int governor;
                    double x, y;
                    ss >> governor >> x >> y;
                    node->getGovernors().push_back(scene.getPrimitives().at(governor));
                    node->getClosestPoints().push_back(std::pair<double,double>(x,y));
                }
                
                if(type == "connection"){
                    int n1,n2,connectionType;
                    double x,y,distance;
                    ss >> n1 >> n2 >> connectionType;
                    ss >> distance;
                    
                    NavMesh2DConnection *connection = new NavMesh2DConnection(graphNodes.at(n1),graphNodes.at(n2),connectionType, distance);
            
                    Polygon polygon;
                    for(int i=0; i<6; i++){
                        bool c;
                        ss >> x >> y >> c;
                        polygon.addVertex(x, y, c);
                    }
                    connection->setPolygon(polygon);
                    
                    for(int i=0; i<20; i++){
                        ss >> x >> y;
                        connection->getConnectionPoints().push_back(std::pair<double,double>(x,y));
                    }
                    
                    graphNodes.at(n1)->getConnections().insert(connection);
                    
                }
                
            }
            
            
            
        }
        
        in.close();
        
        std::cout<<std::endl<<"NavMesh2D::loadingNavigationMesh - loading finished: "<<graphNodes.size()<<" nodes loaded";
        
    }else{
        std::cout<<std::endl<<"Unable to open navigation mesh file : "<<filename;
    }
    
}

//generateVornoi
void NavMesh2D::generateVoronoi(){
    
    clock_t begin = clock();
    std::cout<<std::endl<<" NavMesh2D::generateVoronoi - Generating voronoi...";
    
    for(int i=0; i<resHeight; i++){
        
        double y = i + 0.5*resolution;
        
        for(int j=0; j<resWidth; j++){
            
            double x = j + 0.5*resolution;
            
            if(!scene.isPointFreeInScene(x, y)){
                voronoi[i][j] = -1;
                continue;
            }
            
            
            double minDistance = 1e100;
            double primitive = -1;
            
            for(int k=0; k<scene.getPrimitives().size(); k++){
                double t;
                double distance = scene.getPrimitives().at(k)->distanceToPoint(x, y, t);
                if(distance < minDistance){
                    minDistance = distance;
                    primitive = k;
                }
            }

            voronoi[i][j] = primitive;
            
        }
    }
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    std::cout<<std::endl<<" NavMesh2D::generateVoronoi - Voronoi generated : " << elapsed << "s";
    
}

//generateMedialAxis
void NavMesh2D::generateMedialAxis(){
    
    clock_t begin = clock();
    std::cout<<std::endl<<" NavMesh2D::generateMedialAxis - Generating medial axis...";
    
    for(int y=0; y<resHeight; y++){
        for(int x=0; x<resWidth; x++){
            
            std::set<int> governors;
            
            int p = voronoi[y][x];
            if(p<0) continue;
            
            governors.insert(p);
            
            
            //8-connected
            addVoronoiGovernor(p, x-1, y-1, governors);
            addVoronoiGovernor(p, x, y-1, governors);
            addVoronoiGovernor(p, x+1, y-1, governors);
            addVoronoiGovernor(p, x-1, y, governors);
            addVoronoiGovernor(p, x+1, y, governors);
            addVoronoiGovernor(p, x-1, y+1, governors);
            addVoronoiGovernor(p, x, y+1, governors);
            addVoronoiGovernor(p, x+1, y+1, governors);
            
            //Check no convexity
            if(governors.size()>1){
                
                //Filter convex governors
                std::vector<int> noConvexGovernors;
                filterConvexGovernors(x, y, governors, noConvexGovernors);
                
                //Create conections
                for(int i=0; i<noConvexGovernors.size(); i++){
                    
                    double t1,px1,py1;
                    int p1 = noConvexGovernors.at(i);
                    PrimitiveSegment *prim1 = scene.getPrimitives().at(p1);
                    prim1->minDistancePositionToPoint(x+0.5*resolution, y+0.5*resolution, t1, px1, py1, false);
                    
                    for(int j= i+1; j<noConvexGovernors.size(); j++){
                        
                        double t2,px2,py2;
                        int p2 = noConvexGovernors.at(j);
                        PrimitiveSegment *prim2 = scene.getPrimitives().at(p2);
                        prim2->minDistancePositionToPoint(x+0.5*resolution, y+0.5*resolution, t2, px2, py2, false);
                        
                        std::pair<int,int> point(x,y);
                        
                        GovernorGroup gg(prim1,prim2,t1,t2);
                        medialAxisSegments[gg].push_back(point);
                        medialAxisPoints[point].insert(gg);
                        medialAxis[y][x] = gg.getID();
            
                    }
                    
                }
                
                if(noConvexGovernors.size()>2){
                    medialAxis[y][x] = -10;
                }
                
            }
            
        }
    }
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    std::cout<<std::endl<<" NavMesh2D::generateMedialAxis - Medial axis generated : " << elapsed << "s";
    
}

//addVoronoiGovernor
void NavMesh2D::addVoronoiGovernor(int d, int x, int y, std::set<int> &governors){
    
    if( x>=0 && y>=0 && x<resWidth && y <resHeight){
        int p = voronoi[y][x];
        if((p>=0 && p != d)){
            governors.insert(p);
        }
    }
    
}

//filterConvexGovernors
void NavMesh2D::filterConvexGovernors(int x, int y, std::set<int> &governors, std::vector<int> &noConvexGovernors){
        
        
    std::set<int>::iterator it = governors.begin();
    noConvexGovernors.push_back(*it);
    ++it;
    
    for(; it != governors.end(); ++it){
        int p2 = *it;
        size_t N = noConvexGovernors.size();
        bool convex = false;
        
        for(int k=0; k<N; k++){
            int p1 = noConvexGovernors.at(k);
            convex |= checkConvexity(scene.getPrimitives().at(p1), scene.getPrimitives().at(p2), x+0.5*resolution, y+0.5*resolution);
        }
        if(!convex){
            noConvexGovernors.push_back(p2);
        }
    }
        
}

//checkConvexity
bool NavMesh2D::checkConvexity(PrimitiveSegment *p1, PrimitiveSegment *p2, double x, double y){
    
    double t,px,py;
    p1->intersectionPointToSegment(p2, t, px, py);
    
    double a;
    p2->intersectionPointToSegment(p1, a, px, py);
    
    if( (fabs(t)<1e-6 || fabs(t-1)<1e-6) &&  (a>=0 ||a>=1) ){
        addConcaveNode(px, py);
    }
    
    if( (fabs(a)<1e-6 || fabs(a-1)<1e-6) &&  (t>=0 ||t>=1) ){
        addConcaveNode(px, py);
    }
    
    if( (fabs(t)<1e-6 || fabs(t-1)<1e-6) &&  (fabs(a)<1e-6 || fabs(a-1)<1e-6) ){
        
        addConcaveNode(px, py);
        
        double px1,py1;
        double px2,py2;
        p1->middlePoint(px1,py1);
        p2->middlePoint(px2,py2);
        
        PrimitiveSegment pa(x,y,px1,py1);
        PrimitiveSegment pb(x,y,px2,py2);
        
        double t3,t4;
        pa.intersectionPointToSegment(p2, t3, px, py);
        
        if(!(t3<0 || t3>1)) return true;
        
        pb.intersectionPointToSegment(p1, t4, px, py);
        
        
        if( !(t4<0 || t4>1) ) return true;

    }
    
    return false;
}

void NavMesh2D::addConcaveNode(double px, double py){
    
    if(py>=0 && px>=0 && px<resWidth && py<resHeight){
        int xx = px;
        int yy = py;
        medialAxisPts[yy][xx] = 5;
        medialAxis[yy][xx] = -5;
        
        if(yy>0 && xx>0){
            medialAxisPts[yy-1][xx-1] = 5;
            medialAxis[yy-1][xx-1] = -5;
        }
        
        if(yy>0){
            medialAxisPts[yy-1][xx] = 5;
            medialAxis[yy-1][xx] = -5;
        }
        
        if(yy>0 && xx<(resWidth-1)){
            medialAxisPts[yy-1][xx+1] = 5;
            medialAxis[yy-1][xx+1] = -5;
        }
        
        if(xx>0){
            medialAxisPts[yy][xx-1] = 5;
            medialAxis[yy][xx-1] = -5;
        }
        
        if(xx<(resWidth-1)){
            medialAxisPts[yy][xx+1] = 5;
            medialAxis[yy][xx+1] = -5;
        }
        
        if(yy<(resHeight-1) && xx>0){
            medialAxisPts[yy+1][xx-1] = 5;
            medialAxis[yy+1][xx-1] = -5;
        }
        
        if(yy<(resHeight-1)){
            medialAxisPts[yy+1][xx] = 5;
            medialAxis[yy+1][xx] = -5;
        }
        
        if(yy<(resHeight-1) && xx<(resWidth-1)){
            medialAxisPts[yy+1][xx+1] = 5;
            medialAxis[yy+1][xx+1] = -5;
        }
        
    }
    
}

//generateEventPoints
void NavMesh2D::generateEventPoints(){
    
    clock_t begin = clock();
    std::cout<<std::endl<<" NavMesh2D::generateEventPoints - Generating event points...";
    
    for(itmap=medialAxisPoints.begin(); itmap!=medialAxisPoints.end(); ++itmap){
        
        int x = itmap->first.first;
        int y = itmap->first.second;
        
        int p = medialAxis[y][x];
        
        if(addEventPoint(x-1, y-1, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x, y-1, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x+1, y-1, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x-1, y, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x+1, y, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x-1, y+1, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x, y+1, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
        
        if(addEventPoint(x+1, y+1, p)){
            medialAxisPts[y][x] = 10;
            eventPoints.insert(itmap->first);
            continue;
        }
    }
    
    mergeEventPoints();
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    std::cout<<std::endl<<" NavMesh2D::generateEventPoints - Event points generated : " << elapsed << "s";
}

//addEventPoint
bool NavMesh2D::addEventPoint(int x, int y, int p){
    
    if( x>=0 && y>=0 && x<resWidth && y<resHeight ){
        int d = medialAxis[y][x];
        if(d!=0 && p != d){
            return true;
        }
    }
    return false;
}

//mergeEventPoints
void NavMesh2D::mergeEventPoints(){
    clock_t begin = clock();
    std::cout<<std::endl<<"  NavMesh2D::mergeEventPoints - Merging event points...";
    
    for(itep = eventPoints.begin(); itep != eventPoints.end(); ++itep){
        
        std::pair<int, int> pp = *itep;
        int mp = medialAxisPts[pp.second][pp.first];
        if (mp == 0) continue;
        
        double sx=0, sy=0;
        int count = 0;
        std::set<GovernorGroup> ggs;
        mergeEventPoints(pp.first, pp.second, sx, sy, count, ggs);
        
        double xmean = sx/(double)count;
        double ymean = sy/(double)count;
        
        NavMesh2DNode *node = new NavMesh2DNode(xmean, ymean);
        node->setID((int)graphNodes.size());
        graphNodes.push_back(node);
        
        std::set<GovernorGroup>::iterator it;
        std::set<int> governors;
        for(it = ggs.begin(); it != ggs.end(); ++it){
            GovernorGroup gg = *it;
            connectionMap[*it].push_back(node);
            governors.insert(gg.getP1()->getId());
            governors.insert(gg.getP2()->getId());
        }
        
        std::set<int>::iterator it2;
        for(it2=governors.begin(); it2 != governors.end(); ++it2){
            double px,py,t;
            int primitive = *it2;
            PrimitiveSegment *prim = scene.getPrimitives().at(primitive);
            node->getGovernors().push_back(prim);
            scene.getPrimitives().at(primitive)->minDistancePositionToPoint(node->getX(), node->getY(), t, px, py);
            node->getClosestPoints().push_back(std::pair<double,double>(px,py));
            node->getMapClosestPoints()[prim->getId()] = std::pair<double,double>(px,py);
        }
    }
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    std::cout<<std::endl<<"  NavMesh2D::mergeEventPoints - Event points merged : " << elapsed << "s";
}

//mergeEventPoints
void NavMesh2D::mergeEventPoints(int x, int y, double &sx, double &sy, int &count, std::set<GovernorGroup> &governors){
        
    if((x>=0) && (y>=0) && (x<resWidth) && (y<resHeight)){
            
        int d = medialAxisPts[y][x];
            
        if(d > 0){
   
            sx += x;
            sy += y;
            count++;
                
            medialAxisPts[y][x]= 0;
                
            std::pair<int,int> point(x,y);
                
            std::set<GovernorGroup> ggs = medialAxisPoints[point];
            std::set<GovernorGroup>::iterator it4;
            for(it4 = ggs.begin(); it4!=ggs.end(); ++it4){
                governors.insert(*it4);
            }

                
            mergeEventPoints(x-1,y-1, sx, sy, count, governors);
            mergeEventPoints(x,y-1, sx, sy, count, governors);
            mergeEventPoints(x+1,y-1, sx, sy, count, governors);
            mergeEventPoints(x-1,y, sx, sy, count, governors);
            mergeEventPoints(x+1,y, sx, sy, count, governors);
            mergeEventPoints(x-1,y+1, sx, sy, count, governors);
            mergeEventPoints(x,y+1, sx, sy, count, governors);
            mergeEventPoints(x+1,y+1, sx, sy, count, governors);
        }
    }
}

//generateNodeConnections
void NavMesh2D::generateNodeConnections(){
    
    clock_t begin = clock();
    std::cout<<std::endl<<" NavMesh2D::generateNodeConnections - Generating node connections...";
    
    for(itcm = connectionMap.begin(); itcm != connectionMap.end(); ++itcm){
    
        GovernorGroup gg = itcm->first;
        
        std::vector<std::pair<int,int> > points;
        std::vector<NavMesh2DNode*> nodes = itcm->second;
        
        if(nodes.size()<=1) continue;
        
        if(nodes.size()>2){
            
            for(int k=0; k<nodes.size(); k++){
                NavMesh2DNode *n1 = nodes.at(k);
                for(int l=k+1; l<nodes.size(); l++){
                    NavMesh2DNode *n2 = nodes.at(l);
                    double xx = 0.5*(n1->getX() + n2->getX());
                    double yy = 0.5*(n1->getY() + n2->getY());
                    
                    std::pair<int,int> pp(xx,yy);
                    if(medialAxisPoints.count(pp) > 0){
                        std::set<GovernorGroup> ggs = medialAxisPoints[pp];
                        if(ggs.count(gg) > 0){
                            NavMesh2DConnection *c1 = new NavMesh2DConnection(n1, n2, gg);
                            NavMesh2DConnection *c2 = new NavMesh2DConnection(n2, n1, gg);
                            n1->getConnections().insert(c1);
                            n2->getConnections().insert(c2);

                        }
                    }
                    
                }
            }
            
        }else{
            
            
            NavMesh2DConnection *c1 = new NavMesh2DConnection(nodes.at(0), nodes.at(1), gg);
            NavMesh2DConnection *c2 = new NavMesh2DConnection(nodes.at(1), nodes.at(0), gg);
    
            nodes.at(0)->getConnections().insert(c1);
            nodes.at(1)->getConnections().insert(c2);

        }

    }
    
    clock_t end = clock();
    double elapsed = double(end-begin)/CLOCKS_PER_SEC;
    std::cout<<std::endl<<" NavMesh2D::generateNodeConnections - Node connections generated : "<<elapsed<<"s";
    
}

