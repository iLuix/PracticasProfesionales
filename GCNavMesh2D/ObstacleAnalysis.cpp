//
//  ObstacleAnalysis.cpp
//  GCNavMesh2D
//
//  Created by gds1 on 10/5/16.
//  Copyright © 2016 gds1. All rights reserved.
//

#include "ObstacleAnalysis.hpp"
using namespace std;

Graph::Graph()
{
    
    //adj = new map<int,vector<int>>();
    // parents = new  vector<int>();
    
    
}

void Graph::addEdge(int v, int w)
{
    adj[v].insert(w); // Add w to v’s list.
    adj[w].insert(v); // Add v to w’s list.
}

void Graph::addEdge(double x1, double y1, double x2, double y2)
{
    Point2D p1(x1,y1);
    Point2D p2(x2,y2);
    int cont=(int)mapa.size();
    //   cout<<"antes contador mapa p1: "<<mapa.size()<<endl;
    std::map<Point2D,int>::iterator it;
    it = mapa.find(p1);
    if (it == mapa.end())
        mapa[p1]=cont;
    
    cont=(int)mapa.size();
    it = mapa.find(p2);
    if (it == mapa.end())
        mapa[p2]=cont;
    addEdge(mapa[p1],mapa[p2]);
    //   cout<<"point: ("<<p1.getX()<<",  "<<p1.getY()<<") = "<<mapa[p1]<<"| ("<<p2.getX()<<",  "<<p2.getY()<<") = "<<mapa[p2]<<endl;
    
}

// A recursive function that uses visited[] and parent to detect
// cycle in subgraph reachable from vertex v.







vector<vector<Point2D> > *Graph::getPointsPoligons()
{
    return &pointsPoligons;
}
bool Graph::isCyclicUtil(int v, bool visited[], int parent)
{
    // Mark the current node as visited
    visited[v] = true;
    
    // Recur for all the vertices adjacent to this vertex
    set<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
    {
        // If an adjacent is not visited, then recur for that adjacent
        if (!visited[*i])
        {
            if (isCyclicUtil(*i, visited, v))
                return true;
        }
        
        // If an adjacent is visited and not parent of current vertex,
        // then there is a cycle.
        else if (*i != parent)
            return true;
    }
    return false;
}



int Graph::findFirstCyclic(int v, bool visited[],  vector<int>  &way, int parent, int children, bool start)
{
    //cout<<"in: "<<v<<"  "<<children<<"  "<<star<<"  "<<count<<endl;
    
    if((!start && v==children)){
        return 1;
    }
    if(visited[v])
        return 0;
    visited[v] = true;
    
    
    set<int>::iterator i;
    for (i = adj[v].begin(); i != adj[v].end(); ++i)
        if(*i!=parent){
            //cout<<"** objetive:"<<children<<"  actual "<<v<<" find: "<<*i<<endl;
            
            int aux= findFirstCyclic(*i,visited,way, v, children,false);
            if(aux){
                way.push_back(*i);
                return 1;
            }
        }
    return 0;
    
    
}

bool Graph::isInVector(int value, vector<int>& vectora)
{
    int t=(int)vectora.size();
    for(int i=0;i<t;i++)
        if(vectora[i]==value)
            return true;
    return false;
    
}


void Graph::addSetCyclic(vector<int>cycle)
{
    int totalcycle=(int)edgesCloser.size();
    int sizenewcycle=(int)cycle.size();
    bool isInSomeSet=false;
    for( int i=0 ; i<totalcycle;i++){
        int sizecycle=(int)edgesCloser[i].size();
        
        bool  isinedgescloser=true;
        if(sizenewcycle<=sizecycle){
            for( int k=0;k<sizenewcycle;k++)
                if(!isInVector(cycle[k],edgesCloser[i])){
                    isinedgescloser=false;
                    break;
                }
            if(isinedgescloser){
                isInSomeSet=true;
                break;
            }
            
        }
        else{ // si tiene longitud mayor checar si existe un conjunto ya en la lista que contenga estos elementos
            
            for( int k=0;k<sizecycle;k++)
                if(!isInVector(edgesCloser[i][k],cycle)){
                    isinedgescloser=false;
                    break;
                }
            if(isinedgescloser){
                edgesCloser[i]=cycle;
                isInSomeSet=true;
                break;
                
            }
            
        }
        
    } //end for
    if(!isInSomeSet){
        //  cout<<"conjunto agregado"<<endl;
        edgesCloser.push_back(cycle);
    }
    else{
        // cout<<"conjunto no agregado"<<endl;
        
    }
    
}
void Graph::calculateCyclics()
{
    
    //for every edge
    int V=0;
    map<int,set<int>>::iterator it;
    
    for( it = adj.begin(); it != adj.end(); ++it)
        if(it->first > V)
            V=it->first;
    /*
     for( int k=0;k<=V;k++){
     cout<<endl<<"adjuntos: "<<k<<endl;
     set<int>::iterator it2;
     for (it2 = adj[k].begin(); it2 != adj[k].end(); ++it2)
     cout<<*it2<<",     ";
     }*/
    for( int k=0;k<=V;k++){
        
        bool *visited = new bool[V];
        for (int i = 0; i <= V; i++)
            visited[i] = false;
        
        // cout<<endl<<"calculando para: "<<k<<endl;
        vector<int> way;
        findFirstCyclic(k,visited,way,-1,k, true);
        
        if(way.size()>0){
            // cout<<"ciclo encontrado: "<<endl;
            addSetCyclic(way);
        }
        
        
    }
    
    
    int totalcicles=(int)edgesCloser.size();
    /*
     for(int i=0;i<totalcicles;i++){
     cout<<endl<<"ciclo  "<<i<<" es de "<<edgesCloser[i].size()<<"elementos"<<endl;
     for( int k=0;k<edgesCloser[i].size();k++){
     std::map<Point2D,int>::iterator it;
     for (it = mapa.begin(); it != mapa.end(); ++it )
     if (it->second == edgesCloser[i][k]){
     Point2D p=it->first;
     cout<<"<"<<i<<"> "<<edgesCloser[i][k]<<"    ("<<p.getX()<<" "<<p.getY()<<")"<< endl;
     break;
     }
     
     }
     }*/
    
    cout<<"terminado calculo de ciclos "<<totalcicles<<endl;
    for(int i=0;i<totalcicles;i++){
        //cout<<endl<<"ciclo  "<<i<<" es de "<<edgesCloser[i].size()<<"elementos"<<endl;
        int xmin= 10000000000;
        int ymin= 10000000000;
        int xmax= -1000000000;
        int ymax= -1000000000;
        vector<Point2D> poligonocerrado;
        for( int k=0;k<edgesCloser[i].size();k++){
            //  cout<<"<"<<i<<"> "<<edgesCloser[i][k]<<endl;
            //obtener el punto asociado al vertice nodo:
            std::map<Point2D,int>::iterator it;
            for (it = mapa.begin(); it != mapa.end(); ++it )
                if (it->second == edgesCloser[i][k]){
                    Point2D P = it->first;
                    if(P.getX()<xmin) xmin=P.getX();
                    if(P.getY()<ymin) ymin=P.getY();
                    if(P.getX() > xmax) xmax=P.getX();
                    if(P.getY() > ymax) ymax=P.getY();
                    
                    poligonocerrado.push_back(P);
                    break;
                }
            
            
        }
        pointsPoligons.push_back(poligonocerrado);
        
        Point2D p1(xmin,ymin);
        Point2D p2(xmax,ymax);
        vector<Point2D> vectoraux;
        vectoraux.push_back(p1);
        vectoraux.push_back(p2);
        
        renctangleCloserPoligons.push_back(vectoraux);
    }
    
    
}

bool Graph::isGoodPoint(int x, int y)
{
    int totalCicles= (int)this->renctangleCloserPoligons.size();
    bool isgood=true;
    for(int i = 0; i < totalCicles ;i++){
        vector<Point2D> ciclo=renctangleCloserPoligons[i];
        
        Point2D p1=ciclo[0];
        Point2D p2=ciclo[1];
        if(p1.getX() <=  x && p1.getY()<= y && p2.getX() >=x && p2.getY() >=y ){
            isgood=false;
            break;
        }
        
    }
    return isgood;
}

vector<vector<Point2D> > Graph::getEdgesCloserPoints()
{
    int totalcicles= (int)edgesCloser.size();
    vector<vector<Point2D>> out(totalcicles);
    
    for(int i=0;i<totalcicles;i++){
        vector<Point2D> vectorP;
        for( int k=0;k<edgesCloser[i].size();k++){
            std::map<Point2D,int>::iterator it;
            for (it = mapa.begin(); it != mapa.end(); ++it )
                if (it->second == edgesCloser[i][k]){
                    Point2D P = it->first;
                    vectorP.push_back(P);
                    break;
                }
            
        }
        // vectorP.push_back(vectorP[0]);
        out[i]=vectorP;
    }
    return out;
    
    
    
    
}

// Returns true if the graph contains a cycle, else false.
bool Graph::isCyclic()
{
    /*
     for (int u = 0; u < V; u++){
     cout<<endl<<"parents for u  "<<u<<"= "<<parents[u].size()<<endl;
     for( int j=0;j<parents[u].size();j++)
     cout<<parents[u][j]<<"   ";
     cout<<endl<<"camino "<<endl;
     //if(parents[u].size()>1)
     findway(u);
     
     
     }
     
     */
    
    
    return false;
}




double Point2D::getY() const
{
    return y;
}

void Point2D::setY(double value)
{
    y = value;
}


Point2D::Point2D()
{
    x=0;
    y=0;
}

Point2D::Point2D(double x_, double y_):x(x_), y(y_)
{
    
}

double Point2D::getX() const
{
    return x;
}

void Point2D::setX(double value)
{
    x = value;
}

