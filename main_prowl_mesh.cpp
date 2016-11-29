
#include <fstream>
#include <iostream>
#include "prowl_steering_behaviour.hpp"


#include <iostream>
#include "GCNavMesh2D/NavMesh2D.hpp"
#include "GCNavMesh2D/NavMeshScene.hpp"
#include "GCNavMesh2D/Polygon.hpp"
#include "GCNavMesh2D/NavMesh2DTrajectory.hpp"
#include <set>

#define BUILD_NAVMESH

int main(int argc, const char * argv[]) {
    

#ifdef BUILD_NAVMESH
    //CONSTRUCCION DEL ESCENARIO ---------------------
    
    //Definir Polígonos
    Polygon floor;
    floor.addVertex(0,0);
    floor.addVertex(1500,0);
    floor.addVertex(1500,1000);
    floor.addVertex(0,1000);
    
    Polygon o1;
    o1.addVertex(0, 475);
    o1.addVertex(500, 475);
    o1.addVertex(500, 500);
    o1.addVertex(0, 500);
    
    Polygon o2;
    o2.addVertex(750, 0);
    o2.addVertex(750, 475);
    o2.addVertex(550,475);
    o2.addVertex(550, 500);
    o2.addVertex(775, 500);
    o2.addVertex(775, 0);
    
    Polygon o3;
    o3.addVertex(825, 475);
    o3.addVertex(1500, 475);
    o3.addVertex(1500, 500);
    o3.addVertex(825, 500);
    
    Polygon o4;
    o4.addVertex(600, 1000);
    o4.addVertex(600, 600);
    o4.addVertex(200, 600);
    o4.addVertex(200, 575);
    o4.addVertex(1300, 575);
    o4.addVertex(1300, 600);
    o4.addVertex(625, 600);
    o4.addVertex(625, 1000);
    
    //Mesa
    Polygon o5;
    o5.addVertex(150, 150);
    o5.addVertex(600, 150);
    o5.addVertex(600, 300);
    o5.addVertex(150, 300);
    
    //Sillas
    Polygon o6;
    o6.addVertex(150,100);
    o6.addVertex(200,100);
    o6.addVertex(200,125);
    o6.addVertex(150,125);
    
    Polygon o7;
    o7.addVertex(250,100);
    o7.addVertex(300,100);
    o7.addVertex(300,125);
    o7.addVertex(250,125);
    
    Polygon o8;
    o8.addVertex(350,100);
    o8.addVertex(400,100);
    o8.addVertex(400,125);
    o8.addVertex(350,125);
    
    Polygon o9;
    o9.addVertex(450,100);
    o9.addVertex(500,100);
    o9.addVertex(500,125);
    o9.addVertex(450,125);
    
    Polygon o10;
    o10.addVertex(550,100);
    o10.addVertex(600,100);
    o10.addVertex(600,125);
    o10.addVertex(550,125);
    
    Polygon o11;
    o11.addVertex(150,325);
    o11.addVertex(200,325);
    o11.addVertex(200,350);
    o11.addVertex(150,350);
    
    Polygon o12;
    o12.addVertex(250,325);
    o12.addVertex(300,325);
    o12.addVertex(300,350);
    o12.addVertex(250,350);
    
    Polygon o13;
    o13.addVertex(350,325);
    o13.addVertex(400,325);
    o13.addVertex(400,350);
    o13.addVertex(350,350);
    
    Polygon o14;
    o14.addVertex(450,325);
    o14.addVertex(500,325);
    o14.addVertex(500,350);
    o14.addVertex(450,350);
    
    Polygon o15;
    o15.addVertex(550,325);
    o15.addVertex(600,325);
    o15.addVertex(600,350);
    o15.addVertex(550,350);
    
    //Estantes
    
    Polygon o16;
    o16.addVertex(850,75);
    o16.addVertex(925,75);
    o16.addVertex(925,275);
    o16.addVertex(850,275);
    
    Polygon o18;
    o18.addVertex(975,75);
    o18.addVertex(1050,75);
    o18.addVertex(1050,275);
    o18.addVertex(975,275);
    
    Polygon o17;
    o17.addVertex(1100,75);
    o17.addVertex(1175,75);
    o17.addVertex(1175,275);
    o17.addVertex(1100,275);
    
    //Mesa2
    Polygon o19;
    o19.addVertex(950,350);
    o19.addVertex(1200,350);
    o19.addVertex(1200,425);
    o19.addVertex(950,425);
    
    //Escritorio
    Polygon o20;
    o20.addVertex(1300,475);
    o20.addVertex(1500,475);
    o20.addVertex(1500,200);
    o20.addVertex(1450,200);
    o20.addVertex(1450,350);
    o20.addVertex(1375,425);
    o20.addVertex(1300,425);

    //Librero
    Polygon o21;
    o21.addVertex(550, 700);
    o21.addVertex(550, 900);
    o21.addVertex(600, 900);
    o21.addVertex(600, 700);
    
    //Sillon 1
    Polygon o22;
    o22.addVertex(300, 950);
    o22.addVertex(500, 950);
    o22.addVertex(500, 900);
    o22.addVertex(300, 900);
    
    //Sillon 2
    Polygon o23;
    o23.addVertex(250, 875);
    o23.addVertex(325, 700);
    o23.addVertex(265, 675);
    o23.addVertex(190, 850);
    
    //Sillon 3
    Polygon o24;
    o24.addVertex(400, 675);
    o24.addVertex(500, 675);
    o24.addVertex(500, 725);
    o24.addVertex(400, 725);
    
    //Mesas
    Polygon o25;
    o25.addVertex(700, 650);
    o25.addVertex(750, 650);
    o25.addVertex(750, 900);
    o25.addVertex(700, 900);
    
    Polygon o26;
    o26.addVertex(800, 650);
    o26.addVertex(850, 650);
    o26.addVertex(850, 900);
    o26.addVertex(800, 900);
    
    Polygon o27;
    o27.addVertex(900, 650);
    o27.addVertex(950, 650);
    o27.addVertex(950, 900);
    o27.addVertex(900, 900);
    
    Polygon o28;
    o28.addVertex(1000, 650);
    o28.addVertex(1050, 650);
    o28.addVertex(1050, 900);
    o28.addVertex(1000, 900);
    
    Polygon o29;
    o29.addVertex(1100, 650);
    o29.addVertex(1150, 650);
    o29.addVertex(1150, 900);
    o29.addVertex(1100, 900);
    
    Polygon o30;
    o30.addVertex(1200, 650);
    o30.addVertex(1250, 650);
    o30.addVertex(1250, 900);
    o30.addVertex(1200, 900);
    
    Polygon o31;
    o31.addVertex(1475, 650);
    o31.addVertex(1500, 650);
    o31.addVertex(1500, 900);
    o31.addVertex(1475, 900);
    
    //CONSTRUCCION DE LA ESCENA
    NavMeshScene scene(1500,1000,1.0);
    scene.addPolygonFloor(floor);
    scene.addPolygonObstacle(o1);
    scene.addPolygonObstacle(o2);
    scene.addPolygonObstacle(o3);
    scene.addPolygonObstacle(o4);
    scene.addPolygonObstacle(o5);
    scene.addPolygonObstacle(o6);
    scene.addPolygonObstacle(o7);
    scene.addPolygonObstacle(o8);
    scene.addPolygonObstacle(o9);
    scene.addPolygonObstacle(o10);
    scene.addPolygonObstacle(o11);
    scene.addPolygonObstacle(o12);
    scene.addPolygonObstacle(o13);
    scene.addPolygonObstacle(o14);
    scene.addPolygonObstacle(o15);
    scene.addPolygonObstacle(o16);
    scene.addPolygonObstacle(o17);
    scene.addPolygonObstacle(o18);
    scene.addPolygonObstacle(o19);
    scene.addPolygonObstacle(o20);
    scene.addPolygonObstacle(o21);
    scene.addPolygonObstacle(o22);
    scene.addPolygonObstacle(o23);
    scene.addPolygonObstacle(o24);
    scene.addPolygonObstacle(o25);
    scene.addPolygonObstacle(o26);
    scene.addPolygonObstacle(o27);
    scene.addPolygonObstacle(o28);
    scene.addPolygonObstacle(o29);
    scene.addPolygonObstacle(o30);
    scene.addPolygonObstacle(o31);
    scene.drawScene(cv::Vec3b(255,255,255), cv::Vec3b(255,0,0));
   
    
    //posicion inicial del agente
    double x1 = 671;
    double y1 = 240;
    srand(time(NULL));
    do{
    	x1=rand()%1500;
    	y1=rand()%1000;
    }while(scene.isPointFreeInScene(x1,y1)==0);
    
    
//numero de pasos de merodeo
    int n=5000;
    //cin>>n;
//la cosa que merodea
    ProwlSteeringBehaviour cosa;
    double posicioninicial[2]={x1, y1 };
    double velocidadinicial[2]={0,0};
//velocidad y aceleracion maxima
    double maxLV=2;
    double maxLA=1.8;
//archivo de salida del merodeo
    std::ofstream salida;
    salida.open("dibuja_2/salida_prowl_4.txt");
    cosa.inicializa_free2d(posicioninicial, velocidadinicial, maxLV, maxLA,&scene,10.0,maxLV*2);

    salida<<n<<std::endl;
    double *aux;
    for(int i=0;i<n;i++){
        double *pos=cosa.getPosition();
        double  *st=cosa.getSteeringProwl();
        if(st){
        	cosa.locomotion->applyForce(st);
        	aux=cosa.getPosition();
        salida<<aux[0]<<" "<<aux[1]<<" "<<10*st[0]<<" "<<10*st[1]<<std::endl;
        	delete[] st;
        }
        else{
        	aux=cosa.getPosition();
        	salida<<aux[0]<<" "<<aux[1]<<" 0 0"<<std::endl;
        }

        
        //std::cout<<i<<std::endl;
        //cout<<cosa.getCurrentSegment()<<endl;
    }
    salida.close();

    salida.open("dibuja_colisiones/salida_colisiones.txt");
    salida<<n<<std::endl;
    for(int i=0;i<10000;i++){
    	x1=rand()%1500;
    	y1=rand()%1000;
    	
    	salida<<x1<<" "<<y1<<" "<<scene.isPointFreeInScene(x1,y1)<<std::endl;


    }
	salida.close();

	salida.open("dibuja_futuras_colisiones/salida_colisiones.txt");
    salida<<50<<std::endl;
    for(int i=0;i<50;i++){
    	x1=rand()%1500;
    	y1=rand()%1000;
    	if(scene.isPointFreeInScene(x1,y1)==0)
    		salida<<x1<<" "<<y1<<" "<<0<<std::endl;
    	else{
    		double theta=(double(rand())/RAND_MAX)*2*M_PI;
    		double r=100+rand()%50;
    		double xc,yc;
    		PrimitiveSegment *col =scene.predictCollisionPoint(x1,y1,x1+cos(theta)*r,y1+sin(theta)*r,xc,yc);
    		if(!col){
    			xc=x1+cos(theta)*r;
    			yc=y1+sin(theta)*r;
    		}
    		salida<<x1<<" "<<y1<<" "<<1<<" "<<xc<<" "<<yc<<std::endl;
    		
    	}



    }
	salida.close();

    /*
    double x2 = 1707;
    double y2 = 431;
    double xint,yint;
    
    for(int i= 0; i<100; i++){
        
        double x2 = x1 + 500*((double) rand() / (RAND_MAX)) - 250;
        double y2 = y1 + 500*((double) rand() / (RAND_MAX)) - 250;
        PrimitiveSegment *p = scene.predictCollisionPoint(x1, y1, x2, y2, xint, yint);
        
    }*/
    
    //la siguiente parte del codigo no es usado para el prowl...
#else
    //CONSTRUCCION DE LA MALLA DE NAVEGACIÓN --------------- 
    NavMesh2D navmesh(scene);
    navmesh.build();
    navmesh.drawNavMesh();
    navmesh.saveScene("scene.png");
    navmesh.saveNavigationMesh("navmesh3.txt");
    
    
    NavMesh2DTrajectoryBuilder  nvtb("navmesh3.txt");
    
    nvtb.timesearch = 0;
    nvtb.countsearchs = 0;
    nvtb.maxtime = -1e100;
    
    int iterationsNarrowing = 50;
    double distanceBetweenPoints = 10;
    double distanceToObstacles = 10;
    
    for(int i= 0; i<100; i++){
        
        double x1 = 1500*((double) rand() / (RAND_MAX));
        double y1 = 1000*((double) rand() / (RAND_MAX));
        double x2 = 1500*((double) rand() / (RAND_MAX));
        double y2 = 1000*((double) rand() / (RAND_MAX));

        std::cout<<std::endl<<x1<<" "<<y1<<" "<<x2<<" "<<y2;
        
        bool finished = false;
        
        NavMesh2DTrajectory trajectory = NavMesh2DTrajectory();
        bool found = nvtb.searchTrajectory(x1, y1, x2, y2, trajectory, distanceBetweenPoints, iterationsNarrowing, distanceToObstacles, finished);
        
        if(found){
            std::stringstream ss;
            ss<<"trajectory_"<<i<<"_"<<iterationsNarrowing<<"_"<<distanceToObstacles<<".png";
            cv::Mat img;
            nvtb.drawTrajectory(trajectory, img);
            cv::imwrite(ss.str(), img);
        }
        
        
    }
    
    std::cout<<std::endl<<nvtb.countsearchs<<" searchs, "<<1000*nvtb.timesearch<<"ms, "<<1000*nvtb.timesearch/(double)nvtb.countsearchs<<"ms, maxtime = "<<1000*nvtb.maxtime<<"ms";
    
#endif
    
    std::cout << std::endl << std::endl << "END OF PROGRAM!!!\n";
    return 0;
}
