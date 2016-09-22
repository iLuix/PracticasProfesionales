#include <fstream>
#include <iostream>
#include "seek_steering_behaviour_with_arrival.hpp"
using namespace std;
int main(){
	SeekSteeringBehaviourWithArrival cosa;
	double posicioninicial[3]={50, 30 ,50};
	double velocidadinicial[3]={10,30,0};
	double orientacioninicial[3]={0,1,0};
	double maxLV=10;
	double maxLA=0.6;

	double target[3]={100,50,100};
	double target1[3]={300,50,140};
	double target2[3]={500,50,500};
	double target3[3]={550,50,10};
	double target4[3]={600,50,600};
	double target5[3]={300,350,300};
	ofstream salida;
	salida.open("dibuja/salida2.txt");
	cosa.inicializa_free2d(posicioninicial,  orientacioninicial, velocidadinicial, maxLV, maxLA, 150);
	double *aux;
	int n=120;
	salida<<n<<" "<<target[0]<<" "<<target[1]<<" "<<target[2]<<endl;
	for(int i=0;i<n;i++){
//		cout<< "targe\t"<<target[0]<<"\t"<<target[1]<<"\t"<<target[2]<<endl;
		double *pos=cosa.getPosition();
//		cout<< "posit\t"<<pos[0]<<"\t"<<pos[1]<<"\t"<<pos[2]<<endl;
		cosa.seek(target);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
		//cin.get();
	}
	//return 0;
	n=200;
	salida<<n<<" "<<target1[0]<<" "<<target1[1]<<" "<<target1[2]<<endl;
	for(int i=0;i<n;i++){
		cosa.seek(target1);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}

	salida<<n<<" "<<target2[0]<<" "<<target2[1]<<" "<<target2[2]<<endl;
	for(int i=0;i<n;i++){
		cosa.seek(target2);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}
	
	salida<<n<<" "<<target3[0]<<" "<<target3[1]<<" "<<target3[2]<<endl;
	for(int i=0;i<n;i++){
		cosa.seek(target3);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}
	
	salida<<n<<" "<<target4[0]<<" "<<target4[1]<<" "<<target4[2]<<endl;
	for(int i=0;i<n;i++){
		cosa.seek(target4);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}
	double newDir2[3]={1,2,1};
	cosa.setUpDirection(newDir2);
	salida<<n<<" "<<target5[0]<<" "<<target5[1]<<" "<<target5[2]<<endl;
	for(int i=0;i<n;i++){
		cosa.seek(target5);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}


	salida.close();

	
}