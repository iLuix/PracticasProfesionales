#include <fstream>
#include <iostream>
#include "seek_steering_behaviour_with_arrival.hpp"
using namespace std;
int main(){
	SeekSteeringBehaviourWithArrival cosa;
	double posicioninicial[3]={50,50};
	double velocidadinicial[3]={10,0};
	double maxLV=10;
	double maxLA=0.6;

	double target[3]={100,100};
	double target1[3]={300,140};
	double target2[3]={500,500};
	double target3[3]={550,10};
	double target4[3]={600,600};
	double target5[3]={300,300};
	ofstream salida;
	salida.open("dibuja_2/salida_seekwitharrival.txt");
	cosa.inicializa_free2d(posicioninicial, velocidadinicial, maxLV, maxLA,30);
	double *aux;
	int n=720;
	salida<<n<<endl;
	for(int i=0;i<120;i++){
//		cout<< "targe\t"<<target[0]<<"\t"<<target[1]<<"\t"<<target[2]<<endl;
//		cout<< "posit\t"<<pos[0]<<"\t"<<pos[1]<<"\t"<<pos[2]<<endl;
		cosa.seek(target);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
		//cin.get();
	}

	for(int i=0;i<120;i++){
		cosa.seek(target1);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}

	for(int i=0;i<120;i++){
		cosa.seek(target2);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}
	
	for(int i=0;i<120;i++){
		cosa.seek(target3);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}
	
	for(int i=0;i<120;i++){
		cosa.seek(target4);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}
	for(int i=0;i<120;i++){
		cosa.seek(target5);
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<endl;
		/*aux=cosa.getVelocity();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";
		aux=cosa.getOrientation();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<" ";*/
	}


	salida.close();

	
}