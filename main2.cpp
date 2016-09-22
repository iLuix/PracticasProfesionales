#include <fstream>
#include <iostream>
#include "prowl_steering_behaviour.hpp"
using namespace std;
int main(){
	ProwlSteeringBehaviour cosa;
	double posicioninicial[3]={50, 30 ,50};
	double velocidadinicial[3]={10,30,0};
	double orientacioninicial[3]={0,1,0};
	double maxLV=5;
	double maxLA=1;

	ofstream salida;
	salida.open("dibuja_2/salida2.txt");
	cosa.inicializa_free2d(posicioninicial,  orientacioninicial, velocidadinicial, maxLV, maxLA);
	
	int n=500;
	salida<<n<<endl;
	double *aux;
	for(int i=0;i<n;i++){
		double *pos=cosa.getPosition();
		cosa.prowl();
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
	}
	salida.close();

	
}