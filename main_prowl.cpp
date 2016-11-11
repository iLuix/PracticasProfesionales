#include <fstream>
#include <iostream>
#include "prowl_steering_behaviour.hpp"
using namespace std;
int main(){
	int n=500;
	//cin>>n;

	ProwlSteeringBehaviour cosa;
	double posicioninicial[2]={20, 30 };
	double velocidadinicial[2]={0,5};
	
	double maxLV=5;
	double maxLA=1;

	ofstream salida;
	salida.open("dibuja_2/salida_prowl.txt");
	cosa.inicializa_free2d(posicioninicial, velocidadinicial, maxLV, maxLA);

	salida<<n<<endl;
	double *aux;
	for(int i=0;i<n;i++){
		double *pos=cosa.getPosition();
		cosa.prowl();
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<"        "<<endl;

		//cout<<cosa.getCurrentSegment()<<endl;
	}
	salida.close();

	
}