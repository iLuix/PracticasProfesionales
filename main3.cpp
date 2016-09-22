#include <fstream>
#include <iostream>
#include "follow_path_steering_behaviour.hpp"
using namespace std;
int main(){
	FollowPathSteeringBehaviour cosa;
	double posicioninicial[3]={20, 30 ,0};
	double velocidadinicial[3]={0,0,5};
	double orientacioninicial[3]={0,1,0};
	double maxLV=5;
	double maxLA=1;

	ofstream salida;
	salida.open("dibuja_2/salida_follow_path.txt");
	cosa.inicializa_free2d(posicioninicial,  orientacioninicial, velocidadinicial, maxLV, maxLA,4);

	double punto[3];

	punto[0]=0;
	punto[1]=30;
	punto[2]=0;
	cosa.addVertexToThePath(punto);
	
	punto[0]=500;
	punto[1]=30;
	punto[2]=500;
	cosa.addVertexToThePath(punto);

	punto[0]=200;
	punto[1]=30;
	punto[2]=500;
	cosa.addVertexToThePath(punto);

	punto[0]=250;
	punto[1]=30;
	punto[2]=250;
	cosa.addVertexToThePath(punto);


	punto[0]=500;
	punto[1]=30;
	punto[2]=250;
	cosa.addVertexToThePath(punto);

	punto[0]=0;
	punto[1]=100;
	punto[2]=0;
	cosa.addVertexToThePath(punto);


	int n=500;
	salida<<n<<endl;
	double *aux;
	for(int i=0;i<n;i++){
		double *pos=cosa.getPosition();
		cosa.followPath();
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<" "<<aux[2]<<endl;
		//cout<<cosa.getCurrentSegment()<<endl;
	}
	salida.close();

	
}