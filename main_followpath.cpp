#include <fstream>
#include <iostream>
#include "follow_path_steering_behaviour.hpp"
using namespace std;
int main(){
	int n=1500;
	//cin>>n;

	FollowPathSteeringBehaviour cosa;
	double posicioninicial[2]={20, 30 };
	double velocidadinicial[2]={0,5};
	
	double maxLV=5;
	double maxLA=1;

	ofstream salida;
	salida.open("dibuja_2/salida_follow_path.txt");
	cosa.inicializa_free2d(posicioninicial, velocidadinicial, maxLV, maxLA,2.5);

	double punto[2];

	punto[0]=10;
	punto[1]=10;
	cosa.addVertexToThePath(punto);
	
	punto[0]=500;
	punto[1]=500;
	cosa.addVertexToThePath(punto);

	

	punto[0]=400;
	punto[1]=490;
	cosa.addVertexToThePath(punto);
	
	punto[0]=20;
	punto[1]=20;
	cosa.addVertexToThePath(punto);

	punto[0]=0;
	punto[1]=250;
	cosa.addVertexToThePath(punto);


	punto[0]=500;
	punto[1]=250;
	cosa.addVertexToThePath(punto);

	punto[0]=10;
	punto[1]=30;
	cosa.addVertexToThePath(punto);


	
	salida<<n<<endl;
	double *aux;
	for(int i=0;i<n;i++){
		double *pos=cosa.getPosition();
		cosa.followPath();
		aux=cosa.getPosition();
		salida<<aux[0]<<" "<<aux[1]<<"        "<<cosa.getCurrentSegment()<<endl;

		//cout<<cosa.getCurrentSegment()<<endl;
	}
	salida.close();

	
}