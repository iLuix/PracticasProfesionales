#ifndef PROWL_STEERING_BEHAVIOUR
#define PROWL_STEERING_BEHAVIOUR

#include "seek_steering_behaviour.hpp"
#include <cmath>
#include <cstdlib>
#include <ctime>

class ProwlSteeringBehaviour: public SeekSteeringBehaviour{
public:
	void prowl();
	double *getSteeringProwl();//no es tan 'util, pues cada vez que se llama se genera un target nuevo...
	//functions for demos
	void inicializa_free2d(double pos[2],  double LV[2], double maxLV, double maxLA, NavMeshScene *scn, double r, double ar){
		this->locomotion = new Free2DMovement(pos,LV,maxLV,maxLA,r);
		this->scene=scn;
		this->avoidanceRadii=ar;
		srand(time(0));
	}
	


	//

};

#endif