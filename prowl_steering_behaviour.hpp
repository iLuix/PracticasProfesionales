#ifndef PROWL_STEERING_BEHAVIOUR
#define PROWL_STEERING_BEHAVIOUR

#include "seek_steering_behaviour.hpp"
#include <cmath>
#include <cstdlib>
#include <ctime>

class ProwlSteeringBehaviour: public SeekSteeringBehaviour{
public:
	void prowl();
	//functions for demos
	void inicializa_free2d(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA){
		this->locomotion = new Free2DMovement(pos,ori,LV,maxLV,maxLA);
		srand(time(0));
	}
	


	//

};

#endif