#ifndef SEEK_STEERING_BEHAVIOUR
#define SEEK_STEERING_BEHAVIOUR

#include "steering_behaviour.hpp"
#include <cmath>
class SeekSteeringBehaviour : public SteeringBehaviour{
public:
	void seek(double target[3]);
	//functions for demos
	


	//
	void inicializa_free2d(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA){
		this->locomotion = new Free2DMovement(pos,ori,LV,maxLV,maxLA);
	}
	
	double *getPosition(){ return locomotion->getPosition(); }
	double *getOrientation(){ return locomotion->getOrientation(); }
	double *getVelocity(){ return locomotion->getVelocity(); }
	void setUpDirection(double *orientation){ locomotion->setUpDirection(orientation); }

};

#endif