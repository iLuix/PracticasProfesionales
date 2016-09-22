#ifndef SEEK_STEERING_BEHAVIOUR_WITH_ARRIVAL
#define SEEK_STEERING_BEHAVIOUR_WITH_ARRIVAL

#include "steering_behaviour.hpp"
#include <cmath>
class SeekSteeringBehaviourWithArrival : public SteeringBehaviour{
private:
	double arrivalRadii;
public:
	void seek(double target[3]);
	//functions for demos
	
	//
	void inicializa_free2d(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA, double radii){
		this->locomotion = new Free2DMovement(pos,ori,LV,maxLV,maxLA);
		arrivalRadii=radii;
	}
	
	double *getPosition(){ return locomotion->getPosition(); }
	double *getOrientation(){ return locomotion->getOrientation(); }
	double *getVelocity(){ return locomotion->getVelocity(); }
	void setUpDirection(double *orientation){ locomotion->setUpDirection(orientation); }

};

#endif