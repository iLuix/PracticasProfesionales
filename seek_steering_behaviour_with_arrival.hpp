#ifndef SEEK_STEERING_BEHAVIOUR_WITH_ARRIVAL
#define SEEK_STEERING_BEHAVIOUR_WITH_ARRIVAL

#include "steering_behaviour.hpp"
#include <cmath>
class SeekSteeringBehaviourWithArrival : public SteeringBehaviour{
private:
	double arrivalRadii;	//radio para empezar a decrementar la velocidad

public:
	void seek(double target[3]);
	double *getSteeringSeekWithArrival(double target[2]);
	//functions for demos
	
	//
	void inicializa_free2d(double pos[2], double LV[2], double maxLV, double maxLA, double ar, NavMeshScene *scn=NULL, double r=0, double ari=0){
		this->locomotion = new Free2DMovement(pos,LV,maxLV,maxLA,r);
		this->arrivalRadii=ar;
		this->avoidanceRadii=ari;
		scene=scn;
	}
	
	double *getPosition(){ return locomotion->getPosition(); }
	double *getOrientation(){ return locomotion->getOrientation(); }
	double *getVelocity(){ return locomotion->getVelocity(); }
	//void setUpDirection(double *orientation){ locomotion->setUpDirection(orientation); }

};

#endif