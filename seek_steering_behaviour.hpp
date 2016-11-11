#ifndef SEEK_STEERING_BEHAVIOUR
#define SEEK_STEERING_BEHAVIOUR

#include "steering_behaviour.hpp"
#include <cmath>
class SeekSteeringBehaviour : public SteeringBehaviour{
private:
	
public:
	void seek(double target[2]);
	double *getSteeringSeek(double target[2]);
	//functions for demos
	void inicializa_free2d(double pos[2], double LV[2], double maxLV, double maxLA, NavMeshScene *scn=NULL, double r=0, double ar=0){ //inicializa sin colisiones, r es dependiente de scn, no importa el radio si no hay colisiones
		this->locomotion = new Free2DMovement(pos,LV,maxLV,maxLA,r);
		this->scene=scn;
		this->avoidanceRadii=ar;
	}
	//
	
	/*double *getPosition(){ return locomotion->getPosition(); }
	double *getOrientation(){ return locomotion->getOrientation(); }
	double *getVelocity(){ return locomotion->getVelocity(); }*/

//	void setUpDirection(double *orientation){ locomotion->setUpDirection(orientation); }

};

#endif