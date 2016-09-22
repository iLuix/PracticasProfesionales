#include "prowl_steering_behaviour.hpp"

void ProwlSteeringBehaviour::prowl(){
	double *position=this->locomotion->getPosition();
	double *velocity=this->locomotion->getVelocity();
	double maxLinearVelocity= this->locomotion->getMaxLinearVelocity();


	double nextPosition[3]={position[0]+velocity[0],position[1]+velocity[1],position[2]+velocity[2]};
	double angle=2.0*M_PI*(rand()/((double)(RAND_MAX)));
	
	double target[3]={-0.5+(rand()/((double)(RAND_MAX))),-0.5+(rand()/((double)(RAND_MAX))),-0.5+(rand()/((double)(RAND_MAX)))};
	
	target[0]+=nextPosition[0];
	target[1]+=nextPosition[1];
	target[2]+=nextPosition[2];
	this->seek(target);
} 	

