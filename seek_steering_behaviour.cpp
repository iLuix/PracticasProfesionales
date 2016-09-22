#include "seek_steering_behaviour.hpp"

void SeekSteeringBehaviour::seek(double target[3]){
	double *position=this->locomotion->getPosition();
	double *velocity=this->locomotion->getVelocity();

	double desired[3]={target[0]-position[0], target[1]-position[1], target[2]-position[2]};

	if(desired[0]!=0.0 && desired[0]!=0.0 && desired[0]!=0.0){
		double norm=sqrt(desired[0]*desired[0]+desired[1]*desired[1]+desired[2]*desired[2]);
		double maxLinearAcceleration=this->locomotion->getMaxLinearAcceleration();
		double maxLinearVelocity= this->locomotion->getMaxLinearVelocity();
		desired[0]*= maxLinearVelocity/norm;
		desired[1]*= maxLinearVelocity/norm;
		desired[2]*= maxLinearVelocity/norm;
		double steer[3]={desired[0]-velocity[0],desired[1]-velocity[1],desired[2]-velocity[2]};
		norm=sqrt(steer[0]*steer[0]+steer[1]*steer[1]+steer[2]*steer[2]);
		if(norm!=0.0){
			steer[0]*=maxLinearAcceleration/norm;
			steer[1]*=maxLinearAcceleration/norm;
			steer[2]*=maxLinearAcceleration/norm;
		}
		this->locomotion->applyForce(steer);
	}

	else{
		return;
	}
} 	

