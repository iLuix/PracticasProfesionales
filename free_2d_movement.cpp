#include "free_2d_movement.hpp"
	
void Free2DMovement::updateLinearVelocity(double acceleration[2]){
	if(!this->isActive()) return;
	//normalizacion de la aceleracion aplicada
	double norm=sqrt(acceleration[0]*acceleration[0] +acceleration[1]*acceleration[1]);
	
	//si la aceleraci'on es nula, no se hae algo
	if(norm==0.0) return;

	this->linearVelocity[0] += this->maxLinearAcceleration*(acceleration[0]/norm);
	this->linearVelocity[1] += this->maxLinearAcceleration*(acceleration[1]/norm);

	//componente de aceleraci'on respecto al plano definido por la orientaci'on(d'onde es arriba) del cuerpo
	//no relevante para el caso 2D si se define correctamente la orientacion (0,1,0)
	//double alpha=( this->linearVelocity[0]*this->orientation[0] + this->linearVelocity[1]*this->orientation[1] + this->linearVelocity[2]*this->orientation[2] );
	//this->linearVelocity[0]-=alpha*orientation[0];
	//this->linearVelocity[1]-=alpha*orientation[1];
	//this->linearVelocity[2]-=alpha*orientation[2];

	//aplicacion de la aceleracion al cuerpo
	norm=sqrt(this->linearVelocity[0]*this->linearVelocity[0] + this->linearVelocity[1]*this->linearVelocity[1]);
	if(norm!=0 && norm>maxLinearVelocity){
		this->linearVelocity[0]*=this->maxLinearVelocity/norm;
		this->linearVelocity[1]*=this->maxLinearVelocity/norm;
	}
	return;
}
void Free2DMovement::updatePosition(){
	if(!this->isActive()) return;
	this->position[0]+=this->linearVelocity[0];
	this->position[1]+=this->linearVelocity[1];
}
/*void Free2DMovement::setUpDirection(double direction[2]){//no usado en 2D
	this->orientation[0]=direction[0];
	this->orientation[1]=direction[1];
	this->orientation[2]=direction[2];
	//normalize orientation
	double norm=sqrt(this->orientation[0]*this->orientation[0] + this->orientation[1]*this->orientation[1] + this->orientation[2]*this->orientation[2]);
	this->orientation[0]/=norm;
	this->orientation[1]/=norm;
	this->orientation[2]/=norm;
}*/
void Free2DMovement::moveInDirection(double direction[2], double velocity){ //forces the movement in any(!) direction with a certain velocity even if the agent is inactive

	double norm=sqrt(direction[0]*direction[0] + direction[1]*direction[1]);
	if(norm==0.0) return;
	this->position[0]+=velocity*direction[0]/norm;
	this->position[1]+=velocity*direction[1]/norm;

}
void Free2DMovement::applyForce(double steer[2]){
	this->updateLinearVelocity(steer);
	this->updatePosition();
}


Free2DMovement::Free2DMovement(double pos[2], double LV[2], double maxLV, double maxLA, double r){
	this->position[0]=pos[0];
	this->position[1]=pos[1];
/*
	this->orientation[0]=ori[0];//no usado en 2D
	this->orientation[1]=ori[1];
	this->orientation[2]=ori[2];
	//normalize orientation
	double norm=sqrt(this->orientation[0]*this->orientation[0] + this->orientation[1]*this->orientation[1] + this->orientation[2]*this->orientation[2]);
	this->orientation[0]/=norm;
	this->orientation[1]/=norm;
	this->orientation[2]/=norm;*/


	this->linearVelocity[0]=LV[0];
	this->linearVelocity[1]=LV[1];

	this->maxLinearVelocity=maxLV;

	this->maxLinearAcceleration=maxLA;
	this->radii=r;
	this->active=true;
}