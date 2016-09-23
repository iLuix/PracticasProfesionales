#include "two_wheels.hpp"


double sign(double a){ return a>0?1:-1; }
	
void TwoWheels::updateLinearVelocity(double acceleration[3]){
	if(!this->isActive()) return;

	double norm=sqrt(acceleration[0]*acceleration[0] +acceleration[1]*acceleration[1] +acceleration[2]*acceleration[2]);
	if(norm==0.0) return;

	this->linearVelocity[0] += this->maxLinearAcceleration*(acceleration[0]/norm);
	this->linearVelocity[1] = 0;  //+= this->maxLinearAcceleration*(acceleration[1]/norm);
	this->linearVelocity[2] += this->maxLinearAcceleration*(acceleration[2]/norm);

	//required angle in the new velocity
	double th = atan2(this->linearVelocity[2],this->linearVelocity[0]); //angle in (0,1,0)^(1,0,0) local reference frame
	double velocitytn = th - this->theta > this->theta+2*M_PI-th? -this->theta-2*M_PI+th:th - this->theta;//angulo mas cercano
	double accelerationt=velocitytn - this->angularVelocity[1];
	accelerationt=abs(accelerationt)>this->maxLinearVelocity?sign(accelerationt)*this->maxLinearVelocity:accelerationt;
	this->angularVelocity[1]+=accelerationt;
	this->angularVelocity[1] = abs(this->angularVelocity[1])>this->maxAngularVelocity?sign(this->angularVelocity[1])*this->maxAngularVelocity:this->angularVelocity[1];

	double v;
	if(cos(th)>sin(th))
		v=this->linearVelocity[0]/cos(th);
	else
		v=this->linearVelocity[2]/sin(th);
	v=v>this->maxLinearVelocity?sign(v)*this->maxLinearVelocity:v;
	theta+=velocityt;
	if(theta>2*M_PI){ theta-=2*M_PI; }
	return;

}
void TwoWheels::updatePosition(){
	if(!this->isActive()) return;
	this->position[0]+=this->linearVelocity[0];
	this->position[1]+=this->linearVelocity[1];
	this->position[2]+=this->linearVelocity[2];
}
void TwoWheels::setUpDirection(double direction[3]){
	this->orientation[0]=direction[0];
	this->orientation[1]=direction[1];
	this->orientation[2]=direction[2];
	//normalize orientation
	double norm=sqrt(this->orientation[0]*this->orientation[0] + this->orientation[1]*this->orientation[1] + this->orientation[2]*this->orientation[2]);
	this->orientation[0]/=norm;
	this->orientation[1]/=norm;
	this->orientation[2]/=norm;
}
void TwoWheels::moveInDirection(double direction[3], double velocity){ //forces the movement in any(!) direction with a certain velocity even if the agent is inactive

	double norm=sqrt(direction[0]*direction[0] + direction[1]*direction[1] + direction[2]*direction[2]);
	if(norm==0.0) return;

	this->position[0]+=velocity*direction[0]/norm;
	this->position[1]+=velocity*direction[1]/norm;
	this->position[2]+=velocity*direction[2]/norm;


}
void TwoWheels::applyForce(double steer[3]){
	this->updateLinearVelocity(steer);
	this->updatePosition();
}


TwoWheels::TwoWheels(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA){
	this->position[0]=pos[0];
	this->position[1]=pos[1];
	this->position[2]=pos[2];

	this->orientation[0]=ori[0];
	this->orientation[1]=ori[1];
	this->orientation[2]=ori[2];
	//normalize orientation
	double norm=sqrt(this->orientation[0]*this->orientation[0] + this->orientation[1]*this->orientation[1] + this->orientation[2]*this->orientation[2]);
	this->orientation[0]/=norm;
	this->orientation[1]/=norm;
	this->orientation[2]/=norm;


	this->linearVelocity[0]=LV[0];
	this->linearVelocity[1]=LV[1];
	this->linearVelocity[2]=LV[2];

	this->maxLinearVelocity=maxLV;

	this->maxLinearAcceleration=maxLA;
	this->active=true;
}