#include "rigid_body_locomotion.hpp"

void RigidBodyLocomotion::setMaxLinearVelocity(double max){
	this->maxLinearVelocity=max;
}
double RigidBodyLocomotion::getMaxLinearVelocity(){
	return this->maxLinearVelocity;
}
void RigidBodyLocomotion::setMaxAngularVelocity(double max){
	this->maxAngularAcceleration=max;
}
double RigidBodyLocomotion::getMaxAngularVelocity(){
	return this->maxAngularVelocity;
}
void RigidBodyLocomotion::setMaxLinearAcceleration(double max){
	this->maxLinearAcceleration=max;
}
double RigidBodyLocomotion::getMaxLinearAcceleration(){
	return this->maxLinearAcceleration;
}
void RigidBodyLocomotion::setMaxAngularAcceleration(double max){
	this->maxAngularAcceleration=max;
}
double RigidBodyLocomotion::getMaxAngularAcceleration(){
	return this->maxAngularAcceleration;
}
void RigidBodyLocomotion::setActive(bool activated){
	this->active=activated;
}
bool RigidBodyLocomotion::isActive(){
	return this->active;
}
//
/*
double* RigidBodyLocomotion::getPosition(){
	return this->position;
}

double* RigidBodyLocomotion::getVelocity(){
	return this->linearVelocity;
}*/