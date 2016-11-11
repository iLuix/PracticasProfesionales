#ifndef FREE_2D_MOVEMENT
#define FREE_2D_MOVEMENT

#include "rigid_body_locomotion.hpp"
#include <cmath>

class Free2DMovement: public RigidBodyLocomotion {
private:	
	void updateLinearVelocity(double acceleration[2]);
	void updatePosition();
public:
	Free2DMovement(double pos[2], double LV[2], double maxLV, double maxLA, double r);
	void setUpDirection(double direction[2]);
	void moveInDirection(double direction[2], double velocity);
	//funcion virtual
	void applyForce(double steer[2]);
};

#endif