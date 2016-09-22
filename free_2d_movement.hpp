#ifndef FREE_2D_MOVEMENT
#define FREE_2D_MOVEMENT

#include "rigid_body_locomotion.hpp"
#include <cmath>

class Free2DMovement: public RigidBodyLocomotion {
private:	
	void updateLinearVelocity(double acceleration[3]);
	void updatePosition();
public:
	Free2DMovement(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA);
	void setUpDirection(double direction[3]);
	void moveInDirection(double direction[3], double velocity);
	//funcion virtual
	void applyForce(double steer[3]);
};

#endif