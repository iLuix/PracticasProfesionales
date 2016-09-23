#ifndef TWO_WHEELS
#define TWO_WHEELS

#include "rigid_body_locomotion.hpp"
#include <cmath>

class TwoWheels: public RigidBodyLocomotion {
private:	
	void updateLinearVelocity(double acceleration[3]);
	void updatePosition();
public:
	TwoWheels(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA);
	void setUpDirection(double direction[3]);
	void moveInDirection(double direction[3], double velocity);
	//funcion virtual
	void applyForce(double steer[3]);
};

#endif