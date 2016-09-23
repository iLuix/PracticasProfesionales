#ifndef RIGID_BODY_LOCOMOTION
#define RIGID_BODY_LOCOMOTION

class RigidBodyLocomotion{
protected:
	double position[3];					//position of the body in general coordinates
	double orientation[3];				//where is up for the local coordinates of the body
	double linearVelocity[3];			//
	double theta;						
	double angularVelocity[3];
	double maxLinearVelocity;
	double maxAngularVelocity;
	double maxLinearAcceleration;
	double maxAngularAcceleration;

	bool active;
public:
	void setMaxLinearVelocity(double max);
	double getMaxLinearVelocity();
	void setMaxAngularVelocity(double max);
	double getMaxAngularVelocity();
	void setMaxLinearAcceleration(double max);
	double getMaxLinearAcceleration();
	void setMaxAngularAcceleration(double max);
	double getMaxAngularAcceleration();
	void setActive(bool activated);
	bool isActive();

	double *getPosition(){ return position; }
	double *getVelocity(){ return linearVelocity; }
	double *getOrientation(){ return orientation; }
	double *getAngularVelocity(){ return angularVelocity; }

	virtual void setUpDirection(double [3])=0;
	virtual void applyForce(double steer[3]) = 0;

};

#endif