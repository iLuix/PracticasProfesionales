#ifndef RIGID_BODY_LOCOMOTION
#define RIGID_BODY_LOCOMOTION

class RigidBodyLocomotion{
protected:
	double radii;						//radio del cuerpo
	double position[2];					//Posici'on del cuerpo en coordenadas globales
	double orientation[2];				//d'onde es arriba respecto al cuerpo, no usado en ambientes 2d
//?
	double theta;						//orientaci'on del cuerpo en 2D usando orientaci'on global
	double linearVelocity[2];			//velocidad lineal del cuerpo
	double angularVelocity;				//velocidad angular del cuerpo
	double maxLinearVelocity;			//m'axima velocidad lineal del cuerpo
	double maxAngularVelocity;			//m'axima velocidad angular del cuerpo
	double maxLinearAcceleration;		//m'axima aceleraci'on lineal del cuerpo
	double maxAngularAcceleration;		//m'axima aceleraci'on angular del cuerpo
	bool active;						//booleano para saber si el cuerpo est'a activo

public:
	//set & get de las variables anteriores
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
	double getRadii(){return radii; }
	double *getPosition(){ return position; }
	double *getVelocity(){ return linearVelocity; }
	double *getOrientation(){ return orientation; }
	double getAngularVelocity(){ return angularVelocity; }

	//funciones virtuales 
	//virtual void setUpDirection(double [3]) = 0;	//No usada para versi'on 2D
	virtual void applyForce(double steer[2]) = 0;
	virtual void updatePosition() = 0;

	//inicialzador
	/*RigidBodyLocomotion(double _position[3], double _orientation[3], 
	double _linearVelocity[3], double _orientation[3], double _linearVelocity,
	double _angularVelocity, double _maxLinearVelocity, double _maxAngularVelocity,
	double _maxLinearAcceleration, double _maxAngularAcceleration, bool _active);
	*/
};

#endif