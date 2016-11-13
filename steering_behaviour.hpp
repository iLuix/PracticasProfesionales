#ifndef STEERING_BEHAVIOUR
#define STEERING_BEHAVIOUR

//librerias para poder usar scene.predictCollisionPoint(...);
//#include "NavMesh2D.hpp"
#include "GCNavMesh2D/NavMeshScene.hpp"
#include "GCNavMesh2D/Polygon.hpp"
#include <iostream>
//#include "NavMesh2DTrajectory.hpp"


#include "free_2d_movement.hpp"
// #include "other_rigid_body_movement_models.hpp"

class SteeringBehaviour{
protected:
	//RigidBodyLocomotion *locomotion;
	NavMeshScene *scene;
	double avoidanceRadii;				//radio de evaci'on	
	SteeringBehaviour();
public:
	RigidBodyLocomotion *locomotion;
	double* getPosition(){return this->locomotion->getPosition();}
};
#endif