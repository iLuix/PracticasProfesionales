#ifndef FOLLOW_PATH_STEERING_BEAHAVIOUR
#define FOLLOW_PATH_STEERING_BEAHAVIOUR

#include "seek_steering_behaviour.hpp"
#include <cmath>
#include <vector>

class FollowPathSteeringBehaviour : public SeekSteeringBehaviour{
private:
	//radii of the path
	double pathRadii;
	//every three doubles is a point in 3D that represents a vertex in the path
	std::vector <double> pointsOfPath;
	//index for the number of segment ([3*actualSegment],[3*actualSegment+1],[3*actualSegment+2]) -> ([3*actualSegment+3],[3*actualSegment+4],[3*actualSegment+5])
	int currentSegment;


public:
	
	void followPath();
	double *getSteeringFollowPath();
	void addVertexToThePath(double vertex[3]);

	

	//functions for demos

	//inicializa
	void inicializa_free2d(double pos[2], double LV[2], double maxLV, double maxLA, double pr, NavMeshScene *scn=NULL, double r=0, double ar=0){
		this->locomotion = new Free2DMovement(pos,LV,maxLV,maxLA,r);
		this->currentSegment=0;
		this->pathRadii=pr;
		this->scene=scn;
		this->avoidanceRadii=ar;
	}
	int getCurrentSegment(){ return this->currentSegment; }

	


};

#endif