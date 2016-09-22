#ifndef FOLLOW_PATH_STEERING_BEAHAVIOUR
#define FOLLOW_PATH_STEERING_BEAHAVIOUR

#include "seek_steering_behaviour.hpp"
#include <cmath>
#include <vector>

class FollowPathSteeringBehaviour : public SeekSteeringBehaviour{
private:
	//radii of the path
	double radii;
	//every three doubles is a point in 3D that represents a vertex in the path
	std::vector <double> pointsOfPath;
	//index for the number of segment ([3*actualSegment],[3*actualSegment+1],[3*actualSegment+2]) -> ([3*actualSegment+3],[3*actualSegment+4],[3*actualSegment+5])
	int currentSegment;


public:
	
	void followPath();
	void addVertexToThePath(double vertex[3]);

	

	//functions for demos

	//inicializa
	void inicializa_free2d(double pos[3], double ori[3], double LV[3], double maxLV, double maxLA, double r){
		this->locomotion = new Free2DMovement(pos,ori,LV,maxLV,maxLA);
		this->currentSegment=0;
		this->radii=r;
	}
	int getCurrentSegment(){ return this->currentSegment; }

	


};

#endif