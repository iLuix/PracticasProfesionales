#include "follow_path_steering_behaviour.hpp"


#include <iostream> 
using namespace std;


void FollowPathSteeringBehaviour::followPath(){
	int numberOfSegments=int(this->pointsOfPath.size()/3)-1;
	double *position=this->locomotion->getPosition();
	double *velocity=this->getVelocity();
	double maxLinearVelocity= this->locomotion->getMaxLinearVelocity();




	if(numberOfSegments <= this->currentSegment){ return; }
	double A[3] = { pointsOfPath[3*currentSegment]  ,pointsOfPath[3*currentSegment+1],pointsOfPath[3*currentSegment+2]};
	double B[3] = { pointsOfPath[3*currentSegment+3],pointsOfPath[3*currentSegment+4],pointsOfPath[3*currentSegment+5]};
	double C[3] = { position[0]+velocity[0], position[1]+velocity[1], position[2]+velocity[2] }; //future position
	double distanceAB = sqrt ( (B[0]-A[0])*(B[0]-A[0]) + (B[1]-A[1])*(B[1]-A[1]) + (B[2]-A[2])*(B[2]-A[2]) ) ;
	
	double t= ( (B[0]-A[0])*(C[0]-A[0])+(B[1]-A[1])*(C[1]-A[1])+(B[2]-A[2])*(C[2]-A[2]) )/(distanceAB*distanceAB);



	double D[3] = { A[0] + (B[0]-A[0])*t - C[0], A[1] + (B[1]-A[1])*t - C[1], A[2] + (B[2]-A[2])*t - C[2] };
	double normD= sqrt(D[0]*D[0] + D[1]*D[1] + D[2]*D[2]);


	if(t + maxLinearVelocity/distanceAB > 1.0){
		this->currentSegment++;
		if(this->currentSegment < numberOfSegments ) 
		this->followPath();
		return;
	}
	if(this->currentSegment + 1 < numberOfSegments ){ //checks if the next segment is close than the previus and changes A,B,C,D,normD,normAB,this->currentSegment if is neccesary
		double B2[3] = { pointsOfPath[3*currentSegment+6],pointsOfPath[3*currentSegment+7],pointsOfPath[3*currentSegment+8]};	
		double distanceBB2 = sqrt ( (B2[0]-B[0])*(B2[0]-B[0]) + (B2[1]-B[1])*(B2[1]-B[1]) + (B2[2]-B[2])*(B2[2]-B[2]) ) ;
		double t2=( (B2[0]*C[0] - B2[0]*B[0] -C[0]*B[0]) + (B2[1]*C[1] - B2[1]*B[1] -C[1]*B[1]) + (B2[2]*C[2] - B2[2]*B[2] -C[2]*B[2]) ) / ( distanceBB2*distanceBB2 ) ;
		double D2[3] = { B[0] + (B2[0]-B[0])*t2 - C[0], B[1] + (B2[1]-B[1])*t2 - C[1], B[2] + (B2[2]-B[2])*t2 - C[2] };
		double normD2= sqrt(D2[0]*D2[0] + D2[1]*D2[1] + D2[2]*D2[2]);
		if(normD2<normD){
			A[0]=B[0];
			A[1]=B[1];
			A[2]=B[2];
			B[0]=B2[0];
			B[1]=B2[1];
			B[2]=B2[2];
			D[0]=D2[0];
			D[1]=D2[1];
			D[2]=D2[2];
			distanceAB=distanceBB2;
			normD=normD2;
			t=t2;
			this->currentSegment++;
		}
	}

	if(normD > this->radii){//out of the path?
		double F[3] = { A[0] + (B[0]-A[0])*(t + maxLinearVelocity/distanceAB), A[1] + (B[1]-A[1])*(t + maxLinearVelocity/distanceAB), A[2] + (B[2]-A[2])*(t + maxLinearVelocity/distanceAB) };
		/*if(this->currentSegment == 3 || 	1){
		cout<<"t "<<t<<endl;
		cout<<"A "<<A[0]<<" "<<A[1]<<" "<<A[2]<<endl;
		cout<<"B "<<B[0]<<" "<<B[1]<<" "<<B[2]<<endl;
		cout<<"C "<<C[0]<<" "<<C[1]<<" "<<C[2]<<endl;
		
		cout<<"D "<<D[0]<<" "<<D[1]<<" "<<D[2]<<endl;
		cout<<"F "<<F[0]<<" "<<F[1]<<" "<<F[2]<<endl;

		cout<<"P "<<position[0]<<" "<<position[1]<<" "<<position[2]<<endl;
		cout<<"V "<<velocity[0]<<" "<<velocity[1]<<" "<<velocity[2]<<endl;
		cout<<this->currentSegment<< " "<< numberOfSegments;
		cin.get();
		}*/
		this->seek(F);
	}
	else{
		//this->locomotion->updtatePosition();
		velocity[0]*=2;
		velocity[1]*=2;
		velocity[2]*=2;
		this->seek(velocity); //keeps moving in the same direction

	}
	








}


void FollowPathSteeringBehaviour::addVertexToThePath(double vertex[3]){
	this->pointsOfPath.push_back(vertex[0]);
	this->pointsOfPath.push_back(vertex[1]);
	this->pointsOfPath.push_back(vertex[2]);
	return;
}