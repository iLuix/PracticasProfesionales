#include "follow_path_steering_behaviour.hpp"


#include <iostream> 
using namespace std;


void FollowPathSteeringBehaviour::followPath(){
	int numberOfSegments=int(this->pointsOfPath.size()/2)-1;
	double* steer=getSteeringFollowPath();
	if(steer){
		locomotion->applyForce(steer);
		delete[] steer;
		return;
	}
	if(numberOfSegments <= this->currentSegment){ return; }//no se mueve si ya se lleg'o al final del camino

	//como no hubo steering y aun no se llega al final del camino, se actualiza la posicion
	locomotion->updatePosition();
	return;



	/*//recaba informaci'on del camino
	
	//recaba informaci'on del cuerpo
	double *position=this->locomotion->getPosition();
	double *velocity=this->locomotion->getVelocity();
	double maxLinearVelocity=this->locomotion->getMaxLinearVelocity();

	//pregunta si l camino se termin'o
	if(numberOfSegments <= this->currentSegment){ return; }

	//Segmentos actual A->B
	double A[3] = { pointsOfPath[3*currentSegment]  ,pointsOfPath[3*currentSegment+1],pointsOfPath[3*currentSegment+2]};
	double B[3] = { pointsOfPath[3*currentSegment+3],pointsOfPath[3*currentSegment+4],pointsOfPath[3*currentSegment+5]};
	//posicion futura si ning'un steer es aplicado	
	double C[3] = { position[0]+velocity[0], position[1]+velocity[1], position[2]+velocity[2] };
	
	//variables para obtener proyeccion de la posicion del cuerpo en el segmento A->B
	double distanceAB = sqrt ( (B[0]-A[0])*(B[0]-A[0]) + (B[1]-A[1])*(B[1]-A[1]) + (B[2]-A[2])*(B[2]-A[2]) ) ;
	double t= ( (B[0]-A[0])*(C[0]-A[0])+(B[1]-A[1])*(C[1]-A[1])+(B[2]-A[2])*(C[2]-A[2]) )/(distanceAB*distanceAB);
	//proyeccion del cuerpo en segmento
	double D[3] = { A[0] + (B[0]-A[0])*t - C[0], A[1] + (B[1]-A[1])*t - C[1], A[2] + (B[2]-A[2])*t - C[2] };
	double normD= sqrt(D[0]*D[0] + D[1]*D[1] + D[2]*D[2]);

	//pregunta si el cuerpo debe seguir el siguiente segmento por desborde
	if(t + maxLinearVelocity/distanceAB > 1.0){
		//prosigue al siguiente segmento rellamando a la funci'on
		this->currentSegment++;
		if(this->currentSegment < numberOfSegments ) 
		this->followPath();
		return;
	}
	//verifica si el siguiente segmento es m'as cercano(si lo hay) que el actual y de serlo hace los cambios necesarios en A->B, C, y currentSegment++
	if(this->currentSegment + 1 < numberOfSegments ){
		//fin del siguiente segmento, el inicio es B
		double B2[3] = { pointsOfPath[3*currentSegment+6],pointsOfPath[3*currentSegment+7],pointsOfPath[3*currentSegment+8]};	
		//variables para calcular proyecci'on de B->B2		
		double distanceBB2 = sqrt ( (B2[0]-B[0])*(B2[0]-B[0]) + (B2[1]-B[1])*(B2[1]-B[1]) + (B2[2]-B[2])*(B2[2]-B[2]) ) ;
		double t2=( (B2[0]*C[0] - B2[0]*B[0] -C[0]*B[0]) + (B2[1]*C[1] - B2[1]*B[1] -C[1]*B[1]) + (B2[2]*C[2] - B2[2]*B[2] -C[2]*B[2]) ) / ( distanceBB2*distanceBB2 ) ;
		//proyecci'on
		double D2[3] = { B[0] + (B2[0]-B[0])*t2 - C[0], B[1] + (B2[1]-B[1])*t2 - C[1], B[2] + (B2[2]-B[2])*t2 - C[2] };
		double normD2= sqrt(D2[0]*D2[0] + D2[1]*D2[1] + D2[2]*D2[2]);
		//hace el cambio si la proyeccion nueva es m'as cercana
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
	//aplica fuerza si el cuerpo se sale del radio del camino
	if(normD > this->pathRadii){//out of the path?
		//nuevo steer un paso adelante de la proyeccion del cuerpo definido como:
		double F[3] = { A[0] + (B[0]-A[0])*(t + maxLinearVelocity/distanceAB), A[1] + (B[1]-A[1])*(t + maxLinearVelocity/distanceAB), A[2] + (B[2]-A[2])*(t + maxLinearVelocity/distanceAB) };
		this->seek(F);
	}
	else{
		//si no, aplica un steer en direcci'on en la que ya va
		this->locomotion->updatePosition();
		return;
		velocity[0]*=2;
		velocity[1]*=2;
		velocity[2]*=2;
		this->seek(velocity); //keeps moving in the same direction

	}*/
}

//?
double *FollowPathSteeringBehaviour::getSteeringFollowPath(){
	//recaba informaci'on del camino
	int numberOfSegments=int(this->pointsOfPath.size()/2)-1;
	//recaba informaci'on del cuerpo
	double *position=this->locomotion->getPosition();
	double *velocity=this->locomotion->getVelocity();
	double maxLinearVelocity= this->locomotion->getMaxLinearVelocity();

	//pregunta si l camino se termin'o
	if(numberOfSegments <= this->currentSegment){ return NULL; }

	//Segmentos actual A->B
	double A[2] = { pointsOfPath[2*currentSegment]  ,pointsOfPath[2*currentSegment+1]};
	double B[2] = { pointsOfPath[2*currentSegment+2],pointsOfPath[2*currentSegment+3]};
	//posicion futura si ning'un steer es aplicado	
	double C[2] = { position[0]+velocity[0], position[1]+velocity[1]};
	
	//variables para obtener proyeccion de la posicion del cuerpo en el segmento A->B
	double distanceAB = sqrt ( (B[0]-A[0])*(B[0]-A[0]) + (B[1]-A[1])*(B[1]-A[1]) ) ;
	double t= ( (B[0]-A[0])*(C[0]-A[0])+(B[1]-A[1])*(C[1]-A[1]) )/(distanceAB*distanceAB);
	//proyeccion del cuerpo en segmento
	double D[2] = { A[0] + (B[0]-A[0])*t - C[0], A[1] + (B[1]-A[1])*t - C[1] };
	double normD= sqrt(D[0]*D[0] + D[1]*D[1]);

	//pregunta si el cuerpo debe seguir el siguiente segmento por desborde
	if(t + maxLinearVelocity/distanceAB > 1.0){
		//prosigue al siguiente segmento rellamando a la funci'on
		this->currentSegment++;
		if(this->currentSegment < numberOfSegments ) 
		return this->getSteeringFollowPath();
	}
	//verifica si el siguiente segmento es m'as cercano(si lo hay) que el actual y de serlo hace los cambios necesarios en A->B, C, y currentSegment++
	if(this->currentSegment + 1 < numberOfSegments ){
		//fin del siguiente segmento, el inicio es B
		double B2[2] = { pointsOfPath[2*currentSegment+4],pointsOfPath[2*currentSegment+5]};	
		//variables para calcular proyecci'on de B->B2		
		double distanceBB2 = sqrt ( (B2[0]-B[0])*(B2[0]-B[0]) + (B2[1]-B[1])*(B2[1]-B[1]) ) ;
		double t2=( (B2[0]*C[0] - B2[0]*B[0] -C[0]*B[0]) + (B2[1]*C[1] - B2[1]*B[1] -C[1]*B[1]) ) / ( distanceBB2*distanceBB2 ) ;
		//proyecci'on
		double D2[2] = { B[0] + (B2[0]-B[0])*t2 - C[0], B[1] + (B2[1]-B[1])*t2 - C[1] };
		double normD2= sqrt(D2[0]*D2[0] + D2[1]*D2[1]);
		//hace el cambio si la proyeccion nueva es m'as cercana
		if(normD2<normD && t>0.9){
			A[0]=B[0];
			A[1]=B[1];

			B[0]=B2[0];
			B[1]=B2[1];

			D[0]=D2[0];
			D[1]=D2[1];

			distanceAB=distanceBB2;
			normD=normD2;
			t=t2;
			this->currentSegment++;
		}
	}
	//aplica fuerza si el cuerpo se sale del radio del camino
	if(normD > this->pathRadii || t<0){//out of the path or going back
		//nuevo steer un paso adelante de la proyeccion del cuerpo definido como:
		double F[2] = { A[0] + (B[0]-A[0])*(t + maxLinearVelocity/distanceAB), A[1] + (B[1]-A[1])*(t + maxLinearVelocity/distanceAB)};
		return this->getSteeringSeek(F);
	}
	else{
		return NULL;
	}
}





void FollowPathSteeringBehaviour::addVertexToThePath(double vertex[2]){
	this->pointsOfPath.push_back(vertex[0]);
	this->pointsOfPath.push_back(vertex[1]);
	return;
}