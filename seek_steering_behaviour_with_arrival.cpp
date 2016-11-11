#include "seek_steering_behaviour_with_arrival.hpp"

void SeekSteeringBehaviourWithArrival::seek(double target[2]){
	//test de inicializaci'on de locomotion
	double *steer=this->getSteeringSeekWithArrival(target);
	if(steer!=NULL){ //test de steer v'alido
		this->locomotion->applyForce(steer);
		delete[] steer;
	}
	return;

	/*
	double *position=this->locomotion->getPosition();
	double *velocity=this->locomotion->getVelocity();

	double desired[3]={target[0]-position[0], target[1]-position[1], target[2]-position[2]};



	if(desired[0]!=0.0 && desired[0]!=0.0 && desired[0]!=0.0){
		double norm=sqrt(desired[0]*desired[0]+desired[1]*desired[1]+desired[2]*desired[2]);
		double maxLinearAcceleration=this->locomotion->getMaxLinearAcceleration();
		double maxLinearVelocity= this->locomotion->getMaxLinearVelocity();
		if(norm<this->arrivalRadii){
			double nuevaMagnitud=norm*maxLinearVelocity/this->arrivalRadii;
			desired[0]*= nuevaMagnitud/norm;
			desired[1]*= nuevaMagnitud/norm;
			desired[2]*= nuevaMagnitud/norm;
		}
		else{
			desired[0]*= maxLinearVelocity/norm;
			desired[1]*= maxLinearVelocity/norm;
			desired[2]*= maxLinearVelocity/norm;
		}

		double steer[3]={desired[0]-velocity[0],desired[1]-velocity[1],desired[2]-velocity[2]};
		norm=sqrt(steer[0]*steer[0]+steer[1]*steer[1]+steer[2]*steer[2]);
		if(norm!=0.0){
			steer[0]*=maxLinearAcceleration/norm;
			steer[1]*=maxLinearAcceleration/norm;
			steer[2]*=maxLinearAcceleration/norm;
		}
		this->locomotion->applyForce(steer);
	}

	else{
		return;
	}*/
} 	


double *SeekSteeringBehaviourWithArrival::getSteeringSeekWithArrival(double target[2]){
	//test de inicializaci'on
	if(locomotion==NULL){std::cout<<"el cuerpo locomotion no ha sido inicializado";return NULL;}
	//recaba informaci'on del cuerpo locomotion
	double *position=this->locomotion->getPosition();
	double *velocity=this->locomotion->getVelocity();
	double radii=locomotion->getRadii();
	double avoidanceRadii=this->avoidanceRadii;
	double maxLinearAcceleration=this->locomotion->getMaxLinearAcceleration();
	double maxLinearVelocity= this->locomotion->getMaxLinearVelocity();
	//aceleraci'on deseada
	double desired[2]={target[0]-position[0], target[1]-position[1]};
	double *steer=NULL;
	double norm;


	if(desired[0]!=0.0 && desired[1]!=0.0){
		double norm=sqrt(desired[0]*desired[0]+desired[1]*desired[1]);
		double newMagnitud;

		if(norm<this->arrivalRadii)
			newMagnitud=norm*maxLinearVelocity/this->arrivalRadii;
		else
			newMagnitud=maxLinearVelocity;

		desired[0]*= newMagnitud/norm;
		desired[1]*= newMagnitud/norm;

		steer=new double[2]{desired[0]-velocity[0],desired[1]-velocity[1]};
		norm=sqrt(steer[0]*steer[0]+steer[1]*steer[1]);
		if(norm!=0.0){
			steer[0]*=maxLinearAcceleration/norm;
			steer[1]*=maxLinearAcceleration/norm;
		}

		//test de colision aplicando el steer, solo funciona en 2D hasta ahora
		if(scene==NULL) return steer;

		//los dos puntos no son en realidad el de la izq y der, solo son ambos, pueden estar cambiados
		double leftPoint[2] = {position[0]-radii*steer[1]/maxLinearAcceleration,position[1]+radii*steer[0]/maxLinearAcceleration};
		double rightPoint[2]= {position[0]+radii*steer[1]/maxLinearAcceleration,position[1]-radii*steer[0]/maxLinearAcceleration};


		double leftCollision[2],rightCollision[2];
		PrimitiveSegment *collisionSegmentLeft =this->scene->predictCollisionPoint(leftPoint[0] ,leftPoint[1] ,leftPoint[0] +velocity[0]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[0],leftPoint[1] +velocity[1]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[1],leftCollision[0] ,leftCollision[1] );
		PrimitiveSegment *collisionSegmentRight=this->scene->predictCollisionPoint(rightPoint[0],rightPoint[1],rightPoint[0]+velocity[0]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[0],rightPoint[1]+velocity[1]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[1],rightCollision[0],rightCollision[1]);
		
		if(collisionSegmentLeft && collisionSegmentRight){
			if(collisionSegmentRight==collisionSegmentRight){//no esquina
				double normal[2]={-collisionSegmentRight->getY1()+collisionSegmentRight->getY2(),collisionSegmentRight->getX1()-collisionSegmentRight->getX2()};
				//dadas dos normales, define cu'al es la normal usando producto punto del steer y de la normal obtenida, el cual debe de ser negativo por la direccion de ambos
				if(normal[0]*steer[0]+normal[1]+steer[1]>0){
					normal[0]*=-1;
					normal[1]*=-1;
				}
				//normalizacion de normal
				norm=sqrt(normal[0]*normal[0]+normal[1]*normal[1]);
				normal[0]=(leftCollision[0]+rightCollision[0])/2.0+normal[0]*(avoidanceRadii)/norm;//?
				normal[1]=(leftCollision[1]+rightCollision[1])/2.0+normal[1]*(avoidanceRadii)/norm;//?
				return this->getSteeringSeekWithArrival(normal); //sets new target
			}
			else{//esquina detectada
				double centerCollision[2];
				PrimitiveSegment *collisionSegmentCenter =this->scene->predictCollisionPoint(position[0] ,position[1] ,position[0] +velocity[0]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[0],position[1] +velocity[1]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[1],centerCollision[0] ,centerCollision[1] );
				if(collisionSegmentCenter==0){
					//un caso donde hay estrechos m'as peque~nos que el radio del cuerpo
					std::cout<<"hay un estrecho m'as peque~no que el radio del robot, este caso no est'a optimizado";
					collisionSegmentCenter=collisionSegmentRight;
					centerCollision[0]=rightCollision[0];
					centerCollision[1]=rightCollision[1];
				}
				double normal[2]={-collisionSegmentCenter->getY1()-collisionSegmentCenter->getY2(),collisionSegmentCenter->getX1()-collisionSegmentCenter->getX2()};
				//dadas dos normales, define cu'al es la normal usando producto punto del steer y de la normal obtenida, el cual debe de ser negativo por la direccion de ambos
				if(normal[0]*steer[0]+normal[1]+steer[1]>0){
					normal[0]*=-1;
					normal[1]*=-1;
				}
				//normalizacion de normal
				norm=sqrt(normal[0]*normal[0]+normal[1]*normal[1]);
				normal[0]=centerCollision[0]+normal[0]*(avoidanceRadii)/norm;//normal al punto de colisi'on
				normal[1]=centerCollision[1]+normal[1]*(avoidanceRadii)/norm;
				return this->getSteeringSeekWithArrival(normal); //sets new target
			}
		}
		else if(collisionSegmentRight){ //solo colisi'on en la derecha, genera target a la izquierda
			double new_target[2]={leftPoint[0] +velocity[0]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[0],leftPoint[1] +velocity[1]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[1]};
			return this->getSteeringSeekWithArrival(new_target);
		}
		else if(collisionSegmentLeft){ //si no genera un target a la derecha
			double new_target[2]={rightPoint[0]+velocity[0]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[0],rightPoint[1]+velocity[1]+((maxLinearAcceleration+radii)/maxLinearAcceleration)*steer[1]};
			return this->getSteeringSeekWithArrival(new_target);
		}
		else{
			//...
		}
		//this->locomotion->applyForce(steer);
	}

	return steer;
} 	
