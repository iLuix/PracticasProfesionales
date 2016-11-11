todo:	
	@echo '   [      COMPILANDO      ]    '
	@ g++ -std=c++11 main.cpp -o main  -L/.usr/local/lib/ GCNavMesh2D/PrimitiveSegment.cpp GCNavMesh2D/Polygon.cpp GCNavMesh2D/Obstacle.cpp GCNavMesh2D/NavMeshScene.cpp GCNavMesh2D/NavMesh2DTrajectory.cpp GCNavMesh2D/NavMesh2DNode.cpp GCNavMesh2D/NavMesh2D.cpp GCNavMesh2D/Math.cpp GCNavMesh2D/CustomPriorityQueue.cpp -lopencv_core -lopencv_highgui  -lopencv_features2d -lopencv_imgproc -lopencv_imgcodecs prowl_steering_behaviour.cpp rigid_body_locomotion.cpp free_2d_movement.cpp steering_behaviour.cpp seek_steering_behaviour.cpp seek_steering_behaviour_with_arrival.cpp 
	@echo '   [      EJECUTANDO      ]    '
	@./main
	@echo '   [      TERMINANDO      ]    '
