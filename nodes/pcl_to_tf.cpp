

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// MICHI: May20
#include <pcl_odometry.cpp>


////////////////////////////////////////////////////
//////////// GLOBAL SCOPE VARIABLES ////////////////
////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////
//////////// PARAMETERS FOR THE FILTERS AND ALGORITHM ////////////////
//////////////////////////////////////////////////////////////////////

///////// ICP ALGORITHM (from the IterativeClosestPoint Detailed Description):
int ARG_maxIterations = 100;
// Maximum amount of iterations of the ICP algorithm for approaching one group of points to another.
double ARG_epsilon = 1e-8;
// Maximum epsilon (difference) between the previous transformation and the current estimated transformation.
double ARG_eclideanDistance=1;
// The sum of Euclidean squared errors between the pairs of points.
int ARG_maxRansacIterations = 100;
// Maximum amount of iterations for each execution of the RANSAC algorithm
double ARG_ransacInlierThreshold = 0.1;
// Maximum distance between the points considered inliers for the RANSAc estimation.
double ARG_maxDistance=1.2;
// Maximum distance between the homolog points after the transformation.






///////////////////////////////////////////
//////////// MAIN FUNCTION ////////////////
///////////////////////////////////////////

int main(int argc, char **argv) {
	ros::init(argc, argv, "pcl_to_tf");
	
	ros::NodeHandle nHandle;
	ros::NodeHandle *nHandlePtr = &nHandle;
	
	pcl_odometry::odometryComm odoComm(nHandlePtr);

	odoComm.startCommunications();
	
	int entrada;
	std::cout << "Press 0 to continue" << std::endl; std::cin >> entrada;
	

	ros::spin();
	//	while (ros::ok()) ros::spinOnce();
	return 0;
}
