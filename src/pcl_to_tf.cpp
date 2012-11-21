

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// MICHI: May20
#include <pcl_odometry.hpp>
// MICHI: May24
#include <boost/thread.hpp>
#include <signal.h>


//=============================================================================
//Function prototypes
//=============================================================================
/**
 * Signal handler.
 *
 *   @param signalNumber [in] - received signal number
 */
static void signalHandler(int signalNumber);

/**
 * Check if we should just print usage information, and if so, print it.
 *
 *   @param nodeName [in] - node name
 *   @param argc [in] - number of command line arguments
 *   @param argv [in] - command line arguments
 *
 *   @return true if usage was printed
 */
static bool printUsage(std::string nodeName, int argc, char** argv);




//=============================================================================
//Globals
//=============================================================================
/** Global atomic flag used to indicate if an interrupt has been requested. */
static sig_atomic_t gInterruptRequested = 0;




//=============================================================================
//Function definitions
//=============================================================================
static void signalHandler(int signalNumber) {
	gInterruptRequested = 1;
}
//=============================================================================



static bool printUsage(std::string nodeName, int argc, char** argv) {
  //Check for "-h" or "--help", if found, print usage
  for (int i = 1; i < argc; i++) {
    if ((0 == strcmp(argv[i], "-h")) || (0 == strcmp(argv[i], "--help"))) {
      printf("\n");
      printf("Usage:\n");
      printf("    %s [params]\n", nodeName.c_str());
      printf("\n");
      printf("Parameters and their default values\n");
      
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ICP_ITERATIONS,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ICP_ITERATIONS);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ICP_DISTANCE,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ICP_DISTANCE);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ICP_EPSILON,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ICP_EPSILON);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ICP_EUCLIDEAN_DISTANCE,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ICP_EUCLIDEAN_DISTANCE);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ICP_RANSAC_ITERATIONS,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ICP_RANSAC_ITERATIONS);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ICP_RANSAC_THRESHOLD,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ICP_RANSAC_THRESHOLD);
	  
	  printf("\n");
	  
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_RESOLUTION_LEAF_SIZE,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_RESOLUTION_LEAF_SIZE);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_RESOLUTION_FILTERING,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_RESOLUTION_FILTERING);

	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_MIN_DEPTH,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_MIN_DEPTH);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_MAX_DEPTH,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_MAX_DEPTH);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_DEPTH_FILTERING,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_DEPTH_FILTERING);
	  
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_NOISE_NEIGHBOURS,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_NOISE_NEIGHBOURS);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_NOISE_RADIUS,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_NOISE_RADIUS);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_NOISE_FILTERING,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_NOISE_FILTERING);
	  
	  printf("\n");
	  
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_INPUT_STR,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_INPUT_STR).c_str());
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_INPUT_PCL,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_INPUT_PCL).c_str());
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_OUTPUT_GLOBAL_ODOMETRY,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_GLOBAL_ODOMETRY).c_str());
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_OUTPUT_RELATIVE_ODOMETRY,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_RELATIVE_ODOMETRY).c_str());

	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_OUTPUT_ALIGNED_CLOUD,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_ALIGNED_CLOUD).c_str());
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_OUTPUT_INITIAL_CLOUD,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_INITIAL_CLOUD).c_str());
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_OUTPUT_FILTERED_CLOUD,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_FILTERED_CLOUD).c_str());
	  printf("    _%s:=%s\n",
			  pcl_odometry::odometryComm::PARAM_KEY_TOPIC_ODOMETRY_ANSWER,
			 (nodeName + "/" + pcl_odometry::odometryComm::PARAM_DEFAULT_TOPIC_ODOMETRY_ANSWER).c_str());
	  
	  printf("\n");
	  
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_MANUAL_MODE,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_MANUAL_MODE);
	  printf("    _%s:=%f\n",
			  pcl_odometry::odometryComm::PARAM_KEY_ACCURACY,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_ACCURACY);
	  
      printf("\n");

	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_KINECT_X,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_KINECT_X);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_KINECT_Y,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_KINECT_Y);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_KINECT_Z,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_KINECT_Z);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_KINECT_ROLL,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_KINECT_ROLL);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_KINECT_PITCH,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_KINECT_PITCH);
	  printf("    _%s:=%d\n",
			  pcl_odometry::odometryComm::PARAM_KEY_KINECT_YAW,
			  pcl_odometry::odometryComm::PARAM_DEFAULT_KINECT_YAW);
      return true;
    }
  }
  return false;
}
//=============================================================================



///////////////////////////////////////////
//////////// MAIN FUNCTION ////////////////
///////////////////////////////////////////

int main(int argc, char **argv) {
	//Use first argument (executable name) as node name
	std::string nodeName(basename(argv[0]));
	
	//Check if we should just print usage information and quit
	if (printUsage(nodeName, argc, argv)) {
		return 0;
	}
	
	//Set SIGINT signal handler
	signal(SIGINT, signalHandler);

	pcl_odometry::odometryComm odoComm;

	//Init node object
	if (!odoComm.init(argc, argv, nodeName, ros::init_options::NoSigintHandler)) {
		ROS_ERROR("main: error initialising node");
		return 1;
	}
	
	//Periodically check for interruption
	while (0 == gInterruptRequested) {
		ros::spinOnce();
//		boost::this_thread::sleep(boost::posix_time::milliseconds(500));
	}
	
	return 0;
}
