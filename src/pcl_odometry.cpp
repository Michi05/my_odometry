
//=============================================================================
//Includes
//=============================================================================

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
//May24
#include <pcl_odometry.hpp>



//=============================================================================
//Namespace
//=============================================================================
namespace pcl_odometry {




//=============================================================================
//Static member definitions
//=============================================================================
// Parameter keys
const char   odometryComm::PARAM_KEY_ICP_ITERATIONS[] =      "icp_max_iterations";
const char   odometryComm::PARAM_KEY_ICP_DISTANCE[] =    "icp_max_distance";
const char   odometryComm::PARAM_KEY_ICP_EPSILON[] =    "icp_epsilon";
const char   odometryComm::PARAM_KEY_ICP_EUCLIDEAN_DISTANCE[] =    "icp_euclidean_distance";
const char   odometryComm::PARAM_KEY_ICP_RANSAC_ITERATIONS[] =    "icp_ransac_iterations";
const char   odometryComm::PARAM_KEY_ICP_RANSAC_THRESHOLD[] =    "icp_ransac_threshold";
const char   odometryComm::PARAM_KEY_ICP_SCORE[] =    "icp_min_score";

const char   odometryComm::PARAM_KEY_CLOUD_TRIM_X[] =    "cloud_trim_x";
const char   odometryComm::PARAM_KEY_CLOUD_TRIM_Y[] =    "cloud_trim_y";
const char   odometryComm::PARAM_KEY_CLOUD_TRIM_Z[] =    "cloud_trim_z";

const char   odometryComm::PARAM_KEY_RESOLUTION_LEAF_SIZE[] =    "resolution_leaf_size";
const char   odometryComm::PARAM_KEY_RESOLUTION_FILTERING[] =    "activate_resolution_filter";

const char   odometryComm::PARAM_KEY_MIN_DEPTH[] =    "min_depth";
const char   odometryComm::PARAM_KEY_MAX_DEPTH[] =    "max_depth";
const char   odometryComm::PARAM_KEY_DEPTH_FILTERING[] =    "activate_depth_filter";

const char   odometryComm::PARAM_KEY_NOISE_NEIGHBOURS[] =    "noise_neighbours";
const char   odometryComm::PARAM_KEY_NOISE_RADIUS[] =    "noise_radius";
const char   odometryComm::PARAM_KEY_NOISE_FILTERING[] =    "activate_noise_filter";

const char   odometryComm::PARAM_KEY_TOPIC_INPUT_STR[] =    "topic_input_str";
const char   odometryComm::PARAM_KEY_TOPIC_INPUT_PCL[] =    "topic_input_pcl";
const char   odometryComm::PARAM_KEY_TOPIC_INPUT_CAMERA_TF[] =    "topic_camera_tf";
const char   odometryComm::PARAM_KEY_TOPIC_OUTPUT_GLOBAL_ODOMETRY[] =    "global_odometry_out_topic";
const char   odometryComm::PARAM_KEY_TOPIC_OUTPUT_RELATIVE_ODOMETRY[] =    "relative_odometry_out_topic";

const char   odometryComm::PARAM_KEY_TOPIC_OUTPUT_ALIGNED_CLOUD[] =    "topic_aligned_cloud";
const char   odometryComm::PARAM_KEY_TOPIC_OUTPUT_INITIAL_CLOUD[] =    "topic_initial_cloud";
const char   odometryComm::PARAM_KEY_TOPIC_OUTPUT_FILTERED_CLOUD[] =    "topic_filtered_cloud";
const char   odometryComm::PARAM_KEY_TOPIC_ODOMETRY_ANSWER[] =    "topic_odometry_answer";


const char   odometryComm::PARAM_KEY_MANUAL_MODE[] =    "odometry_manual_mode";
const char   odometryComm::PARAM_KEY_IGNORE_TIME[] =    "odometry_ignore_time";
const char   odometryComm::PARAM_KEY_ACCURACY[] =    "odometry_measure_accuracy";
const char   odometryComm::PARAM_KEY_ROTATION_ACCURACY[] =    "odometry_rotation_accuracy";

const char   odometryComm::PARAM_KEY_KINECT_X[] =    "kinect_x";
const char   odometryComm::PARAM_KEY_KINECT_Y[] =    "kinect_y";
const char   odometryComm::PARAM_KEY_KINECT_Z[] =    "kinect_z";

const char   odometryComm::PARAM_KEY_KINECT_ROLL[] =    "kinect_roll";
const char   odometryComm::PARAM_KEY_KINECT_PITCH[] =    "kinect_pitch";
const char   odometryComm::PARAM_KEY_KINECT_YAW[] =    "kinect_yaw";

// Parameter default values
const int   odometryComm::PARAM_DEFAULT_ICP_ITERATIONS =  150;
const double   odometryComm::PARAM_DEFAULT_ICP_DISTANCE =  0.8;
const double   odometryComm::PARAM_DEFAULT_ICP_EPSILON =  0.000005;
const double   odometryComm::PARAM_DEFAULT_ICP_EUCLIDEAN_DISTANCE =  0;
const int   odometryComm::PARAM_DEFAULT_ICP_RANSAC_ITERATIONS =  0;
const double   odometryComm::PARAM_DEFAULT_ICP_RANSAC_THRESHOLD =  0;
const double   odometryComm::PARAM_DEFAULT_ICP_SCORE =  0.02;

const double odometryComm::PARAM_DEFAULT_CLOUD_TRIM_X = 0.85;
const double odometryComm::PARAM_DEFAULT_CLOUD_TRIM_Y = 0.9;
const double odometryComm::PARAM_DEFAULT_CLOUD_TRIM_Z = 0.9;

const double   odometryComm::PARAM_DEFAULT_RESOLUTION_LEAF_SIZE =  0.01f;
const bool   odometryComm::PARAM_DEFAULT_RESOLUTION_FILTERING =  true;

const double   odometryComm::PARAM_DEFAULT_MIN_DEPTH =  1.1;
const double   odometryComm::PARAM_DEFAULT_MAX_DEPTH =  2.9;
const bool   odometryComm::PARAM_DEFAULT_DEPTH_FILTERING =  true;

const int   odometryComm::PARAM_DEFAULT_NOISE_NEIGHBOURS =  0;
const double   odometryComm::PARAM_DEFAULT_NOISE_RADIUS =  0;
const bool   odometryComm::PARAM_DEFAULT_NOISE_FILTERING =  false;

const char   odometryComm::PARAM_DEFAULT_TOPIC_INPUT_STR[] =    "inputStr";
const char   odometryComm::PARAM_DEFAULT_TOPIC_INPUT_PCL[] =    "inputPCL";
const char   odometryComm::PARAM_DEFAULT_TOPIC_INPUT_CAMERA_TF[] =    "input_camera_tf";
const char   odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_GLOBAL_ODOMETRY[] =    "globalOdometry";
const char   odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_RELATIVE_ODOMETRY[] =    "lastRelativeOdometry";

const char   odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_ALIGNED_CLOUD[] =    "aligned_cloud";
const char   odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_INITIAL_CLOUD[] =    "initial_cloud";
const char   odometryComm::PARAM_DEFAULT_TOPIC_OUTPUT_FILTERED_CLOUD[] =    "final_filtered_cloud";
const char   odometryComm::PARAM_DEFAULT_TOPIC_ODOMETRY_ANSWER[] =    "odometry_answer";

const bool   odometryComm::PARAM_DEFAULT_MANUAL_MODE = true;
const bool   odometryComm::PARAM_DEFAULT_IGNORE_TIME = true;
const double   odometryComm::PARAM_DEFAULT_ACCURACY = 0.01;
const double   odometryComm::PARAM_DEFAULT_ROTATION_ACCURACY = 0.0001;

const double   odometryComm::PARAM_DEFAULT_KINECT_X = 0.0f;
const double   odometryComm::PARAM_DEFAULT_KINECT_Y = 0.0f;
const double   odometryComm::PARAM_DEFAULT_KINECT_Z = 0.0f;

const double   odometryComm::PARAM_DEFAULT_KINECT_ROLL = 0.0f;
const double   odometryComm::PARAM_DEFAULT_KINECT_PITCH = 0.0f;
const double   odometryComm::PARAM_DEFAULT_KINECT_YAW = 0.0f;


	////////////////////////////////////////////////////
	//////////// GLOBAL SCOPE DECLARATIONS /////////////
	////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;



//=============================================================================
//Method definitions
//=============================================================================
	
	
	////////////////////////////////////////////////////////////////////
	//////////////////////// CLASS CONSTRUCTORS ////////////////////////
	////////////////////////////////////////////////////////////////////	
	
	odometryComm::odometryComm() :
		globalTF(tf::Transform::getIdentity()),
		lastRelativeTF(tf::Transform::getIdentity()),
		mIsInitialised(false),
		mSpinner(0),
		odomStatus(UNLOADED) {
	}
	//=============================================================================
	odometryComm::~odometryComm() {
	  //Shutdown (should always succeed)
	  if (!shutdown()) {
	    ROS_WARN("TeleopSourceNode::~TeleopSourceNode: ignoring error in shutdown");
	  }
	}
	
	
	
	
	
	//=============================================================================
	  bool odometryComm::init(int argc, char** argv, std::string nodeName, uint32_t rosInitOptions) {
		  //Lock access to is initialised
		  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

		  //If already initialised shutdown first (shutdown should always succeed)
		  if (mIsInitialised && !shutdown()) {
		    ROS_WARN("odometryComm::init: ignoring error in shutdown()");
		  }

		  //Init node
		  try {
			  ros::init(argc, argv, nodeName, rosInitOptions);
		  } catch(ros::InvalidNodeNameException& e) {
			  ROS_ERROR("odometryComm::init: error initialising node");
			  return false;
		  }

		  //Start node manually to avoid node shutdown when last node handle is destroyed
		  ros::start();
		  


		  //Init parameters
		  try {
		    //Create private node handle for parameters
		    ros::NodeHandle nodeHandlePrivate("~");

		    //Read parameters and/or set default values
		    nodeHandlePrivate.param(PARAM_KEY_ICP_ITERATIONS,    maxIterations, PARAM_DEFAULT_ICP_ITERATIONS);
		    nodeHandlePrivate.param(PARAM_KEY_ICP_DISTANCE, maxDistance, PARAM_DEFAULT_ICP_DISTANCE);
		    nodeHandlePrivate.param(PARAM_KEY_ICP_EPSILON,    epsilon, PARAM_DEFAULT_ICP_EPSILON);
		    nodeHandlePrivate.param(PARAM_KEY_ICP_EUCLIDEAN_DISTANCE, euclideanDistance, PARAM_DEFAULT_ICP_EUCLIDEAN_DISTANCE);
		    nodeHandlePrivate.param(PARAM_KEY_ICP_RANSAC_ITERATIONS,    maxRansacIterations, PARAM_DEFAULT_ICP_RANSAC_ITERATIONS);
		    nodeHandlePrivate.param(PARAM_KEY_ICP_RANSAC_THRESHOLD, ransacInlierThreshold, PARAM_DEFAULT_ICP_RANSAC_THRESHOLD);
		    nodeHandlePrivate.param(PARAM_KEY_ICP_SCORE, ICPMinScore, PARAM_DEFAULT_ICP_SCORE);

		    nodeHandlePrivate.param(PARAM_KEY_CLOUD_TRIM_X, cloud_trim_x, PARAM_DEFAULT_CLOUD_TRIM_X);
		    nodeHandlePrivate.param(PARAM_KEY_CLOUD_TRIM_Y, cloud_trim_y, PARAM_DEFAULT_CLOUD_TRIM_Y);
		    nodeHandlePrivate.param(PARAM_KEY_CLOUD_TRIM_Z, cloud_trim_z, PARAM_DEFAULT_CLOUD_TRIM_Z);

		    nodeHandlePrivate.param(PARAM_KEY_RESOLUTION_LEAF_SIZE,    leafSize, PARAM_DEFAULT_RESOLUTION_LEAF_SIZE);
		    nodeHandlePrivate.param(PARAM_KEY_RESOLUTION_FILTERING, doDownsampleFiltering, PARAM_DEFAULT_RESOLUTION_FILTERING);

		    nodeHandlePrivate.param(PARAM_KEY_MIN_DEPTH,    minDepth, PARAM_DEFAULT_MIN_DEPTH);
		    nodeHandlePrivate.param(PARAM_KEY_MAX_DEPTH, maxDepth, PARAM_DEFAULT_MAX_DEPTH);
		    nodeHandlePrivate.param(PARAM_KEY_DEPTH_FILTERING, doDepthFiltering, PARAM_DEFAULT_DEPTH_FILTERING);

		    nodeHandlePrivate.param(PARAM_KEY_NOISE_NEIGHBOURS,    neighbours, PARAM_DEFAULT_NOISE_NEIGHBOURS);
		    nodeHandlePrivate.param(PARAM_KEY_NOISE_RADIUS, radius, PARAM_DEFAULT_NOISE_RADIUS);
		    nodeHandlePrivate.param(PARAM_KEY_NOISE_FILTERING, doNoiseFiltering, PARAM_DEFAULT_NOISE_FILTERING);

		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_INPUT_STR, inputStrRequest_topic, std::string(PARAM_DEFAULT_TOPIC_INPUT_STR));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_INPUT_PCL, inputPCL_topic, std::string(PARAM_DEFAULT_TOPIC_INPUT_PCL));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_INPUT_CAMERA_TF, cameraTF_topic, std::string(PARAM_DEFAULT_TOPIC_INPUT_CAMERA_TF));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_OUTPUT_GLOBAL_ODOMETRY, outputGlobalOdometry_topic, std::string(PARAM_DEFAULT_TOPIC_OUTPUT_GLOBAL_ODOMETRY));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_OUTPUT_RELATIVE_ODOMETRY, outputRelativeOdometry_topic, std::string(PARAM_DEFAULT_TOPIC_OUTPUT_RELATIVE_ODOMETRY));

		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_OUTPUT_ALIGNED_CLOUD, outputAlignedCloud_topic, std::string(PARAM_DEFAULT_TOPIC_OUTPUT_ALIGNED_CLOUD));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_OUTPUT_INITIAL_CLOUD, outputInitialCloud_topic, std::string(PARAM_DEFAULT_TOPIC_OUTPUT_INITIAL_CLOUD));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_OUTPUT_FILTERED_CLOUD, outputFilteredCloud_topic, std::string(PARAM_DEFAULT_TOPIC_OUTPUT_FILTERED_CLOUD));
		    nodeHandlePrivate.param(PARAM_KEY_TOPIC_ODOMETRY_ANSWER, outputOdometryAnswer_topic, std::string(PARAM_DEFAULT_TOPIC_ODOMETRY_ANSWER));

		    nodeHandlePrivate.param(PARAM_KEY_MANUAL_MODE, manualMode, PARAM_DEFAULT_MANUAL_MODE);
		    nodeHandlePrivate.param(PARAM_KEY_IGNORE_TIME, ignoreTimestamp, PARAM_DEFAULT_IGNORE_TIME);
		    nodeHandlePrivate.param(PARAM_KEY_ACCURACY, measureAccuracy, PARAM_DEFAULT_ACCURACY);
		    nodeHandlePrivate.param(PARAM_KEY_ROTATION_ACCURACY, rotationAccuracy, PARAM_DEFAULT_ROTATION_ACCURACY);


		    nodeHandlePrivate.param(PARAM_KEY_KINECT_X, kinectX, PARAM_DEFAULT_KINECT_X);
		    nodeHandlePrivate.param(PARAM_KEY_KINECT_Y, kinectY, PARAM_DEFAULT_KINECT_Y);
		    nodeHandlePrivate.param(PARAM_KEY_KINECT_Z, kinectZ, PARAM_DEFAULT_KINECT_Z);

		    nodeHandlePrivate.param(PARAM_KEY_KINECT_ROLL, kinectRoll, PARAM_DEFAULT_KINECT_ROLL);
		    nodeHandlePrivate.param(PARAM_KEY_KINECT_PITCH, kinectPitch, PARAM_DEFAULT_KINECT_PITCH);
		    nodeHandlePrivate.param(PARAM_KEY_KINECT_YAW, kinectYaw, PARAM_DEFAULT_KINECT_YAW);
		    
		    
// nodeHandlePrivate.param(PARAM_KEY_ICP_ITERATIONS,    aaaa, nodeName + "/" + bbbbb);
		    
		    //Advertise all parameters to allow introspection
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_ITERATIONS,    maxIterations);
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_DISTANCE, maxDistance);
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_EPSILON,    epsilon);
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_EUCLIDEAN_DISTANCE, euclideanDistance);
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_RANSAC_ITERATIONS,    maxRansacIterations);
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_RANSAC_THRESHOLD, ransacInlierThreshold);
		    nodeHandlePrivate.setParam(PARAM_KEY_ICP_SCORE, ICPMinScore);

		    nodeHandlePrivate.setParam(PARAM_KEY_CLOUD_TRIM_X, cloud_trim_x);
		    nodeHandlePrivate.setParam(PARAM_KEY_CLOUD_TRIM_Y, cloud_trim_y);
		    nodeHandlePrivate.setParam(PARAM_KEY_CLOUD_TRIM_Z, cloud_trim_z);

		    nodeHandlePrivate.setParam(PARAM_KEY_RESOLUTION_LEAF_SIZE, leafSize);
		    nodeHandlePrivate.setParam(PARAM_KEY_RESOLUTION_FILTERING, doDownsampleFiltering);

		    nodeHandlePrivate.setParam(PARAM_KEY_MIN_DEPTH,    minDepth);
		    nodeHandlePrivate.setParam(PARAM_KEY_MAX_DEPTH, maxDepth);
		    nodeHandlePrivate.setParam(PARAM_KEY_DEPTH_FILTERING, doDepthFiltering);

		    nodeHandlePrivate.setParam(PARAM_KEY_NOISE_NEIGHBOURS,    neighbours);
		    nodeHandlePrivate.setParam(PARAM_KEY_NOISE_RADIUS, radius);
		    nodeHandlePrivate.setParam(PARAM_KEY_NOISE_FILTERING, doNoiseFiltering);
		    
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_INPUT_STR, inputStrRequest_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_INPUT_PCL, inputPCL_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_INPUT_CAMERA_TF, cameraTF_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_OUTPUT_GLOBAL_ODOMETRY, outputGlobalOdometry_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_OUTPUT_RELATIVE_ODOMETRY, outputRelativeOdometry_topic);
			
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_OUTPUT_ALIGNED_CLOUD, outputAlignedCloud_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_OUTPUT_INITIAL_CLOUD, outputInitialCloud_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_OUTPUT_FILTERED_CLOUD, outputFilteredCloud_topic);
		    nodeHandlePrivate.setParam(PARAM_KEY_TOPIC_ODOMETRY_ANSWER, outputOdometryAnswer_topic);

		    nodeHandlePrivate.setParam(PARAM_KEY_MANUAL_MODE, manualMode);
		    nodeHandlePrivate.setParam(PARAM_KEY_IGNORE_TIME, ignoreTimestamp);
		    nodeHandlePrivate.setParam(PARAM_KEY_ACCURACY, measureAccuracy);
		    nodeHandlePrivate.setParam(PARAM_KEY_ROTATION_ACCURACY, rotationAccuracy);

		    
		    nodeHandlePrivate.setParam(PARAM_KEY_KINECT_X, kinectX);
		    nodeHandlePrivate.setParam(PARAM_KEY_KINECT_Y, kinectY);
		    nodeHandlePrivate.setParam(PARAM_KEY_KINECT_Z, kinectZ);

		    nodeHandlePrivate.setParam(PARAM_KEY_KINECT_ROLL, kinectRoll);
		    nodeHandlePrivate.setParam(PARAM_KEY_KINECT_PITCH, kinectPitch);
		    nodeHandlePrivate.setParam(PARAM_KEY_KINECT_YAW, kinectYaw);

		    printConfiguration();
		    
		  } catch (ros::InvalidNameException& e) {
		    ROS_ERROR("TeleopSourceNode::init: error initialising parameters");
		    ros::shutdown();
		    return false;
		  }
		  
		  

		  //Create publisher
		  try {
		    //Create relative node handle for topics
//		    ros::NodeHandle nodeHandleRelative = ros::NodeHandle();
			ros::NodeHandle nodeHandlePrivate("~");
			
		    //Create publisher with buffer size set to 1 and latching on. 
		    // The publisher should basically just always contain the latest state.
			int output_queue_size = 1;
			pcl_pub_aligned = nodeHandlePrivate.advertise<PointCloudT> (outputAlignedCloud_topic, output_queue_size, true);
			pcl_pub_initial = nodeHandlePrivate.advertise<PointCloudT> (outputInitialCloud_topic, output_queue_size, true);
			pcl_pub_final = nodeHandlePrivate.advertise<PointCloudT> (outputFilteredCloud_topic, output_queue_size, true);
			odom_answer_publisher = nodeHandlePrivate.advertise<my_odometry::odom_answer> (outputOdometryAnswer_topic, output_queue_size, true);


			int input_queue_size = 1;
			if (manualMode==true)
				pcl_sub = nodeHandlePrivate.subscribe<std_msgs::String> (inputStrRequest_topic, input_queue_size, &odometryComm::STR_callback, this);
			else
				pcl_sub = nodeHandlePrivate.subscribe<PointCloudT> (inputPCL_topic, input_queue_size, &odometryComm::PCL_callback, this);
			camera_tf_sub = nodeHandlePrivate.subscribe<tf::tfMessage> (cameraTF_topic, input_queue_size, &odometryComm::cameraTF_callback, this);
			
		  } catch (ros::InvalidNameException& e) {
		    ROS_ERROR("odometryComm::init: error creating publisher");
		    ros::shutdown();
		    return false;
		  }
		  
			ROS_INFO("Trying to launch Visual Odometry with the next parameters for epsilon=%f, maxIterations=%d and maxDistance=%f.\r\n", epsilon, maxIterations, maxDistance);

			setDownsampleFiltering(leafSize);
			setDistanceFiltering(minDepth, maxDepth);
			// setNoiseFiltering(radius, neighbours);

		  //Update init done flag
		  mIsInitialised = true;
		  

		  // Launch services
		  ros::NodeHandle nHandle("~");
		  server_updateOdometry = new ros::ServiceServer(nHandle.advertiseService("updateOdometry", &odometryComm::update_odometry, this));
		  server_resetGlobals = new ros::ServiceServer(nHandle.advertiseService("resetGlobals", &odometryComm::reset_globals, this));
		  server_getLastStatus = new ros::ServiceServer(nHandle.advertiseService("getLastStatus", &odometryComm::get_last_status, this));

		  //Create and start single-threaded asynchronous spinner to handle incoming
		  //ROS messages via our sole subscriber.  Use only one thread, since the
		  //callback method is not thread-safe.
		  mSpinner = new ros::AsyncSpinner(1);
		  mSpinner->start();
		  
		  // The fixed camera tf needs to be initialized according to rotations
		  fixed_camera_transform = tf::Transform::getIdentity();
		  rotate_tf(fixed_camera_transform, kinectRoll, kinectPitch, kinectYaw);
		  std::cout << "***initial camera fixed tf:" << std::endl; printTransform(fixed_camera_transform);

			// Initialise status (no error and not-yet waiting for data)
		  setOdomStatus(INITIALIZED);
		  
		  ROS_INFO("Completely Initialized");

		  //Return result
		  return true;
		}
	  
		//=============================================================================
		bool odometryComm::shutdown() {
		  //Lock access to is initialised
		  boost::lock_guard<boost::recursive_mutex> isInitialisedLock(mIsInitialisedMutex);

		  //Check if done
		  if (!mIsInitialised) {
		    return true;
		  }


		  //Delete the spinner
		  delete mSpinner;

			//Shutdown node
		  ros::shutdown();

		  //Set init done flag
		  mIsInitialised = false;

		  //Return result
		  return true;
		}
		
		
		
		
		
		
		
		
		
		
		

	
	
	////////////////////////////////////////////////////////////////////
	//////////////////////// CLASS METHODS ////////////////////////
	////////////////////////////////////////////////////////////////////

	
//////////////////////////////////////////////////////
///////////// PUBLIC CONTROL METHODS /////////////////
//////////////////////////////////////////////////////

	void odometryComm::setDownsampleFiltering(float newLeafSize) {
		leafSize = newLeafSize;
		doDownsampleFiltering = true;
	}
	//=============================================================================
	void odometryComm::setDistanceFiltering(float minimum, float maximum) {
		minDepth = minimum;
		maxDepth = maximum;
		doDepthFiltering = true;
	}
	//=============================================================================
	void odometryComm::setNoiseFiltering(float radDist, int noOfNeighbours) {
		radius = radDist;
		neighbours = noOfNeighbours;
		doNoiseFiltering = true;
	}

	
	
	
	
	////////////////////////////////////////////////////////
	///////////// PRIVATE AUXILIAR METHODS /////////////////
	////////////////////////////////////////////////////////
	void odometryComm::printConfiguration(){
	      printf("\n");
	      printf("Parameters and their CURRENT values\n");

		  printf("    _%s:=%d\n",
				  PARAM_KEY_ICP_ITERATIONS,
				  maxIterations);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_ICP_DISTANCE,
				  maxDistance);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_ICP_EPSILON,
				  epsilon);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_ICP_EUCLIDEAN_DISTANCE,
				  euclideanDistance);
		  printf("    _%s:=%u\n",
				  PARAM_KEY_ICP_RANSAC_ITERATIONS,
				  maxRansacIterations);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_ICP_RANSAC_THRESHOLD,
				  ransacInlierThreshold);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_ICP_SCORE,
				  ICPMinScore);

		  printf("    _%s:=%f\n",
				  PARAM_KEY_CLOUD_TRIM_X,
				  cloud_trim_x);
		  printf("    _%s:=%f\n",
				  PARAM_KEY_CLOUD_TRIM_Y,
				  cloud_trim_y);
		  printf("    _%s:=%f\n",
				  PARAM_KEY_CLOUD_TRIM_Z,
				  cloud_trim_z);
		  
		  printf("    _%s:=%E\n",
				  PARAM_KEY_RESOLUTION_LEAF_SIZE,
				  leafSize);
		  printf("    _%s:=%d\n",
				  PARAM_KEY_RESOLUTION_FILTERING,
				  doDownsampleFiltering);

		  printf("    _%s:=%E\n",
				  PARAM_KEY_MIN_DEPTH,
				  minDepth);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_MAX_DEPTH,
				  maxDepth);
		  printf("    _%s:=%d\n",
				  PARAM_KEY_DEPTH_FILTERING,
				  doDepthFiltering);
		  
		  printf("    _%s:=%d\n",
				  PARAM_KEY_NOISE_NEIGHBOURS,
				  neighbours);
		  printf("    _%s:=%E\n",
				  PARAM_KEY_NOISE_RADIUS,
				  radius);
		  printf("    _%s:=%d\n",
				  PARAM_KEY_NOISE_FILTERING,
				  doNoiseFiltering);

		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_INPUT_STR,
				  inputStrRequest_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_INPUT_PCL,
				  inputPCL_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_INPUT_CAMERA_TF,
				  cameraTF_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_OUTPUT_GLOBAL_ODOMETRY,
				  outputGlobalOdometry_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_OUTPUT_RELATIVE_ODOMETRY,
				  outputRelativeOdometry_topic.c_str());

		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_OUTPUT_ALIGNED_CLOUD,
				  outputAlignedCloud_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_OUTPUT_INITIAL_CLOUD,
				  outputInitialCloud_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_OUTPUT_FILTERED_CLOUD,
				  outputFilteredCloud_topic.c_str());
		  printf("    _%s:=%s\n",
				  PARAM_KEY_TOPIC_ODOMETRY_ANSWER,
				  outputOdometryAnswer_topic.c_str());
		  
		  printf("    _%s:=%d\n",
				  PARAM_KEY_MANUAL_MODE,
				  manualMode);
		  printf("    _%s:=%d\n",
				  PARAM_KEY_IGNORE_TIME,
				  ignoreTimestamp);
		  printf("    _%s:=%f\n",
				  PARAM_KEY_ACCURACY,
				  measureAccuracy);
		  printf("    _%s:=%f\n",
				  PARAM_KEY_ROTATION_ACCURACY,
				  rotationAccuracy);

		  printf("    _%s:=%f\n",
				  PARAM_KEY_KINECT_ROLL,
				  kinectRoll);
		  printf("    _%s:=%f\n",
				  PARAM_KEY_KINECT_PITCH,
				  kinectPitch);
		  printf("    _%s:=%f\n",
				  PARAM_KEY_KINECT_YAW,
				  kinectYaw);
	      printf("\n");
	}


	//=============================================================================
	void odometryComm::publishOdom(ros::Publisher &odom_publisher, tf::Transform tMatrix) {
		odom_publisher.publish(transformToOdometry(tMatrix));
	}

	//=============================================================================
	PointCloudT::ConstPtr odometryComm::fetchPointCloud (std::string topicName, int timeout){
	    ros::NodeHandle nodeHandleRelative = ros::NodeHandle();
		return ros::topic::waitForMessage<PointCloudT>(topicName, nodeHandleRelative, ros::Duration(timeout));
	}

	//=============================================================================
	void odometryComm::apply_filters(PointCloudT::ConstPtr pointCloud1_in, PointCloudT::Ptr &pointCloud1_out) {
		// During the filtering, "pointCloud1_out" is always the last correct cloud and satisfies
		//"pointCloud1_in==pointCloud1_out" at the beginning and the end of each different filter.

		// The pointCloud1_out needs to be already initialized
		if (pointCloud1_out == 0)
			pointCloud1_out = PointCloudT::Ptr(new PointCloudT);

		// Downsample filtering
		if (doDownsampleFiltering ==true) {
			pcl::VoxelGrid<PointT> sor;
			sor.setInputCloud (pointCloud1_in);
			sor.setDownsampleAllData (true);
			sor.setLeafSize (leafSize, leafSize, leafSize);
			sor.filter (*pointCloud1_out);

			pointCloud1_in=pointCloud1_out;
		}

		// Distance filtering
		if (doDepthFiltering ==true) {
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (pointCloud1_in);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (minDepth, maxDepth);
			pass.filter (*pointCloud1_out);

			pointCloud1_in=pointCloud1_out;
		}

		// Noise filtering
		if (doNoiseFiltering ==true) {
			pcl::RadiusOutlierRemoval<PointT> radius_outlier_removal;
			radius_outlier_removal.setInputCloud (pointCloud1_in);
			// set radius for neighbor search
			radius_outlier_removal.setRadiusSearch (radius);
			// set threshold for minimum required neighbours
			radius_outlier_removal.setMinNeighborsInRadius (neighbours);
			radius_outlier_removal.filter (*pointCloud1_out);
		}
		
		pointCloud1_in = pointCloud1_out;
	}

	//=============================================================================
	void odometryComm::trimPreviousCloud(PointCloudT::Ptr &ptcld, double x, double y, double z) {
		// A temporary cloud is used as the "pcl::PassThrough" specification doesn't 
		//guarantee to work if storing result back into the original cloud.
		PointCloudT::Ptr temp(new PointCloudT);
		if (ptcld == 0 || ptcld->points.size() < 1) {
			return; // No action without points
		}

// Once the absolute value is taken,
//values above 1 mean 100% of the image
		x = std::abs(x);
		if (x < 1) {
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (ptcld);
			pass.setFilterFieldName ("x");
			pass.setFilterLimits (-x, x);
			// Filter between -x and x. Then store it back to ptcld
			pass.filter (*temp);
			ptcld = temp;
		}

		y = std::abs(y);
		if (y < 1) {
			pcl::PassThrough<PointT> pass;
			pass.setInputCloud (ptcld);
			pass.setFilterFieldName ("y");
			pass.setFilterLimits (-y, y);
			// Filter between -y and y. Then store it back to ptcld
			pass.filter (*temp);
			ptcld = temp;
		}


		z = std::abs(z);
		if (z < 1) {
			// The max/min Depth parameters determine the possible values of depth
			double d = (maxDepth - minDepth);
			//the new min/max (after trimming) are calculated according to 'z'
			double minTrim = minDepth + d*(1.0f-z), maxTrim = minDepth + d*z;
			pcl::PassThrough<PointT> pass;
			std::cout << " \t trimming z in " << minTrim << ", " << maxTrim << std::endl << std::endl;
			pass.setInputCloud (ptcld);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (minTrim, maxTrim);
			// Filter between min and max. Then store it back to ptcld
			pass.filter (*temp);
			ptcld = temp;
		}
	}

	//=============================================================================

	double odometryComm::round(double value, int noDecimals) {
		return floor(value*pow(10, noDecimals) + 0.5) / pow(10, noDecimals);
	}

	//=============================================================================

	void odometryComm::applyRestrictions(tf::Transform &tf_input) {
		/* TODO: 
	 * This method can help to clean errors from the algorithm in yaw calculations:
	 * It may be improved for removing wrong measures (like height) or removed
	 * but right now it's not doing anything, just the place to do it.
		*/
		double R, P, Y;			tf_input.getBasis().getRPY(R, P, Y);
		tf::Vector3 origin = tf_input.getOrigin(); // tf::Vector3(0, 0, 0);
		tf::Transform tf;		generate_tf(tf, R, P, Y);
		tf.setOrigin(tf::Vector3(origin.x(), origin.y(), origin.z()));
		tf_input = tf;
	}

	void odometryComm::toROSCoordinates(tf::Transform &tf) {
		// Adapting the values to the ROS convention:
		// "3D coordinate systems in ROS are always right-handed, with X
		//forward, Y left, and Z up." (http://ros.org/wiki/tf/Overview/Transformations)
		// But (CAUTION!) Point Clouds have ALWAYS Y to represent DOWN (not up) and Z for depth,
		//which is ((MINUS))Forward! So the change is:
		//(X, Y, Z)_transform == (-Z_cloud, X_cloud, Y_cloud)
		
		
		// The initial RPY and XYZ values are stored firstly
		double R, P, Y;			tf.getBasis().getRPY(R, P, Y);
		btVector3 origin(-tf.getOrigin()[2], tf.getOrigin()[0], -tf.getOrigin()[1]);
		// Finally they are stored in the 'this' instance
		tf.generate_tf(P, Y, -R);
		tf.setOrigin(origin);
	}

	//=============================================================================
	void odometryComm::filter_resulting_TF(tf::Transform &tf_result) {
		// A first rounding is needed in order to avoid the noise
		//(a 0.00001 from the camera must be physical error but
		//0.001 after transformation is not so clear)
		round_near_zero_values(tf_result, measureAccuracy);

		// As the fixed_camera_transform is in ROS coordinates, the
		//transform must be changed to the same convention before the
		//"get_robot_relative_tf" method
		targetTF->toROSCoordinates(tf_result);
		
		// The known tf is the movement of the camera.
		//The robot-relative tf is the needed one.
		get_robot_relative_tf(tf_result, fixed_camera_transform);
		
		// This filter removes impossible calculations (like the
		//robot seemly flying or so)
		applyRestrictions(tf_result);

		// The last rounding can be removed but it's useful for
		//removing residual errors because of the maths
		round_near_zero_values(tf_result, measureAccuracy);
	}

	//=============================================================================
	void odometryComm::round_near_zero_values(tf::Transform &tf_result, double accuracy){
		// NOTE: I remove the near-0 values in the rotations if they seem to belower than
		//the real detectable accuracy. This is done in the first place as the accuracy
		//is from the camera POV, not after the rotations.

		// Check rotations for near-zero values:
		if (transformToRotation(tf_result) == 0 ) { //< 0.001f) {
			tf::Quaternion newRotation(0, 0, 0, 1); // XYZW constructor
	//		newRotation.setRPY(x, y, z);
			tf_result.setRotation(newRotation);
		}
		
		// Check the origin vector for near-zero values:
		for (int i=0; i<3; i++) {
			double val = tf_result.getOrigin()[i];
			if (val < accuracy && val > -accuracy) {
				tf_result.getOrigin()[i] = 0;
			}
		}
	}

	//=============================================================================
	
	void odometryComm::printTransform(tf::Transform &newTF){
		/*// This is ommitted as the whole matrix is not needed anymore
		  for (int i=0; i<3; i++) {
			  tf::Vector3 tmp = newTF.getBasis()[i];
			  std::cout << " \t " << tmp.x();
			  if (tmp.x()==1 || tmp.x()==0)
				  std::cout << " \t ";
			  std::cout << " \t " << tmp.y();
			  if (tmp.y()==1 || tmp.y()==0)
				  std::cout << " \t ";
			  std::cout << " \t " << tmp.z() << std::endl;
		  }
		  */
		  std::cout << "(x, y, z) = \t " << newTF.getOrigin().x() << " \t " << newTF.getOrigin().y() << " \t " << newTF.getOrigin().z() << std::endl;
		  std::cout << "(R, P, Y :: W) normalized = \r\n\t\t " << newTF.getRotation().getX() << " \t " << newTF.getRotation().getY() << " \t " << newTF.getRotation().getZ() << " \t ++ " << newTF.getRotation().getW() << std::endl;
		  double R, P, Y; newTF.getBasis().getRPY(R, P, Y);
		  std::cout << "(R, P, Y) = \t " << R << " \t " << P << " \t " << Y << std::endl;
		  std::cout << std::endl << std::endl;
	}
	
	//=============================================================================
	
	void odometryComm::printMatrix4f(Eigen::Matrix4f &tMatrix){
		  tf::Transform newTF = eigenToTransform(tMatrix);
		  printTransform(newTF);
	}
	
	//=============================================================================

	void odometryComm::setOdomStatus(int status) {
		odomStatus = status;
	}
	
	//=============================================================================

	bool odometryComm::generate_tf(tf::Transform &tf, double roll, double pitch, double yaw) {
		tf = tf::Transform::getIdentity();
		
		tf::Quaternion newRotation(0, 0, 0, 1); // XYZW quaternion constructor
		newRotation.setRPY(roll, pitch, yaw);
//		std::cout << "setRotation(" << x << ", " << y << ", " << z << ")" << std::endl;

		tf.setRotation(newRotation);
		tf.setOrigin(tf::Vector3(0, 0, 0));

		return true;
	}

	//=============================================================================

	bool odometryComm::generate_tf(Eigen::Matrix4f &mat, double roll, double pitch, double yaw) {
		tf::Transform tf = tf::Transform::getIdentity();
		generate_tf(tf, roll, pitch, yaw);
		
		mat = *(new Eigen::Matrix4f());
		
		pcl_ros::transformAsMatrix(tf, mat);
		
		return true;
	}

	//=============================================================================

	void odometryComm::rotate_tf(tf::Transform &original_tf, double roll, double pitch, double yaw) {
		// Starts by creating a fixedTF with the rotations. Then rotates the original tf
		tf::Transform fixedTF = tf::Transform::getIdentity();
		generate_tf(fixedTF, roll, pitch, yaw);

		original_tf.mult(original_tf, fixedTF);
	}

	
	//=============================================================================

	void odometryComm::get_robot_relative_tf(tf::Transform &original_tf, tf::Transform &fixedTF) {

		// (( Remember: matrices are asociative, but not conmutative ))

		tf::Transform resultTF1;
		// The fixed camera position is relative to the (0, 0, 0) of the robot
		//and the odometry estimation is about the camera movement
		// The fixedTF times camera movement times the inverse of the first must
		//be the robot's movement as the said relative position is fixed.
		std::cout << "*** Camera movement:" << std::endl; printTransform(original_tf);

		resultTF1 = fixedTF * original_tf * fixedTF.inverse();
		std::cout << "*** Robot movement:" << std::endl; printTransform(resultTF1);
				
		original_tf = resultTF1;
	}

	
	//////////////////////////////////////////////////////
	///////// TRANSFORMATION FRAMES' METHODS /////////////
	//////////////////////////////////////////////////////
	void odometryComm::broadcastTransform(tf::Transform &newTF, std::string tfChannel, std::string tfParent){
	  tf::TransformBroadcaster br;
	  std::cout << "Broadcasting TF \"" << tfChannel << "\" at time: " << ros::Time::now() << std::endl;
	  printTransform(newTF);
	  br.sendTransform(tf::StampedTransform(newTF, ros::Time::now(), tfParent, tfChannel));
	}
	
	//=============================================================================
	tf::Transform odometryComm::eigenToTransform(Eigen::Matrix4f tMatrix) {
	  Eigen::Matrix4d md(tMatrix.cast<double>());
	  Eigen::Affine3d affine(md);
	  tf::Transform result;
 	  tf::TransformEigenToTF(affine, result);
	  
	  return result;
	}

	//=============================================================================
	double odometryComm::transformToDistance(tf::Transform t){
		tf::Vector3 origin(t.getOrigin());
		double sqDistance = 0;
		for (int i=0; i< 3; i++)
			sqDistance = origin[i] * origin[i] + sqDistance;
	    return sqrt(sqDistance);
	}

	//=============================================================================
	double odometryComm::transformToRotation(tf::Transform t){
		double rotSqr = t.getRotation().getW()*t.getRotation().getW();
		double diff = double(1.00000) - rotSqr;
		// If the diff between 1.0 and rotSqr is too small
		//then it is considered 0.0
		if (diff < rotationAccuracy)
			return double(0.0);
//		printf("ROTATION %f: \r\n sqrt(1-%f) = sqrt(%f) = %f != %f\r\n\r\n", t.getRotation().getW(), rotSqr, double(1.0)-rotSqr, sqrt(double(1.0)-rotSqr));
	    return diff;
	}

	//=============================================================================
	double odometryComm::matToDist(Eigen::Matrix4f t){
	    return sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
	}

	//=============================================================================
	void odometryComm::matToXYZ(Eigen::Matrix4f t, double& x, double& y, double& z){
		x = t(2,3); y = t(0,3); z = t(1,3);
	}


	//=============================================================================
	void odometryComm::matToRPY(Eigen::Matrix4f t, double& roll, double& pitch, double& yaw) {
	    roll = atan2(t(2,1),t(2,2));
	    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
	    yaw = atan2(t(1,0),t(0,0));
	}
	
	
	//////////////////////////////////////////////////////
	///////// CONVERTERS FROM TRANSFORMATIONS ////////////
	//////////////////////////////////////////////////////
	nav_msgs::Odometry odometryComm::transformToOdometry(tf::Transform tMatrix, ros::Time stamp, std::string frameID, std::string childID) {
		nav_msgs::Odometry odom_msg_output;

		odom_msg_output.header.stamp = stamp;
		odom_msg_output.header.frame_id = frameID;
		odom_msg_output.child_frame_id = childID;

		// Translation data
		odom_msg_output.pose.pose.position.x = tMatrix.getOrigin().getX();
		odom_msg_output.pose.pose.position.y = tMatrix.getOrigin().getY();
		odom_msg_output.pose.pose.position.z = tMatrix.getOrigin().getZ();
		// Rotation data
		odom_msg_output.pose.pose.orientation.x = tMatrix.getRotation().getX();
		odom_msg_output.pose.pose.orientation.y = tMatrix.getRotation().getY();
		odom_msg_output.pose.pose.orientation.z = tMatrix.getRotation().getZ();
	//	odom_msg_output.pose.pose.orientation.w = rw;

		return odom_msg_output;
	}

	//=============================================================================
	tfMessage odometryComm::transformToTFMsg(tf::Transform tMatrix, ros::Time stamp, std::string frameID, std::string childID) {
		geometry_msgs::TransformStamped tf2_msg_output;

		tf2_msg_output.header.stamp = stamp;
		tf2_msg_output.header.frame_id = frameID;
		tf2_msg_output.child_frame_id = childID;
		
		// Translation data
		tf2_msg_output.transform.translation.x = tMatrix.getOrigin().getX();
		tf2_msg_output.transform.translation.y = tMatrix.getOrigin().getY();
		tf2_msg_output.transform.translation.z = tMatrix.getOrigin().getZ();

		// Rotation data
		tf2_msg_output.transform.rotation.x = tMatrix.getRotation().getX();
		tf2_msg_output.transform.rotation.y = tMatrix.getRotation().getY();
		tf2_msg_output.transform.rotation.z = tMatrix.getRotation().getZ();

		tfMessage newTFMessage;
		newTFMessage.transforms.push_back(tf2_msg_output);
		return newTFMessage;
	}


	
	
	
	
	
	/////////////////////////////////////////////////////////
	//////////// MAIN ALGORITHM PROCESSING //////////////////
	/////////////////////////////////////////////////////////
	Eigen::Matrix4f odometryComm::process2CloudsICP(PointCloudT::Ptr &cloud_initial, PointCloudT::Ptr &cloud_final, double *final_score_out) {
		// If no hint passed, an identity matrix is the hint.
		Eigen::Matrix4f neutralHint = Eigen::Matrix4f::Identity();
		return process2CloudsICP(cloud_initial, cloud_final, neutralHint, final_score_out);
	}

	Eigen::Matrix4f odometryComm::process2CloudsICP(PointCloudT::Ptr &cloud_initial, PointCloudT::Ptr &cloud_final, Eigen::Matrix4f &hint, double *final_score_out) {
		if (cloud_final == 0 || cloud_initial == 0) {
			ROS_INFO("Empty clouds received; returning!"); //ROS_WARN
			return Eigen::Matrix4f::Identity();
		}

		pcl::IterativeClosestPoint<PointT, PointT> icp;
		PointCloudT cloud_aligned; // The final cloud though it will be ignored

		//*****************************
		// ** Main processing
		//*****************************
			double ini_time = ros::Time::now().toSec();
			// Sets both clouds into the algorithm
			icp.setInputCloud(cloud_initial); icp.setInputTarget(cloud_final);

			// 
			if (maxIterations > 0 && epsilon > 0 && maxDistance > 0) {
				icp.setMaximumIterations(maxIterations);
				icp.setTransformationEpsilon(epsilon);
				icp.setMaxCorrespondenceDistance(maxDistance);
			}
			else
				ROS_ERROR("The algorithm is supposed to be run with positive values for the MAIN parameters but one or several aren't: maxIterations = %d, epsilon = %f, maxDistance = %f", maxIterations, epsilon, maxDistance);
			std::cout << std::endl << std::endl;
			if (euclideanDistance > 0)
				icp.setEuclideanFitnessEpsilon (euclideanDistance);
			else
				printf("euclideanDistance is: %E\r\n", icp.getEuclideanFitnessEpsilon());
			if (maxRansacIterations != 0)
				icp.setRANSACIterations (maxRansacIterations);
			else
				printf("maxRansacIterations is: %E\r\n", icp.getRANSACIterations());
			if (ransacInlierThreshold > 0)
				icp.setRANSACOutlierRejectionThreshold (ransacInlierThreshold);
			else
				printf("ransacInlierThreshold is: %E\r\n", icp.getRANSACOutlierRejectionThreshold());
			std::cout << std::endl << std::endl;
			
			
// **********************************************************			
// ************** Trying 3 hints version *********************
// **********************************************************

// Run the calculations (returns nothing but copies in "cloud_aligned" the
//input PC (cloud_initial) transformed accordingly to the result
// Method specification:			icp.align(cloud_aligned, const Eigen::Matrix4f &guess); // If there's any guess
			
			
			Eigen::Matrix4f final_result, guess;
			double final_score = 1;

			// Without any guess:
			icp.align(cloud_aligned);
			if (icp.hasConverged() && icp.getFitnessScore() < final_score) {
				final_score = icp.getFitnessScore();
				final_result = icp.getFinalTransformation();
				std::cout << "Score after first try is " << final_score << std::endl;
			}

			// Guessing negative pitch:
			generate_tf(guess, 0, -0.1, 0);
			icp.align(cloud_aligned, guess); // If there's any guess
			if (icp.hasConverged() && icp.getFitnessScore() < final_score) {
				std::cout << "Result improved with -pitch from " << final_score << " to " << icp.getFitnessScore() << std::endl;
				final_score = icp.getFitnessScore();
				final_result = icp.getFinalTransformation();
			}
			
			// Guessing positive pitch:
			generate_tf(guess, 0, 0.1, 0);
			icp.align(cloud_aligned, guess);
			if (icp.hasConverged() && icp.getFitnessScore() < final_score) {
				std::cout << "Result improved with +pitch from " << final_score << " to " << icp.getFitnessScore() << std::endl;
				final_score = icp.getFitnessScore();
				final_result = icp.getFinalTransformation();
			}


			
			
			
			
			
			
			
			
			
			
			
			newTransf.generate_tf(0, P*1.2, 0); // Real Yaw is cloud's Pitch
			newTransf.setOrigin(tf::Vector3(0, 0, 0));
			pcl_ros::transformAsMatrix(newTransf, hint);

			
			// Guessing with HINT
			if (hint == Eigen::Matrix4f::Identity()) {
				std::cout << "No hint received, creating one." << std::endl;
				// Getting the Transform version of the matrix for the calculations
				tf::Transform newTransf = eigenToTransform(final_result);
				
				// Storing the current original values before the modifications
				double R, P, Y;			newTransf.getBasis().getRPY(R, P, Y); // Unused for now. This values remain
				tf::Vector3 origin = newTransf.getOrigin(); // tf::Vector3(0, 0, 0);
				
				// Generate the new tf and apply rules
				origin = tf::Vector3(0, 0, 0);
//				btVector3 origin(0, 0, 0); // Changing origin of coordinates ((tf::Vector3(0, 0, 0)))

				newTransf.generate_tf(0, P*1.2, 0); // Real Yaw is cloud's Pitch
				newTransf.setOrigin(origin);
				
				// Back to the matrix version for the hint
				pcl_ros::transformAsMatrix(newTransf, hint);
			}
			else std::cout << "Using passed hint." << std::endl;
			icp.align(cloud_aligned, hint); // If there's any guess
			if (icp.hasConverged() && icp.getFitnessScore() < final_score) {
				std::cout << "Result improved with hint00 from " << final_score << " to " << icp.getFitnessScore() << std::endl;
				final_score = icp.getFitnessScore();
				final_result = icp.getFinalTransformation();
			}
			std::cout << "Score after HINT00 is " << icp.getFitnessScore() << std::endl;
			std::cout << std::flush;
			
			// NOTE: I REALLY would use the last correct estimation as a hint.
			
			
			
			
			
			
			
			
			
			
			
			// Just in case the cloud_aligned is published
			//is better to tag it as timestamp==0 to avoid problems
			cloud_aligned.header.stamp = ros::Time(0);
			double total_time = ros::Time::now().toSec()-ini_time;

			
			if (final_score < 1.0) { // final-score is '1' unless at least one test converged and assigned a new value
				std::cout << "has converged with score: " << final_score << " after " << total_time << " for " << cloud_aligned.points.size() << " points." << std::endl;
			}
			else
				std::cout << "has NOT converged after " << total_time << std::endl;
			// Output the final score:
			*final_score_out = final_score;
		return final_result;
	}
	
	
	// Alternative transformation estimator with "SVD"-based algorithm
	Eigen::Matrix4f odometryComm::estimateTransformation(PointCloudT::Ptr &cloud_initial, PointCloudT::Ptr &cloud_final) {
		if (cloud_final == 0 || cloud_initial == 0) {
			ROS_INFO("Empty clouds received; returning!"); //ROS_WARN
			return Eigen::Matrix4f::Identity();
		}
		
		
//		pcl::registration::TransformationEstimation<PointT, PointT> MyEstimation;
		pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
		
		//*****************************
		// ** Main processing
		//*****************************
		Eigen::Matrix4f transformationResult;
		svd.estimateRigidTransformation(*cloud_initial, *cloud_final, transformationResult);

		return transformationResult;
	}
	

	
	
	
	////////////////////////////////////////////////////////////
	//////////// SERVICE HANDLERS AND LAUNCHER /////////////////
	////////////////////////////////////////////////////////////
	
	bool odometryComm::fill_in_answer(my_odometry::odom_answer &res) {
		res.globalTF = transformToTFMsg(globalTF);
		res.relativeTF = transformToTFMsg(lastRelativeTF);
		res.globalOdometry = transformToOdometry(globalTF);
		res.relativeOdometry = transformToOdometry(lastRelativeTF);
		res.statusCode = odomStatus;

		return true;
	}

	bool odometryComm::update_odometry(	my_odometry::odom_update_srv::Request  &req,
							my_odometry::odom_update_srv::Response &res ) {
		ROS_INFO("Request for update received. Reading from %s", inputPCL_topic.c_str());
		
// Running algorithm:
		// Fetch a new pointcloud
		PointCloudT::ConstPtr cloud_msg = fetchPointCloud(inputPCL_topic);
		common_callback_routine(cloud_msg, inputPCL_topic);
		
		// Writting the answer
		return fill_in_answer(res.answer);
	}

	//=============================================================================
	bool odometryComm::get_last_status(	my_odometry::emptyRequest::Request  &req,
							my_odometry::emptyRequest::Response &res ) {
		ROS_INFO("Request for status received, sending back empty response");
		return fill_in_answer(res.answer);
	}

	//=============================================================================
	bool odometryComm::reset_globals(my_odometry::emptyRequest::Request  &req,
			my_odometry::emptyRequest::Response &res ) {
		boost::mutex::scoped_lock lock(callback_mutex); // Lock
		ROS_INFO("Request reset global measures received.");
		previous=PointCloudT::Ptr(new PointCloudT);
		globalTF = tf::Transform::getIdentity();
		lastRelativeTF =  tf::Transform::getIdentity();
		
		// This can be done or not as it's not a meassure but a config
		  fixed_camera_transform = tf::Transform::getIdentity();
		  rotate_tf(fixed_camera_transform, kinectRoll, kinectPitch, kinectYaw);
		  std::cout << "***initial camera fixed tf:" << std::endl; printTransform(fixed_camera_transform);
	
		  return fill_in_answer(res.answer);
	}
	

	// Callbacks!!

	void odometryComm::cameraTF_callback(const tf::tfMessageConstPtr& newTF) {
		const tf::tfMessage& msg_in = *newTF;
		tf::transformMsgToTF(msg_in.transforms[0].transform, fixed_camera_transform);

		std::cout << "***new camera fixed tf:" << std::endl; fixed_camera_transform.printTransform();
	}
	
	///////////////////////////////////////////////////////////
	//////////// POINTCLOUD RECEPTION CALLBACK ////////////////
	///////////////////////////////////////////////////////////
	void odometryComm::PCL_callback(const PointCloudT::ConstPtr & cloud_msg) {
		boost::mutex::scoped_lock lock(callback_mutex); // Lock
		
		// It is necessary to copy everything in a new variable as the original
		//cloud is constant.
		PointCloudT::ConstPtr pointCloud1_aux(cloud_msg);
		
		// Notice: that cloud_msg is constant and the parameter inside of
		//the method is not, this is necessary for the routine to work.
		common_callback_routine(pointCloud1_aux, inputPCL_topic);
	}

	//////////////////////////////////////////////////////////
	//////////// STR_REQUEST!! RECEPTION CALLBACK ////////////
	//////////////////////////////////////////////////////////
	void odometryComm::STR_callback(const std_msgs::String::ConstPtr & nextTopic) {
		boost::mutex::scoped_lock lock(callback_mutex); // Lock
		
		ROS_INFO("Requested cloud at %f.", static_cast<double>(ros::Time::now().toSec()));

		// Fetch new pointcloud from the topic specified in the request
		PointCloudT::ConstPtr cloud_msg = fetchPointCloud (nextTopic->data);

		// Notice: that cloud_msg is constant and the parameter inside of
		//the method is not, this is necessary for the routine to work.
		common_callback_routine(cloud_msg, nextTopic->data);
	}

	//////////////////////////////////////////////////////////
	////////////	ACTUAL CALLBACK ROUTINE		////////////
	//////////////////////////////////////////////////////////
	void odometryComm::common_callback_routine(PointCloudT::ConstPtr & pointCloud1_aux, std::string nextTopic) {

		// The cloud must be accessible and have points as a precondition
		if (!(pointCloud1_aux != 0 && pointCloud1_aux->points.size() > 0)) {
			ROS_INFO("Empty PointCloud or Null pointer received.");
			return; // Nothing can be done without points
		}
		
		// If "ignoreTimestamp" is false and the timestamp is old,
		//the cloud is rejected and a new one is fetched.
		if (ignoreTimestamp==false) {
			double maxTime = 2.0; // TODO: this must end up being an external parameter
			while (pointCloud1_aux->header.stamp.toSec() < ros::Time::now().toSec()-maxTime) {
				ROS_INFO("Old PCL received; asking for a new one.");
				pointCloud1_aux=fetchPointCloud (nextTopic);
			}
		}
		
		ROS_INFO("Got cloud with timestamp %f.", static_cast<double>(pointCloud1_aux->header.stamp.toSec()));
		PointCloudT::Ptr pointCloud1_last(new PointCloudT);

		// The point cloud is filtered at the beginning
		apply_filters(pointCloud1_aux, pointCloud1_last);

		pointCloud1_aux=pointCloud1_last;

		// If no valid points after filtering, the PC is not valid itself
		if (pointCloud1_last->points.size() < 10) {
			ROS_INFO("Not enough points after the filtering. Need better more data.");
			return; // Nothing can be done without points
		}
		// Timestamp set to 0 for RViz not to go crazy about it
		pointCloud1_last->header.stamp = ros::Time(0);
		
		
		
		
		
		
		
		
		
		
		
		
		// If no previous PointCloud, the one read is stored
		//(already filtered) as previous and the callback ends
		if (previous == 0 || previous->points.size() < 1) {
			ROS_INFO("No previous PointCloud. First PointCloud read.");
			trimPreviousCloud(pointCloud1_last, cloud_trim_x, cloud_trim_y, cloud_trim_z);
			previous = pointCloud1_last;
			// Status waiting
			setOdomStatus(WAITING);
			return;
		} // else:
				
		
		// Actual processing
		double final_score = 1.0;
//		Eigen::Matrix4f tf_matrix_result = process2CloudsICP(previous, pointCloud1_last, &final_score);
		Eigen::Matrix4f tf_matrix_result = estimateTransformation(previous, pointCloud1_last);
		
		tf::Transform tf_result = eigenToTransform(tf_matrix_result);
		
		
//***********************************************************************************************
//**************** In case it is needed to make any changes in the transform
		filter_resulting_TF(tf_result);
//***********************************************************************************************
		//WATCH OUT: from here on, the transform doesn't represent the relation
		//between the clouds anymore. So in order to publish the aligned clouds,
		//the "back_to_cloud_matrix" method must be used.
//***********************************************************************************************

		
		// In case the lineal distance is bigger than the maximum, it's unlikely
		//that the result is correct since either the maximum is false or the
		//algorithm failed.
		ROS_WARN("Raw distance/rotation: %f / %f", transformToDistance(tf_result), transformToRotation(tf_result));
		if (transformToDistance(tf_result) > maxDistance || final_score > ICPMinScore) {
			ROS_INFO("UNRELIABLE RESULT with distance %f > maxDistance OR score %f > ICPMinScore", transformToDistance(tf_result), final_score);
			
			// Status running
			setOdomStatus(UNRELIABLE_RESULT);
		}
		else setOdomStatus(WAITING);

		if (transformToDistance(tf_result)  == 0) ROS_WARN("Position has NOT changed!");
		if (transformToRotation(tf_result)  == 0) ROS_WARN("Orientation has NOT changed!");

		// ONLY if there are changes (any movement), then:
		if (transformToDistance(tf_result)  != 0 || transformToRotation(tf_result)  != 0) {
			// Calculate the new aligned cloud:
			PointCloudT::Ptr pointCloud1_aligned(new PointCloudT);
			pcl_ros::transformAsMatrix(tf_result, tf_matrix_result);
			pcl::transformPointCloud (*previous, *pointCloud1_aligned, tf_matrix_result);

			// Publish all the results
			pcl_pub_initial.publish(previous);
			pcl_pub_final.publish(pointCloud1_last);
			pcl_pub_aligned.publish(pointCloud1_aligned);
			

			
			// ****************************************
			// ** Scary test!!
//			std::cout << "ATENTION!!!! *********************************" << std::endl;
//			Eigen::Matrix4f mat_res2 = process2CloudsICP(pointCloud1_aligned, pointCloud1_last);
//			tf::Transform tf_result2 = eigenToTransform(mat_res2);
//			printTransform(tf_result2);
			// ****************************************

			// NOTE: About which pointcloud to store for comparison:
			//-I store the last REAL cloud because I'm sure that it is correct and so it will be the next estimation, then
			//-But if the LAST ESTIMATION is wrong, the error will be accumulated while it could be balance if the aligned
			//versión was used.
			// Another problem for this is that it's not clear there will be too much difference between the clouds if one
			//is lost.
			trimPreviousCloud(pointCloud1_last, cloud_trim_x, cloud_trim_y, cloud_trim_z);
			// The last PointCloud is stored as the previous one for the next call
			previous = pointCloud1_last;

			// The "tf_matrix_result" transform is the last movement known relatively to the
			//last position, the "globalTF" is the total accumulated position.
			lastRelativeTF = tf_result;
			globalTF = globalTF * tf_result;
			
			
			// In the end, both known transforms are published both as transforms and
			//odometry messages.
			broadcastTransform(globalTF, "globalTF");
			broadcastTransform(lastRelativeTF, "relativeTF");
			
			my_odometry::odom_answer newAnswer;
			fill_in_answer(newAnswer);
			odom_answer_publisher.publish(newAnswer);
		}
	}

//=============================================================================
} //namespace
//=============================================================================
