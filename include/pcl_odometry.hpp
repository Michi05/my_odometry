
//=============================================================================
#ifndef PCL_ODOMETRY_HPP
#define PCL_ODOMETRY_HPP
//=============================================================================


//=============================================================================
//Includes
//=============================================================================

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h> // (de)serialize ros messages directly
#include <pcl/filters/filter.h>
// May24
#include <boost/thread.hpp>
#include <string>
#include <pcl_odometry.hpp>
// Jul02
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <cmath> // For the float std::abs()

// Message types for services
#include "my_odometry/emptyRequest.h"
#include "my_odometry/odom_update_srv.h"
#include "my_odometry/statusMsg.h"
#include "my_odometry/odom_answer.h"



//=============================================================================
//Namespace
//=============================================================================
namespace pcl_odometry {





	////////////////////////////////////////////////////
	//////////// GLOBAL SCOPE DECLARATIONS /////////////
	////////////////////////////////////////////////////
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef tf2_msgs::TFMessage tfMessage; // alternative is "tf::tfMessage"





////////////////////////////////////////////////////////////////////////////////////////
// ////////////////// odometryComm class for odometry calculations //////////////////////
// This class is meant to read Point Cloud messages and execute ICP algorithm to estimate
//the transformations between each couple of them.
// Usage: A call to the complete constructor includes the 3 algorithm parameters
// "int maxOfIterations, double epsilonValue, double maxTransformDistance" while the
// other one automatically reads the parameters from the parameter server.
/*
*
* The init() and shutdown() methods control the life cycle of the object.
* Note that shutdown() is always called on destruction.
*
* Optional parameters for the node are the following:
*
*   topic:           topic to which to publish the teleop state
*   ...
*   */
////////////////////////////////////////////////////////////////////////////////////////
class odometryComm {
public:
	  //Parameter keys
	  static const char PARAM_KEY_ICP_ITERATIONS[];
	  static const char PARAM_KEY_ICP_DISTANCE[];
	  static const char PARAM_KEY_ICP_EPSILON[];
	  static const char PARAM_KEY_ICP_EUCLIDEAN_DISTANCE[];
	  static const char PARAM_KEY_ICP_RANSAC_ITERATIONS[];
	  static const char PARAM_KEY_ICP_RANSAC_THRESHOLD[];

	  static const char PARAM_KEY_CLOUD_TRIM_X[];
	  static const char PARAM_KEY_CLOUD_TRIM_Y[];
	  static const char PARAM_KEY_CLOUD_TRIM_Z[];

	  static const char PARAM_KEY_RESOLUTION_LEAF_SIZE[];
	  static const char PARAM_KEY_RESOLUTION_FILTERING[];

	  static const char PARAM_KEY_MIN_DEPTH[];
	  static const char PARAM_KEY_MAX_DEPTH[];
	  static const char PARAM_KEY_DEPTH_FILTERING[];

	  static const char PARAM_KEY_NOISE_NEIGHBOURS[];
	  static const char PARAM_KEY_NOISE_RADIUS[];
	  static const char PARAM_KEY_NOISE_FILTERING[];

	  static const char PARAM_KEY_TOPIC_INPUT_STR[];
	  static const char PARAM_KEY_TOPIC_INPUT_PCL[];
	  static const char PARAM_KEY_TOPIC_INPUT_CAMERA_TF[];
	  static const char PARAM_KEY_TOPIC_OUTPUT_GLOBAL_ODOMETRY[];
	  static const char PARAM_KEY_TOPIC_OUTPUT_RELATIVE_ODOMETRY[];

	  static const char PARAM_KEY_TOPIC_OUTPUT_ALIGNED_CLOUD[];
	  static const char PARAM_KEY_TOPIC_OUTPUT_INITIAL_CLOUD[];
	  static const char PARAM_KEY_TOPIC_OUTPUT_FILTERED_CLOUD[];
	  static const char PARAM_KEY_TOPIC_ODOMETRY_ANSWER[];

	  static const char PARAM_KEY_MANUAL_MODE[];
	  static const char PARAM_KEY_IGNORE_TIME[];
	  static const char PARAM_KEY_ACCURACY[];

	  static const char PARAM_KEY_KINECT_ROLL[];
	  static const char PARAM_KEY_KINECT_PITCH[];
	  static const char PARAM_KEY_KINECT_YAW[];
	  
	  //Parameter default values
	  static const int PARAM_DEFAULT_ICP_ITERATIONS;
	  static const double PARAM_DEFAULT_ICP_DISTANCE;
	  static const double PARAM_DEFAULT_ICP_EPSILON;
	  static const double PARAM_DEFAULT_ICP_EUCLIDEAN_DISTANCE;
	  static const int PARAM_DEFAULT_ICP_RANSAC_ITERATIONS;
	  static const double PARAM_DEFAULT_ICP_RANSAC_THRESHOLD;

	  static const double PARAM_DEFAULT_CLOUD_TRIM_X;
	  static const double PARAM_DEFAULT_CLOUD_TRIM_Y;
	  static const double PARAM_DEFAULT_CLOUD_TRIM_Z;
	  
	  static const double PARAM_DEFAULT_RESOLUTION_LEAF_SIZE;
	  static const bool PARAM_DEFAULT_RESOLUTION_FILTERING;
	  
	  static const double PARAM_DEFAULT_MIN_DEPTH;
	  static const double PARAM_DEFAULT_MAX_DEPTH;
	  static const bool PARAM_DEFAULT_DEPTH_FILTERING;
	  
	  static const int PARAM_DEFAULT_NOISE_NEIGHBOURS;
	  static const double PARAM_DEFAULT_NOISE_RADIUS;
	  static const bool PARAM_DEFAULT_NOISE_FILTERING;

	  static const char PARAM_DEFAULT_TOPIC_INPUT_STR[];
	  static const char PARAM_DEFAULT_TOPIC_INPUT_PCL[];
	  static const char PARAM_DEFAULT_TOPIC_INPUT_CAMERA_TF[];
	  static const char PARAM_DEFAULT_TOPIC_OUTPUT_GLOBAL_ODOMETRY[];
	  static const char PARAM_DEFAULT_TOPIC_OUTPUT_RELATIVE_ODOMETRY[];

	  static const char PARAM_DEFAULT_TOPIC_OUTPUT_ALIGNED_CLOUD[];
	  static const char PARAM_DEFAULT_TOPIC_OUTPUT_INITIAL_CLOUD[];
	  static const char PARAM_DEFAULT_TOPIC_OUTPUT_FILTERED_CLOUD[];
	  static const char PARAM_DEFAULT_TOPIC_ODOMETRY_ANSWER[];

	  static const bool PARAM_DEFAULT_MANUAL_MODE;
	  static const bool PARAM_DEFAULT_IGNORE_TIME;
	  static const double PARAM_DEFAULT_ACCURACY;


	  static const double PARAM_DEFAULT_KINECT_ROLL;
	  static const double PARAM_DEFAULT_KINECT_PITCH;
	  static const double PARAM_DEFAULT_KINECT_YAW;
	
	
	

	//////////////////////////////////////////////////////////////////////
	//////////// PARAMETERS FOR THE FILTERS AND ALGORITHM ////////////////
	//////////////////////////////////////////////////////////////////////

	///////// ICP ALGORITHM (from the IterativeClosestPoint Detailed Description):
		// Maximum amount of iterations of the ICP algorithm for approaching one group of points to another.
	int maxIterations;
	// Maximum distance between the homolog points after the transformation.
	double maxDistance;
	// Maximum epsilon (difference) between the previous transformation and the current estimated transformation.
	double epsilon;
	// The sum of Euclidean squared errors between the pairs of points.
	double euclideanDistance;
	// Maximum amount of iterations for each execution of the RANSAC algorithm
	int maxRansacIterations;
	// Maximum distance between the points considered inliers for the RANSAc estimation.
	double ransacInlierThreshold;

	///////// x,y,z values to TRIM POINTCLOUD CLOUD
	// Ratios (0 to 1) of the previous point cloud for
	//ICP to use in the matching with the next cloud.
	double cloud_trim_x, cloud_trim_y, cloud_trim_z;

	///////// RESOLUTION FILTER ("VoxelGrid" class):
	bool doDownsampleFiltering; // To turn it on an off
	double leafSize;
	// The number determines the accuracy of the PCL; the smaller the number, the higher the amount of points

	///////// DISTANCE FILTER ("PashThrough" class):
	bool doDepthFiltering; // To turn it on an off
	double minDepth, maxDepth;
	// Depth distance boundaries for filtering points that are too far or too close.

	///////// NOISE FILTER ("RadiusOutlierRemoval"):
	bool doNoiseFiltering; // To turn it on an off
	int neighbours;
	// Minimum amount of neighbour points for a point to be considered "alone" and classify it as noise.
	double radius;
	// Maximum radius distance in which to look for the above neighbours.
	
	///////// PUBLIC TOPIC NAMES:
	std::string inputStrRequest_topic;
	std::string inputPCL_topic;
	std::string cameraTF_topic;
	std::string outputGlobalOdometry_topic;
	std::string outputRelativeOdometry_topic;
	
	std::string outputAlignedCloud_topic;
	std::string outputInitialCloud_topic;
	std::string outputFilteredCloud_topic;
	std::string outputOdometryAnswer_topic;
	
	///////// PCL_Odometry Configuration:
	bool manualMode;
	bool ignoreTime;
	double measureAccuracy;

	//////// Physical Description Configuration:
	double kinectRoll,kinectPitch, kinectYaw;
	
	
	////////////////////////////////////////////////////
	//////////// CLASS SCOPE DATA MEMBERS //////////////
	////////////////////////////////////////////////////
	// TODO: document as well??
	boost::mutex callback_mutex;
	Eigen::Matrix4f globalTransform, lastRelativeTransform;
	tf::Transform globalTF, lastRelativeTF, fixed_camera_transform;
//	ros::Publisher global_odom_publisher, relative_odom_publisher;
	ros::Publisher pcl_pub_aligned, pcl_pub_final, pcl_pub_initial, odom_answer_publisher;
	ros::Subscriber pcl_sub, camera_tf_sub;
	PointCloudT::Ptr previous;
	
	/** Is initialised flag */
	bool mIsInitialised;
	
	/** Mutex to protect is initialised flag */
	boost::recursive_mutex mIsInitialisedMutex;

	/** Spinner to receive ROS events */
	ros::AsyncSpinner *mSpinner; //MUST!! be the last one
	//So it isn't initialized before the mutex!!
	
	
	/** Services **/
	//TODO: group services or structure them in a coherent way
	ros::ServiceServer *server_resetGlobals;
	// Status of the odometry process
	int odomStatus;
	//...still in need of some agreement...
	// - Option 1: -1=error, 0=waiting, 1=busy (this is the one being used)
	// - Option 2: binary interpret: waiting|error. 00
	// (( e.g. 00 == working, no error // 11 = waiting for next, error found
	// - Option 3: obviously several flags can be used separately according to needs
	
	////////////////////////////////////////////////////////////////////
	//////////////////////// CLASS CONSTRUCTORS ////////////////////////
	////////////////////////////////////////////////////////////////////	
	
	odometryComm();
	//=============================================================================
	~odometryComm();
	
	

	  /**
	   * Initialise object.  If object is already initialised it is shutdown and
	   * reinitialised.
	   *
	   *   @param argc - number of command line arguments to process
	   *   @param argv - command line arguments
	   *   @param nodeName - node name
	   *   @param initOptions - init options
	   *
	   *   @return true on success
	   */
	bool init(int argc, char** argv, std::string nodeName, uint32_t rosInitOptions=0);

	  /**
	   * Shutdown object.  If object is already shutdown this has no effect.  This
	   * method always cleans up as much as possible, even if there are errors.
	   * This method is always called on destruction.
	   *
	   *   @return true on success
	   */
	bool shutdown();
		
		
		
		
		
		
		
		
		
		

	
	
	////////////////////////////////////////////////////////////////////
	//////////////////////// CLASS METHODS ////////////////////////
	////////////////////////////////////////////////////////////////////

	
//////////////////////////////////////////////////////
///////////// PUBLIC CONTROL METHODS /////////////////
//////////////////////////////////////////////////////

	  /**
	   * Activate the downsample filter (that is, put
	   * the flag as "true" and sets it according to the parameters
	   *
	   *   @param newLeafSize - distance between sample points in the
	   *   						point cloud after downsampling.
	   */
	void setDownsampleFiltering(float newLeafSize);
	  /**
	   * Activate the distance filter (that is, put
	   * the flag as "true" and sets it according to the parameters
	   *   @param minimum/maximum - allowed min/max values of the
	   *   							points kept in the cloud.
	   */
	void setDistanceFiltering(float minimum, float maximum);
	  /**
	   * Activate the noise filter (that is, put
	   * the flag as "true" and sets it according to the parameters
	   *   @param radDist - maximums radius distance in (...TODO: meters?...)
	   *					between each point and the others for them to be
	   *					considered neighbours.
	   *   @param noOfNeighbours - minimum number of neighbours for a
	   *						point to be considered not-alone and
	   *						keep it.
	   */
	void setNoiseFiltering(float radDist, int noOfNeighbours);

	
	
	
	
	////////////////////////////////////////////////////////
	///////////// PRIVATE AUXILIAR METHODS /////////////////
	////////////////////////////////////////////////////////
private:

	  /**
	   * Outputs, through the console, the list of current avaliable
	   * parameters and their current values.
	   */
	void printConfiguration();

    
	  /**
	   * Publishes an odometry message to the ROS environment. (TODO: something else to say?)
	   *   
	   *   @param odom_publisher - the topic publisher object in which to
	   *   							put the message in order to send it.
	   *   @param tMatrix - the tf::Transform from which to read the data for publishing.
	   */
	void publishOdom(ros::Publisher &odom_publisher, tf::Transform tMatrix);
	
	  /**
	   * Fetches a new cloud from the passed topic as inmediately as possible
	   * (as it is suppossed to be the newest existing one).
	   * It doesn't check the timestamp so that's a responsibility of the calling method.
	   *   
	   *   @param topicName - the name of the topic from which to fetch the PC.
	   *   @param timeout - time after which to give up if no PC is found.
	   *
	   *   @return A constant pointer to the fetched pointcloud if possible or a
	   *   null pointer if the timeout runs out.
	   */
	PointCloudT::ConstPtr fetchPointCloud (std::string topicName, int timeout = 20);

	  /**
	   * Applies the different filters on the input point cloud and stores the result
	   * in the output PC.
	   *   
	   *   @param pointCloud1_in - input PointCloud for filtering
	   *   @param pointCloud1_out - reference for storing the new resulting
	   *   							PointCloud.
	   */
	void apply_filters(PointCloudT::ConstPtr pointCloud1_in, PointCloudT::Ptr &pointCloud1_out);

	// TODO: Document
	void trimPreviousCloud(PointCloudT::Ptr &pointCloud1_out, double x = 0.75, double y = 0.9, double z = 0.9);

	  /**
	   * Receives a TF and cleans the data in different possible ways:
	   *  - Removes measures smaller than the expected accuracy (as they
	   *  are not reliable).
	   *  - Changes the coordinates if needed in order to be according to
	   *  the receiver (RViz visor in this case)
	   *   
	   *   @param tf_result - the TF data stored in a tf::Transform
	   */
	void filter_resulting_TF(tf::Transform &tf_result);
	
	// TODO: Document
	double round(double value, int noDecimals);
	void round_near_zero_values(tf::Transform &tf_result, double margin);
	bool generate_tf(tf::Transform &tf, double roll, double pitch, double yaw);	void rotate_tf(tf::Transform &tf_result, double roll, double pitch, double yaw);
	void rotate_tf(tf::Transform &tf_result, tf::Transform &fixedTF);
	void printTransform(tf::Transform &newTF);
	void printMatrix4f(Eigen::Matrix4f &tMatrix);



	//////////////////////////////////////////////////////
	///////// TRANSFORMATION FRAMES' METHODS /////////////
	//////////////////////////////////////////////////////

	  /**
	   * Publishes a transform to the ROS environment using a
	   * "TransformBroadcaster" object, so it's visible system-wide.
	   *   
	   *   @param tMatrix - the tf::Transform to be published.
	   *   @param tfChannel - The name (id) of the coordinate frame this
	   *   					transform defines (child).
	   *   @param tfParent - The name (id) of the coordinate frame in which
	   *   					the transform is defined. 
	   */
	void broadcastTransform(tf::Transform &newTF, std::string tfChannel, std::string tfParent="map");

	/**
	 * TODO: document this
	 */
	double transformToDistance(tf::Transform t);
	double transformToRotation(tf::Transform t);

	  /**
	   * Extracts the dist data from a transform frame passed as Matrix4f.
	   *   
	   *   @param t - transform frame from which to obtain data
	   */
	double matToDist(Eigen::Matrix4f t);
	  /**
	   * Extracts the XYZ data from a transform frame passed as Matrix4f.
	   *   
	   *   @param t - transform frame from which to obtain data
	   *   @param x/y/z - XYZ values described by the transform
	   */
	void matToXYZ(Eigen::Matrix4f t, double& x, double& y, double& z);
	  /**
	   * Extracts the RPY data from a transform frame passed as Matrix4f.
	   *   
	   *   @param t - transform frame from which to obtain data
	   *   @param roll/pitch/yaw - RPY values described by the transform
	   */
	void matToRPY(Eigen::Matrix4f t, double& roll, double& pitch, double& yaw);

	  /**
	   * Converts a transform frame 
	   * into Odometry/transform frame/tfMessage
	   *   
	   *   @param tMatrix - original tf::Transform to return in odometry format.
	   *   @param stamp - timestamp for the frame. 0 is understood as unknown
	   *   				or just irrelevant.
	   *   @param frameID - The name (id) of the coordinate frame in which
	   *   					the transform is defined. 
	   *   @param childID - The name (id) of the coordinate frame this
	   *   					transform defines (child).
	   *
	   *   @return the resulting tf with the same data with the new format or type
	   */
	nav_msgs::Odometry transformToOdometry(tf::Transform tMatrix, ros::Time stamp = ros::Time(0), std::string frameID = "my_odom_tf", std::string childID = "base_link");

	  /**
	   * Converts the transform frame stored as Matrix4f from the eigen library
	   * into Odometry/transform frame/tfMessage
	   *   
	   *   @param tMatrix - original tf matrix
	   *
	   *   @return the resulting tf with the same data with the new format or type
	   */
	tf::Transform eigenToTransform(Eigen::Matrix4f tMatrix);
	tf::Transform eigenToTransform(Eigen::Matrix4f tMatrix, bool overloaded);


	  /**
	   * Converts the transform frame stored as Matrix4f from the eigen library
	   * into Odometry/transform frame/tfMessage
	   *   
	   *   @param tMatrix - original tf::Transform frame
	   *   @param stamp - timestamp for the frame. 0 is understood as unknown
	   *   				or just irrelevant.
	   *   @param frameID - The name (id) of the coordinate frame in which
	   *   					the transform is defined. 
	   *   @param childID - The name (id) of the coordinate frame this
	   *   					transform defines (child).
	   *
	   *   @return the resulting tf with the same data with the new format or type
	   */
	tfMessage transformToTFMsg(tf::Transform tMatrix, ros::Time stamp = ros::Time(0), std::string frameID = "my_odom_tf", std::string childID = "base_link");

	
	
	
	
	
	/////////////////////////////////////////////////////////
	//////////// MAIN ALGORITHM PROCESSING //////////////////
	/////////////////////////////////////////////////////////
// Point Cloud main stuff

	  /**
	   * Calculates the transformation frame to describe the relative position between
	   * two given pointclouds using the ICP algorithm with the already parameters which
	   * where set at the beginning from the "init" method.
	   *   
	   *   @param cloud_initial - first PC for the algorithm. This one is suppossed to be the
	   *   					initial one in chronological order.
	   *   @param cloud_final - second PC for the algorithm, which should be the final
	   *   						position to which to compare with.
	   *
	   *   @return The transform frame representing hte relative position between the
	   *   two pointclouds stored in a Matrix4f.
	   */
	Eigen::Matrix4f process2CloudsICP(PointCloudT::Ptr &cloud_initial, PointCloudT::Ptr &cloud_final);
	



	
	
	
	

	////////////////////////////////////////////////////////////
	//////////// SERVICE HANDLERS AND LAUNCHER /////////////////
	////////////////////////////////////////////////////////////

	// TODO: document
	bool fill_in_answer(my_odometry::odom_answer &res);
	  /**
	   * Triggers the ICP calculantion of a new tf when requested from the corresponding service
	   * by using the last stored PCL and the first one received after the request.
	   *   
	   *   @param req - empty structure from the service request message.
	   *   @param res - variable reference to store the answer to the service.
	   *   				Stores the global and relative tfs both as TFMessage and Odometry,
	   *   				and also an integer for the status code after running the algorithm.
	   *
	   *   @return true if no error was found.
	   */
	bool update_odometry(	my_odometry::odom_update_srv::Request  &req,
							my_odometry::odom_update_srv::Response &res );
	  /**
	   * Sends the last status code as an answer to the corresponding service.
	   *   
	   *   @param req - empty structure from the service request message.
	   *   @param res - variable reference to store the answer to the service.
	   *   				Stores an integer for the status code after running the algorithm.
	   *
	   *   @return true if no error was found.
	   */
	bool get_last_status(	my_odometry::statusMsg::Request  &req,
							my_odometry::statusMsg::Response &res );
	  /**
	   * Reset the accumulative transform frame which stores the current position after
	   * accumulating several relative transforms.
	   *   
	   *   @param req - empty structure from the service request message.
	   *   @param res - variable reference to store the answer to the service.
	   *   				Stores a boolean according to the return value.
	   *
	   *   @return true if no error was found.
	   */
	bool reset_globals(	my_odometry::emptyRequest::Request  &req,
						my_odometry::emptyRequest::Response &res );

	// Callbacks!!
public:
	  /**
TODO:
	   */
	void cameraTF_callback(const tf::tfMessageConstPtr& newTF);

	///////////////////////////////////////////////////////////
	//////////// POINTCLOUD RECEPTION CALLBACK ////////////////
	///////////////////////////////////////////////////////////
	  /**
	   * Triggered when a new pointcloud is received. After that, the
	   * "common_callback_routine" method is called for finding the new tf.
	   *   
	   *   @param cloud_msg - new received PointCloud.
	   */
	void PCL_callback(const PointCloudT::ConstPtr & cloud_msg);


	//////////////////////////////////////////////////////////
	//////////// STR_REQUEST!! RECEPTION CALLBACK ////////////
	//////////////////////////////////////////////////////////
	  /**
	   * Triggered when a new request message is received. The callback fetches
	   * a single new PointCloud and calls the "common_callback_routine" method
	   * is called for finding the new tf.
	   *   
	   *   @param nextTopic - new received topic name.
	   */
	void STR_callback(const std_msgs::String::ConstPtr & nextTopic);


	/////////////////////////////////////////////////
	//////////// common_callback_routine ////////////
	/////////////////////////////////////////////////
	  /**
	   * These method triggers the calculantion of a new tf between the last
	   * and the second last received pointclouds.
	   *   
	   *   @param pointCloud1_aux - new PointCloud to filter and then apply the
	   *   							algorithm on it.
	   *   @param nextTopic - name of the topic from which to get new PCs if
							necessary (in case the received one is too old).
	   */
	void common_callback_routine(PointCloudT::ConstPtr & pointCloud1_aux, std::string nextTopic);
};

//=============================================================================
} //namespace
//=============================================================================


//=============================================================================
#endif //#ifndef PCL_ODOMETRY_HPP
//=============================================================================