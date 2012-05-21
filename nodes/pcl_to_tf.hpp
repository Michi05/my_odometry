#ifndef PCL_TO_TF_HPP
#define PCL_TO_TF_HPP

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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


// Main PCL handle Callback

void cloudIndicesModelCallback(const std_msgs::String::ConstPtr & nextTopic) {
	boost::mutex::scoped_lock lock(callback_mutex); // Lock
	PointCloudT::ConstPtr cloud_msg = fetchPointCloud (nextTopic->data, *nHandlePtr);
	PointCloudT::Ptr pointCloud1_BB(new PointCloudT), pointCloud1_A(new PointCloudT);

	if (!(cloud_msg != 0 && cloud_msg->points.size() > 0))
		ROS_INFO("Empty PointCloud or Pointer");
		return; // Nothing can be done without points

	//	std::stringstream ss;
	//	ss << "n/a";
		ROS_INFO("Got cloud with timestamp %li.", static_cast<long>(cloud_msg->header.stamp.toNSec()));

	// Downsample filtering
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud_msg);
	sor.setDownsampleAllData (true);
	sor.setLeafSize (leafSize, leafSize, leafSize);
	sor.filter (*pointCloud1_A);
//	std::cout << "Removed with <VoxelGrid>: " << sor.getRemovedIndices()->size();
	
//	std::cout << "with ratio = " << pointsRatio << " and leafSize = " << leafSize << ", got " << inputFiltered->points.size() << " of " << clud_msg->points.size() << std::endl << std::endl;

	// Create the filtering object
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (pointCloud1_A);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (minDepth, maxDepth);
	pass.filter (*pointCloud1_BB);
	
	/*
	pcl::RadiusOutlierRemoval<PointT> radius_outlier_removal;
	radius_outlier_removal.setInputCloud (pointCloud1_BB);
	// set radius for neighbor search
	radius_outlier_removal.setRadiusSearch (radius);
	// set threshold for minimum required neighbours
	radius_outlier_removal.setMinneighboursInRadius (neighbours);
	radius_outlier_removal.filter (*pointCloud1_A);
	*/
//	std::cout << "Removed with <removeNaN>: " << tempIntVec.size();
	
	if (pointCloud1_BB->points.size() < 1)
		return; // Nothing can be done without points
	
	if (previous != 0 && previous->points.size() > 0)
	{
//		pcl_pub.publish(pointCloud1_A);
		Eigen::Matrix4f tf_result = processCloud(pointCloud1_BB, previous);
		pcl_pub.publish(previous);
	    GlobalTransform = tf_result * GlobalTransform;
		broadcastTransform(GlobalTransform);

//		publishOdom(odom_pub, t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(),
//							t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ(), t.getRotation().getW());
	}
	previous = pointCloud1_BB;
}


////////////////////////////////////////////////////////////////////////////////////////
// ////////////////// nodeComm class for odometry calculations //////////////////
// Reads Point Cloud messages and executes ICP algorithm to estimate transformations
// Usage: A call to the complete constructor includes the 3 algorithm parameters
// "int maxOfIterations, double epsilonValue, double maxTransformDistance" while the
// other one automatically reads the parameters from the parameter server.
////////////////////////////////////////////////////////////////////////////////////////
class nodeComm {
public:
	// Data members
	int maxIterations = 10, neighbours=80;
	double epsilon = 1e-8, maxDistance=7.2;
	float leafSize=0.04f, minDepth=0.6, maxDepth=3.2, radius=0.5;
	bool doDownsampleFiltering = False, doDistanceFiltering = False, doNoiseFiltering = False;


	// GLOBAL MUTEX
	boost::mutex callback_mutex;
	ros::Publisher odom_pub;
	PointCloudT::Ptr previous;
	tf::TransformBroadcaster* br;
	Eigen::Matrix4f GlobalTransform;


	//Ctors
	nodeComm(): br(new tf::TransformBroadcaster), GlobalTransform(Eigen::Matrix4f::Identity ()), maxIterations(10), neighbours(80); {

		// Reads parameters from parameter server if possible
		nHandle.getParam("epsilon", maxIterations);
		nHandle.getParam("maxIterations", maxIterations);
		nHandle.getParam("maxDistance", maxDistance);
		
		nodeComm(maxIterations, epsilon, maxDistance); // Derivation to complete constructor
	}
		
	nodeComm(int maxOfIterations, double epsilonValue, double maxTransformDistance):
br(new tf::TransformBroadcaster), GlobalTransform(Eigen::Matrix4f::Identity ()), maxIterations(maxOfIterations),
maxDistance(maxTransformDistance), epsilon(epsilonValue); {

		std::cout << "Trying to launch Visual Odometry with the next parameters for epsilon, maxIterations and maxDistance: " << epsilon << "," << maxIterations << "," << maxDistance << std::endl;

		setDownsampleFiltering(leafSize);
		setDistanceFiltering(minDepth, maxDepth);
//		setNoiseFiltering(radius, neighbours);

		startComms();
	}
		
	// Methods
	
private:
	// Methods
	// Special algorithm-control methods
	void startCommunications() {
		// %Tag(PUBLISHER)%
		int output_queue_size = 1;
		odom_pub=nHandle.advertise<nav_msgs::Odometry> ("output2", output_queue_size); // nav_msgs/Odometry - primary output
		pcl_pub = nHandle.advertise<PointCloudT> ("filteredPCL2", output_queue_size);
		//	ros::Publisher debug_pub = nHandle.advertise<my_odometry::Debug> ("debug", output_queue_size); // starmac_kinect/Debug - debugging info
		// %EndTag(PUBLISHER)%
		
		// %Tag(SUBSCRIBER)%
		int input_queue_size = 1;
		ros::Subscriber pcl_sub = nHandle.subscribe<PointCloudT > ("input2", input_queue_size, cloudIndicesModelCallback);
		// %EndTag(SUBSCRIBER)%
		
	}

	setDownsampleFiltering(int newLeafSize) {
		leafSize = newLeafSize;
		doDownsampleFiltering = True
	}
	setDistanceFiltering(float minimum, float maximum) {
		leafSize = newLeafSize;
		minDepth = minimum;
		maxDepth = maximum;
		doDistanceFiltering = True
	}
	setNoiseFiltering(float radDist, int noOfNeighbours) {
		radius = radDist;
		neighbours = noOfNeighbours;
		doNoiseFiltering = True
	}
	
	std::string getParam(std::string param_name) {
		std::string result;
//	    if (nHandle.getParam(param_name, result))
		if (!nHandle.hasParam(param_name, result))
	    	ROS_ERR("ERROR: Parameter '%s' missing in the parameter server.");
	    return result
	}
	    
	// Transformation Frames' methods
	tf::Transform tfFromEigen(Eigen::Matrix4f trans) {
	  btMatrix3x3 btm;
	  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
	             trans(1,0),trans(1,1),trans(1,2),
	             trans(2,0),trans(2,1),trans(2,2));
	  btTransform ret;
	  ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
	  ret.setBasis(btm);
	  return ret;
	}
	
	void broadcastTransform(Eigen::Matrix4f trans) {
	  tf::Transform newTF = tfFromEigen(trans);
	//	  tf::Transform transform;
	//    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
	//    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
	//	  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
	//	  transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
	  std::cout << "I'm broadcasting one tf at " << ros::Time::now();
	  br->sendTransform(tf::StampedTransform(newTF, ros::Time::now(), "camera_link", "my_new_shiny_tfBroadcaster"));
	}


	double mat2dist(Eigen3::Matrix4f t) {
	    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
	}


	void mat2RPY(Eigen3::Matrix4f t, double& roll, double& pitch, double& yaw) {
	    roll = atan2(t(2,1),t(2,2));
	    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
	    yaw = atan2(t(1,0),t(0,0));
	}

	void printTransform(tf::Transform tfTransform) {
	   //visualize the transformation
	    Eigen3::Matrix4f matrixTransform;
	    pcl_ros::transformAsMatrix(tfTransform, matrixTransform);

	    double roll, pitch,yaw,dist;
	    dist = mat2dist(matrixTransform);
	    mat2RPY(matrixTransform, roll, pitch, yaw);
	    ROS_INFO("winkel: %f %f %f  dist: %f cm",  roll/M_PI*180,pitch/M_PI*180,yaw/M_PI*180, dist*100);
	}
};



abstract class ifaceOdometry {
protected:
	// Data members
	int input_queue_size;
	ros::Subscriber topic_sub;
	
	// Methods


	//Ctors
	manualOdometry(): input_queue_size(1){
		pcl_sub = nHandle.subscribe<std_msgs::String> (PCL_TOPIC, input_queue_size, newPCL_Callback);
	}


private:
	// Methods
	virtual PointCloudT::ConstPtr fetchPointCloud (std::string topicName, ros::NodeHandle &nHandler, int timeout = 20);
	virtual void cloudIndicesModelCallback(const std_msgs::String::ConstPtr & nextTopic) = 0; // Pure virtual

protected:
	//Methods
	
// Point Cloud main stuff
	Eigen::Matrix4f processCloud(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out) {
		if (cloud_out == 0 || cloud_in == 0) {
			ROS_INFO("Empty clouds received; returning!");
			return Eigen::Matrix4f(0,0);
		}
		
	//	if (cloud_in->points.size () != cloud_out->points.size ()) {
	//		ROS_INFO("The amount of points in each cloud is different");
	//	}
	//	for (size_t i = 0; i < cloud_out->points.size (); ++i)
	//		std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	//	std_msgs::String msg;
	//	std::stringstream ss;
	//	ss << "Checkpoint " << 1 << " at: " << ros::Time::now();
	//	ROS_INFO("%s", ((ss).str()).c_str());
		PointCloudT Final;
		pcl::IterativeClosestPoint<PointT, PointT> icp;

	//*****************************	
		ros::Time ini_time = ros::Time::now();
		ros::Duration maxTime = (ini_time-ini_time);
		ros::Duration minTime = (ini_time-(ros::Time()));
		double minScore=1, maxScore=0;
		
		for (int i=0; i< 1; i++) {
			ini_time = ros::Time::now();
			for (int j=0; j<1; j++) {
				icp.setInputCloud(cloud_in);
				icp.setInputTarget(cloud_out);
				icp.setMaximumIterations(maxIterations);
				icp.setTransformationEpsilon(epsilon);
				icp.setMaxCorrespondenceDistance(maxDistance);
				//setRANSACIterations (int ransac_iterations);
				//setRANSACOutlierRejectionThreshold (double inlier_threshold);
				icp.align(Final);
				*cloud_out = Final;
			}
			if (icp.hasConverged()) {
				std::cout << "has converged with score: " << icp.getFitnessScore() << " after " << (ros::Time::now()-ini_time) << " for " << Final.points.size() << " points." << std::endl;
				/*
				if ((ros::Time::now()-ini_time) < minTime)
					minTime = (ros::Time::now()-ini_time);
				else if ((ros::Time::now()-ini_time) > maxTime)
					maxTime = (ros::Time::now()-ini_time);
				if (icp.getFitnessScore() < minScore)
					minScore = icp.getFitnessScore();
				else if (icp.getFitnessScore() > maxScore)
					maxScore = icp.getFitnessScore();
				*/
			}
			else
				std::cout << "has NOT converged after " << (ros::Time::now()-ini_time) << std::endl;
		}
	//	std::cout << "minMax scores between " << minScore << " and " << maxScore << ": " << maxScore-minScore << std::endl;
	//	std::cout << "minMax times between " << minTime << " and " << maxTime << ": " << maxTime-minTime << std::endl;
		return icp.getFinalTransformation(); // * Eigen::Matrix4f::Identity();
	}
};



class manualOdometry: public ifaceOdometry {
public:
	// Methods

	//Ctors
	manualOdometry(): input_queue_size(1){
		pcl_sub = nHandle.subscribe<std_msgs::String> (PCL_TOPIC, input_queue_size, newPCL_Callback);
	}


private:
	// Methods
};



class autoOdometry: public ifaceOdometry {
public:
	// Data members
	
	// Methods

	//Ctors
	autoOdometry(): input_queue_size(10){
		pcl_sub = nHandle.subscribe<std_msgs::String> (PCL_TOPIC, input_queue_size, requestCallback);
	}
		

private:
	// Methods
	PointCloudT::ConstPtr fetchPointCloud (std::string topicName, ros::NodeHandle &nHandler, int timeout = 20){
		return ros::topic::waitForMessage<PointCloudT>(topicName, nHandler, ros::Duration(timeout));
	}
};



class sonarOdometry: public ifaceOdometry {
public:
	// Data members
	
	// Methods

	//Ctors
	autoOdometry(): input_queue_size(10){
		pcl_sub = nHandle.subscribe<std_msgs::String> (PCL_TOPIC, input_queue_size, requestCallback);
	}
		

private:
	// Methods
};

#endif //#define PCL_TO_TF_HPP
