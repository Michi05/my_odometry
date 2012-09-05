

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
//#include <my_odometry/Debug.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>

// MICHI: April27
#include <tf/transform_broadcaster.h>
// MICHI: April30
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// GLOBAL MUTEX
boost::mutex callback_mutex;
ros::Publisher odom_pub;
PointCloudT::Ptr previous;
static tf::TransformBroadcaster* br;
Eigen::Matrix4f GlobalTransform;

int maxIterations = 100, neighbors=80;
double epsilon = 1e-8, maxDistance=1.2;
float leafSize=0.001f, minDepth=0.6, maxDepth=1.8, radius=0.5;

ros::Publisher pcl_pub;
std::string PCL_TOPIC = "input";
ros::NodeHandle *nHandlePtr;


PointCloudT::ConstPtr fetchPointCloud (std::string topicName, ros::NodeHandle &nHandler, int timeout = 20){
	return ros::topic::waitForMessage<PointCloudT>(topicName, nHandler, ros::Duration(timeout));
}

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
//			setEuclideanFitnessEpsilon (double epsilon);
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

void publishOdom(ros::Publisher &odom_pub, int x, int y, int z, int rx, int ry, int rz, int rw) {
	nav_msgs::Odometry odom_msg_output = nav_msgs::Odometry();
//	odom_msg_output.pose.pose.position.z = pos;
	odom_msg_output.twist.twist.linear.x = rx;
	odom_msg_output.twist.twist.linear.y = ry;
	odom_msg_output.twist.twist.linear.z = rz;
	odom_msg_output.pose.pose.orientation.x = x;
	odom_msg_output.pose.pose.orientation.y = y;
	odom_msg_output.pose.pose.orientation.z = z;
	odom_msg_output.pose.pose.orientation.w = rw;
	odom_msg_output.header.stamp = ros::Time::now();
	odom_pub.publish(odom_msg_output);
}

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  btMatrix3x3 btm;
  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
             trans(1,0),trans(1,1),trans(1,2),
             trans(2,0),trans(2,1),trans(2,2));
  btTransform ret;
  ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
  ret.setBasis(btm);
  return ret;
}

void broadcastTransform(Eigen::Matrix4f trans){
  tf::Transform newTF = tfFromEigen(trans);
//	  tf::Transform transform;
//    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
//    cam2rgb.setOrigin(tf::Point(0,-0.04,0));
//	  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
//	  transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
  std::cout << "I'm broadcasting one tf at " << ros::Time::now() << std::endl;
  std::cout << "...with the next values: " << std::endl;
  for (int i=0; i<3; i++) {
	  tf::Vector3 tmp = newTF.getBasis()[i];
	  std::cout << " \t " << tmp.x();
	  if (tmp.x()==1 || tmp.x()==0)
		  std::cout << " \t ";
	  std::cout << " \t " << tmp.y();
	  if (tmp.x()==1 || tmp.x()==0)
		  std::cout << " \t ";
	  std::cout << " \t " << tmp.z() << std::endl;
  }
  std::cout << "(x, y z) = \t " << newTF.getOrigin().x() << " \t " << newTF.getOrigin().y() << " \t " << newTF.getOrigin().z();
  std::cout << std::endl << std::endl;

  br->sendTransform(tf::StampedTransform(newTF, ros::Time::now(), "camera_link", "my_new_shiny_tfBroadcaster"));
}


//double mat2dist(Eigen3::Matrix4f t){
//    dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
//}


//void mat2RPY(Eigen3::Matrix4f t, double& roll, double& pitch, double& yaw)
//{
//    roll = atan2(t(2,1),t(2,2));
//    pitch = atan2(-t(2,0),sqrt(t(2,1)*t(2,1)+t(2,2)*t(2,2)));
//    yaw = atan2(t(1,0),t(0,0));
//}

/*
void printTransform(tf::Transform tfTransform) {
   //visualize the transformation
    Eigen3::Matrix4f matrixTransform;
    pcl_ros::transformAsMatrix(tfTransform, matrixTransform);

    double roll, pitch,yaw,dist;
    dist = mat2dist(matrixTransform);
    mat2RPY(matrixTransform, roll, pitch, yaw);
    ROS_INFO("winkel: %f %f %f  dist: %f cm",  roll/M_PI*180,pitch/M_PI*180,yaw/M_PI*180, dist*100);
}
*/

void cloudIndicesModelCallback(const std_msgs::String::ConstPtr & nextTopic) {
	boost::mutex::scoped_lock lock(callback_mutex); // Lock
	PointCloudT::ConstPtr cloud_msg = fetchPointCloud (nextTopic->data, *nHandlePtr);
	PointCloudT::Ptr pointCloud1_BB(new PointCloudT), pointCloud1_A(new PointCloudT);

	if (!(cloud_msg != 0 && cloud_msg->points.size() > 0)) {
		ROS_INFO("Empty PointCloud or Pointer");
		return; // Nothing can be done without points
	}

	//	std::stringstream ss;
	//	ss << "n/a";
	ROS_INFO("Got cloud with timestamp %f.", static_cast<double>(cloud_msg->header.stamp.toNSec()));

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
	// set threshold for minimum required neighbors
	radius_outlier_removal.setMinNeighborsInRadius (neighbors);
	radius_outlier_removal.filter (*pointCloud1_A);
	*/
//	std::cout << "Removed with <removeNaN>: " << tempIntVec.size();
	
	if (pointCloud1_BB->points.size() < 1) {
		ROS_INFO("No points after the filtering.");
		return; // Nothing can be done without points
	}

	pcl_pub.publish(pointCloud1_BB);
	
	if (previous != 0 && previous->points.size() > 0)
	{
		
//		pcl_pub.publish(pointCloud1_BB);
		pointCloud1_BB->header.stamp = ros::Time(0);
		//"marker.header.stamp = rospy.Time(0)"

		Eigen::Matrix4f tf_result = processCloud(pointCloud1_BB, previous);
		pcl_pub.publish(previous);
		

		for (int i=0; i<3; i++) {
			if (tf_result(i,3) < 0.001 && tf_result(i,3) > -0.001) {
				ROS_INFO("ANULANDO %f", tf_result(i,3));
				  tf_result(i,3) = 0;
			}
		}
		if (!(tf_result(0,3) == tf_result(1,3) == tf_result(2,3) == 0)) {
			previous = pointCloud1_BB; // ONLY IF THERE'S CHANGES!!!
		}
	//		std::cout << tf_result << std::endl;
			GlobalTransform = tf_result * GlobalTransform;
			broadcastTransform(tf_result);
	//		broadcastTransform(GlobalTransform);
			
	//		publishOdom(odom_pub, t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(),
	//							t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ(), t.getRotation().getW());
	}
	else
		previous = pointCloud1_BB;


}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pcl_to_tf");
	if (argc>2) {
		epsilon = atof(argv[1]);
		maxIterations = atoi(argv[2]);
		maxDistance = atof(argv[3]);
//		int maxIterations = 10, neighbors=80;
//		double epsilon = 1e-8, maxDistance=7.2;
//		float leafSize=0.04f, minDepth=0.6, maxDepth=3.2, radius=0.5;
	} /*
	else {
		std::cout << "I need the epsilon (as a float):";
		std::cin >> epsilon;
		std::cout << "I need the maxIterations (as a int):";
		std::cin >> maxIterations;
		std::cout << "I need the maxDistance (as a float):";
		std::cin >> maxDistance;
	}
*/
	
	ros::NodeHandle nHandle;
	nHandlePtr = &nHandle;

//	if (nHandle.hasParam("epsilon"))
		nHandle.getParam("epsilon", maxIterations);
//	if (nHandle.hasParam("maxIterations"))
		nHandle.getParam("maxIterations", maxIterations);
//	if (nHandle.hasParam("maxDistance"))
		nHandle.getParam("maxDistance", maxDistance);

	std::cout << "I read the next parameters for epsilon, maxIterations and maxDistance: " << epsilon << "," << maxIterations << "," << maxDistance << std::endl;

	br = new tf::TransformBroadcaster;
	GlobalTransform = Eigen::Matrix4f::Identity ();
	
	// %Tag(PUBLISHER)%
	int output_queue_size = 1;
	odom_pub=nHandle.advertise<nav_msgs::Odometry> ("output", output_queue_size); // nav_msgs/Odometry - primary output
	pcl_pub = nHandle.advertise<PointCloudT> ("filteredPCL", output_queue_size);
	//	ros::Publisher debug_pub = nHandle.advertise<my_odometry::Debug> ("debug", output_queue_size); // starmac_kinect/Debug - debugging info
	// %EndTag(PUBLISHER)%
	
	// %Tag(SUBSCRIBER)%
	int input_queue_size = 1;
	ros::Subscriber pcl_sub = nHandle.subscribe<std_msgs::String> (PCL_TOPIC, input_queue_size, cloudIndicesModelCallback);
	// %EndTag(SUBSCRIBER)%
	
	/**
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << count;
	msg.data = ss.str();
	*/

  ros::spin();
//	while (ros::ok()) ros::spinOnce();


  return 0;
}
