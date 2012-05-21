
// %Tag(FULLTEXT)%


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <pcl/registration/icp_nl.h>

std::string turtle_name;


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef sensor_msgs::PointCloud2 PointCloud2;

// GLOBAL MUTEX
boost::mutex callback_mutex;
ros::Publisher odom_pub;
PointCloudT::Ptr previous;
static tf::TransformBroadcaster* br;
Eigen::Matrix4f GlobalTransform;
/*
void broadcastTransform(){

    tf::Transform cam2rgb;
    cam2rgb.setRotation(tf::createQuaternionFromRPY(-1.57,0,-1.57));
    cam2rgb.setOrigin(tf::Point(0,-0.04,0));


    tf::Transform world2cam_ = cam2rgb*kinect_transform_;
    printTransform(kinect_transform_);
//    br_.sendTransform(tf::StampedTransform(world2cam_, time_of_last_transform_, "/openni_camera", "/slam_transform"));
    //visualize the transformation
    Eigen3::Matrix4f transform;
    pcl_ros::transformAsMatrix(world2cam_, transform);

    //double roll, pitch,yaw,dist;
    //mat2RPY(transform, roll, pitch, yaw);
    //mat2dist(transform,dist);
    //ROS_INFO("winkel: %f %f %f  dist: %f cm",  roll/M_PI*180,pitch/M_PI*180,yaw/M_PI*180, dist*100);
}
*/

Eigen::Matrix4f processCloud(PointCloudT::Ptr &cloud_in, PointCloudT::Ptr &cloud_out) {
	if (cloud_out == 0 || cloud_in == 0) {
		ROS_INFO("Empty clouds received; returning!");
		return Eigen::Matrix4f(0,0);
	}
	
	if (cloud_in->points.size () != cloud_out->points.size ()) {
		ROS_INFO("The amount of points in each cloud is different");
	}
	
	for (size_t i = 0; i < cloud_in->points.size (); ++i)
		cloud_in->points[i].x = cloud_in->points[i].x + 0.7f;

	std::cout << "Transformed " << cloud_in->points.size () << " data points:" << std::endl;

//	for (size_t i = 0; i < cloud_out->points.size (); ++i)
//		std::cout << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

//	std_msgs::String msg;
//	std::stringstream ss;
//	ss << "Checkpoint " << 1 << " at: " << ros::Time::now();
//	ROS_INFO("%s", ((ss).str()).c_str());


	PointCloudT Final;
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);
	icp.setMaximumIterations(10);
//	icp.setTransformationEpsilon(1e-8);
//	icp.setMaxCorrespondenceDistance(7.2);
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
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
//	  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
//	  transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
  std::cout << "I'm broadcasting one tf at " << ros::Time::now();
  br->sendTransform(tf::StampedTransform(newTF, ros::Time::now(), "openni_camera", "my_new_shiny_tfBroadcaster"));
}

void cloudIndicesModelCallback(const PointCloud2::ConstPtr& cloud_msg) {
	boost::mutex::scoped_lock lock(callback_mutex); // Lock
	PointCloudT::Ptr pointCloud1_BB(new PointCloudT), pointCloud1_A(new PointCloudT);

	// Downsample filtering
//	float pointsRatio = (float)(cloud_msg->points.size())/2000;
//	float leafSize = 0.001 * pointsRatio;
	sensor_msgs::PointCloud2::Ptr inputFiltered (new sensor_msgs::PointCloud2 ());
	pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
	sor.setInputCloud (cloud_msg);
	sor.setDownsampleAllData (true);
	sor.setLeafSize (0.04f, 0.04f, 0.04f);
	sor.filter (*inputFiltered);
//	std::cout << "Removed with <VoxelGrid>: " << sor.getRemovedIndices()->size();
	
//	std::cout << "with ratio = " << pointsRatio << " and leafSize = " << leafSize << ", got " << inputFiltered->points.size() << " of " << clud_msg->points.size() << std::endl << std::endl;

//	std::stringstream ss;
//	ss << "n/a";
	ROS_INFO("Got cloud with timestamp %s", "n/a");
	pcl::fromROSMsg(*inputFiltered, *pointCloud1_A); // ros PCL2 to pcl PCL
	
	// Create the filtering object
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (pointCloud1_A);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.6, 3.2);
	pass.filter (*pointCloud1_BB);
	
	std::vector<int> tempIntVec;
	pcl::removeNaNFromPointCloud(*pointCloud1_BB, *pointCloud1_A, tempIntVec); // Removing NaN values
	
//	std::cout << "Removed with <removeNaN>: " << tempIntVec.size();
	
	if (pointCloud1_A->points.size() < 1)
		return; // Nothing can be done without points
	
	if (previous != 0 && previous->points.size() > 0)
	{
		Eigen::Matrix4f tf_result = processCloud(pointCloud1_A, previous);
//		for (int i=0; i<4; i++)
//			for (int j=0; j<4; j++)
//				tf_result(i, j) = float(int(tf_result(i, j)*100))/100; // MICHI: Brutal rounding
		std::cout << tf_result << std::endl;
	    GlobalTransform = tf_result * GlobalTransform;
		broadcastTransform(GlobalTransform);
		
//		tf::Transform t = ;
//		publishOdom(odom_pub, t.getOrigin().x(), t.getOrigin().y(), t.getOrigin().z(),
//							t.getRotation().getX(), t.getRotation().getY(), t.getRotation().getZ(), t.getRotation().getW());
	}
	previous = pointCloud1_A;

}


int main(int argc, char **argv)
{
	// %Tag(INIT)%
	ros::init(argc, argv, "pcl_to_tf");
	// %EndTag(INIT)%
	
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	// %Tag(NODEHANDLE)%
	ros::NodeHandle n;
	// %EndTag(NODEHANDLE)%

	br = new tf::TransformBroadcaster;
	GlobalTransform = Eigen::Matrix4f::Identity ();
	
	// %Tag(PUBLISHER)%
	int output_queue_size = 1;
	odom_pub=n.advertise<nav_msgs::Odometry> ("output", output_queue_size); // nav_msgs/Odometry - primary output
	//	ros::Publisher debug_pub = n.advertise<my_odometry::Debug> ("debug", output_queue_size); // starmac_kinect/Debug - debugging info
	// %EndTag(PUBLISHER)%
	
	// %Tag(SUBSCRIBER)%
	int input_queue_size = 1;
	ros::Subscriber pcl_sub = n.subscribe<PointCloud2> ("input", input_queue_size, cloudIndicesModelCallback);
	// %EndTag(SUBSCRIBER)%

	// %Tag(LOOP_RATE)%
	ros::Rate loop_rate(10);
	// %EndTag(LOOP_RATE)%
	
	/**
	* A count of how many messages we have sent. This is used to create
	* a unique string for each message.
	*/
	// %Tag(ROS_OK)%
	while (ros::ok())
	{
	// %EndTag(ROS_OK)%
	
	
	/*
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << count;
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());
	odom_pub.publish(msg);
	// %Tag(SPINONCE)%
	// %EndTag(SPINONCE)%
	*/
	ros::spinOnce();
  }

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%
  ros::spin();
// %EndTag(SPIN)%


  return 0;
}
// %EndTag(FULLTEXT)%
