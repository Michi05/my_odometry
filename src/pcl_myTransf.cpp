
//=============================================================================
//Includes
//=============================================================================

#include "ros/ros.h"

#include <pcl_myTransf.hpp>








//=============================================================================
//Namespace
//=============================================================================
namespace pcl_odometry {


//=============================================================================
//Method definitions
//=============================================================================
	


//////////////////////////////////////////////////////
///////// CONVERTERS FROM TRANSFORMATIONS ////////////
//////////////////////////////////////////////////////

nav_msgs::Odometry myTransf::transformToOdometry(ros::Time stamp, std::string frameID, std::string childID) {
	nav_msgs::Odometry odom_msg_output;

	odom_msg_output.header.stamp = stamp;
	odom_msg_output.header.frame_id = frameID;
	odom_msg_output.child_frame_id = childID;

	// Translation data
	odom_msg_output.pose.pose.position.x = this->getOrigin().getX();
	odom_msg_output.pose.pose.position.y = this->getOrigin().getY();
	odom_msg_output.pose.pose.position.z = this->getOrigin().getZ();
	// Rotation data
	odom_msg_output.pose.pose.orientation.x = this->getRotation().getX();
	odom_msg_output.pose.pose.orientation.y = this->getRotation().getY();
	odom_msg_output.pose.pose.orientation.z = this->getRotation().getZ();
//	odom_msg_output.pose.pose.orientation.w = rw;

	return odom_msg_output;
}

//=============================================================================
tfMessage myTransf::transformToTFMsg(ros::Time stamp, std::string frameID, std::string childID) {
	geometry_msgs::TransformStamped tf2_msg_output;

	tf2_msg_output.header.stamp = stamp;
	tf2_msg_output.header.frame_id = frameID;
	tf2_msg_output.child_frame_id = childID;
	
	// Translation data
	tf2_msg_output.transform.translation.x = this->getOrigin().getX();
	tf2_msg_output.transform.translation.y = this->getOrigin().getY();
	tf2_msg_output.transform.translation.z = this->getOrigin().getZ();

	// Rotation data
	tf2_msg_output.transform.rotation.x = this->getRotation().getX();
	tf2_msg_output.transform.rotation.y = this->getRotation().getY();
	tf2_msg_output.transform.rotation.z = this->getRotation().getZ();

	tfMessage newTFMessage;
	newTFMessage.transforms.push_back(tf2_msg_output);
	return newTFMessage;
}

//=============================================================================























////////////////////////////////////////////////
////////// EXTRACT DATA / PUBLISH / PRINT TFs /////////////////
////////////////////////////////////////////////

void myTransf::printTransform(){
	/*// This is ommitted as the whole matrix is not needed anymore
	  for (int i=0; i<3; i++) {
		  tf::Vector3 tmp = this->getBasis()[i];
		  std::cout << " \t " << tmp.x();
		  if (tmp.x()==1 || tmp.x()==0)
			  std::cout << " \t ";
		  std::cout << " \t " << tmp.y();
		  if (tmp.y()==1 || tmp.y()==0)
			  std::cout << " \t ";
		  std::cout << " \t " << tmp.z() << std::endl;
	  }
	  */
	
	double R, P, Y; this->getBasis().getRPY(R, P, Y);
	tf::Vector3 origin = this->getOrigin();
	double d = 0.2;
	double x=d*cos(Y)-(origin.x()+d), y=d*sin(Y)-origin.y(), z=origin.z()/2.0;
	
	  std::cout << "(x, y, z) = \t " << this->getOrigin().x() << " \t " << this->getOrigin().y() << " \t " << this->getOrigin().z() << std::endl;
//	  std::cout << "(x', y', z') = \t " << x << " \t " << y << " \t " << z << std::endl;

//	  std::cout << "(R, P, Y :: W) normalized = \r\n\t\t " << this->getRotation().getX() << " \t " << this->getRotation().getY() << " \t " << this->getRotation().getZ() << " \t ++ " << this->getRotation().getW() << std::endl;
	  std::cout << "(R, P, Y) = \t " << R << " \t " << P << " \t " << Y << std::endl;
	  std::cout << std::endl << std::endl;
}

//=============================================================================
void myTransf::publishOdom(ros::Publisher &odom_publisher) {
	odom_publisher.publish(this->transformToOdometry());
}

//=============================================================================
void myTransf::broadcastTransform(std::string tfChannel, std::string tfParent){
  tf::TransformBroadcaster br;
  std::cout << "Broadcasting TF \"" << tfChannel << "\" at time: " << ros::Time::now() << std::endl;
  this->printTransform();
  br.sendTransform(tf::StampedTransform(*this, ros::Time::now(), tfParent, tfChannel));
}

//=============================================================================
double myTransf::transformToDistance(){
	tf::Vector3 origin(this->getOrigin());
	double sqDistance = 0;
	for (int i=0; i< 3; i++)
		sqDistance = origin[i] * origin[i] + sqDistance;
    return sqrt(sqDistance);
}

//=============================================================================
double myTransf::transformToRotation(double rotationAccuracy){
	double rotSqr = this->getRotation().getW()*(this->getRotation().getW());
	double diff = double(1.00000) - rotSqr;
	// If the diff between 1.0 and rotSqr is too small
	//then it is considered 0.0
	if (diff < rotationAccuracy)
		return double(0.0);
//		printf("ROTATION %f: \r\n sqrt(1-%f) = sqrt(%f) = %f != %f\r\n\r\n", this->getRotation().getW(), rotSqr, double(1.0)-rotSqr, sqrt(double(1.0)-rotSqr));
    return diff;
}















////////////////////////////////////////////////////
////////// HANDLERS FOR MODIFYING TFs //////////////
////////////////////////////////////////////////////

void myTransf::applyRestrictions() {
	/* TODO: 
	 * This method can help to clean errors from the algorithm in yaw calculations:
	 * It may be improved for removing wrong measures (like height) or removed
	 * but right now it's not doing anything, just the place to do it.
	*/
	double R, P, Y;			this->getBasis().getRPY(R, P, Y);
	tf::Vector3 origin = this->getOrigin(); // tf::Vector3(0, 0, 0);
	this->generate_tf(R, P, Y); // this->generate_tf(R, P, Y);
	this->setOrigin(tf::Vector3(origin.x(), origin.y(), origin.z()));
}

/*
void myTransf::fromCloudToCamera() {
	// If the cloud reference system is inside of the cloud, this is what it takes to change it
	//if the same is in (x/2,y/2,0) it's trivial because it's the camera!
	double R, P, Y;			this->getBasis().getRPY(R, P, Y);
	tf::Vector3 origin = this->getOrigin();
	
	// This is the supposed distance to the cloud
	double d=0.1;
	
	double x=d*cos(Y)-(origin.x()+d), y=d*sin(Y)-origin.y(), z=origin.z()/2.0;
	this->generate_tf(R, P, Y);
	this->setOrigin(tf::Vector3(x, y, z));
}
*/

//=============================================================================

void myTransf::toROSCoordinates() {
	// Adapting the values to the ROS convention:
	// "3D coordinate systems in ROS are always right-handed, with X
	//forward, Y left, and Z up." (http://ros.org/wiki/tf/Overview/Transformations)
	// But (CAUTION!) Point Clouds have ALWAYS Y to represent DOWN (not up) and Z for depth,
	//which is ((MINUS))Forward! So the change is:
	//(X, Y, Z)_transform == (-Z_cloud, X_cloud, -Y_cloud)
	
	
	// The initial RPY and XYZ values are stored firstly
	double R, P, Y;			this->getBasis().getRPY(R, P, Y);
	btVector3 origin(-this->getOrigin()[2], this->getOrigin()[0], -this->getOrigin()[1]);
	// Finally they are stored in the 'this' instance
	this->generate_tf(-Y, R, P);
	this->setOrigin(origin);
}

//=============================================================================

// Just temporal method for testing
void myTransf::toCloudCoordinates() {
	// The initial RPY and XYZ values are stored firstly
	double R, P, Y;			this->getBasis().getRPY(Y, R, P);
	btVector3 origin(this->getOrigin()[1], this->getOrigin()[2], -this->getOrigin()[0]);
	// Finally they are stored in the 'this' instance
	this->generate_tf(R, P, -Y);
	this->setOrigin(origin);
}

//=============================================================================

void myTransf::round_near_zero_values(double accuracy, double rotationAccuracy){
	// NOTE: I remove the near-0 values in the rotations if they seem to belower than
	//the real detectable accuracy. This is done in the first place as the accuracy
	//is from the camera POV, not after the rotations.

	// Check rotations for near-zero values:
	if (this->transformToRotation(rotationAccuracy) == 0 ) { //< 0.001f) {
		tf::Quaternion newRotation(0, 0, 0, 1); // XYZW constructor
//		newRotation.setRPY(x, y, z);
		this->setRotation(newRotation);
	}
	
	// Check the origin vector for near-zero values:
	for (int i=0; i<3; i++) {
		double val = this->getOrigin()[i];
		if (val < accuracy && val > -accuracy) {
			this->getOrigin()[i] = 0;
		}
	}
}


//=============================================================================
// Temporal method for tests.
//DOESN'T change the " "this" " object
void myTransf::back_to_cloud_matrix(myTransf &fixedTF, Eigen::Matrix4f &tf_matrix_result){
	myTransf resultTF1;
//	std::cout << "*** Robot movement:" << std::endl; this->printTransform();
	resultTF1 = fixedTF.inverse() * (*this) * fixedTF;
//	std::cout << "*** Camera movement:" << std::endl; resultTF1.printTransform();

	resultTF1.toCloudCoordinates();
//	std::cout << "*** Cloud transform:" << std::endl; resultTF1.printTransform();
	
	pcl_ros::transformAsMatrix(resultTF1, tf_matrix_result);
}

//=============================================================================

void myTransf::get_robot_relative_tf(myTransf &fixedTF) {
	// (( Remember: matrices are asociative, but not conmutative ))

	myTransf resultTF1;
	// The fixed camera position is relative to the (0, 0, 0) of the robot
	//and the odometry estimation is about the camera movement
	// The fixedTF times camera movement times the inverse of the first must
	//be the robot's movement as the said relative position is fixed.
//	std::cout << "*** Camera movement:" << std::endl; this->printTransform();

	resultTF1 = fixedTF * (*this) * fixedTF.inverse();
//	std::cout << "*** Robot movement:" << std::endl; resultTF1.printTransform();
	
	*this = resultTF1;
}

//=============================================================================

void myTransf::rotate_tf(double roll, double pitch, double yaw) {
	// Starts by creating a fixedTF with the rotations. Then rotates the current tf
	myTransf fixedTF = myTransf::getIdentity();
	fixedTF.generate_tf(roll, pitch, yaw);

	this->mult(*this, fixedTF);
}
















////////////////////////////////////////////////////
////////// HANDLERS FOR CREATING TFs  //////////////
////////////////////////////////////////////////////


bool myTransf::generate_tf(double roll, double pitch, double yaw) {
	*this = myTransf::getIdentity();
	
	tf::Quaternion newRotation(0, 0, 0, 1); // XYZW quaternion constructor
	newRotation.setRPY(roll, pitch, yaw);
//		std::cout << "setRotation(" << x << ", " << y << ", " << z << ")" << std::endl;

	this->setRotation(newRotation);
	this->setOrigin(tf::Vector3(0, 0, 0));

	return true;
}

//=============================================================================




//=============================================================================
} //namespace
//=============================================================================
