
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
//	odom_msg_output.pose.pose.orientation.w = tMatrix.getRotation().getW();;
	// TODO: check this again; I don't trust it

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
	  std::cout << "(x, y, z) = \t " << this->getOrigin().x() << " \t " << this->getOrigin().y() << " \t " << this->getOrigin().z() << std::endl;
	  std::cout << "(R, P, Y :: W) normalized = \r\n\t\t " << this->getRotation().getX() << " \t " << this->getRotation().getY() << " \t " << this->getRotation().getZ() << " \t ++ " << this->getRotation().getW() << std::endl;
	  double R, P, Y; this->getBasis().getRPY(R, P, Y);
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
	 * Dirty trick that until now helps to clean
	 * errors from the algorithm in yaw calculations:
	*/
	double R, P, Y;			this->getBasis().getRPY(R, P, Y);
//		myTransf tf;		generate_tf(tf, R/10.0, P, Y/10.0);
	myTransf tf;		tf.generate_tf(R, P, Y);
	tf.setOrigin(this->getOrigin());
	*this = tf.getTransform();
}

//=============================================================================

void myTransf::changeCoordinates() {
	// Adapting the values to the ROS convention:
	// "3D coordinate systems in ROS are always right-handed, with X
	//forward, Y left, and Z up." (http://ros.org/wiki/tf/Overview/Transformations)
	// But (CAUTION!) Point Clouds have ALWAYS Y to represent up and Z for depth,
	//which is Forward! So the change is: (X, Y, Z)_cloud == (Z_cloud, X_cloud, Y_cloud)_transform
	
	// The initial RPY and XYZ values are stored firstly
	double R, P, Y;			this->getBasis().getRPY(R, P, Y);
	btVector3 origin(this->getOrigin()[2], this->getOrigin()[0], this->getOrigin()[1]);
	// Finally they are stored in the 'this' instance
	this->generate_tf(P, Y, R);
	this->setOrigin(origin);
}

//=============================================================================

void myTransf::round_near_zero_values(double accuracy, double rotationAccuracy){
	// MICHI: I remove the near-0 values in the rotations as they are lower than the real detected accuracy
	// This is done in the first place as the accuracy is from the camera POV, not after the rotations.

//TODO: BE CAREFUL!!!! the rotation accuracy is missing, just for testing purposes; it needs to be fixed!!
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

void myTransf::get_robot_relative_tf(myTransf &fixedTF) {
	// (( Remember: matrices are asociative, but not conmutative ))

	myTransf resultTF1;
	// La posición FIJA de la cámara es con respecto a cero (el robot)
	//el movimiento estimado es el de la cámara
	//el movimiento compuesto con la posición de la cámara es la nueva posición de la cámara
	//la nueva posición de la cámara por la inversa de la posición FIJA de la cámara es la
	//posición del robot, que no cambió su posición relativa
	std::cout << "***original_tf:" << std::endl; this->printTransform();

	resultTF1 = fixedTF * (*this) * fixedTF.inverse(); // == fixedTF * original_TF
	std::cout << "***resultTF3:" << std::endl; resultTF1.printTransform();
// TODO: change this for the final version and add the new parameters in the main
	
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
