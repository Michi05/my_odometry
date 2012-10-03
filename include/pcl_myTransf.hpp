
//=============================================================================
#ifndef PCL_MYTRANSF_HPP
#define PCL_MYTRANSF_HPP
//=============================================================================

#include "ros/ros.h"
#include <string>
#include <cmath> // For the float std::abs()

// Transform Frame data types-related
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Geometry>

// ROS Messages and formats
#include <message_filters/subscriber.h> // topic subscriber
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"

typedef tf2_msgs::TFMessage tfMessage; // alternative is "tf::tfMessage"







//=============================================================================
namespace pcl_odometry { //Namespace
//=============================================================================





/////////////////////////////////////////////////////////
///////////// odometryComm Forward Declaration //////////
/////////////////////////////////////////////////////////

class odometryComm;








//////////////////////////////////////////////////////
///////////// myTransf class declaration  ////////////
//////////////////////////////////////////////////////

class myTransf: public tf::Transform {
public:
	myTransf(){}
	myTransf(tf::Transform newTF): tf::Transform(newTF){}
	
	tf::Transform getTransform(){
		return *this;
	}

	//////////////////////////////////////////////////////
	///////// CONVERTERS FROM TRANSFORMATIONS ////////////
	//////////////////////////////////////////////////////

	/**
	   * Converts current transform frame 
	   * into Odometry/transform frame/tfMessage
	   *   
	   *   @param stamp - timestamp for the frame. 0 is understood as unknown
	   *   				or just irrelevant.
	   *   @param frameID - The name (id) of the coordinate frame in which
	   *   					the transform is defined. 
	   *   @param childID - The name (id) of the coordinate frame this
	   *   					transform defines (child).
	   *
	   *   @return the resulting tf with the same data with the new format or type
	   */
	nav_msgs::Odometry transformToOdometry(ros::Time stamp = ros::Time(0), std::string frameID = "my_odom_tf", std::string childID = "base_link");


	  /**
	   * Converts the current TF into
	   * Odometry/transform frame/tfMessage
	   *   
	   *   @param stamp - timestamp for the frame. 0 is understood as unknown
	   *   				or just irrelevant.
	   *   @param frameID - The name (id) of the coordinate frame in which
	   *   					the transform is defined. 
	   *   @param childID - The name (id) of the coordinate frame this
	   *   					transform defines (child).
	   *
	   *   @return the resulting tf with the same data with the new format or type
	   */
	tfMessage transformToTFMsg(ros::Time stamp = ros::Time(0), std::string frameID = "my_odom_tf", std::string childID = "base_link");

	
	////////////////////////////////////////////////
	////////// EXTRACT DATA / PUBLISH / PRINT TFs /////////////////
	////////////////////////////////////////////////

	  /**
	   * Publishes an ROS-odometry-type message to the ROS environment with
	   * data from current transform and using the passed publisher
	   *   
	   *   @param odom_publisher - the topic publisher object in which to
	   *   							put the message in order to send it.
	   */
	void publishOdom(ros::Publisher &odom_publisher);
	
	
	  /**
	   * Automatically prints all the data in the myTransf transform frame
	   */
	void printTransform();


	  /**
	   * Publishes a transform to the ROS environment using a
	   * "TransformBroadcaster" object, so it's visible system-wide.
	   *   
	   *   @param tMatrix - the myTransf to be published.
	   *   @param tfChannel - The name (id) of the coordinate frame this
	   *   					transform defines (child).
	   *   @param tfParent - The name (id) of the coordinate frame in which
	   *   					the transform is defined. 
	   */
	void broadcastTransform(std::string tfChannel, std::string tfParent="map");

	  /**
	   * Calculates the linear movement in a euclidean space in order to
	   * determine if the movement is to big to be reliable.
	   *   
	   *   @param t - transform frame from which to obtain data
	   */
	double transformToDistance();

	  /**
	   * Calculates the amount of rotation in order to have a measure
	   * to determine if the rotation is to big to be reliable.
	   *   
	   *   @param t - transform frame from which to obtain data
	   */
	double transformToRotation(double accuracy);


	////////////////////////////////////////////////////
	////////// HANDLERS FOR MODIFYING TFs //////////////
	////////////////////////////////////////////////////

	// TODO: document (again)
	void applyRestrictions();
	void changeCoordinates();

		
	/**
	   * Rounds small values of a transform frame to prevent noise from
	   * being interpreted as movement.
	   *   
	   *   @param tf_result - the tf to be rounded
	   *   @param margin - the value for the interval to be considered
	   *   "too near to zero": -margin < tooSmall < margin
	   */
	void round_near_zero_values(double accuracy, double rotationAccuracy);
	
	/**
	   * Rotates a transform frame according to the fixed transform that
	   * describes a relative position. The resulting transform should
	   * represent the same movement just from a different reference system.
	   *   
	   *   @param tf_result - the transform to be rotated
	   *   @param fixedTF - the fixed transform with the rotation to be applied
	   */
	void get_robot_relative_tf(myTransf &fixedTF);

	/**
	   * Rotates a transform frame according to the passed values.
	   * The resulting transform should represent the same movement just from
	   * a different reference system.
	   *   
	   *   @param tf_result - the calculated transform to be rotated
	   *   @param roll - the X axis rotation to be applied to tf_result
	   *   @param pitch - the Y axis rotation to be applied to tf_result
	   *   @param yaw - the Z axis rotation to be applied to tf_result
	   */
	void rotate_tf(double roll, double pitch, double yaw);

	
	
	////////////////////////////////////////////////////
	////////// HANDLERS FOR CREATING TFs  //////////////
	////////////////////////////////////////////////////


	/**
	   * Generates a 6D transform frame describing a position (0, 0, 0) and
	   * orientation according to the passed values.
	   *   
	   *   @param tf - the tf variable in which to store the result
	   *   @param roll - the X axis rotation to be applied to tf
	   *   @param pitch - the Y axis rotation to be applied to tf
	   *   @param yaw - the Z axis rotation to be applied to tf
	   */
	bool generate_tf(double roll, double pitch, double yaw);
	


private:
};

//=============================================================================
} //namespace
//=============================================================================


//=============================================================================
#endif //#ifndef PCL_ODOMETRY_HPP
//=============================================================================
