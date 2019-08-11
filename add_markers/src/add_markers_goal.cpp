//This node controls the marker behaviour by leveraging the move_base goal result topic
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <string>
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include <sstream>

//declare publishers globally. Could also have used functors but this is adequate for a small file.
ros::Publisher marker_pub;
ros::Publisher setSwCase;
visualization_msgs::Marker marker1;
//initialize the variable used in decision logic
std::string goalCase = "0";

//subscribe to the move_base/result message which indicates when and which goal has been reached.
void goalCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& act)
{

	//ROS_INFO("Callback called: ");
// Separate out the goal id/id data from the message
	std::string var = act->status.goal_id.id;
	ROS_INFO("output: %s ", var.c_str());
	std::string pickup = "pick_objects-1";
	std::string dropoff = "pick_objects-2";
//check to see which goal object was reached by comparing the data to the above strings:
	if (var.find(pickup) != std::string::npos) 
	{
	ROS_INFO("Marker found! I'll pick it up");
	goalCase = "1";
	}
	else if (var.find(dropoff) != std::string::npos) 
	{
	ROS_INFO("Here we are, time to drop off the marker!");
	goalCase = "2";
	}
	else
	{
	ROS_INFO("I'm not sure what happened");
	}
	   

	std_msgs::String msg;
	std::stringstream ss;
	ss << goalCase;
	msg.data = ss.str();
	//ROS_INFO("the callback received: [%s]", msg.data.c_str());
	goalCase = msg.data.c_str();
}

void publishPickup(){
	uint32_t shape = visualization_msgs::Marker::CUBE;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker1.header.frame_id = "map";
	marker1.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker1.ns = "basic_shapes";
	marker1.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker1.type = shape;

	//ros::Duration(5).sleep();

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker1.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker1.pose.position.x = 1.0;
	marker1.pose.position.y = 7.8;
	marker1.pose.position.z = 0;
	marker1.pose.orientation.x = 0.0;
	marker1.pose.orientation.y = 0.0;
	marker1.pose.orientation.z = 0.0;
	marker1.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker1.scale.x = 0.2;
	marker1.scale.y = 0.2;
	marker1.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker1.color.r = 0.0f;
	marker1.color.g = 1.0f;
	marker1.color.b = 1.0f;
	marker1.color.a = 1.0;
	//std::cout << "test before first marker publisher ****************: " << std::endl; 
	marker_pub.publish(marker1);
	//std::cout << "test after first marker publisher ****************: " << std::endl;
}

void publishDropoff(){
	uint32_t shape = visualization_msgs::Marker::CUBE;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker1.header.frame_id = "map";
	marker1.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker1.ns = "basic_shapes";
	marker1.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker1.type = shape;

	//ros::Duration(5).sleep();

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker1.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker1.pose.position.x = 8.0;
	marker1.pose.position.y = 7.8;
	marker1.pose.position.z = 0;
	marker1.pose.orientation.x = 0.0;
	marker1.pose.orientation.y = 0.0;
	marker1.pose.orientation.z = 0.0;
	marker1.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker1.scale.x = 0.2;
	marker1.scale.y = 0.2;
	marker1.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker1.color.r = 0.0f;
	marker1.color.g = 1.0f;
	marker1.color.b = 1.0f;
	marker1.color.a = 1.0;

	marker_pub.publish(marker1);
	//Wait a moment and then close the node
	//ros::Duration(2).sleep();
	//ros::shutdown();

}

void publishHideMarker(){
	marker1.header.frame_id = "map";
	marker1.header.stamp = ros::Time::now();
	marker1.ns = "basic_shapes";
	marker1.id = 0;
	marker1.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker1);

}



int main( int argc, char** argv )
{
	ROS_INFO("Navigating to pickup point!"); 
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	ros::Rate r(1);

	//publisher for the markers
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	//This publisher/callback sets the logic for the marker placement
	setSwCase = n.advertise<std_msgs::String>("set_sw_case", 5);
	//This subscriber/callback monitors the goal messages.
	ros::Subscriber result = n.subscribe("move_base/result", 5, goalCallback);

	while (ros::ok())
	{
		//ROS_INFO("goal case is: [%s]", goalCase.c_str());
		//logic for markers
		if (goalCase == "0"){publishPickup();}
		else if(goalCase == "1"){publishHideMarker();}
		else if(goalCase == "2"){publishDropoff();}
		
		ros::spinOnce();

  }
}
