//This node adds the markers to RVIZ with the required timing
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
	ros::init(argc, argv, "add_markers");
	ros::NodeHandle n;
	//ros::Rate r(0.01);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;

	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;

	ros::Duration(5).sleep();

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 1.0;
	marker.pose.position.y = 7.8;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	//set the interval for the marker to be published using ros time
	ros::Time beginTime = ros::Time::now();
	//std::cout << "beginTime: " << beginTime << std::endl;
	ros::Duration five_seconds = ros::Duration(5.0); 
	ros::Time endTime =  beginTime + five_seconds;
	//std::cout << "endTime: " << endTime << std::endl;

	//while loop to publish marker for 5 seconds
	while(ros::Time::now() < endTime )
	{
	    marker_pub.publish(marker);
	    //std::cout << "in loop 1" << std::endl;

	    // Time between messages
	    ros::Duration(0.1).sleep();
	}


	marker.action = visualization_msgs::Marker::DELETE;
	marker_pub.publish(marker);


	ros::Duration(5).sleep();
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = 8.0;
	marker.pose.position.y = 7.8;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.2;
	marker.scale.y = 0.2;
	marker.scale.z = 0.2;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;

	ros::Time beginTime2 = ros::Time::now();
	//std::cout << "beginTime2: " << beginTime2 << std::endl;
	ros::Duration five_seconds2 = ros::Duration(5.0); 
	ros::Time endTime2 =  beginTime2 + five_seconds2;
	//std::cout << "beginTime2: " << beginTime2 << std::endl;
	//std::cout << "endTime2: " << endTime2 << std::endl;

	while(ros::Time::now() < endTime2 )
	{
	    marker_pub.publish(marker);
	    //std::cout << "in loop 2" << std::endl;

	    // Time between messages
	    ros::Duration(0.1).sleep();
	}

}
