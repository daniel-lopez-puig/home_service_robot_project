#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


visualization_msgs::Marker create_marker(const float x, const float y, const float qz=0.0, const float qw=1.0, const int id=0, uint32_t action=visualization_msgs::Marker::ADD){
    visualization_msgs::Marker marker;
    uint32_t shape = visualization_msgs::Marker::CUBE;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "pick_and_drop";
    marker.id = id;
    marker.type = shape;

    marker.lifetime = ros::Duration();
    marker.action = action;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.orientation.z = qz;
    marker.pose.orientation.w = qw;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    return marker;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    visualization_msgs::Marker marker;
    marker = create_marker(8.72774124146,1.64151453972,0.0,1.0,0,visualization_msgs::Marker::ADD);
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    marker = create_marker(8.72774124146,1.64151453972,0.0,1.0,0,visualization_msgs::Marker::DELETE);
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep();
    marker = create_marker(1.11344361305,1.08418285847,1.0,0.0,1,visualization_msgs::Marker::ADD);
    marker_pub.publish(marker);
    // ros::Duration(5.0).sleep();
    // marker = create_marker(1.11344361305,1.08418285847,1.0,0.0,1,visualization_msgs::Marker::DELETE);
    // marker_pub.publish(marker);

}