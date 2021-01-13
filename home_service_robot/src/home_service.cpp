#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

visualization_msgs::Marker create_marker(const double x, const double y, const double qz = 0.0, const double qw = 1.0, const int id = 0, uint32_t action = visualization_msgs::Marker::ADD)
{
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

class NavigationGoal
{
public:
  NavigationGoal(double gx, double gy, double gqz, double gqw, const ros::NodeHandle &n_)
  {
    x = gx;
    y = gy;
    qz = gqz;
    qw = gqw;
  }

  double x;
  double y;
  double qz;
  double qw;
  bool goal_achieved = false;

private:
  ros::NodeHandle n_;

  ros::Subscriber odom_sub = n_.subscribe("/odom", 100, &NavigationGoal::sub_callback, this);

  void sub_callback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    double goal_distance = sqrt(pow(msg->pose.pose.position.x - this->x, 2) + pow(msg->pose.pose.position.y - this->y, 2));
    ROS_INFO_STREAM("Distance to goal: "<<goal_distance);
    if (goal_distance < 0.5)
    {
      this->goal_achieved = true;
    }
  }
};

main(int argc, char **argv)
{
  ros::init(argc, argv, "home_service");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  visualization_msgs::Marker marker;
  NavigationGoal ng(8.72774124146, 1.64151453972, 0.0, 1.0, n);
  marker = create_marker(ng.x, ng.y, ng.qz, ng.qw, 0, visualization_msgs::Marker::ADD);
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
  while (!ng.goal_achieved)
  {
    r.sleep();
    ros::spinOnce();
  }
  marker = create_marker(ng.x, ng.y, ng.qz, ng.qw, 0, visualization_msgs::Marker::DELETE);
  marker_pub.publish(marker);
  ROS_INFO_STREAM("Picking object");
  ros::Duration(5.0).sleep();
  ng.x = 1.11344361305;
  ng.y = 1.08418285847;
  ng.qz = 1.0;
  ng.qw = 0.0;
  ng.goal_achieved=false;

  while (!ng.goal_achieved)
  {
    r.sleep();
    ros::spinOnce();
  }
  marker = create_marker(ng.x, ng.y, ng.qz, ng.qw, 1, visualization_msgs::Marker::ADD);
  marker_pub.publish(marker);
  ROS_INFO_STREAM("Object delivered successfully!");
  ros::Duration(5.0).sleep();
  // marker = create_marker(1.11344361305,1.08418285847,1.0,0.0,1,visualization_msgs::Marker::DELETE);
  // marker_pub.publish(marker);
  return 0;
}