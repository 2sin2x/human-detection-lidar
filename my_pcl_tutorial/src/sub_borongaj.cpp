#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <math.h>
#include <visualization_msgs/Marker.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
std_msgs::Bool zone1_detect;
std_msgs::Bool zone2_detect;
std_msgs::Bool zone3_detect;
geometry_msgs::Vector3 predicted_position;

void vector_prediction_callback(const geometry_msgs::Vector3 msg)
{
	predicted_position.x = msg.x;
	predicted_position.y = msg.y;
	predicted_position.z = msg.z;
}

void callback(const PointCloud::ConstPtr& msg)
{
  float distance;

  zone1_detect.data=false;
  zone2_detect.data=false;	
  zone3_detect.data=false;

  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
	distance=sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
	if (distance<5){
	zone3_detect.data=true;	
	}
	if (distance<3.2){
	zone2_detect.data=true;	
	}
	if (distance<2){
	zone1_detect.data=true;	
	}	
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_obstacles", 1, callback);
  ros::Publisher zone1_pub=nh.advertise<std_msgs::Bool>("zone1_detect", 1000);
  ros::Publisher zone2_pub=nh.advertise<std_msgs::Bool>("zone2_detect", 1000);
  ros::Publisher zone3_pub=nh.advertise<std_msgs::Bool>("zone3_detect", 1000);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::Subscriber vector_prediction = nh.subscribe("vector_prediction", 1, vector_prediction_callback);

  ros::Rate loop_rate(10);
  while (nh.ok()) {

  float predicted_distance;
  bool  zone1_prediction_detect=false;
  bool  zone2_prediction_detect=false;
  bool  zone3_prediction_detect=false;

  predicted_distance=sqrt(predicted_position.x*predicted_position.x+predicted_position.y*predicted_position.y);
    if ((predicted_distance<5)&&(predicted_distance>0)){
	zone3_prediction_detect=true;	
	}
	if ((predicted_distance<3.2)&&(predicted_distance>0)){
	zone2_prediction_detect=true;	
	}
	if ((predicted_distance<2)&&(predicted_distance>0)){
	zone1_prediction_detect=true;	
	}	

  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time();
  marker.ns = "zone1";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 2*2;
  marker.scale.y = 2*2;
  marker.scale.z = 2*2;

  if ((zone1_detect.data==false)&&(zone1_prediction_detect==false)){
  marker.color.a = 0.7;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  }

  if ((zone1_detect.data==false)&&(zone1_prediction_detect==true)){
  marker.color.a = 0.7;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  }

  if (zone1_detect.data==true){
  marker.color.a = 0.7;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  }


  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "velodyne";
  marker2.header.stamp = ros::Time();
  marker2.ns = "zone2";
  marker2.id = 0;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = 0;
  marker2.pose.position.y = 0;
  marker2.pose.position.z = 0;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;

  marker2.scale.x = 3.2*2;
  marker2.scale.y = 3.2*2;
  marker2.scale.z = 3.2*2;

  if ((zone2_detect.data==false)&&(zone2_prediction_detect==false)){
  marker2.color.a = 0.4;
  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  }

  if ((zone2_detect.data==false)&&(zone2_prediction_detect==true)){
  marker2.color.a = 0.4;
  marker2.color.r = 1.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  }

  if (zone2_detect.data==true){
  marker2.color.a = 0.4;
  marker2.color.r = 1.0;
  marker2.color.g = 0.0;
  marker2.color.b = 0.0;
  }

  visualization_msgs::Marker marker3;
  marker3.header.frame_id = "velodyne";
  marker3.header.stamp = ros::Time();
  marker3.ns = "zone3";
  marker3.id = 0;
  marker3.type = visualization_msgs::Marker::SPHERE;
  marker3.action = visualization_msgs::Marker::ADD;
  marker3.pose.position.x = 0;
  marker3.pose.position.y = 0;
  marker3.pose.position.z = 0;
  marker3.pose.orientation.x = 0.0;
  marker3.pose.orientation.y = 0.0;
  marker3.pose.orientation.z = 0.0;
  marker3.pose.orientation.w = 1.0;

  marker3.scale.x = 5*2;
  marker3.scale.y = 5*2;
  marker3.scale.z = 5*2;

  if ((zone3_detect.data==false)&&(zone3_prediction_detect==false)){
  marker3.color.a = 0.2;
  marker3.color.r = 0.0;
  marker3.color.g = 1.0;
  marker3.color.b = 0.0;
  }

  if ((zone3_detect.data==false)&&(zone3_prediction_detect==true)){
  marker3.color.a = 0.2;
  marker3.color.r = 1.0;
  marker3.color.g = 1.0;
  marker3.color.b = 0.0;
  }

  if (zone3_detect.data==true){
  marker3.color.a = 0.2;
  marker3.color.r = 1.0;
  marker3.color.g = 0.0;
  marker3.color.b = 0.0;
  }

  marker_pub.publish(marker);
  marker_pub.publish(marker2);
  marker_pub.publish(marker3);

  zone1_pub.publish(zone1_detect);
  zone2_pub.publish(zone2_detect);
  zone3_pub.publish(zone3_detect);
  ros::spinOnce();
  loop_rate.sleep();
  }
}
