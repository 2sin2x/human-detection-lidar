#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <math.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
std_msgs::Bool zone1_detect;
std_msgs::Bool zone2_detect;
std_msgs::Bool zone3_detect;

std_msgs::Bool zone1_prediction_detect;
std_msgs::Bool zone2_prediction_detect;
std_msgs::Bool zone3_prediction_detect;
geometry_msgs::Vector3 predicted_position;

geometry_msgs::Vector3 vector_vel_position;
geometry_msgs::Vector3 vector1min;
geometry_msgs::Vector3 vector1max;
geometry_msgs::Vector3 vector2min;
geometry_msgs::Vector3 vector2max;
geometry_msgs::Vector3 vector3min;
geometry_msgs::Vector3 vector3max;

void vector_prediction_callback(const geometry_msgs::Vector3 msg)
{
	predicted_position.x = msg.x;
	predicted_position.y = msg.y;
	predicted_position.z = msg.z;
}
void vectorCallback_vel_position(const geometry_msgs::Vector3 msg)
{
	vector_vel_position.x = msg.x;
	vector_vel_position.y = msg.y;
	vector_vel_position.z = msg.z;
}
void vectorCallback1min(const geometry_msgs::Vector3 msg)
{
	vector1min.x = msg.x;
	vector1min.y = msg.y;
	vector1min.z = msg.z;
}
void vectorCallback1max(const geometry_msgs::Vector3 msg)
{
	vector1max.x = msg.x;
	vector1max.y = msg.y;
	vector1max.z = msg.z;
}
void vectorCallback2min(const geometry_msgs::Vector3 msg)
{
	vector2min.x = msg.x;
	vector2min.y = msg.y;
	vector2min.z = msg.z;
}
void vectorCallback2max(const geometry_msgs::Vector3 msg)
{
	vector2max.x = msg.x;
	vector2max.y = msg.y;
	vector2max.z = msg.z;
}
void vectorCallback3min(const geometry_msgs::Vector3 msg)
{
	vector3min.x = msg.x;
	vector3min.y = msg.y;
	vector3min.z = msg.z;
}
void vectorCallback3max(const geometry_msgs::Vector3 msg)
{
	vector3max.x = msg.x;
	vector3max.y = msg.y;
	vector3max.z = msg.z;
}
void callback(const PointCloud::ConstPtr& msg)
{ 

  zone1_detect.data=false;
  zone2_detect.data=false;	
  zone3_detect.data=false;
  float xtest,ytest,ztest;

  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){

	xtest=pt.x+vector_vel_position.x;
	ytest=pt.y+vector_vel_position.y;
	ztest=pt.z+vector_vel_position.z;

	if ((xtest>vector1min.x)&&(xtest<vector1max.x)&&(ytest>vector1min.y)&&(ytest<vector1max.y)&&(ztest>vector1min.z)&&(ztest<vector1max.z)){
		zone1_detect.data=true;
	}

	if ((xtest>vector2min.x)&&(xtest<vector2max.x)&&(ytest>vector2min.y)&&(ytest<vector2max.y)&&(ztest>vector2min.z)&&(ztest<vector2max.z)){
		zone2_detect.data=true;
	}

	if ((xtest>vector3min.x)&&(xtest<vector3max.x)&&(ytest>vector3min.y)&&(ytest<vector3max.y)&&(ztest>vector3min.z)&&(ztest<vector3max.z)){
		zone3_detect.data=true;

	}	

  }

}

int main(int argc, char** argv)
{


  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_obstacles_new", 1, callback);
  ros::Subscriber vectorSub_vel_position = nh.subscribe("vector_vel_position", 1, vectorCallback_vel_position);
  ros::Subscriber vectorSub1min = nh.subscribe("vector1min", 1, vectorCallback1min);
  ros::Subscriber vectorSub1max = nh.subscribe("vector1max", 1, vectorCallback1max);
  ros::Subscriber vectorSub2min = nh.subscribe("vector2min", 1, vectorCallback2min);
  ros::Subscriber vectorSub2max = nh.subscribe("vector2max", 1, vectorCallback2max);
  ros::Subscriber vectorSub3min = nh.subscribe("vector3min", 1, vectorCallback3min);
  ros::Subscriber vectorSub3max = nh.subscribe("vector3max", 1, vectorCallback3max);
  ros::Publisher zone1_pub=nh.advertise<std_msgs::Bool>("zone1_detect", 1000);
  ros::Publisher zone2_pub=nh.advertise<std_msgs::Bool>("zone2_detect", 1000);
  ros::Publisher zone3_pub=nh.advertise<std_msgs::Bool>("zone3_detect", 1000);
  ros::Publisher zone1_prediction_pub=nh.advertise<std_msgs::Bool>("zone1_prediction_detect", 1000);
  ros::Publisher zone2_prediction_pub=nh.advertise<std_msgs::Bool>("zone2_prediction_detect", 1000);
  ros::Publisher zone3_prediction_pub=nh.advertise<std_msgs::Bool>("zone3_prediction_detect", 1000);
  ros::Subscriber vector_prediction = nh.subscribe("vector_prediction", 1, vector_prediction_callback);


  ros::Rate loop_rate(10);
  while (nh.ok()) {

  zone1_prediction_detect.data=false;
  zone2_prediction_detect.data=false;	
  zone3_prediction_detect.data=false;
  float xpr,ypr,zpr;
	xpr=predicted_position.x+vector_vel_position.x;
	ypr=predicted_position.y+vector_vel_position.y;
	zpr=predicted_position.z+vector_vel_position.z;

	if ((xpr>vector1min.x)&&(xpr<vector1max.x)&&(ypr>vector1min.y)&&(ypr<vector1max.y)&&(zpr>vector1min.z)&&(zpr<vector1max.z)){
		zone1_prediction_detect.data=true;
	}

	if ((xpr>vector2min.x)&&(xpr<vector2max.x)&&(ypr>vector2min.y)&&(ypr<vector2max.y)&&(zpr>vector2min.z)&&(zpr<vector2max.z)){
		zone2_prediction_detect.data=true;
	}

	if ((xpr>vector3min.x)&&(xpr<vector3max.x)&&(ypr>vector3min.y)&&(ypr<vector3max.y)&&(zpr>vector3min.z)&&(zpr<vector3max.z)){
		zone3_prediction_detect.data=true;

	}
    if ((predicted_position.x==0)&&(predicted_position.y==0)&&(predicted_position.z==0)){
  		zone1_prediction_detect.data=false;
  		zone2_prediction_detect.data=false;	
  		zone3_prediction_detect.data=false;

	}


  zone1_pub.publish(zone1_detect);
  zone2_pub.publish(zone2_detect);
  zone3_pub.publish(zone3_detect);

  zone1_prediction_pub.publish(zone1_prediction_detect);
  zone2_prediction_pub.publish(zone2_prediction_detect);
  zone3_prediction_pub.publish(zone3_prediction_detect);

  ros::spinOnce();
  loop_rate.sleep();

  }
}
