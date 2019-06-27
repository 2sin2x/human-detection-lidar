/*!
 * \file robot_pose_publisher.cpp
 * \brief Publishes the robot's position in a geometry_msgs/Pose message.
 *
 * Publishes the robot's position in a geometry_msgs/Pose message based on the TF
 * difference between /map and /base_link.
 *
 * \author Russell Toris - rctoris@wpi.edu
 * \date April 3, 2014
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"
#include <sstream>
#include "std_msgs/Bool.h"
  std_msgs::Bool zone1_detect;
  std_msgs::Bool zone2_detect;
  std_msgs::Bool zone3_detect;

  std_msgs::Bool zone1_prediction_detect;
  std_msgs::Bool zone2_prediction_detect;
  std_msgs::Bool zone3_prediction_detect;

void zone1Callback(const std_msgs::Bool msg)
{
  zone1_detect.data=msg.data;
}


void zone2Callback(const std_msgs::Bool msg)
{
  zone2_detect.data=msg.data;
}


void zone3Callback(const std_msgs::Bool msg)
{
  zone3_detect.data=msg.data;
}



void zone1_pr_Callback(const std_msgs::Bool msg)
{
  zone1_prediction_detect.data=msg.data;
}


void zone2_pr_Callback(const std_msgs::Bool msg)
{
  zone2_prediction_detect.data=msg.data;
}


void zone3_pr_Callback(const std_msgs::Bool msg)
{
  zone3_prediction_detect.data=msg.data;
}



int main(int argc, char ** argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  // configuring parameters
  std::string map_frame, base_frame ,tool_frame, vel_frame;
  double publish_frequency;
  bool is_stamped;
  ros::Publisher p_pub;

  zone1_detect.data=false;
  zone2_detect.data=false;	
  zone3_detect.data=false;



  //publishers and subscribers
  ros::Subscriber zone1_sub = nh.subscribe("zone1_detect", 1000, zone1Callback);
  ros::Subscriber zone2_sub = nh.subscribe("zone2_detect", 1000, zone2Callback);
  ros::Subscriber zone3_sub = nh.subscribe("zone3_detect", 1000, zone3Callback);

  ros::Subscriber zone1_pr_sub = nh.subscribe("zone1_prediction_detect", 1000, zone1_pr_Callback);
  ros::Subscriber zone2_pr_sub = nh.subscribe("zone2_prediction_detect", 1000, zone2_pr_Callback);
  ros::Subscriber zone3_pr_sub = nh.subscribe("zone3_prediction_detect", 1000, zone3_pr_Callback);

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub2 = nh.advertise<std_msgs::String>("chatter2", 1000);
  ros::Publisher vector_vel_position = nh.advertise<geometry_msgs::Vector3>("vector_vel_position", 1);
  ros::Publisher vector1min = nh.advertise<geometry_msgs::Vector3>("vector1min", 1);
  ros::Publisher vector1max = nh.advertise<geometry_msgs::Vector3>("vector1max", 1);
  ros::Publisher vector2min = nh.advertise<geometry_msgs::Vector3>("vector2min", 1);
  ros::Publisher vector2max = nh.advertise<geometry_msgs::Vector3>("vector2max", 1);
  ros::Publisher vector3min = nh.advertise<geometry_msgs::Vector3>("vector3min", 1);
  ros::Publisher vector3max = nh.advertise<geometry_msgs::Vector3>("vector3max", 1);
  uint32_t shape = visualization_msgs::Marker::CUBE;
  int counttt=0;
  float xmin,xmax,xmin2,xmax2,xmin3,xmax3;
  float ymin,ymax,ymin2,ymax2,ymin3,ymax3;
  float zmin,zmax,zmin2,zmax2,zmin3,zmax3;


  nh_priv.param<std::string>("map_frame",map_frame,"/base_footprint");
  nh_priv.param<std::string>("tool_frame",tool_frame,"/link_6");
  nh_priv.param<std::string>("base_frame",base_frame,"/link_1");
  nh_priv.param<std::string>("vel_frame",vel_frame,"/velodyne");

  nh_priv.param<double>("publish_frequency",publish_frequency,10);
  nh_priv.param<bool>("is_stamped", is_stamped, false);

  if(is_stamped)
    p_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
  else 
    p_pub = nh.advertise<geometry_msgs::Pose>("robot_pose", 1);

  // create the listener
  tf::TransformListener listener;
  listener.waitForTransform(map_frame, base_frame, ros::Time(), ros::Duration(1.0));

  tf::TransformListener listener2;
  listener2.waitForTransform(map_frame, tool_frame, ros::Time(), ros::Duration(1.0));

  tf::TransformListener listener3;
  listener3.waitForTransform(map_frame, vel_frame, ros::Time(), ros::Duration(1.0));

  ros::Rate rate(publish_frequency);
  while (nh.ok())
  {
    tf::StampedTransform transform;
	tf::StampedTransform transform2;
    tf::StampedTransform transform3;


	  listener.lookupTransform(map_frame, base_frame, ros::Time(0), transform);
      // construct a pose message for footprint to base
      geometry_msgs::PoseStamped pose_stamped;
      pose_stamped.header.frame_id = map_frame;
      pose_stamped.header.stamp = ros::Time::now();

      pose_stamped.pose.orientation.x = transform.getRotation().getX();
      pose_stamped.pose.orientation.y = transform.getRotation().getY();
      pose_stamped.pose.orientation.z = transform.getRotation().getZ();
      pose_stamped.pose.orientation.w = transform.getRotation().getW();

      pose_stamped.pose.position.x = transform.getOrigin().getX();
      pose_stamped.pose.position.y = transform.getOrigin().getY();
      pose_stamped.pose.position.z = transform.getOrigin().getZ();

      if(is_stamped)
        p_pub.publish(pose_stamped);
      else
        p_pub.publish(pose_stamped.pose);

	  listener2.lookupTransform(map_frame, tool_frame, ros::Time(0), transform2);
      // construct a pose message for footprint to tool
      geometry_msgs::PoseStamped pose_stamped2;
      pose_stamped2.header.frame_id = map_frame;
      pose_stamped2.header.stamp = ros::Time::now();

      pose_stamped2.pose.orientation.x = transform2.getRotation().getX();
      pose_stamped2.pose.orientation.y = transform2.getRotation().getY();
      pose_stamped2.pose.orientation.z = transform2.getRotation().getZ();
      pose_stamped2.pose.orientation.w = transform2.getRotation().getW();

      pose_stamped2.pose.position.x = transform2.getOrigin().getX();
      pose_stamped2.pose.position.y = transform2.getOrigin().getY();
      pose_stamped2.pose.position.z = transform2.getOrigin().getZ();

      if(is_stamped)
        p_pub.publish(pose_stamped2);
      else
        p_pub.publish(pose_stamped2.pose);

	  listener3.lookupTransform(map_frame, vel_frame, ros::Time(0), transform3);
      // construct a pose message for footprint to velodyne
      geometry_msgs::PoseStamped pose_stamped3;
      pose_stamped3.header.frame_id = map_frame;
      pose_stamped3.header.stamp = ros::Time::now();

      pose_stamped3.pose.orientation.x = transform3.getRotation().getX();
      pose_stamped3.pose.orientation.y = transform3.getRotation().getY();
      pose_stamped3.pose.orientation.z = transform3.getRotation().getZ();
      pose_stamped3.pose.orientation.w = transform3.getRotation().getW();

      pose_stamped3.pose.position.x = transform3.getOrigin().getX();
      pose_stamped3.pose.position.y = transform3.getOrigin().getY();
      pose_stamped3.pose.position.z = transform3.getOrigin().getZ();

      if(is_stamped)
        p_pub.publish(pose_stamped3);
      else
        p_pub.publish(pose_stamped3.pose);

	  geometry_msgs::Vector3 msg_vel_position;	
	  msg_vel_position.x=pose_stamped3.pose.position.x;
	  msg_vel_position.y=pose_stamped3.pose.position.y;
	  msg_vel_position.z=pose_stamped3.pose.position.z;
	  vector_vel_position.publish(msg_vel_position);
/*
	  std::cout<<"x1: "<<pose_stamped.pose.position.x<<" x2: "<<pose_stamped2.pose.position.x<<std::endl;
      std::cout<<"y1: "<<pose_stamped.pose.position.y<<" y2: "<<pose_stamped2.pose.position.y<<std::endl;
	  std::cout<<"z1: "<<pose_stamped.pose.position.z<<" z2: "<<pose_stamped2.pose.position.z<<std::endl;
*/

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_footprint";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robot";
    marker.id = 0;
	marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = (pose_stamped2.pose.position.x+pose_stamped.pose.position.x)/2;
    marker.pose.position.y = (pose_stamped2.pose.position.y+pose_stamped.pose.position.y)/2;
    marker.pose.position.z = (pose_stamped2.pose.position.z+pose_stamped.pose.position.z)/2;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1.0;
/*
    marker.scale.x = fabs(pose_stamped2.pose.position.x-pose_stamped.pose.position.x)+3;
    marker.scale.y = fabs(pose_stamped2.pose.position.y-pose_stamped.pose.position.y)+3;
    marker.scale.z = fabs(pose_stamped2.pose.position.z-pose_stamped.pose.position.z)+1;
*/
    marker.scale.x = 3;
    marker.scale.y = 3;
    marker.scale.z = 1.8;


/*
if (zone1_detect.data==false){
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
}
else{
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;
}
*/

  if ((zone1_detect.data==false)&&(zone1_prediction_detect.data==false)){
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  }

  if ((zone1_detect.data==false)&&(zone1_prediction_detect.data==true)){
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  }

  if (zone1_detect.data==true){
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  }

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = "/base_footprint";
    marker2.header.stamp = ros::Time::now();
    marker2.ns = "disable robot";
    marker2.id = 0;
	marker2.type = visualization_msgs::Marker::CUBE;
    marker2.action = visualization_msgs::Marker::ADD;

    marker2.pose.position.x = marker.pose.position.x;
    marker2.pose.position.y = marker.pose.position.y;
    marker2.pose.position.z = marker.pose.position.z;
    marker2.pose.orientation.x = 0;
    marker2.pose.orientation.y = 0;
    marker2.pose.orientation.z = 0;
    marker2.pose.orientation.w = 1.0;
/*
    marker2.scale.x = fabs(pose_stamped2.pose.position.x-pose_stamped.pose.position.x)+5.6;
    marker2.scale.y = fabs(pose_stamped2.pose.position.y-pose_stamped.pose.position.y)+5.6;
    marker2.scale.z = fabs(pose_stamped2.pose.position.z-pose_stamped.pose.position.z)+1.3;
*/
    marker2.scale.x = 5.6;
    marker2.scale.y = 5.6;
    marker2.scale.z = 1.8;

/*
if (zone2_detect.data==false){
    marker2.color.r = 0.0f;
    marker2.color.g = 1.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 0.3;
}
else{
    marker2.color.r = 1.0f;
    marker2.color.g = 0.0f;
    marker2.color.b = 0.0f;
    marker2.color.a = 0.3;
}
*/
  if ((zone2_detect.data==false)&&(zone2_prediction_detect.data==false)){
  marker2.color.a = 0.3;
  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  }

  if ((zone2_detect.data==false)&&(zone2_prediction_detect.data==true)){
  marker2.color.a = 0.3;
  marker2.color.r = 1.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;
  }

  if (zone2_detect.data==true){
  marker2.color.a = 0.3;
  marker2.color.r = 1.0;
  marker2.color.g = 0.0;
  marker2.color.b = 0.0;
  }

    visualization_msgs::Marker marker3;
    marker3.header.frame_id = "/base_footprint";
    marker3.header.stamp = ros::Time::now();
    marker3.ns = "attention";
    marker3.id = 0;
	marker3.type = visualization_msgs::Marker::CUBE;
    marker3.action = visualization_msgs::Marker::ADD;

    marker3.pose.position.x = marker.pose.position.x;
    marker3.pose.position.y = marker.pose.position.y;
    marker3.pose.position.z = marker.pose.position.z;
    marker3.pose.orientation.x = 0;
    marker3.pose.orientation.y = 0;
    marker3.pose.orientation.z = 0;
    marker3.pose.orientation.w = 1.0;
/*
    marker3.scale.x = fabs(pose_stamped2.pose.position.x-pose_stamped.pose.position.x)+8.2;
    marker3.scale.y = fabs(pose_stamped2.pose.position.y-pose_stamped.pose.position.y)+8.2;
    marker3.scale.z = fabs(pose_stamped2.pose.position.z-pose_stamped.pose.position.z)+1.3;
*/
    marker3.scale.x = 8.2;
    marker3.scale.y = 8.2;
    marker3.scale.z = 1.8;
/*
if (zone3_detect.data==false){
    marker3.color.r = 0.0f;
    marker3.color.g = 1.0f;
    marker3.color.b = 0.0f;
    marker3.color.a = 0.2;
}
else{
    marker3.color.r = 1.0f;
    marker3.color.g = 0.0f;
    marker3.color.b = 0.0f;
    marker3.color.a = 0.2;
}
*/

  if ((zone3_detect.data==false)&&(zone3_prediction_detect.data==false)){
  marker3.color.a = 0.2;
  marker3.color.r = 0.0;
  marker3.color.g = 1.0;
  marker3.color.b = 0.0;
  }

  if ((zone3_detect.data==false)&&(zone3_prediction_detect.data==true)){
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
//math cube



// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
    marker2.lifetime = ros::Duration();
    marker3.lifetime = ros::Duration();
// %EndTag(LIFETIME)%


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
    marker_pub.publish(marker2);
    marker_pub.publish(marker3);

//math
	xmin=marker.pose.position.x-(marker.scale.x/2);
	ymin=marker.pose.position.y-(marker.scale.y/2);
	zmin=marker.pose.position.z-(marker.scale.z/2);

    xmax=marker.pose.position.x+(marker.scale.x/2);
    ymax=marker.pose.position.y+(marker.scale.y/2);
    zmax=marker.pose.position.z+(marker.scale.z/2);


	xmin2=marker.pose.position.x-(marker2.scale.x/2);
	ymin2=marker.pose.position.y-(marker2.scale.y/2);
	zmin2=marker.pose.position.z-(marker2.scale.z/2);

    xmax2=marker.pose.position.x+(marker2.scale.x/2);
    ymax2=marker.pose.position.y+(marker2.scale.y/2);
    zmax2=marker.pose.position.z+(marker2.scale.z/2);


	xmin3=marker.pose.position.x-(marker3.scale.x/2);
	ymin3=marker.pose.position.y-(marker3.scale.y/2);
	zmin3=marker.pose.position.z-(marker3.scale.z/2);

    xmax3=marker.pose.position.x+(marker3.scale.x/2);
    ymax3=marker.pose.position.y+(marker3.scale.y/2);
    zmax3=marker.pose.position.z+(marker3.scale.z/2);


	geometry_msgs::Vector3 msg1min;
	msg1min.x=xmin;
	msg1min.y=ymin;
	msg1min.z=zmin;
	vector1min.publish(msg1min);

	geometry_msgs::Vector3 msg1max;
	msg1max.x=xmax;
	msg1max.y=ymax;
	msg1max.z=zmax;
	vector1max.publish(msg1max);

	geometry_msgs::Vector3 msg2min;
	msg2min.x=xmin2;
	msg2min.y=ymin2;
	msg2min.z=zmin2;
	vector2min.publish(msg2min);

	geometry_msgs::Vector3 msg2max;
	msg2max.x=xmax2;
	msg2max.y=ymax2;
	msg2max.z=zmax2;
	vector2max.publish(msg2max);

	geometry_msgs::Vector3 msg3min;
	msg3min.x=xmin3;
	msg3min.y=ymin3;
	msg3min.z=zmin3;
	vector3min.publish(msg3min);

	geometry_msgs::Vector3 msg3max;
	msg3max.x=xmax3;
	msg3max.y=ymax3;
	msg3max.z=zmax3;
	vector3max.publish(msg3max);


//proba za msg
  std_msgs::String msg2;
  std::stringstream ss;
  std_msgs::String msgg;
  std::stringstream ssg;
  ss << "position of velodyne is x:" <<transform3.getOrigin().getX()<<" y:"<<transform3.getOrigin().getY()<<" z:"<<transform3.getOrigin().getZ();
  ssg << "cube x:"<<xmin<<" to "<<xmax<< " y:"<<ymin<<" to "<<ymax<< " z:"<<zmin<<" to "<<zmax<<" ";
  ssg << "cube x:"<<xmin2<<" to "<<xmax2<< " y:"<<ymin2<<" to "<<ymax2<< " z:"<<zmin2<<" to "<<zmax2<<" ";
  ssg << "cube x:"<<xmin3<<" to "<<xmax3<< " y:"<<ymin3<<" to "<<ymax3<< " z:"<<zmin3<<" to "<<zmax3;
  msg2.data = ss.str();
  msgg.data = ssg.str();
  //ROS_INFO("%s", msg2.data.c_str());
  chatter_pub.publish(msg2);
  chatter_pub2.publish(msgg);
  ros::spinOnce();
  ++counttt;



    rate.sleep();
  }

  return EXIT_SUCCESS;
}
