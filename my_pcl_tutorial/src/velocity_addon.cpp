#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

std_msgs::Bool empty;
std_msgs::Bool second_point_found;
std_msgs::Bool quad1;
std_msgs::Bool quad2;
std_msgs::Bool quad3;
std_msgs::Bool quad4;
ros::Time temp;

int counter=0;
float xfirst,xsecond;
float yfirst,ysecond;
float zfirst,zsecond;

float xoffset,yoffset,zoffset;
float distance;
double time_elapsed;
float velocity;

void callback(const PointCloud::ConstPtr& msg)
{


  float minimum1=100.0;
  float minimum2=100.0;
  float broj;

  float sumx=0;
  float sumy=0;
  float sumz=0;

  quad1.data=false;
  quad2.data=false;
  quad3.data=false;
  quad4.data=false;

  second_point_found.data=false;

  empty.data=true;
  if (msg->width==0){
  empty.data=true;
  }
  else{
  empty.data=false;
  }

  ros::Time begin=ros::Time::now();

  if ((counter==0)&&(empty.data==false)){
  temp=begin;
  }
  if ((counter==1)&&(empty.data==false)){
  begin=temp;
}
  if ((counter==1)&&(empty.data==true)){
  temp=begin;
  counter=0;
}
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){

	broj=sqrt(pt.x*pt.x+pt.y*pt.y+pt.z*pt.z);
	if ((empty.data==false)&&(counter==0)){
	sumx=sumx+pt.x;
	sumy=sumy+pt.y;
	sumz=sumz+pt.z;
	}
	if ((empty.data==false)&&(counter==1)){
	sumx=sumx+pt.x;
	sumy=sumy+pt.y;
	sumz=sumz+pt.z;
	}	
/*
	if ((broj>0)&&(broj<minimum1)&&(counter==0)){
	minimum1=broj;	
	xfirst=pt.x;
	yfirst=pt.y;
	}
	if ((broj>0)&&(broj<minimum2)&&(counter==9)){
	minimum2=broj;	
	xsecond=pt.x;
	ysecond=pt.y;
	}
*/


  }
 	ros::Time end=ros::Time::now();


	if ((counter==1)&&(empty.data==false)){
	xsecond=sumx/msg->width;
	ysecond=sumy/msg->width;
	zsecond=sumz/msg->width;
    second_point_found.data=true;
    counter=0;

	time_elapsed=(end-begin).toSec();
	xoffset=xfirst-xsecond;
	yoffset=yfirst-ysecond;
	zoffset=zfirst-zsecond;
	distance=sqrt(xoffset*xoffset+yoffset*yoffset);
	velocity=distance/time_elapsed;

		if ((xfirst<xsecond)&&(yfirst<ysecond)){
		//std::cout<<"velocity: "<<velocity<<" going to +x and to +y "<<std::endl;
		quad1.data=true;
		}
		if ((xfirst<xsecond)&&(yfirst>ysecond)){
		//std::cout<<"velocity: "<<velocity<<" going to +x and to -y"<<std::endl;
		quad4.data=true;
		}
		if ((xfirst>xsecond)&&(yfirst<ysecond)){
		//std::cout<<"velocity: "<<velocity<<" going to -x and to +y"<<std::endl;
		quad2.data=true;
		}
		if ((xfirst>xsecond)&&(yfirst>ysecond)){
		//std::cout<<"velocity: "<<velocity<<" going to -x and to -y"<<std::endl;
		quad3.data=true;
		}

	}

    if ((counter==0)&&(empty.data==false)&&(second_point_found.data==false)){
	xfirst=sumx/msg->width;
	yfirst=sumy/msg->width;
	zfirst=sumz/msg->width;
	counter=1;
	}


	//std::cout<<"elapsed: "<<time_elapsed<<" distance: "<<distance<<" vel "<<velocity<<std::endl;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_addon");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_obstacles_new", 1, callback);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
  ros::Publisher vector_prediction = nh.advertise<geometry_msgs::Vector3>("vector_prediction", 1);


  float array_point_x[1000]={0};
  float array_point_y[1000]={0};
  float array_calculated_point_x[1000]={0};
  float array_calculated_point_y[1000]={0};
  float xnew,ynew;
  float k_ratio;
  int count_loop=0;
  ros::Rate loop_rate(10);

  while (nh.ok()) {


  geometry_msgs::Point temp_point;


  visualization_msgs::Marker marker;
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time();
  marker.ns = "arrow";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  temp_point.x = xfirst;
  temp_point.y = yfirst; 
  temp_point.z = zfirst;
  marker.points.push_back(temp_point);

  temp_point.x = xsecond;
  temp_point.y = ysecond; 
  temp_point.z = zsecond;
  marker.points.push_back(temp_point);

  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  marker.color.a = 0.7;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;


  k_ratio=yoffset/xoffset;

  if ((quad1.data==true)||(quad4.data==true)){
  xnew=sqrt((pow((velocity),2.0))/(k_ratio*k_ratio+1));
  }
  if ((quad2.data==true)||(quad3.data==true)){
  xnew=-sqrt((pow((velocity),2.0))/(k_ratio*k_ratio+1));
  }

  ynew=k_ratio*xnew;

  geometry_msgs::Vector3 predicted_position;
/*	
  predicted_position.x=xnew+xfirst;
  predicted_position.y=ynew+yfirst;
*/
  predicted_position.x=xnew+xsecond;
  predicted_position.y=ynew+ysecond;
  predicted_position.z=0;

  if (second_point_found.data==true){
  vector_prediction.publish(predicted_position);
} 

//begin line

/*
  if (second_point_found.data==true){
  array_point_x[count_loop]=xfirst;
  array_point_y[count_loop]=yfirst;
  //array_point_x[count_loop+1]=xsecond;
  //array_point_y[count_loop+1]=ysecond;
  array_calculated_point_x[count_loop]=xnew+xfirst;
  array_calculated_point_y[count_loop]=ynew+yfirst;
  //std::cout<<"value "<<array_point_x[count_loop]<<std::endl;
  count_loop=count_loop+1;
  }


  if ((empty.data==true)&&(array_point_x[2]!=0)){

  visualization_msgs::Marker line;
  line.header.frame_id = "velodyne";
  line.header.stamp = ros::Time();
  line.ns = "line";
  line.id = 0;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  line.action = visualization_msgs::Marker::ADD;

  visualization_msgs::Marker line_calculated;
  line_calculated.header.frame_id = "velodyne";
  line_calculated.header.stamp = ros::Time();
  line_calculated.ns = "line_calculated";
  line_calculated.id = 0;
  line_calculated.type = visualization_msgs::Marker::LINE_STRIP;
  line_calculated.action = visualization_msgs::Marker::ADD;

  for (int a=0;a<count_loop;a=a+1){
    geometry_msgs::Point point;
    geometry_msgs::Point calculated_point;
    point.x=array_point_x[a];
    point.y=array_point_y[a];
    calculated_point.x=array_calculated_point_x[a];
    calculated_point.y=array_calculated_point_y[a];

    line.points.push_back(point);
    line_calculated.points.push_back(calculated_point);
    }

  line.scale.x = 0.2;
  line.color.a = 0.7;
  line.color.r = 0.0;
  line.color.g = 1.0;
  line.color.b = 0.0;

  line_calculated.scale.x = 0.2;
  line_calculated.color.a = 0.7;
  line_calculated.color.r = 1.0;
  line_calculated.color.g = 0.0;
  line_calculated.color.b = 0.0;


  memset(array_point_x, 0, sizeof(array_point_x));
  memset(array_point_y, 0, sizeof(array_point_y));
  memset(array_calculated_point_x, 0, sizeof(array_calculated_point_x));
  memset(array_calculated_point_y, 0, sizeof(array_calculated_point_y));   

  marker_pub.publish(line);
  marker_pub.publish(line_calculated);
  }

  if (empty.data==true){
  count_loop=0;
  }
  

*/
//end line




  visualization_msgs::Marker marker2;
  marker2.header.frame_id = "velodyne";
  marker2.header.stamp = ros::Time();
  marker2.ns = "after 1second1";
  marker2.id = 0;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = xnew+xsecond;
  marker2.pose.position.y = ynew+ysecond;
/*
  marker2.pose.position.x = xnew+xfirst;
  marker2.pose.position.y = ynew+yfirst;
*/
  marker2.pose.position.z = 0;
  marker2.pose.orientation.x = 0.0;
  marker2.pose.orientation.y = 0.0;
  marker2.pose.orientation.z = 0.0;
  marker2.pose.orientation.w = 1.0;
  marker2.scale.x = 0.2;
  marker2.scale.y = 0.2;
  marker2.scale.z = 0.2;
  marker2.color.a = 1.0; // Don't forget to set the alpha!
  marker2.color.r = 0.0;
  marker2.color.g = 1.0;
  marker2.color.b = 0.0;

  if (second_point_found.data==true){

  marker_pub.publish(marker);
  marker_pub.publish(marker2);
  }

  ros::spinOnce();
  loop_rate.sleep();
  }
}
