#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/Vector3.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
int count=0;
int count2=0;

float bok=0;
float x1[2520]={0};
float x2[2520]={0};
float yp[2520]={0};
float y2[2520]={0};
float z1[2520]={0};
float z2[2520]={0};
geometry_msgs::Vector3 vec_back;
geometry_msgs::Vector3 vec_full;
geometry_msgs::Vector3 vec_motion;
void callback_full(const PointCloud::ConstPtr& msg)
{ 

  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
	vec_full.x=pt.x;
	vec_full.y=pt.y;
	vec_full.z=pt.z;
	x1[count]=pt.x;
 	yp[count]=pt.y;
	z1[count]=pt.z;
	count=count+1;
	if (count==msg->width)
		count=0;
	if ((pt.y<0)&&(pt.y>-0.05)&&(pt.x<-3)){
    //printf ("\t(FULL: %f, %f, %f, %f,%f)\n", pt.x, pt.y, pt.z, vec_full.z,count);
	//printf ("\t(%f, %f, %f)\n", vec_full.x, vec_full.y, vec_full.z);
	}
}
}

void callback_back(const PointCloud::ConstPtr& msg)
{ 

  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points){
	vec_back.x=pt.x;
	vec_back.y=pt.y;
	vec_back.z=pt.z;
	x2[count2]=pt.x;
 	y2[count2]=pt.y;
	z2[count2]=pt.z;
	count2=count2+1;
	if (count2==msg->width)
		count2=0;
	if ((pt.y<0)&&(pt.y>-0.05)&&(pt.x<-3)){
	//printf ("\t(%f, %f, %f)\n", vec_full.x, vec_full.y, vec_full.z);
	//printf ("\t(%f, %f, %f)\n", vec_full.x-vec_back.x, vec_full.y-vec_back.y, vec_full.z-vec_back.z);
    //printf ("\t(BACK: %f, %f, %f, %f,%f)\n", pt.x, pt.y, pt.z, vec_full.z,count2);
		}
	}
/*
  if ((vec_full.x-vec_back.x==0)&&(vec_full.y-vec_back.y==0)&&(vec_full.z-vec_back.z==0)){
  vec_motion.x=0;
  vec_motion.y=0;
  vec_motion.z=0;
}
else {
  vec_motion.x=vec_full.x;
  vec_motion.y=vec_full.y;
  vec_motion.z=vec_full.z;
}
*/
}

int main(int argc, char** argv)
{
  int count3=0;
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  ros::Subscriber sub_full = nh.subscribe<PointCloud>("velodyne_points", 1, callback_full);
  ros::Subscriber sub_back = nh.subscribe<PointCloud>("bar", 1, callback_back);
  //ros::spin();
  

  //if ((vec_full.x-vec_back.x<0.01)&&(vec_full.x-vec_back.x>-0.01)&&(vec_full.y-vec_back.y<0.01)&&(vec_full.y-vec_back.y>-0.01)&&(vec_full.z-vec_back.z<0.01)&&(vec_full.z-vec_back.z>-0.01)){

  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
  for (int i=0;i<2520;i=i+1){
printf("%f %f %f",x1[i],yp[i],z1[i]);
/*
	if ((x1[i]-x2[i]>-0.05)&&(x1[i]-x2[i]<0.05)&&(yp[i]-y2[i]>-0.05)&&(yp[i]-y2[i]<0.05)&&(z1[i]-z2[i]>-0.05)&&(z1[i]-z2[i]<0.05)){
}
	else
		count3=count3+1;
*/


}




  ros::Rate loop_rate(10);
  while (nh.ok())
  {
	//printf("Ovoliko puta se mijenja: %d",count3);
   	PointCloud::Ptr msg (new PointCloud);
  	msg->header.frame_id = "base_footprint";
 	msg->height = msg->width = 1;
    msg->points.push_back (pcl::PointXYZ(1, 1, 1));
    pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
    pub.publish (msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }


}
