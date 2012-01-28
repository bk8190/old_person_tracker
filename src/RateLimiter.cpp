

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
// PCL includes
#include "pcl_ros/point_cloud.h"
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// OpenCV includes
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;

class RateLimiter
{
	public:
		int sleep_time_;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		image_transport::SubscriberFilter image_sub_;
  	image_transport::Publisher  image_pub_;

		message_filters::Subscriber<PointCloudXYZRGB> cloud_sub_;
		ros::Publisher  cloud_pub_;

		message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;
		ros::Publisher cam_info_pub_;
	
		RateLimiter(int sleep_time);

	private:
		void bothCB(const sensor_msgs::ImageConstPtr& image_msg,
                const PointCloudXYZRGB::ConstPtr& cloud_msg,
                const sensor_msgs::CameraInfo::ConstPtr& cam_msg);

		typedef message_filters::sync_policies::ExactTime< sensor_msgs::Image, PointCloudXYZRGB, sensor_msgs::CameraInfo > MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > sync_;
};


RateLimiter::RateLimiter(int sleep_time) : 
	it_(nh_),
	image_sub_( it_, "in_image", 1 ),
	cloud_sub_( nh_, "in_cloud", 1 ),
	cam_info_sub_(nh_, "in_cam_info", 1),
	sync_( MySyncPolicy(10), image_sub_, cloud_sub_, cam_info_sub_ )
{
	sleep_time_ = sleep_time;

	// publisher for the image
	std::string image_topic = nh_.resolveName("out_image");
  image_pub_ = it_.advertise(image_topic, 1);

	// publisher for the point cloud
	std::string cloud_topic = nh_.resolveName("out_cloud");
	cloud_pub_ = nh_.advertise<PointCloudXYZRGB> (cloud_topic, 1);

	// publisher for the cam info
	std::string cam_info_topic = nh_.resolveName("out_cam_info");
	cam_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo> (cam_info_topic, 1);

	sync_.registerCallback( boost::bind( &RateLimiter::bothCB, this, _1, _2, _3) );
	ROS_INFO("RateLimiter constructor finished");
}

void RateLimiter::bothCB(const sensor_msgs::ImageConstPtr& image_msg, 
                         const PointCloudXYZRGB::ConstPtr& cloud_msg,
                         const sensor_msgs::CameraInfo::ConstPtr& cam_msg)
{
	// Publish the data and wait to enforce rate
	ROS_INFO("Got data");
  image_pub_.publish(image_msg);
	cloud_pub_.publish (cloud_msg);
	cam_info_pub_.publish(cam_msg);

	// Sleep for sleep_time ms to enforce rate
	float sleep_seconds = ((float)sleep_time_) / 1000;
	ros::Duration(sleep_seconds).sleep();
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "rate_limiter");

	int sleep_time;
	if( argc == 2 )
	{
		sleep_time = atoi(argv[1]);
		ROS_INFO("Using sleep time %d miliseconds", sleep_time);
	}
	else
	{
		sleep_time = 500;
		ROS_INFO("Using standard sleep time %d miliseconds", sleep_time);
	}

	RateLimiter rl(sleep_time);

  ros::spin();
	return(0);
}
