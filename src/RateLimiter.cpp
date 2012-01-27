

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";

class RateLimiter
{
	public:
		int sleep_time_;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

  	//image_transport::Subscriber image_sub_;
		image_transport::SubscriberFilter image_sub_;
  	image_transport::Publisher  image_pub_;

		//ros::Subscriber cloud_sub_;
		message_filters::Subscriber<PointCloudXYZRGB> cloud_sub_;
		ros::Publisher  cloud_pub_;

		RateLimiter(int sleep_time);

		~RateLimiter()
		{
			cv::destroyWindow(WINDOW);
		}

	private:
		void imageCB(const sensor_msgs::ImageConstPtr& image_msg);
		void cloudCB(const PointCloudXYZRGB::ConstPtr& cloud_msg);
		void bothCB(const sensor_msgs::ImageConstPtr& image_msg, const PointCloudXYZRGB::ConstPtr& cloud_msg);

		typedef message_filters::sync_policies::ExactTime< sensor_msgs::Image, PointCloudXYZRGB > MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > sync_;
};


RateLimiter::RateLimiter(int sleep_time) : 
	it_(nh_),
	image_sub_( it_, "in_image", 1 ),
	cloud_sub_( nh_, "in_cloud", 1 ),
	sync_( MySyncPolicy(10), image_sub_, cloud_sub_ )
{
	cv::namedWindow(WINDOW);

	sleep_time_ = sleep_time;

	// Subscriber/publisher for the image
	std::string image_topic = nh_.resolveName("out_image");
  image_pub_ = it_.advertise(image_topic, 1);
  //image_sub_ = it_.subscribe("in_image", 1, &RateLimiter::imageCB, this);
  


	// Subscriber/publisher for the point cloud
	std::string cloud_topic = nh_.resolveName("out_cloud");
	cloud_pub_ = nh_.advertise<PointCloudXYZRGB> (cloud_topic, 1);
	//cloud_sub_ = nh_.subscribe<PointCloudXYZRGB>("in_cloud", 1, &RateLimiter::cloudCB, this);

	sync_.registerCallback( boost::bind( &RateLimiter::bothCB, this, _1, _2 ) );
}

void RateLimiter::bothCB(const sensor_msgs::ImageConstPtr& image_msg, const PointCloudXYZRGB::ConstPtr& cloud_msg)
{
	// Publish the data and wait to enforce rate
	ROS_INFO("Both");
  image_pub_.publish(image_msg);
	cloud_pub_.publish (cloud_msg);
	cv::waitKey(sleep_time_);
}


void RateLimiter::imageCB(const sensor_msgs::ImageConstPtr& image_msg)
{
	ROS_INFO("Image");
  image_pub_.publish(image_msg);
}


void RateLimiter::cloudCB(const PointCloudXYZRGB::ConstPtr& cloud_msg)
{
	// Publish the data and wait to enforce rate
	ROS_INFO("Cloud");
	cloud_pub_.publish (cloud_msg);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
	std::cout << "WTFMATE\n";
	int sleep_time;
	if( argc == 2 )
	{
		sleep_time = atoi(argv[1]);
		ROS_INFO("Using sleep time %d", sleep_time);
	}
	else
	{
		sleep_time = 500;
		ROS_INFO("Using standard sleep time %d", sleep_time);
	}


	RateLimiter rl(sleep_time);

	for( int i=0; i<20; i++ )
		ROS_INFO("Helloooo");

  ros::spin();
	return(0);
}
