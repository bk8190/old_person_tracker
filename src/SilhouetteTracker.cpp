

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

class SilhouetteTracker
{
	public:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		image_transport::SubscriberFilter image_sub_;
  	image_transport::Publisher  image_pub_;

		message_filters::Subscriber<PointCloudXYZRGB> cloud_sub_;
	
		SilhouetteTracker();

	private:
		void bothCB(const sensor_msgs::ImageConstPtr& image_msg,
                const PointCloudXYZRGB::ConstPtr& cloud_msg);

		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, PointCloudXYZRGB> MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > sync_;
};


SilhouetteTracker::SilhouetteTracker() : 
	it_(nh_),
	image_sub_( it_, "in_image", 1 ),
	cloud_sub_( nh_, "in_cloud", 1 ),
	sync_( MySyncPolicy(10), image_sub_, cloud_sub_ )
{
	// publisher for the image
  image_pub_ = it_.advertise(nh_.resolveName("out_image"), 1);
	sync_.registerCallback( boost::bind( &SilhouetteTracker::bothCB, this, _1, _2) );
}

void SilhouetteTracker::bothCB(const sensor_msgs::ImageConstPtr& image_msg, 
                         const PointCloudXYZRGB::ConstPtr& cloud_msg)
{
	// Publish the data and wait to enforce rate
	ROS_INFO("Silhouette tracker got data");

  image_pub_.publish(image_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "silhouette_tracker");
	ROS_INFO("Initializing silhouette tracker.");
	SilhouetteTracker st();
  ros::spin();
	return(0);
}
