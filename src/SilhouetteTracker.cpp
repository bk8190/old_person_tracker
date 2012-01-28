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
#include <opencv2/imgproc/imgproc.hpp>

#define MAX_DIST (7.0)

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
static const char WINDOW[] = "Image window";
static const char WINDOW_EDGES[] = "Edges";


class SilhouetteTracker
{
	public:
		int var_;
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		image_transport::SubscriberFilter image_sub_;
  	image_transport::Publisher  image_pub_;

		message_filters::Subscriber<PointCloudXYZRGB> cloud_sub_;
	
		SilhouetteTracker(int var);
		~SilhouetteTracker()
		{
			cv::destroyWindow(WINDOW);
			cv::destroyWindow(WINDOW_EDGES);
		}

	private:
		void bothCB(const sensor_msgs::ImageConstPtr& image_msg,
                const PointCloudXYZRGB::ConstPtr& cloud_msg);

		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, PointCloudXYZRGB> MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > sync_;
};


SilhouetteTracker::SilhouetteTracker(int var) : 
	nh_("silhouette_tracker"),	
	it_(nh_),
//	image_sub_( it_, "in_image", 1 ),
//	cloud_sub_( nh_, "in_cloud", 1 ),
	image_sub_( it_, "/camera/depth_registered/image_rect", 1),
	cloud_sub_( nh_, "/camera/depth_registered/points", 1 ),
	sync_( MySyncPolicy(2), image_sub_, cloud_sub_ )
{
	// publisher for the image
	std::string image_topic = nh_.resolveName("out_image");
  image_pub_ = it_.advertise(image_topic, 1);

	sync_.registerCallback( boost::bind( &SilhouetteTracker::bothCB, this, _1, _2) );
	cv::namedWindow(WINDOW);
	cv::namedWindow(WINDOW_EDGES);
	ROS_INFO("Silhouette tracker constructor finished");
}


void SilhouetteTracker::bothCB(const sensor_msgs::ImageConstPtr& image_msg, 
                               const PointCloudXYZRGB::ConstPtr& cloud_msg)
{
	ROS_WARN("Silhouette tracker got data, image format %s", image_msg->encoding.c_str());

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	return;
	}

	//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	//	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

	double minval, maxval;
	cv::Point minloc, maxloc;

	// Convert the depth image into 8-bit for edge detection, scaling by 255/MAX_DIST
	cv::Mat img_8bit;
	cv_ptr->image.convertTo(img_8bit, CV_8U, 255/((double)MAX_DIST), 0);

	cv::Mat src_blurred, detected_edges;

	//cv::medianBlur( img_8bit, detected_edges, 3 );

	int lowThreshold;
	int ratio;

	nh_.param<int>("canny/low_threshold", lowThreshold, 50);
	nh_.param<int>("canny/ratio"        , ratio       , 3 );
	nh_.setParam("canny/ratio", 2);
	ROS_INFO("Low threshold=%d Ratio=%d",lowThreshold, ratio);

	cv::blur(img_8bit, detected_edges, cv::Size(3,3));
	int kernel_size = 5;
	cv::Canny(detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size);

	// Copy the detected edges to a new image, dst
	cv::Mat dst;
	dst.create(img_8bit.size(), img_8bit.type());
	dst = cv::Scalar::all(0);
	img_8bit.copyTo(dst, detected_edges);

	//cv::minMaxLoc(cv_ptr->image, &minval, &maxval, &minloc, &maxloc);

	//ROS_INFO("Max element = %f, min = %f", maxval, minval);

	// Convert to a fixed point image

	cv::imshow(WINDOW, img_8bit);
	cv::imshow(WINDOW_EDGES, dst);
	cv::waitKey(3);
  image_pub_.publish(image_msg);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "silhouette_tracker");
	SilhouetteTracker st(3);
	ROS_INFO("Initialization done.");
  ros::spin();
	return(0);
}
