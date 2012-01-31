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
#include <opencv2/ml/ml.hpp>

#include <fstream>
#include <stdlib.h>
#include <stdio.h>

#define MAX_DIST (7.0)

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
static const char WINDOW[] = "Image window";

class CSVSaver
{
	public:	
		CSVSaver(char* file_prefix);

		~CSVSaver()
		{
			cv::destroyWindow(WINDOW);
		}

	private:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

		image_transport::SubscriberFilter image_sub_;
		message_filters::Subscriber<PointCloudXYZRGB> cloud_sub_;

		bool first_run_;
		std::string file_prefix_;
		int filenum_;

		void bothCB(const sensor_msgs::ImageConstPtr& image_msg,
                const PointCloudXYZRGB::ConstPtr& cloud_msg);

		typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, PointCloudXYZRGB> MySyncPolicy;
		message_filters::Synchronizer< MySyncPolicy > sync_;
};

CSVSaver::CSVSaver(char* file_prefix) : 
	nh_("csv_saver"),	
	it_(nh_),
//	image_sub_( it_, "in_image", 1 ),
//	cloud_sub_( nh_, "in_cloud", 1 ),
	image_sub_( it_, "/camera/depth_registered/image_rect", 1),
	cloud_sub_( nh_, "/camera/depth_registered/points", 1 ),
	sync_( MySyncPolicy(2), image_sub_, cloud_sub_ )
{
	this->first_run_   = true;
	this->file_prefix_ = std::string(file_prefix);
	this->filenum_     = 0;

	sync_.registerCallback( boost::bind( &CSVSaver::bothCB, this, _1, _2) );
	cv::namedWindow(WINDOW);
	ROS_INFO("CSV Saver constructor finished");
}

void CSVSaver::bothCB(const sensor_msgs::ImageConstPtr& image_msg, 
                      const PointCloudXYZRGB::ConstPtr& cloud_msg)
{

	if(!this->first_run_)
	{	
		ROS_WARN("CSV saver got data");

		// Convert the image message to OpenCV format
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

		// Synthesize the file name (prefix + number)
		this->filenum_++;
		std::stringstream number_str;
		number_str << this->filenum_;
		std::string filename = this->file_prefix_ + number_str.str() + ".csv";

		// Save the raw image data in CSV format
		std::ofstream myfile;
		myfile.open(filename.c_str());
		myfile << format(cv_ptr->image, "csv");
		myfile.close();

		cv::imshow(WINDOW, cv_ptr->image/MAX_DIST);
		ROS_INFO("Image data saved (%s)", filename.c_str());
	}

	this->first_run_ = false;
	ROS_INFO("Paused.  On key press, will record next image received.");
	cv::waitKey(0);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "csv_saver");

	// The commandline arg will be a file prefix
	if( argc <= 1 ){
		ROS_ERROR("CSV saver needs an argument for a file prefix (something like \"2012_01_31\"");
		return(1);
	}	
	ROS_INFO("CSV saver called with file prefix \"%s\"", argv[1]);

	CSVSaver csvsaver(argv[1]);
  ros::spin();
	return(0);
}
