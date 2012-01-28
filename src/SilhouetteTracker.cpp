#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";
// /camera/depth_registered/points /camera/depth_registered/image_rect
class SilhouetteTracker
{
	public:
		ros::NodeHandle nh_;
		image_transport::ImageTransport it_;

  	image_transport::Subscriber image_sub_;
		ros::Subscriber cloud_sub_;
		ros::Publisher  cloud_pub_;

	public SilhouetteTracker() : it_(nh_)
	{

    image_sub_ = it_.subscribe("in_image", 1, &SilhouetteTracker::imageCb, this);


		// Create a ROS subscriber for the input point cloud
		cloud_sub_ = nh.subscribe<PointCloudXYZRGB> ("in_cloud", 1, cloud_cb);

		// Create a ROS publisher for the output point cloud
		std::string cloud_topic = nh.resolveName("out_cloud");
		cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2> (cloud_topic, 1);

    cv::namedWindow(WINDOW);
	}

  ~SilhouetteTracker()
  {
    cv::destroyWindow(WINDOW);
  }

	private:
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		void cloud_cb(const PointCloudXYZRGB::ConstPtr& cloud_msg);
};

void SilhouetteTracker::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

  cv::imshow(WINDOW, cv_ptr->image);
  cv::waitKey(3);
  
  image_pub_.publish(cv_ptr->toImageMsg());
}


void SilhouetteTracker::cloud_cb(const PointCloudXYZRGB::ConstPtr& cloud_msg);
{
	// ... do data processing

	sensor_msgs::PointCloud2 output;
	// Publish the data
	pub.publish (output);
}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");

	SilhouetteTracker st;
  ros::spin ();
	return(0);
}
