#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h" 
#include "geometry_msgs/Point.h"

#include "sensor_msgs/image_encodings.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

class PubSubNode
{
public:
	PubSubNode(){
		depth_pub_ = nh_.advertise<std_msgs::String>("/cameron/depth_at_center", 1);
		point_pub_ = nh_.advertise<geometry_msgs::Point>("/cameron/center_depth_point", 1);
		depth_img_sub_ = nh_.subscribe("/camera/depth/image_rect_raw", 1, &PubSubNode::get_depth_callback, this);

	}

	void get_depth_callback(const sensor_msgs::ImageConstPtr & img){
		cv_bridge::CvImagePtr cv_ptr;
		try{
	      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
	    }catch (cv_bridge::Exception& e){
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	    }
	    int center_x = img->width/2;
	    int center_y = img->height/2;
	    float dist = 0.001*cv_ptr->image.at<u_int16_t>(center_x, center_y);
	    geometry_msgs::Point pt;
	    pt.x = dist;
	    pt.y = 0;
	    pt.z = 0;
	    point_pub_.publish(pt);
	    
	    /*
		std_msgs::String msg;
		ss_ = ("The distance is: %f",dist);
		msg.data = ss_;
		depth_pub_.publish(msg);
		*/
	}

private:
	ros::Publisher depth_pub_;
	ros::Publisher point_pub_;
	ros::Subscriber depth_img_sub_;
	ros::NodeHandle nh_;
	std::string ss_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "center_depth");

    PubSubNode PBN;

    ros::spin();

	return 0;
}