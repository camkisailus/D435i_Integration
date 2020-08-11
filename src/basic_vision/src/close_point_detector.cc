#include "ros/ros.h" 

#include "sensor_msgs/image_encodings.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>


using namespace sensor_msgs;
using namespace message_filters;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
typedef Synchronizer<MySyncPolicy> Sync;

class Close_Point_Detector
{
public:
	Close_Point_Detector(){
		image_pub_ = nh_.advertise<Image>("/cameron/close_points", 10);
		color_image_sub.subscribe(nh_, "/camera/color/image_raw", 10);
		depth_image_sub.subscribe(nh_, "/camera/aligned_depth_to_color/image_raw", 10);
		sync.reset(new Sync(MySyncPolicy(10), color_image_sub, depth_image_sub));
		sync->registerCallback(boost::bind(&Close_Point_Detector::callback, this, _1, _2));

	}

	void callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image)
	{
		cv_bridge::CvImagePtr depth_ptr;
		cv_bridge::CvImagePtr color_ptr;
		try{
	      depth_ptr = cv_bridge::toCvCopy(depth_image, image_encodings::TYPE_16UC1);
	      color_ptr = cv_bridge::toCvCopy(color_image, image_encodings::BGR8);
	    }catch (cv_bridge::Exception& e){
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	    }
	    for(int x = 0; x < depth_image->height; ++x){
	    	for(int y = 0; y < depth_image->width; ++y){
	    		float dist = 0.001*depth_ptr->image.at<u_int16_t>(x, y);
	    		if (dist > 0.05 && dist < 1.00)
	    		{
	    			cv::circle(color_ptr->image, cv::Point(y, x),  2, cv::Scalar(0, 0, 255), 1, 1);
	    		}
	    	}
	    }
	    image_pub_.publish(color_ptr->toImageMsg());
	}


private:
	ros::NodeHandle nh_;
	ros::Publisher image_pub_;
	message_filters::Subscriber<Image> color_image_sub;
	message_filters::Subscriber<Image> depth_image_sub;
	boost::shared_ptr<Sync> sync;

};

int main(int argc, char **argv){
	ros::init(argc, argv, "close_points");

	Close_Point_Detector cbn;

	ros::spin();

	return 0;
}