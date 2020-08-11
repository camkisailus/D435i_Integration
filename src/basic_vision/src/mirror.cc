#include "ros/ros.h"
#include "sensor_msgs/Image.h"

class PubSubNode
{
public:
	PubSubNode(){

		img_sub_ = nh_.subscribe("/camera/color/image_raw", 1000, &PubSubNode::mirror_callback, this);
		img_pub_ = nh_.advertise<sensor_msgs::Image>("/cameron/mirror_image",1000);
	}

	void mirror_callback(const sensor_msgs::ImageConstPtr & img){

		img_pub_.publish(img);

	}

private:
	ros::NodeHandle nh_;
	ros::Publisher img_pub_;
	ros::Subscriber img_sub_;

};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mirror");

	PubSubNode PBN;

	ros::spin();

	return 0;
}