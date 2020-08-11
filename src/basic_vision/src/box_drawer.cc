#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>


class PubSubNode
{
public:
  PubSubNode(){

    img_sub_ = nh_.subscribe("/camera/color/image_raw", 1000, &PubSubNode::box_drawer_callback, this);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/cameron/box_drawer",1000);
  }

  void box_drawer_callback(const sensor_msgs::Image::Ptr& img){
    cv_bridge::CvImagePtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2),  100, cv::Scalar(0, 0, 255), 10, 1);
    img_pub_.publish(cv_ptr->toImageMsg());

  }

private:
  ros::NodeHandle nh_;
  ros::Publisher img_pub_;
  ros::Subscriber img_sub_;

};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "box_drawer");

  PubSubNode PBN;

  ros::spin();

  return 0;
}