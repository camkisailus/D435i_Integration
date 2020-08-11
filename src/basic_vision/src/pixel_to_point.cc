#include "ros/ros.h" 

#include "sensor_msgs/image_encodings.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
//#include "opencv2/core/core.hpp"
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>


using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
typedef Synchronizer<MySyncPolicy> Sync;


class Pixel_To_Point
{
public:
	Pixel_To_Point(){
		image_pub_ = nh_.advertise<Image>("/cameron/threshold", 10);
		color_image_sub_.subscribe(nh_, "/camera/color/image_raw", 10);
		depth_image_sub_.subscribe(nh_, "/camera/aligned_depth_to_color/image_raw", 10);
		camera_info_sub_.subscribe(nh_, "/camera/aligned_depth_to_color/camera_info", 10);
		sync.reset(new Sync(MySyncPolicy(10), color_image_sub_, depth_image_sub_, camera_info_sub_));
		sync->registerCallback(boost::bind(&Pixel_To_Point::callback, this, _1, _2, _3));
	}



	void callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const CameraInfoConstPtr& cam_info ){
		cv_bridge::CvImagePtr depth_ptr;
		cv_bridge::CvImagePtr color_ptr;
		try{
	      depth_ptr = cv_bridge::toCvCopy(depth_image, image_encodings::TYPE_16UC1);
	      color_ptr = cv_bridge::toCvCopy(color_image, image_encodings::BGR8);
	    }catch (cv_bridge::Exception& e){
	      ROS_ERROR("cv_bridge exception: %s", e.what());
	    }

	    cv_bridge::CvImage theshold = Pixel_To_Point::detect(color_ptr);
	    image_pub_.publish(theshold->toImageMsg());
	    

	    // Convert to points

	}

	cv_bridge::CvImage detect(const cv_bridge::CvImagePtr& cv_image){
		cv_bridge::CvImage gray_scale;
		cvtColor(cv_image->image, gray_scale, COLOR_BGR2GRAY);
		cv_bridge::CvImage blur;
		GaussianBlur(gray_scale, blur, Size(3,3), 0, 0);
		cv_bridge::CvImage thresh;
		threshold(blur, thresh, 200, 255, THRESH_TOZERO);
		return thresh;

	}
	/*
	void detect(const ImageConstPtr& cv_image){
		Mat gray_image;
		cvtColor(cv_image, gray_image, COLOR_BGR2GRAY);
        Mat blurred;
        GaussianBlur(gray, blurred, Size(3, 3), 0, 0);
        Mat thresh;
        threshold(blurred, thresh, 200, 255, cv2.THRESH_TOZERO);
        size = np.size(thresh4)
        skel = np.zeros(thresh4.shape, np.uint8)

        skeleton = self.skeletize(thresh4, size, skel)

        dst = cv2.Laplacian(skeleton, self._ddepth, ksize=self._KERNEL_SIZE)
        abs_dst = cv2.convertScaleAbs(dst)

        linesP = cv2.HoughLinesP(abs_dst, rho=self._RHO, theta=self._THETA, threshold=self._THRESHOLD,
                                 lines=np.array([]), minLineLength=self._MINLINELENGTH, maxLineGap=self._MAXLINEGAP)

        if linesP is not None:
            point_arr.points = []
            right_points = []
            left_points = []
            for line in linesP:
                for x1, y1, x2, y2 in line:
                    if(0 < x1 < 640 and 0 < x2 < 640 and 0 < y1 < 480 and 0 < y2 < 480):
                        slope = (y2 - y1) / (x2 - x1)
                        p1 = Point()
                        p2 = Point()
                        p1.x = x1
                        p1.y = y1
                        p1.z = 0
                        point_arr.points.append(p1)
                        p2.x = x2
                        p2.y = y2
                        p2.z = 0
                        point_arr.points.append(p2)
                        if math.fabs(slope) < .5:
                            continue
                        if x1 < cv_image.shape[1]/2 and x2 < cv_image.shape[1]/2:
                            left_points.append([x1, y1])
                            left_points.append([x2, y2])
                            cv2.circle(cv_image, (x1, y1), (5), (0, 0, 255), 3)
                            cv2.circle(cv_image, (x2, y2), (5), (0, 0, 255), 3)
                        else:
                            right_points.append([x1, y1])
                            right_points.append([x2, y2])
                            cv2.circle(cv_image, (x1, y1), (5), (0, 255, 0), 3)
                            cv2.circle(cv_image, (x2, y2), (5), (0, 255, 0), 3)

            self.interpolate_spline(right_points, cv_image)
            self.interpolate_spline(left_points, cv_image)
        return cv_image
	}*/


private:
	ros::NodeHandle nh_;
	ros::Publisher image_pub_;
	message_filters::Subscriber<Image> color_image_sub_;
	message_filters::Subscriber<Image> depth_image_sub_;
	message_filters::Subscriber<CameraInfo> camera_info_sub_;
	boost::shared_ptr<Sync> sync;
	
};