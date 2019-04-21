#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

static const std::string OPENCV_WINDOW = "Image window";

class colour_detection
{
    ros::NodeHandle nh_;
    cv::Matx33f camera_intrinsic;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;

public:
    colour_detection():it_(nh_){
        // Subscribe to input video and publish to output video
        sub_=it_.subscribe("/camera/image_raw",1,
                &colour_detection::image_callback, this);
        pub_=it_.advertise("/colour_detection/output_video",1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~colour_detection(){
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, 
                    sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Draw something on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        //Output modified video stream
        pub_.publish(cv_ptr->toImageMsg());
    }

    bool camera_initialisation(){
        camera_intrinsic(0,1) = 0;
        camera_intrinsic(1,0) = 0;
        camera_intrinsic(2,0) = 0;
        camera_intrinsic(2,1) = 0;
        camera_intrinsic(2,2) = 1;    
        return 1;
    }

    void camera_info_callback(const sensor_msgs::ImageConstPtr& img,
                              const sensor_msgs::CameraInfoConstPtr& msg) {
        camera_intrinsic(0,0) = msg->K[0];
        camera_intrinsic(0,2) = msg->K[2];
        camera_intrinsic(1,1) = msg->K[4];
        camera_intrinsic(1,2) = msg->K[5];
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "colour_detection");
    colour_detection cd;
    ros::spin();
    return 0;
}
