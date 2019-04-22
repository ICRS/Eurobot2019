#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

class puck_localisation
{
    ros::NodeHandle nh_;
    cv::Matx33f camera_intrinsic;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;

public:
    puck_localisation():it_(nh_){
        // Subscribe to input video and publish to output video
        sub_=it_.subscribe("/camera/image_raw",1,
                &puck_localisation::image_callback, this);
        pub_=it_.advertise("/puck_localisation/output_video",1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~puck_localisation(){
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

        // detect circles
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
            cv::Mat gray_im;
            cv::cvtColor(cv_ptr->image, gray_im, CV_BGR2GRAY);
            // gaussian blur to reduce noise
            cv::GaussianBlur(gray_im, gray_im, cv::Size(9,9),2,2);
            vector<Vec3f> circles;
            // hough transform to find the circles
            cv::HoughCircles(gray_im, circles, CV_HOUGH_GRADIENT,1,30,200,100,0,0);
        for( size_t i = 0; i < circles.size(); i++ ){   
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            cv::circle(cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
            // circle outline
            cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
        }
 }
//            cv::circle(cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0));

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
    ros::init(argc, argv, "puck_localisation");
    puck_localisation pl;
    ros::spin();
    return 0;
}
