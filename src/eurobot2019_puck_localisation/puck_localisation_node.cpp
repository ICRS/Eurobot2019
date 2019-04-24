#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>

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
    int redParams[6];
    int blueParams[6];
    int greenParams[6];
    double thetaX, thetaZ;
    double camera_translation[3];
    Matx31f puck_w;
    Matx31f puck_c;
    Mat R;

public:
    puck_localisation():it_(nh_){
        // Subscribe to input video and publish to output video
        sub_=it_.subscribe("/camera/image_raw",1,
                &puck_localisation::image_callback, this);
        pub_=it_.advertise("/puck_localisation/output_video",1);

        nh_.getParam("puck_localisation/redLowH",redParams[0]);
        nh_.getParam("puck_localisation/redHighH",redParams[1]);
        nh_.getParam("puck_localisation/redLowS",redParams[2]);
        nh_.getParam("puck_localisation/redHighS",redParams[3]);
        nh_.getParam("puck_localisation/redLowV",redParams[4]);
        nh_.getParam("puck_localisation/redHighV",redParams[5]);
        nh_.getParam("puck_localisation/blueLowH",blueParams[0]);
        nh_.getParam("puck_localisation/blueHighH",blueParams[1]);
        nh_.getParam("puck_localisation/blueLowS",blueParams[2]);
        nh_.getParam("puck_localisation/blueHighS",blueParams[3]);
        nh_.getParam("puck_localisation/blueLowV",blueParams[4]);
        nh_.getParam("puck_localisation/blueHighV",blueParams[5]);
        nh_.getParam("puck_localisation/greenLowH",greenParams[0]);
        nh_.getParam("puck_localisation/greenHighH",greenParams[1]);
        nh_.getParam("puck_localisation/greenLowS",greenParams[2]);
        nh_.getParam("puck_localisation/greenHighS",greenParams[3]);
        nh_.getParam("puck_localisation/greenLowV",greenParams[4]);
        nh_.getParam("puck_localisation/greenHighV",greenParams[5]);
        nh_.getParam("puck_localisation/thetaX",thetaX);
        nh_.getParam("puck_localisation/thetaZ",thetaZ);
        nh_.getParam("puck_localisation/worldX_c",camera_translation[0]);
        nh_.getParam("puck_localisation/worldY_c",camera_translation[1]);
        nh_.getParam("puck_localisation/worldZ_c",camera_translation[2]);
        cv::namedWindow(OPENCV_WINDOW);
        camera_initialisation();
        // Rotation matrix
        Mat R_x = (Mat_<double>(3,3) <<
               1,       0,              0,
               0,       cos(thetaX),   -sin(thetaX),
               0,       sin(thetaX),   cos(thetaX));
        Mat R_z = (Mat_<double>(3,3) <<
               cos(thetaZ),    -sin(thetaZ),      0,
               sin(thetaZ),    cos(thetaZ),       0,
               0,               0,                1);
        R = R_z * R_x;
    }

    ~puck_localisation(){
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void image_callback(const sensor_msgs::ImageConstPtr& msg)
    {
        static auto last_time = ros::Time::now();
        auto time = ros::Time::now();
        if(time.toSec() - last_time.toSec() < 1)
            return;

        last_time = time;

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

        Mat imgHSV, redThreshold, blueThreshold, greenThreshold;
        // detect pucks
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
            /*
            cv::cvtColor(cv_ptr->image, imgHSV, CV_BGR2HSV);

            inRange(imgHSV, Scalar(redParams[0], redParams[2], redParams[4]), 
                    Scalar(redParams[1], redParams[3], redParams[5]),redThreshold);
            //morphological opening (removes small objects from the foreground)
            erode(redThreshold, redThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate(redThreshold, redThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            //morphological closing (removes small holes from the foreground)
            dilate(redThreshold, redThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(redThreshold, redThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            inRange(imgHSV, Scalar(blueParams[0], blueParams[2], blueParams[4]), 
                    Scalar(blueParams[1], blueParams[3], blueParams[5]),blueThreshold);
            //morphological opening (removes small objects from the foreground)
            erode(blueThreshold, blueThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate(blueThreshold, blueThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            //morphological closing (removes small holes from the foreground)
            dilate(blueThreshold, blueThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(blueThreshold, blueThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            inRange(imgHSV, Scalar(greenParams[0], greenParams[2], greenParams[4]), 
                    Scalar(greenParams[1], greenParams[3], greenParams[5]),greenThreshold);
            //morphological opening (removes small objects from the foreground)
            erode(greenThreshold, greenThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            dilate(greenThreshold, greenThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            //morphological closing (removes small holes from the foreground)
            dilate(greenThreshold, greenThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
            erode(greenThreshold, greenThreshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

            vector<Vec3f> redPucks;
            GaussianBlur(redThreshold,redThreshold,Size(9,9),2,2);
            // hough transform to find the circles
            cv::HoughCircles(redThreshold, redPucks, CV_HOUGH_GRADIENT,1,100,100,20,0,50);
            if(redPucks.size())
                ROS_INFO("%d red pucks found!",redPucks.size());
            for( size_t i = 0; i < redPucks.size(); i++ ){   
                Point center(cvRound(redPucks[i][0]), cvRound(redPucks[i][1]));
                int radius = cvRound(redPucks[i][2]);
                // circle center
                cv::circle(cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
            }

            vector<Vec3f> bluePucks;
            GaussianBlur(blueThreshold,blueThreshold,Size(9,9),2,2);
            // hough transform to find the circles
            cv::HoughCircles(blueThreshold, bluePucks, CV_HOUGH_GRADIENT,1,100,100,20,0,50);
            if(bluePucks.size())
                ROS_INFO("%d blue pucks found!",redPucks.size());
            for( size_t i = 0; i < bluePucks.size(); i++ ){   
                Point center(cvRound(bluePucks[i][0]), cvRound(bluePucks[i][1]));
                int radius = cvRound(bluePucks[i][2]);
                // circle center
                cv::circle(cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
                find_position(center, blueThreshold);
                ROS_INFO("blue puck %d position is:\tx: %f\ty: %f",i, x_w, y_w);
            }

            vector<Vec3f> greenPucks;
            GaussianBlur(greenThreshold,greenThreshold,Size(9,9),2,2);
            // hough transform to find the circles
            cv::HoughCircles(greenThreshold, greenPucks, CV_HOUGH_GRADIENT,1,100,100,20,0,50);
            if(greenPucks.size())
                ROS_INFO("%d green pucks found!",greenPucks.size());
            for( size_t i = 0; i < greenPucks.size(); i++ ){   
                Point center(cvRound(greenPucks[i][0]), cvRound(greenPucks[i][1]));
                int radius = cvRound(greenPucks[i][2]);
                // circle center
                cv::circle(cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                // circle outline
                cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
                cout<<"found " << greenPucks.size() <<" green puck!" << endl;
            }*/    
            Mat gray;
            cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);
            GaussianBlur(gray, gray, Size(9,9),2,2);
            vector<Vec3f> circles;
            cv::HoughCircles(gray, circles, CV_HOUGH_GRADIENT,1,100,100,20,0,50);
            for(size_t i = 0; i < circles.size(); i++){
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                cv::circle(cv_ptr->image, center, 3, Scalar(0,255,0), -1,8,0);
                cv::circle(cv_ptr->image,center, radius, Scalar(0,0,255), 3, 8, 0);
                find_position(center);
                ROS_INFO("find a circle at position: \tx: %f\ty: %f\tz: %f",puck_w(0), puck_w(1), puck_w(2));
            }
        }
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
        nh_.getParam("puck_localisation/fu", camera_intrinsic(0,0));
        nh_.getParam("puck_localisation/fv", camera_intrinsic(1,1));
        nh_.getParam("puck_localisation/u0", camera_intrinsic(0,2));
        nh_.getParam("puck_localisation/v0", camera_intrinsic(1,2));

        return 1;
    }

    void camera_info_callback(const sensor_msgs::ImageConstPtr& img,
                              const sensor_msgs::CameraInfoConstPtr& msg) {
        camera_intrinsic(0,0) = msg->K[0];
        camera_intrinsic(0,2) = msg->K[2];
        camera_intrinsic(1,1) = msg->K[4];
        camera_intrinsic(1,2) = msg->K[5];

        ROS_INFO("CALLBACK");

        image_callback(img);
    }

    void find_position(Point center) {
        cv::Matx31f hom(center.x, center.y, 1);
        
        hom = camera_intrinsic.inv() * hom;
        float lambd = sqrt(2)/(hom(0)-sqrt(2)*hom(1)+hom(2));
        // position in camera frame (meters)
        puck_c(0) = lambd*hom(0);
        puck_c(1) = lambd*hom(1);
        puck_c(2) = lambd*hom(2);
        // transform to world frame (meters)
        // P_w = R_transpose*P_c - C, C: translation between two frame
        Matx44d R_(
                R.at<double>(0,0),     R.at<double>(0,1),     R.at<double>(0,2),     camera_translation[0],
                R.at<double>(1,0),     R.at<double>(1,1),     R.at<double>(1,2),     camera_translation[1],
                R.at<double>(2,0),     R.at<double>(2,1),     R.at<double>(2,2),     camera_translation[2],
                0,          0,          0,          1);
        Matx44d R_inv = R_.inv();
        Matx41d p_c(puck_c(0), puck_c(1), puck_c(2),1.0);
        Matx41d p_w = R_inv * p_c;
        puck_w(0) = p_w(0);
        puck_w(1) = p_w(1);
        puck_w(2) = p_w(2);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "puck_localisation");
    puck_localisation pl;
    ros::spin();
    return 0;
}
