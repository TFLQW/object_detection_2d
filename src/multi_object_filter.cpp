/* Copyright by Huong Do Van - 10/11/2018
Any suggestion or advice, pls send via email: vanhuong.robotics@gmail.com
*/
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv_object_tracking/position_publish.h"
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include "std_msgs/String.h"
#include<sstream>
// Add new topic
#include "geometry_msgs/Point.h"
#include <iostream>
#include <vector>
// Create new for circle drawing
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Add global variable for pixel.
bool flag;
int posX_1 = 0;
int posY_1 = 0;
float X_111_1 = 0.0;
float Y_111_1 = 0.0;
float Z_111_1 = 0.0;
float x_value_1, y_value_1, z_value_1;
int x_position_1, y_position_1, z_position_1;
int posX_2 = 0;
int posY_2 = 0;
float X_111_2 = 0.0;
float Y_111_2 = 0.0;
float Z_111_2 = 0.0;
float x_value_2, y_value_2, z_value_2;
int x_position_2, y_position_2, z_position_2;
//End of global variable.
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::PointCloud2 my_pcl;
using namespace std;
using namespace cv;
int H_MIN_Blue = 100;
int H_MAX_Blue = 124;
int S_MIN_Blue = 43;
int S_MAX_Blue = 255;
int V_MIN_Blue = 46;
int V_MAX_Blue = 255;

int H_MIN_Black = 0;
int H_MAX_Black = 180;
int S_MIN_Black = 0;
int S_MAX_Black = 255;
int V_MIN_Black = 0;
int V_MAX_Black = 46;
//Publishe new topic
ros::Publisher *pub;
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;
const int MAX_NUM_OBJECTS = 50;
const int MIN_OBJECT_AREA = 20 * 20;
const int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH / 1.5;

static const std::string OPENCV_WINDOW = "Image Window";
static const std::string windowName1 = "HSV image";
static const std::string windowName2 = "Thresholded Blue Image"
;
static const std::string windowName3 = "Thresholded Black Image";
//static const std::string windowName3 = "After Morphological Operations";
//static const std::string trackbarWindowName = "Track bars";

void on_trackbar(int, void*){}
string intToString(int number)
{
        std::stringstream ss;
        ss << number;
        return ss.str();
}
//void createTrackbars()
//{
//        //Create window for trackbars
//        namedWindow("Track_bars", 0);
//        char TrackbarName[50];
//        createTrackbar("H_MIN", "Track_bars", &H_MIN, H_MAX, on_trackbar);
//        createTrackbar("H_MAX", "Track_bars", &H_MAX, H_MAX, on_trackbar);
//        createTrackbar("S_MIN", "Track_bars", &S_MIN, S_MAX, on_trackbar);
//        createTrackbar("S_MAX", "Track_bars", &S_MAX, S_MAX, on_trackbar);
//        createTrackbar("V_MIN", "Track_bars", &V_MIN, V_MAX, on_trackbar);
//        createTrackbar("V_MAX", "Track_bars", &V_MAX, V_MAX, on_trackbar);
//}

void drawObject(int x, int y, Mat &frame, int id)
{
        circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2); //50
        if (y - 25 > 0)
                line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
        if (y + 25 < FRAME_HEIGHT)
                line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
        if (x - 25 > 0)
                line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
        if (x + 25 < FRAME_WIDTH)
                line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
        else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

        putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);
	if (id == 1)
	{
	    ::posX_1 = x;
            ::posY_1 = y;
            putText(frame, "Object-1", Point(20, 200), 2, 0.8, Scalar(0, 255, 0), 1);
            putText(frame, "X = " + intToString(x_position_1) + "(mm)" , Point(20, 220), 2, 0.6, Scalar(0, 255, 0), 1);
            putText(frame, "Y = " + intToString(y_position_1) + "(mm)" , Point(20, 240), 2, 0.6, Scalar(0, 255, 0), 1);
            putText(frame, "Z = " + intToString(z_position_1) + "(mm)" , Point(20, 260), 2, 0.6, Scalar(0, 255, 0), 1);
	} 
	else if (id == 2)
	{
	    ::posX_2 = x;
            ::posY_2 = y;	
            putText(frame, "Object-2", Point(20, 300), 2, 0.8, Scalar(255, 255, 0), 1);
            putText(frame, "X = " + intToString(x_position_2) + "(mm)" , Point(20, 320), 2, 0.6, Scalar(255, 255, 0), 1);
            putText(frame, "Y = " + intToString(y_position_2) + "(mm)" , Point(20, 340), 2, 0.6, Scalar(255, 255, 0), 1);
            putText(frame, "Z = " + intToString(z_position_2) + "(mm)" , Point(20, 360), 2, 0.6, Scalar(255, 255, 0), 1);
	}       



}
void morphOps(Mat &thresh)
{
        Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3, 3));
        Mat dilateElement = getStructuringElement(MORPH_RECT, Size(8, 8));

        erode(thresh, thresh, erodeElement);
        erode(thresh, thresh, erodeElement);

        dilate(thresh, thresh, dilateElement);
        dilate(thresh, thresh, dilateElement);
}
void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed, int id)
{
        Mat temp;
        threshold.copyTo(temp);
        //These two vectors needed for output of findContours
        vector< vector<Point> > contours;
        vector<Vec4i> hierarchy;
        //Find contours of filtered image using openCV findContours function
        findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
        //Use moments method to find our filtered object
        double refArea = 0;
        bool objectFound = false;
        if (hierarchy.size() > 0)
        {
                int numObjects = hierarchy.size();
                //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
                if (numObjects < MAX_NUM_OBJECTS)
                {
                        for (int index = 0; index >= 0; index = hierarchy[index][0])
                        {
                                Moments moment = moments((cv::Mat)contours[index]);
                                double area = moment.m00;
                                //if the area is less than 20 px by 20px then it is probably just noise
                                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                                //we only want the object with the largest area so we safe a reference area each
                                //iteration and compare it to the area in the next iteration.
                                if (area > MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
                                {
                                        x = moment.m10 / area;
                                        y = moment.m01 / area;
                                        objectFound = true;
                                        ::flag = true;
                                        refArea = area;
                                }
                                else
                                {
                                 objectFound = false;
                                 ::flag = false;
                                }
                        }
                        //let user know you found an object
                        if (objectFound == true)
                        {
                                //::flag = true;
				if (id == 1)
                                    putText(cameraFeed, "Position Object 1 tracking ", Point(0, 50), 2, 1, Scalar(0, 255, 0), 1.5);
				else if (id == 2)
				    putText(cameraFeed, "Position Object 2 tracking ", Point(0, 80), 2, 1, Scalar(0, 255, 0), 1.5);
                                //draw object location on screen x y is the positon in the image(pixel postion)
                                drawObject(x, y, cameraFeed, id);
                        }

                }
                else putText(cameraFeed, "TOO MUCH NOISE! ADJUST FILTER", Point(0, 50), 1, 2, Scalar(0, 0, 255), 2);
        }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  //the callback function of the color/image_raw
{
    cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy (msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    bool trackObjects = true; //false
    bool useMorphOps = true;  //false

    Mat HSV;
    Mat threshold_blue;
    Mat threshold_black;
    int x_blue = 0, y_blue = 0;
    int x_black = 0, y_black = 0;
    //createTrackbars();
    //std::cout << " The output of Object tracking by OpenCV!\n";


    cvtColor(cv_ptr->image, HSV, COLOR_BGR2HSV);

    inRange(HSV, Scalar(H_MIN_Blue, S_MIN_Blue, V_MIN_Blue), Scalar(H_MAX_Blue, S_MAX_Blue, V_MAX_Blue), threshold_blue);
    inRange(HSV, Scalar(H_MIN_Black, S_MIN_Black, V_MIN_Black), Scalar(H_MAX_Black, S_MAX_Black, V_MAX_Black), threshold_black);

    if (useMorphOps)
        morphOps(threshold_blue);
        morphOps(threshold_black);

    if (trackObjects)
        trackFilteredObject(x_blue, y_blue, threshold_blue, cv_ptr->image, 1);
        trackFilteredObject(x_black, y_black, threshold_black, cv_ptr->image, 2);
    //show frames
    imshow(windowName2, threshold_blue);
    imshow(windowName3, threshold_black);
    //imshow(OPENCV_WINDOW, cameraFeed);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //imshow(windowName1, HSV);
    cv::waitKey(3);

}

void getXYZ(int x, int y, int id)
{
    int arrayPosition = y*my_pcl.row_step + x*my_pcl.point_step;
    int arrayPosX = arrayPosition + my_pcl.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + my_pcl.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + my_pcl.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    geometry_msgs::Point p;

    memcpy(&X, &my_pcl.data[arrayPosX], sizeof(float));
    memcpy(&Y, &my_pcl.data[arrayPosY], sizeof(float));
    memcpy(&Z, &my_pcl.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;
    if (id == 1)
    {
        ::X_111_1 = X;
        ::Y_111_1 = Y;
        ::Z_111_1 = Z;
        ::x_position_1 = int(X*1000);
        ::y_position_1 = int(Y*1000);
        ::z_position_1 = int(Z*1000);
    }
    else if (id == 2)
    {
        ::X_111_2 = X;
        ::Y_111_2 = Y;
        ::Z_111_2 = Z;
        ::x_position_2 = int(X*1000);
        ::y_position_2 = int(Y*1000);
        ::z_position_2 = int(Z*1000);
    }

    //printf("Position in X coordinate X = %.4f\n", X);
    //printf("Position in Y coordinate Y = %.4f\n", Y);
    //printf("Position in Z coordinate Z = %.4f\n", Z);

    //return 0;
}

void depthcallback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)  //the callback function of the /depth_registered/points
{
    my_pcl = *cloud_msg;
    getXYZ(posX_1 , posY_1, 1);
    getXYZ(posX_2 , posY_2, 2);
}

int main(int argc, char** argv)
{


    ros::init (argc, argv, "image_converter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw",1, imageCallback);
    ros::Subscriber dep;
    dep = nh.subscribe ("/camera/depth_registered/points", 1, depthcallback);

    //Publish new topic.
    ros::Publisher pub = nh.advertise<opencv_object_tracking::position_publish>("position_object", 1);
    //Set the loop period with 0.1 second.
    ros::Rate loop_rate(10);

    opencv_object_tracking::position_publish msg1, msg2;
    msg1.counter = 0;
    msg2.counter = 0;

    //int count = 0;
    while ((ros::ok()))
      {

       //if ((posX != 0) && (posY != 0) && (flag == true))
       //int count = 0;
       if ((flag == true))
       {
       //int posX_1, posY_1;
       //printf("------------\n");
       //printf("Object: 1\n");
       //printf("Position in X coordinate X = %.4f\n", X_111_1);
       //printf("Position in Y coordinate Y = %.4f\n", Y_111_1);
       //printf("Position in Z coordinate Z = %.4f\n", Z_111_1);
       //printf("---***---\n");
       //printf("Object: 2\n");
       //printf("Position in X coordinate X = %.4f\n", X_111_2);
       //printf("Position in Y coordinate Y = %.4f\n", Y_111_2);
       //printf("Position in Z coordinate Z = %.4f\n", Z_111_2);
       //printf("------------\n");
       //int count = 0;
       //posX_1 = posX;
       //posY_1 = posY;
       msg1.Position_XYZ.clear();
       msg1.center_pixel_x = posX_1;
       msg1.center_pixel_y = posY_1;
       msg1.counter = 1;
       msg2.Position_XYZ.clear();
       msg2.center_pixel_x = posX_2;
       msg2.center_pixel_y = posY_2;
       msg2.counter = 2;

       geometry_msgs::Point Position_XYZ;
       Position_XYZ.x = X_111_1;
       Position_XYZ.y = Y_111_1;
       Position_XYZ.z = Z_111_1;
       msg1.Position_XYZ.push_back(Position_XYZ);
       Position_XYZ.x = X_111_2;
       Position_XYZ.y = Y_111_2;
       Position_XYZ.z = Z_111_2;
       msg2.Position_XYZ.push_back(Position_XYZ);

       pub.publish(msg1);
       pub.publish(msg2);

       loop_rate.sleep();
       //++count;
       }

       if (flag == false)
       {
        //count = 0;
        msg1.Position_XYZ.clear();
        msg1.center_pixel_x = 0;
        msg1.center_pixel_y = 0;
        msg1.counter = 0;
        msg2.Position_XYZ.clear();
        msg2.center_pixel_x = 0;
        msg2.center_pixel_y = 0;
        msg2.counter = 0;

        geometry_msgs::Point Position_XYZ;
        Position_XYZ.x = 0;
        Position_XYZ.y = 0;
        Position_XYZ.z = 0;
        msg1.Position_XYZ.push_back(Position_XYZ);
        msg2.Position_XYZ.push_back(Position_XYZ);

        pub.publish(msg1);
        pub.publish(msg2);
        loop_rate.sleep();
       }

      ros::spinOnce();
      }


    return 0;
}


