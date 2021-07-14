#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/UInt8MultiArray.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "opencv_pub");    // node 이름을 opencv_pub로 설정
    
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>("camera/image", 1);

    cv::VideoCapture cap(0);
    // std::cout << cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920) << std::endl;
    // std::cout << cap.get(CV_CAP_PROP_FRAME_HEIGHT) << std::endl;
    cv::Mat frame;
        
    while(nh.ok())
    {
        cap >> frame;
        // frame = 255 - frame;
        // std::cout << frame.channels() << std::endl;
        if(!frame.empty())
        {

            // cv::imshow("frame", frame);

            // Encode, Decode image example            
            std::vector<uchar> encode;
            std::vector<int> encode_param;
            
            encode_param.push_back(CV_IMWRITE_JPEG_QUALITY);
            encode_param.push_back(20);
            
            cv::imencode(".jpg", frame, encode, encode_param);
            std::cout << frame.size() << std::endl;
            cv::Mat decode = cv::imdecode(encode, 1);  // 1에서 바꾼것
            cv::imshow("decode", decode);

            // Convert encoded image to ROS std_msgs format
            std_msgs::UInt8MultiArray msgArray;
            msgArray.data.clear();
            msgArray.data.resize(encode.size());
            std::copy(encode.begin(), encode.end(), msgArray.data.begin()); 
            // input first, input last, output first 순서!

            // Publish msg
            pub.publish(msgArray);
            cv::waitKey(1);
            // cv::waitKey(100);
          
        }

        ros::spinOnce();
    }

    return 0;
    
}