//
// Created by xiang on 11/29/17.
//

// 该文件将打开给定的视频文件，并将图像传递给ORB-SLAM2进行定位

// 需要opencv
#include <opencv2/opencv.hpp>

// ORB-SLAM的系统接口
#include "System.h"

#include <string>
#include <chrono>   // for time stamp
#include <iostream>

// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward{
//     backward::SignalHandling sh;
// }

using namespace std;

// 参数文件与字典文件
// 如果你系统上的路径不同，请修改它
string parameterFile = "/home/dbstg/slam/ORB_SLAM2/orb2_ws/src/ORB_SLAM2-master/Examples/Monocular/myvideo.yaml";
string vocFile = "/home/dbstg/slam/ORB_SLAM2/orb2_ws/src/ORB_SLAM2-master/Vocabulary/ORBvoc.txt";

// 视频文件
string videoFile = "/home/dbstg/slam/ORB_SLAM2/orb2_ws/src/ORB_SLAM2-master/Examples/Monocular/myvideo2.mp4";

int main(int argc, char **argv) {

    // 声明 ORB-SLAM2 系统
    ORB_SLAM2::System SLAM(vocFile, parameterFile, ORB_SLAM2::System::MONOCULAR, true);

    // 获取视频图像
    cv::VideoCapture cap(videoFile);    // change to 1 if you want to use USB camera.

    // 记录系统时间
    auto start = chrono::system_clock::now();

    while (1) {
        cv::Mat frame;
        cap >> frame;   // 读取相机数据
        if ( frame.data == nullptr )
            break;

        // rescale because image is too large
        cv::Mat frame_resized;
        cv::resize(frame, frame_resized, cv::Size(640,360));

        auto now = chrono::system_clock::now();
        auto timestamp = chrono::duration_cast<chrono::milliseconds>(now - start);
        SLAM.TrackMonocular(frame_resized, double(timestamp.count())/1000.0);
        cv::waitKey(30);
    }

    SLAM.Shutdown();
    return 0;
}
