#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {
    String rtsp_addr = "rtsp://admin:ASHIT123456@192.168.1.64:554/h264/ch1/main/av_stream";

    VideoCapture cap(rtsp_addr);
//    cap.open(rtsp_addr);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cout<<"get:"<<cap.get(CV_CAP_PROP_FRAME_WIDTH)<<endl;
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    Mat frame;

    for (;;) {
        cap >> frame;
        Size size(640, 480);
        resize(frame,frame,size);
        if (frame.empty())
            break;
        namedWindow("Video Stream", 1);
        imshow("Video Stream", frame);
        cout<<"width:"<<frame.rows<<endl<<"longth:"<<frame.cols<<endl;

        if (waitKey(10) == 'q')
            break;
    }
}
