/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include <cv.h>
#include "ellipse/EllipseDetectorYaed.h"
#include <thread>//多线程
#include <fstream>

using namespace cv;
using namespace std;

vector<loc_t> target_gps_position;//全局变量——圆心目标的坐标
vector<coordinate> ellipse_out1;
// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	parse_commandline(argc, argv, uart_name, baudrate);


	// --------------------------------------------------------------------------
	//   PORT and THREAD STARTUP
	// --------------------------------------------------------------------------

	/*
	 * Instantiate a serial port object
	 *
	 * This object handles the opening and closing of the offboard computer's
	 * serial port over which it will communicate to an autopilot.  It has
	 * methods to read and write a mavlink_message_t object.  To help with read
	 * and write in the context of pthreading, it gaurds port operations with a
	 * pthread mutex lock.
	 *
	 */
	Serial_Port serial_port(uart_name, baudrate);


	/*
	 * Instantiate an autopilot interface object
	 *
	 * This starts two threads for read and write over MAVlink. The read thread
	 * listens for any MAVlink message and pushes it to the current_messages
	 * attribute.  The write thread at the moment only streams a position target
	 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
	 * is changed by using the method update_setpoint().  Sending these messages
	 * are only half the requirement to get response from the autopilot, a signal
	 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
	 * method.  Signal the exit of this mode with disable_offboard_control().  It's
	 * important that one way or another this program signals offboard mode exit,
	 * otherwise the vehicle will go into failsafe.
	 *
	 */
	Autopilot_Interface autopilot_interface(&serial_port);

	/*
	 * Setup interrupt signal handler
	 *
	 * Responds to early exits signaled with Ctrl-C.  The handler will command
	 * to exit offboard mode if required, and close threads and the port.
	 * The handler in this example needs references to the above objects.
	 *
	 */
	serial_port_quit         = &serial_port;
	autopilot_interface_quit = &autopilot_interface;
	signal(SIGINT,quit_handler);

	/*
	 * Start the port and autopilot_interface
	 * This is where the port is opened, and read and write threads are started.
	 */

//视觉定位线程
//    thread t1(videothread);//ref可以使autopilot_interface引用被正确传递给videothread.
	serial_port.start();
	autopilot_interface.start();

	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */

   commands(autopilot_interface);

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------
	ofstream outf;
	outf.open("location.txt");
	static uint32_t lasttime;
	while (1) {

		/*判断是否读入GPS信号*/
		if(autopilot_interface.current_messages.global_position_int.time_boot_ms == lasttime){
			continue;
//			break;
		}
		else {
			lasttime = autopilot_interface.current_messages.global_position_int.time_boot_ms;
			for (auto &p:ellipse_out1) {

				//在相机坐标系下椭圆圆心的坐标（相机坐标系正东为x，正北为y）
				float x = (p.x - cx) / fx * autopilot_interface.current_messages.global_position_int.relative_alt / 1000;//单位为：m
				float y = -(p.y - cy) / fy * autopilot_interface.current_messages.global_position_int.relative_alt / 1000;
				//将相机坐标系坐标转换为以摄像头所在中心的导航坐标系下坐标（正东为x,正北为y）
				float x_r = y * cos(autopilot_interface.current_messages.global_position_int.hdg * 3.1415926 / 180 / 100) - x * sin(autopilot_interface.current_messages.global_position_int.hdg * 3.1415926 / 180 / 100);//单位是:m
				float y_r = x * cos(autopilot_interface.current_messages.global_position_int.hdg * 3.1415926 / 180 / 100) + y * sin(autopilot_interface.current_messages.global_position_int.hdg * 3.1415926 / 180 / 100);
				cout << "x:" << x << endl
					 << "y:" << y << endl
					 << "order:" << p.order << endl;
				cout << "times" << autopilot_interface.current_messages.time_stamps.global_position_int << endl
					 << "lat:" << autopilot_interface.current_messages.global_position_int.lat << endl
					 << "lon:" << autopilot_interface.current_messages.global_position_int.lon << endl
					 << "hight" << autopilot_interface.current_messages.global_position_int.relative_alt << endl
					 << "yaw" << autopilot_interface.current_messages.global_position_int.hdg << endl;
				outf << "x:" << x << endl
					 << "y:" << y << endl
					 << "order:" << p.order << endl;
				outf << "times" << autopilot_interface.current_messages.time_stamps.global_position_int << endl
					 << "lat:" << autopilot_interface.current_messages.global_position_int.lat << endl
					 << "lon:" << autopilot_interface.current_messages.global_position_int.lon << endl
					 << "hight" << autopilot_interface.current_messages.global_position_int.relative_alt << endl
					 << "yaw" << autopilot_interface.current_messages.global_position_int.hdg << endl;
				cout << "result_x:" << x_r << endl
					 << "result_y:" << y_r << endl
					 << "order:" << p.order << endl;
				outf << "result_x:" << x_r << endl
					 << "result_y:" << y_r << endl
					 << "order:" << p.order << endl;
				outf << "local_x:" << autopilot_interface.current_messages.local_position_ned.x << endl
					 << "local_y:" << autopilot_interface.current_messages.local_position_ned.y << endl
					 << "local_z:" << autopilot_interface.current_messages.local_position_ned.z << endl;
			}
			ellipse_out1.clear();
		}
	};
	/*
	 * Now that we are done we can stop the threads and close the port
	 */
	autopilot_interface.stop();
	serial_port.stop();
//	t1.join();

	// --------------------------------------------------------------------------
	//   DONE
	// --------------------------------------------------------------------------

	// woot!
	return 0;

}


// ------------------------------------------------------------------------------
//   COMMANDS
// ------------------------------------------------------------------------------

void
commands(Autopilot_Interface &api)
{

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    //      设置guided（offboard）模式
    // --------------------------------------------------------------------------

    api.enable_offboard_control();
    usleep(100); // give some time to let it sink in

    // now the autopilot is accepting setpoint commands
    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    printf("SEND OFFBOARD COMMANDS\n");

    // initialize command data strtuctures
//////////////////////guided模式
	mavlink_set_mode_t com5 = { 0 };
	com5.base_mode = 1;
	com5.target_system = 01;
	com5.custom_mode = 04;
	// Encode
	mavlink_message_t message5;
	mavlink_msg_set_mode_encode(255, 190, &message5, &com5);
	// Send the message
	int len5 = api.write_message(message5);
	// Done!

 	// --------------------------------------------------------------------------
    // 给定局部坐标(local_ned)位置，并执行
    // --------------------------------------------------------------------------
        mavlink_set_position_target_local_ned_t sp;
        mavlink_local_position_ned_t locsp = api.local_position;
        sp.coordinate_frame = MAV_FRAME_BODY_NED;
     //mavlink_set_position_target_local_ned_t ip = api.initial_position;
     // -------------------------------------------------------------------------
     // 设置位置，速度，加速度三选一
     // -------------------------------------------------------------------------

        // Example 1 - Set Velocity
//	    set_velocity(  -1.0       , // [m/s]
//				       -1.0       , // [m/s]
//				        0.0       , // [m/s]
//				      sp        );
        // Example 2 - Set Position
        set_position(  - 5.0, // [m]
                       - 5.0, // [m]
                       - 10.0, // [m]
                     sp);

        // Example 1.2 - Append Yaw Command
//	    set_yaw( locsp.yaw , // [rad]
//			     sp     );
//
        // SEND THE COMMAND
        api.update_local_setpoint(sp);

     //Wait for 8 seconds, check position
        for (int i=0; i < 8; i++)
        {
        	mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
        	printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i, pos.x, pos.y, pos.z);
        	sleep(1);
        }

        printf("\n");

    // -------------------------------------------------------------------------
    // --------------- 全局坐标系下设置目标位置坐标 -------------------------------
    // -------------------------------------------------------------------------
	mavlink_set_position_target_global_int_t gsp;
	mavlink_global_position_int_t ggsp = api.global_position;
	mavlink_set_position_target_global_int_t global_int_pos = api.initial_global_position;
	global_int_pos.lat_int = ggsp.lat;
	global_int_pos.lon_int = ggsp.lon;
	global_int_pos.alt = ggsp.alt;
	global_int_pos.vx = ggsp.vx;
	global_int_pos.vy = ggsp.vy;
	global_int_pos.vz = ggsp.vz;
	global_int_pos.yaw = ggsp.hdg;
	gsp.time_boot_ms = (uint32_t) (get_time_usec() / 1000);
	gsp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
	float High = 15;
	//set global_point 经度，纬度，相对home高度
	set_global_position(global_int_pos.lat_int,
						global_int_pos.lon_int,
						High,
						gsp);
//		set_global_velocity( global_int_pos.vx = -1.0       , // [m/s]
//					         global_int_pos.vy = -1.0       , // [m/s]
//							 global_int_pos.vz = 0.0       , // [m/s]
//				              gsp );
//		set_global_yaw( global_int_pos.yaw , // [rad]
//			            gsp );

	api.update_global_setpoint(gsp);
	mavlink_message_t Gmessage;
	mavlink_msg_set_position_target_global_int_encode(255, 190, &Gmessage, &gsp);
	// 写入数据
	int lenG = api.write_message(Gmessage);
	if (lenG <= 0) {
		printf("global position write wrong!");
		printf("[%f,%f,%f]", gsp.lon_int, gsp.lat_int, gsp.alt);
	} else {
		printf("成功写入全局坐标点\n");
		// printf("[%f,%f,%f]\n", api.global_position.lat, gsp.lon_int, gsp.alt);
		printf("[%f,%f,%f]\n", (float)gsp.lon_int,  (float)gsp.lat_int, gsp.alt);
	}
/*
	// -------------------------------------------------------------------------
    // --------------------本体坐标系写入目标点
    // -------------------------------------------------------------------------
    mavlink_mission_item_t commission;
    commission.target_system= 01;
    commission.target_component = 01;
    commission.command = 16;
    commission.frame = MAV_FRAME_BODY_FLU;
    commission.autocontinue = 1;
    commission.current = 1;
    commission.seq = 1;
    commission.param1 = 1;
    commission.x = 5;
    commission.y = 5;
    commission.z = 10;
    mavlink_message_t Bmessage;
    mavlink_msg_mission_item_encode(255, 190, &Bmessage, &commission);
    int lenB = api.write_message(Bmessage);
    printf("成功写入本体坐标系坐标点");
*/
    // --------------------------------------------------------------------------
    // RETURN Home 以及
    //
    // STOP OFFBOARD MODE
    // --------------------------------------------------------------------------

    //返航
    mavlink_command_long_t com3 = { 0 };
    com3.target_system= 01;
    com3.target_component = 01;
    com3.command = 20;

    mavlink_message_t message3;
    mavlink_msg_command_long_encode(255, 190, &message3, &com3);
    int len3 = api.write_message(message3);

    //停止offboard
    //api.disable_offboard_control();

    // now pixhawk isn't listening to setpoint commands


    // --------------------------------------------------------------------------
    //   GET A MESSAGE
    // --------------------------------------------------------------------------
    printf("READ SOME MESSAGES \n");

    // copy current messages
    Mavlink_Messages messages = api.current_messages;

    // local position in ned frame
    mavlink_local_position_ned_t pos = messages.local_position_ned;
    printf("Got message LOCAL_POSITION_NED (spec: https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
    printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z );

    // hires imu
//	mavlink_highres_imu_t imu = messages.highres_imu;
//	printf("Got message HIGHRES_IMU (spec: https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
//	printf("    ap time:     %llu \n", imu.time_usec);
//	printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc , imu.yacc , imu.zacc );
//	printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro, imu.zgyro);
//	printf("    mag  (NED):  % f % f % f (Ga)\n"   , imu.xmag , imu.ymag , imu.zmag );
//	printf("    baro:        %f (mBar) \n"  , imu.abs_pressure);
//	printf("    altitude:    %f (m) \n"     , imu.pressure_alt);
//	printf("    temperature: %f C \n"       , imu.temperature );
//
//	printf("\n");


    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------

    return;

}


// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_quit->handle_quit(sig);
	}
	catch (int error){}

	// serial port
	try {
		serial_port_quit->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}

///////////////视觉定位线程
void videothread(){

    float areanum = 0.215;
	VideoCapture cap(0);
	if(!cap.isOpened()) return;

	int width = 640;
	int height = 480;

//	 Parameters Settings (Sect. 4.2)
	int		iThLength = 16;
	float	fThObb = 3.0f;
	float	fThPos = 1.0f;
	float	fTaoCenters = 0.05f;
	int 	iNs = 16;
	float	fMaxCenterDistance = sqrt(float(width*width + height*height)) * fTaoCenters;

	float	fThScoreScore = 0.4f;

	// Other constant parameters settings.

	// Gaussian filter parameters, in pre-processing
	Size	szPreProcessingGaussKernelSize = Size(5, 5);
	double	dPreProcessingGaussSigma = 1.0;

	float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
	float	fMinReliability = 0.4f;	// Const parameters to discard bad ellipses


	// Initialize Detector with selected parameters
    CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
	yaed->SetParameters(szPreProcessingGaussKernelSize,
						dPreProcessingGaussSigma,
						fThPos,
						fMaxCenterDistance,
						iThLength,
						fThObb,
						fDistanceToEllipseContour,
						fThScoreScore,
						fMinReliability,
						iNs
	);

	Mat1b gray;
	while(true)
	{

		Mat3b image;
		cap >> image;
		cvtColor(image, gray, COLOR_RGB2GRAY);

		vector<Ellipse> ellsYaed;
		Mat1b gray2 = gray.clone();
		yaed->Detect(gray, ellsYaed);
		Mat3b resultImage = image.clone();
		vector<coordinate> ellipse_out;
		yaed->DrawDetectedEllipses(resultImage, ellipse_out, ellsYaed);
        ellipse_out1 = ellipse_out;
		int num_ellipses = ellsYaed.size();
//		计算中心圆的半径
		float ellipses_a = 0;
		vector<int> a;
		Mat gauss, thresh, canny;
		vector< vector<Point> > contours;
		vector< vector<Point> > rects;

		if(ellsYaed.size() != 0) {
			if(ellsYaed.size() == 1){
				ellipses_a = ellsYaed[0]._a;
			}else
				ellipses_a =
						ellsYaed[num_ellipses - 1]._a < ellsYaed[num_ellipses - 2]._a ? ellsYaed[num_ellipses - 1]._a
																					  : ellsYaed[num_ellipses - 2]._a;
			a.push_back(ellipses_a);
		}else{
			/*
            int j = 0, r = 0;
			for(auto i = a.cend() - 5; i < a.cend(); i++){
				if( (*i) == 0 )
					continue;
				else {
					j = j + 1;
					r = r + (*i);
				}
			}
			ellipses_a = r/j;
			 */
			ellipses_a = 176;
		}
//		将中心圆的半径与全局的rows进行比例换算，若超过1/6，则进行方框判断。
		if (ellipses_a - 175 > 0){
			threshold(gray, thresh, 120, 255, CV_THRESH_BINARY);
//            imshow("threshold", thresh);
//            morphologyEx(gauss, gauss, MORPH_CLOSE, (5, 5) );
//
//			Canny(thresh, canny, 50, 150, 3);
			findContours(thresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
			for (int i = 0; i < contours.size(); i++) {
//				拟合出轮廓外侧最小的矩形
				RotatedRect rotate_rect = minAreaRect(contours[i]);
				Point2f *vertices = new Point2f[4];
				rotate_rect.points(vertices);
				if(rotate_rect.size.height < 10 || rotate_rect.size.height > 500 || gray.at<uchar>(rotate_rect.center.y, rotate_rect.center.x) > 20)
					continue;

				float x12 = (vertices[1].x + vertices[2].x)/2;
				float y12 = (vertices[1].y + vertices[2].y)/2;
				float xt12 = areanum * (rotate_rect.center.x - x12) + x12;
				float yt12 = y12 - areanum * (y12 - rotate_rect.center.y);

				float x30 = (vertices[3].x + vertices[0].x)/2;
				float y30 = (vertices[3].y + vertices[0].y)/2;
				float yt30 = areanum * (rotate_rect.center.y - y30) + y30;
				float xt30 = x30 - areanum * (x30 - rotate_rect.center.x);

				float x23 = (vertices[2].x + vertices[3].x)/2;
				float y23 = (vertices[2].y + vertices[3].y)/2;
				float xt23 = areanum * (rotate_rect.center.x - x23) + x23;
				float yt23 = y23 - areanum * (y23 - rotate_rect.center.y);

				float x01 = (vertices[1].x + vertices[0].x)/2;
				float y01 = (vertices[1].y + vertices[0].y)/2;
				float yt01 = areanum * (rotate_rect.center.y - y01) + y01;
				float xt01 = x01 - areanum * (x01 - rotate_rect.center.x);

				int flag = ((abs(x12 - x30) + abs(y12 - y30)) > (abs(x23 - x01) + abs(y23 - y01)))?12:23;

				float upx, upy, downx, downy;
				if(flag == 12){
					upx = xt12, upy = yt12, downx = xt30, downy = yt30;
				} else if(flag == 23)
					upx = xt23, upy = yt23, downx = xt01, downy = yt01;

				if(abs((gray.at<uchar>(upy, upx) - gray.at<uchar>(downy, downx))) > 90 )
					cout<<"decide:"<<"F"<<endl;
				else
					cout<<"decide:"<<"T"<<endl;
				circle(resultImage, Point(rotate_rect.center.x,rotate_rect.center.y), 2,Scalar(255, 255, 0), 1);
				vector<Point> contour;
				for (int i = 0; i < 4; i++) {
					contour.push_back(vertices[i]);
				}

				vector< vector<Point> > contours;
				contours.push_back(contour);
				drawContours(resultImage, contours, 0, Scalar(255, 255, 0), 1);
			}
		}
		namedWindow("Yaed",1);
		imshow("Yaed", resultImage);
        ellipse_out.clear();
		waitKey(10);
	}
    };
// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"mavlink_control threw exception %i \n" , error);
		return error;
	}

}


