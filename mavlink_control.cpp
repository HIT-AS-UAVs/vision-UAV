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
#include <cmath>

using namespace cv;
using namespace std;

vector<coordinate> ellipse_out1;
vector<target> target_ellipse_position;
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
    thread t1(videothread);//ref可以使autopilot_interface引用被正确传递给videothread.
//	serial_port.start();
//	autopilot_interface.start();

	// --------------------------------------------------------------------------
	//   RUN COMMANDS
	// --------------------------------------------------------------------------

	/*
	 * Now we can implement the algorithm we want on top of the autopilot interface
	 */

//   commands(autopilot_interface);

	// --------------------------------------------------------------------------
	//   THREAD and PORT SHUTDOWN
	// --------------------------------------------------------------------------
	ofstream outf;
	outf.open("target.txt");
	bool finish = false;
	while (!finish ) {
        cout<<"ellipse_out1.size = "<<ellipse_out1.size()<<endl;
//        for (int i = 0; i < ellipse_out1.size(); ++i) {
//            cout<<"i = "<<i<<endl
//				<<"x = "<<ellipse_out1[i].x<<endl
//                <<"y = "<<ellipse_out1[i].y<<endl
//                <<"a = "<<ellipse_out1[i].a<<endl
//                <<"flag = "<<ellipse_out1[i].flag<<endl;
//        }
	    possible_ellipse(autopilot_interface, ellipse_out1, target_ellipse_position);
		cout<<"target_ellipse.size = "<<target_ellipse_position.size()<<endl;
		for (int i = 0; i < target_ellipse_position.size(); ++i) {
			cout<<"x = "<<target_ellipse_position[i].x<<endl
				<<"y = "<<target_ellipse_position[i].y<<endl
				<<"T = "<<target_ellipse_position[i].T_N<<endl
				<<"F = "<<target_ellipse_position[i].F_N<<endl
				<<"flag = "<<target_ellipse_position[i].possbile<<endl;
		}
		ellipse_out1.clear();
	}
	//  Now that we are done we can stop the threads and close the port

	autopilot_interface.stop();
	serial_port.stop();
	t1.join();

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
    // 设置变量，用于返回相应切换点
    mavlink_global_position_int_t gp;
    mavlink_global_position_int_t flagPoint;
    flagPoint.lat = 411198976;
    flagPoint.lon = 1230987852;
    flagPoint.relative_alt = 20;
    bool flag = true;
    bool goback = true;

    // --------------------------------------------------------------------------
    //   START OFFBOARD MODE
    //   设置guided（offboard）模式/解锁、起飞
    // --------------------------------------------------------------------------

    api.enable_offboard_control();
    usleep(100); // give some time to let it sink in

    // --------------------------------------------------------------------------
    //   SEND OFFBOARD COMMANDS
    // --------------------------------------------------------------------------
    printf("Start Mission!\n");
/*
    while(flag)
    {
        gp = api.global_position;
//		if (flag == false)
//		{
//			break;
//		}
//设置触发节点
        if (Distance(gp.lat,gp.lon,gp.relative_alt,gp.lat,gp.lon,gp.relative_alt) <  2)
        {
            // --------------------------------------------------------------------------
            // 设置guided模式
            // --------------------------------------------------------------------------
            api.Set_Mode(05);
            usleep(100);
            api.Set_Mode(04);
            usleep(100);
            // now the autopilot is accepting setpoint commands
            // --------------------------------------------------------------------------
            // 给定局部坐标(local_ned)位置，并执行
            // --------------------------------------------------------------------------
            mavlink_set_position_target_local_ned_t sp;
            mavlink_local_position_ned_t locsp = api.local_position;
            sp.coordinate_frame = MAV_FRAME_LOCAL_NED;
            // -------------------------------------------------------------------------
            // 设置位置，速度，加速度三选一
            // -------------------------------------------------------------------------

//	    set_velocity(  -1.0       , // [m/s]
//				       -1.0       , // [m/s]
//				        0.0       , // [m/s]
//				      sp        );

            set_position(  locsp.x + 10, // [m]
                           locsp.y + 5, // [m]
                           locsp.z - 10, // [m]
                           sp);

            // Example 1.2 - Append Yaw Command
            float yaw = D2R(gp.hdg);
            set_yaw( yaw, // [rad]
                     sp     );

            // SEND THE COMMAND
            api.update_local_setpoint(sp);

            while(1)
            {
                mavlink_local_position_ned_t pos = api.current_messages.local_position_ned;
                float distance = Distance(pos.x,pos.y,pos.z,sp.x,sp.y,sp.z);
                if(distance < 2)
                {
                    goback = false;
                    usleep(200);
					// ------------------------------------------------------------------------------
					//	驱动舵机：<PWM_Value:1100-1900> 打开：1700、关闭：1250
					//	ServoId：AUX_OUT1-6 对应148-153
					// ------------------------------------------------------------------------------
                    api.Servo_Control(149,1250);
                    break;
                }
                else
                {
                    sleep(1);
//					goback = false;
//					break;
                }
            }
            printf("\n");

            if(goback == false)
            {
                // -------------------------------------------------------------------------
                // --------------- 全局坐标系下设置目标位置坐标 -------------------------------
                // -------------------------------------------------------------------------
                mavlink_set_position_target_global_int_t gsp;
                mavlink_global_position_int_t ggsp = gp; //api.global_position;
                mavlink_set_position_target_global_int_t global_int_pos = api.initial_global_position;
                global_int_pos.lat_int = ggsp.lat;
                global_int_pos.lon_int = ggsp.lon;
                global_int_pos.alt = ggsp.alt;
                global_int_pos.vx = ggsp.vx;
                global_int_pos.vy = ggsp.vy;
                global_int_pos.vz = ggsp.vz;
//				float gyaw = D2R(ggsp.hdg);
//				global_int_pos.yaw = gyaw;
                gsp.time_boot_ms = (uint32_t) (get_time_usec() / 1000);
                gsp.coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
                int32_t High = 20;
                //set global_point 经度，纬度，相对home高度
                set_global_position(global_int_pos.lat_int,
                                    global_int_pos.lon_int,
                                    High,
                                    gsp);
//				set_global_yaw(gyaw,
//						       gsp);

                api.update_global_setpoint(gsp);
                while(1)
                {
                    mavlink_global_position_int_t current_global = api.global_position;
                    float distan = Distance(current_global.lat,current_global.lon,current_global.relative_alt,gp.lat,gp.lon,gp.relative_alt);
                    if(distan < 2)
                    {
                        usleep(200);
                        api.Set_Mode(03);
                        break;
                    }
                    else
                    {
                        sleep(1);
                    }
                }

                break;
            }

        }
        else
        {
            usleep(10000);
        }

        printf("\n");

    }

    // --------------------------------------------------------------------------
    // RETURN Home 以及
    //
    // STOP OFFBOARD MODE
    // --------------------------------------------------------------------------
    sleep(5);
    mavlink_mission_clear_all_t comclearall;
    comclearall.target_system = 01;
    comclearall.target_component = 01;

    mavlink_message_t Clearcom;
    mavlink_msg_mission_clear_all_encode(255,190,&Clearcom,&comclearall);
    int Clearlen = api.write_message(Clearcom);
    sleep(1);

    //返航
    mavlink_command_long_t com3 = { 0 };
    com3.target_system= 01;
    com3.target_component = 01;
    com3.command = 20;

    mavlink_message_t message3;
    mavlink_msg_command_long_encode(255, 190, &message3, &com3);
    int len3 = api.write_message(message3);


    // now pixhawk isn't listening to setpoint commands

    // --------------------------------------------------------------------------
    //   END OF COMMANDS
    // --------------------------------------------------------------------------
*/
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
	cap.set(CV_CAP_PROP_FRAME_WIDTH, width);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, height);
	cap.set(CAP_PROP_AUTOFOCUS,0);


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

		vector<Ellipse> ellsYaed, ellipse_in;
		yaed->Detect(gray, ellsYaed);
		Mat3b resultImage = image.clone();
		vector<coordinate> ellipse_out, ellipse_TF;
		yaed->OptimizEllipse(ellipse_in, ellsYaed);//对椭圆检测部分得到的椭圆进行预处理，输出仅有大圆的vector
		yaed->DrawDetectedEllipses(resultImage, ellipse_out, ellipse_in);//绘制检测到的椭圆
		vector< vector<Point> > contours;
		if(ellipse_out.size() == 0){
            ellipse_out1 = ellipse_out;
		}
		else
			visual_rec(gray, ellipse_out, ellipse_TF, contours);//T和F的检测程序

//		for(auto &p:ellipse_TF){
//			cout<<"x:"<<p.x<<endl
//				<<"y"<<p.y<<endl
//				<<"flag"<<p.flag<<endl;
//		}
		for(auto &p:contours){
			vector< vector<Point> > contours1;
			contours1.push_back(p);
			drawContours(resultImage, contours1, 0, Scalar(255, 255, 0), 1);
		}
		ellipse_out1 = ellipse_TF;
		namedWindow("Yaed",1);
		imshow("Yaed", resultImage);
        ellipse_out.clear();
		waitKey(10);
	}
}
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


