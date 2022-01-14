// Nagatsuki-realsense-rgbd.cpp : Defines the entry point for the console application.
//

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <System.h>
#include <funcStream.h>
#include <Utils.hpp>

#include "StereoRectifier.h"

int main(int argc, char * argv[]) try
{
	ORB_SLAM2::stereo_rectifier rectifier{argv[2], true};

	// RealSense settings
	rs2::pipeline pipeline;
	rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8, 30);
    config.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8, 30);
	rs2::pipeline_profile cfg = pipeline.start(config);

	// ORB SLAM settings
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);
	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	cv::Mat frame1, frame2, input1, input2;
	bool slam = true;
	double tframe;

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point t2 = t1;
	std::chrono::steady_clock::time_point t3;
	std::chrono::steady_clock::time_point t4;

	typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

	for (;;)
	{
		rs2::frameset data = pipeline.wait_for_frames();

		t3 = std::chrono::steady_clock::now();
		frame1 = funcFormat::frame2Mat(data.get_fisheye_frame(1));
//		cv::resize(frame1, frame1, frame1.size()/2);
		rectifier.rectify(frame1, input1);
		t4 = std::chrono::steady_clock::now();

		std::cout << "Rec" << std::chrono::duration_cast<ms>(t4 - t3).count() << '\n';

		t1 = std::chrono::steady_clock::now();
		tframe = std::chrono::duration_cast<ms>(t1 - t2).count();
		if (slam)
		{
        	PUSH_RANGE("Track image", 4);
			SLAM.TrackMonocular(input1, tframe);
        	POP_RANGE;
		}
		t2 = t1;
		std::ostringstream strs;
		strs << tframe;
		std::string str = strs.str() + " ms";
		std::cout << str << '\n';

		cv::Size size = input1.size();

		char c = (char)cv::waitKey(10);
		if (c == 27 || c == 81 || c == 213)
			break;
		switch (c)
		{
		case 'a':
			slam = !slam;
			if (slam)
			{
				cout << endl << "-------" << endl;
				cout << "Start ORB-SLAM" << endl;
			}
			else
			{
				cout << endl << "-------" << endl;
				cout << "Stop ORB-SLAM" << endl;
			}
			break;
		default:
			;
		}
	}

	// Stop all threads
	SLAM.Shutdown();

	// Save camera trajectory
	SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
	SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

	pipeline.stop();
	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const cv::Exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

