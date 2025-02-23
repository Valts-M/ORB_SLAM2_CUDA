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

int main(int argc, char * argv[]) try
{
	// RealSense settings
	rs2::pipeline pipeline;
	rs2::config config;
	config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::pipeline_profile cfg = pipeline.start(config);

	rs2::align alignTo(RS2_STREAM_COLOR);
	// rs2::decimation_filter filterDec;
	// rs2::spatial_filter filterSpat;
	// rs2::temporal_filter filterTemp;
	// filterSpat.set_option(RS2_OPTION_HOLES_FILL, 5);

	// ORB SLAM settings
	ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::RGBD, true);
	cout << endl << "-------" << endl;
	cout << "Start processing sequence ..." << endl;

	cv::Mat frame1, frame2, input1, input2;
	bool slam = true;
	double tframe;

	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point t2 = t1;
	typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;

	for (;;)
	{
		rs2::frameset data = pipeline.wait_for_frames();
		rs2::frameset alignedFrame = alignTo.process(data);
		rs2::depth_frame depth = alignedFrame.get_depth_frame();
		// depth = filterSpat.process(depth);
		// depth = filterTemp.process(depth);

		frame1 = funcFormat::frame2Mat(alignedFrame.get_color_frame());
		frame2 = funcFormat::frame2Mat(depth);
		input1 = frame1.clone();
		input2 = frame2.clone();

		t1 = std::chrono::steady_clock::now();
		tframe = std::chrono::duration_cast<ms>(t1 - t2).count();
		if (slam)
		{
        	PUSH_RANGE("Track image", 4);
			SLAM.TrackRGBD(input1, input2, tframe);
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

