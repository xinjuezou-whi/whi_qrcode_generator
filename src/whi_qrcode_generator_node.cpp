/******************************************************************
node to generate QR code

Features:
- save generated QR code to image
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-03-07: Initial version
2022-xx-xx: xxx
******************************************************************/
#include <iostream>
#include <signal.h>
#include <functional>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI QR code generator VERSION 00.01" << std::endl;
	std::cout << "Copyright © 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    const std::string nodeName("whi_qrcode_generator"); 
	ros::init(argc, argv, nodeName);
	auto nodeHandle = std::make_shared<ros::NodeHandle>(nodeName);

	/// node logic
	// params
	int codeSize, imageSize;
	nodeHandle->param("code_size", codeSize, 200);
	nodeHandle->param("image_size", imageSize, 500);
	std::string correctionLevel, outputPath, contents;
	nodeHandle->param("correction_level", correctionLevel, std::string("low"));
	nodeHandle->param("output_path", outputPath, std::string("/home"));
	nodeHandle->param("contents", contents, std::string("hello world"));
	if (outputPath.size() > 1 && outputPath.back() == '/')
	{
		outputPath.pop_back();
	}
	bool showGenerated;
	nodeHandle->param("show_generated", showGenerated, true);

	cv::QRCodeEncoder::Params params;
	// parse correction level
	if (correctionLevel.find("middle") != std::string::npos)
	{
		params.correction_level = cv::QRCodeEncoder::CORRECT_LEVEL_M;
	}
	else if (correctionLevel.find("quality") != std::string::npos)
	{
		params.correction_level = cv::QRCodeEncoder::CORRECT_LEVEL_Q;
	}
	else if (correctionLevel.find("high") != std::string::npos)
	{
		params.correction_level = cv::QRCodeEncoder::CORRECT_LEVEL_H;
	}
	else
	{
		params.correction_level = cv::QRCodeEncoder::CORRECT_LEVEL_L;
	}

	auto qrEncoder = cv::QRCodeEncoder::create(params);
	cv::Mat generated;
	qrEncoder->encode(contents, generated);
    while (generated.cols < codeSize)
    {
        cv::resize(generated, generated, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
    }
	if (imageSize > codeSize)
	{
		cv::resize(generated, generated, cv::Size(imageSize, imageSize));
	}
	cv::imwrite(outputPath + "/" + contents + ".png", generated);
	if (showGenerated)
	{
		cv::imshow("generated", generated);
		cv::waitKey(0);
	}

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		// instance = nullptr;

		// all the default sigint handler does is call shutdown()
		ros::shutdown();
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
	ros::AsyncSpinner spinner(0);
	spinner.start();
	ros::waitForShutdown();
#else
	ros::MultiThreadedSpinner spinner(0);
	spinner.spin();
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
