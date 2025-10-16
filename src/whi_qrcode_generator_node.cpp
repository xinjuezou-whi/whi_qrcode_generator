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

#include <rclcpp/rclcpp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

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
	std::cout << "\nWHI QR code generator VERSION 02.02.1" << std::endl;
	std::cout << "Copyright © 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    rclcpp::init(argc, argv);

	// create node
    const std::string nodeName("whi_qrcode_generator"); 
	auto nodeHandle = std::make_shared<rclcpp::Node>(nodeName);

	/// node logic
	// params
	nodeHandle->declare_parameter<std::string>("type", std::string("qr"));
	std::string type = nodeHandle->get_parameter("type").as_string();
	if (!type.empty())
	{
		std::transform(type.begin(), type.end(), type.begin(), [](unsigned char c) { return std::tolower(c); });
	}
	nodeHandle->declare_parameter<int>("image_size", 500);
	int imageSize = nodeHandle->get_parameter("image_size").as_int();
	nodeHandle->declare_parameter<std::string>("output_path", std::string("/home"));
	std::string outputPath = nodeHandle->get_parameter("output_path").as_string();
	nodeHandle->declare_parameter<std::string>("contents", std::string("hello world"));
	std::string contents = nodeHandle->get_parameter("contents").as_string();
	if (outputPath.size() > 1 && outputPath.back() == '/')
	{
		outputPath.pop_back();
	}
	nodeHandle->declare_parameter<bool>("show_generated", true);
	bool showGenerated = nodeHandle->get_parameter("show_generated").as_bool();

	// ArUco
	nodeHandle->declare_parameter<int>("marker_size", 4);
	int markerSize = nodeHandle->get_parameter("marker_size").as_int();
	// QR
	nodeHandle->declare_parameter<int>("code_size", 200);
	int codeSize = nodeHandle->get_parameter("code_size").as_int();
	nodeHandle->declare_parameter<std::string>("correction_level", std::string("low"));
	std::string correctionLevel = nodeHandle->get_parameter("correction_level").as_string();

	/// generate code
	cv::Mat generated;
	if (type == "aruco")
	{
		int id = atoi(contents.c_str());
		std::map<int, std::vector<int>> mapDict
		{
			{4, {cv::aruco::DICT_4X4_50, cv::aruco::DICT_4X4_100, cv::aruco::DICT_4X4_250, cv::aruco::DICT_4X4_1000}},
			{5, {cv::aruco::DICT_5X5_50, cv::aruco::DICT_5X5_100, cv::aruco::DICT_5X5_250, cv::aruco::DICT_5X5_1000}},
			{6, {cv::aruco::DICT_6X6_50, cv::aruco::DICT_6X6_100, cv::aruco::DICT_6X6_250, cv::aruco::DICT_6X6_1000}},
			{7, {cv::aruco::DICT_7X7_50, cv::aruco::DICT_7X7_100, cv::aruco::DICT_7X7_250, cv::aruco::DICT_7X7_1000}}
		};
		int dict = cv::aruco::DICT_4X4_50;
		if (id >= 50 && id < 100)
		{
			dict = mapDict[markerSize][1];
		}
		else if (id >= 100 && id < 250)
		{
			dict = mapDict[markerSize][2];
		}
		else if (id >= 250 && id < 1000)
		{
			dict = mapDict[markerSize][3];
		}
		else
		{
			dict = mapDict[markerSize][0];
		}
		auto dictionary = cv::aruco::getPredefinedDictionary(dict);
    	cv::aruco::generateImageMarker(dictionary, id, std::max(imageSize, 200), generated, 1);
	}
	else if (type == "qr")
	{
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
		qrEncoder->encode(contents, generated);
		while (generated.cols < codeSize)
		{
			cv::resize(generated, generated, cv::Size(0, 0), 2.0, 2.0, cv::INTER_NEAREST);
		}
		if (imageSize > codeSize)
		{
			cv::resize(generated, generated, cv::Size(imageSize, imageSize));
		}
	}

	/// save generated
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
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(nodeHandle);
    executor->spin();  // blocking until shutdown
#else
    rclcpp::spin(nodeHandle);
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
