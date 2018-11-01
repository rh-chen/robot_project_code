/*!
*****************************************************************
* \file
*
* \note
* Copyright (c) 2013 \n
* Fraunhofer Institute for Manufacturing Engineering
* and Automation (IPA) \n\n
*
*****************************************************************
*
* \note
* Project name: care-o-bot
* \note
* ROS stack name: autopnp
* \note
* ROS package name: autopnp_dirt_detection
*
* \author
* Author: Richard Bormann
* \author
* Supervised by:
*
* \date Date of creation: October 2011
*
* \brief
* Module for detecting dirt on surfaces.
*
*****************************************************************
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer. \n
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution. \n
* - Neither the name of the Fraunhofer Institute for Manufacturing
* Engineering and Automation (IPA) nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission. \n
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************/

#ifndef DIRT_DETECTION_H_
#define DIRT_DETECTION_H_

//##################
//#### includes ####

// standard includes
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <time.h>
#include <math.h>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <autopnp_dirt_detection/DirtDetectionConfig.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>

// services
#include <std_srvs/Empty.h>
#include <autopnp_dirt_detection/ActivateDirtDetection.h>
#include <autopnp_dirt_detection/DeactivateDirtDetection.h>
#include <autopnp_dirt_detection/GetDirtMap.h>
#include <autopnp_dirt_detection/ValidateCleaningResult.h>

// topics
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/ml/ml.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/pcd_io.h>

//bridge
#include <cv_bridge/cv_bridge.h>
//#include <cv_bridge/CvBridge.h>

//boost
#include <boost/foreach.hpp>
#include <boost/thread/mutex.hpp>

#include <time.h>
#include "autopnp_dirt_detection/label_box.h"


namespace ipa_DirtDetection {

using namespace std;



/**
 *  Detects dirt from color image.
 */
class DirtDetection
{
protected:

	/**
	 * ROS node handle.
	 */
	ros::NodeHandle node_handle_;

	/// dynamic reconfigure
	dynamic_reconfigure::Server<autopnp_dirt_detection::DirtDetectionConfig> dynamic_reconfigure_server_;

	/**
	 * services
	 */
	ros::ServiceServer activate_dirt_detection_service_server_;		/// server for activating dirt detection
	ros::ServiceServer deactivate_dirt_detection_service_server_;	/// server for deactivating dirt detection
	ros::ServiceServer get_map_service_server_;						/// server for dirt map requests
	ros::ServiceServer validate_cleaning_result_service_server_;	/// server for validating cleaning results
	ros::ServiceServer reset_maps_service_server_;					/// server for resetting dirt maps

	bool activateDirtDetection(autopnp_dirt_detection::ActivateDirtDetection::Request &req, autopnp_dirt_detection::ActivateDirtDetection::Response &res);

	bool deactivateDirtDetection(autopnp_dirt_detection::DeactivateDirtDetection::Request &req, autopnp_dirt_detection::DeactivateDirtDetection::Response &res);

	bool getDirtMap(autopnp_dirt_detection::GetDirtMap::Request &req, autopnp_dirt_detection::GetDirtMap::Response &res);

	// this function assumes that all positions to check are visible at the moment the function is called
	bool validateCleaningResult(autopnp_dirt_detection::ValidateCleaningResult::Request &req, autopnp_dirt_detection::ValidateCleaningResult::Response &res);

	bool resetDirtMaps(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

	int sumOfUCharArray(const std::vector<unsigned char>& vec);

	/**
	 * Used to subscribe and publish images.
	 */
	image_transport::ImageTransport* it_;

	tf::TransformListener transform_listener_;
	tf::TransformBroadcaster transform_broadcaster_;

	/**
	 * Used to receive color image topic from camera.
	 */
	image_transport::Subscriber color_camera_image_sub_;
	/**
	 * Used to receive point cloud topic from camera.
	 */
	ros::Subscriber camera_depth_points_sub_;
	ros::Subscriber floor_plan_sub_;

	//ros::Publisher floor_plane_pub_;
	ros::Publisher camera_depth_points_from_bag_pub_;
	ros::Publisher clock_pub_;
	ros::Publisher ground_truth_map_pub_;
	ros::Publisher detection_map_pub_;
	image_transport::Publisher dirt_detection_image_pub_; ///< topic for publishing the image containing the dirt positions
	image_transport::Publisher dirt_detection_image_with_map_pub_;	///< image containing the map and the found dirt regions (mainly usable for visualization)

	// labeling
	bool labelingStarted_;
	std::vector<labelImage> labeledImages_;

	// grid map
	double gridResolution_;		// resolution of the grid in [cells/m]
	cv::Point2d gridOrigin_;	// translational offset of the grid map with respect to the /map frame origin, in [m], (The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.)
	cv::Point2i gridDimensions_;	// number of grid cells in x and y direction = width and height [in number grid cells]
	cv::Mat gridPositiveVotes_;		// grid map that counts the positive votes for dirt
	cv::Mat gridNumberObservations_;		// grid map that counts the number of times that the visual sensor has observed a grid cell
	std::vector<std::vector<std::vector<unsigned char> > > listOfLastDetections_;	// stores a list of the last x measurements (detection/no detection) for each grid cell (indices: 1=u, 2=v, 3=history)
	cv::Mat historyLastEntryIndex_;	// stores the index of last modified number in the history array (type: 32SC1)
	int detectionHistoryDepth_;		// number of time steps used for the detection history logging
	cv::Mat dirtMappingMask_;	// a mask that defines areas in the map where dirt detections are valid (i.e. this mask can be used to exclude areas from dirt mapping, white=detection area, black=do not detect)

	// evaluation
	int rosbagMessagesProcessed_;	// number of ros messages received by the program
	double meanProcessingTimeSegmentation_;		// average time needed for segmentation
	double meanProcessingTimeDirtDetection_;		// average time needed for dirt detection

	//parameters
	int spectralResidualGaussianBlurIterations_;
	double dirtThreshold_;
	double spectralResidualNormalizationHighestMaxValue_;
	double spectralResidualImageSizeRatio_;
	double dirtCheckStdDevFactor_;
	int modeOfOperation_;
	double birdEyeResolution_;		// resolution for bird eye's perspective [pixel/m]
	bool dirtDetectionActivatedOnStartup_;	// for normal operation mode, specifies whether dirt detection is on right from the beginning
	std::string dirtMappingMaskFilename_;	// if not an empty string, this enables using a mask that defines areas in the map where dirt detections are valid (i.e. this mask can be used to exclude areas from dirt mapping, white=detection area, black=do not detect)
	bool useDirtMappingMask_;

	std::string experimentFolder_;		// storage location of the database index file and writing location for the results of an experiment
	std::string labelingFilePath_;		// path to labeling file storage

	std::map<std::string, bool> debug_;

	bool warpImage_;	// if true, image warping to a bird's eye perspective is enabled
	double maxDistanceToCamera_;	// only those points which are close enough to the camera are taken [max distance in m]
	bool removeLines_;	// if true, strong lines in the image will not produce dirt responses

	// plane search
	int floorSearchIterations_;		// the number of attempts to segment the floor plane in the image
	int minPlanePoints_;		// minimum number of points that are necessary to find the floor plane
	double planeNormalMaxZ_;	// maximum z-value of the plane normal (ensures to have an floor plane)
	double planeMaxHeight_;		// maximum height of the detected plane above the mapped ground

	// further
	ros::Time lastIncomingMessage_;
	bool dirtDetectionCallbackActive_;		///< flag whether incoming messages shall be processed
	bool storeLastImage_;					///< if true, the last (warped) image is stored as well as all variables necessary for conversion
	boost::mutex storeLastImageMutex_;		///< mutex for securing read and write access to the last image data
	struct LastImageDataStorage
	{
		cv::Mat plane_color_image_warped;	// normal image (not warped) if warp is disabled
		cv::Mat R;				// transformation between world and floor plane coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
		cv::Mat t;
		cv::Point2f cameraImagePlaneOffset;		// offset in the camera image plane. Conversion from floor plane to  [xc, yc]
		tf::StampedTransform transformMapCamera;	// 3D transform between camera and map (pointWorldMap = transformMapCamera * pointWorldCamera)
	};
	LastImageDataStorage lastImageDataStorage_;	///< stores the image data of the last image
	nav_msgs::OccupancyGrid floor_plan_;	///< map of the environment
	bool floor_plan_received_;				///< flag whether the florr plan has been received already

public:

	/**
	 * Needed to set pixel color.
	 */
	struct bgr
	{
		uchar b; /**< Blue channel value. */
		uchar g; /**< Green channel value. */
		uchar r; /**< Red channel value. */
	};


	/**
	 * Used to describe a carpet.
	 */
	struct CarpetFeatures
	{
		float min; 	/**< Minimum value in the "C1_saliency_image_with_artifical_dirt" image. */
		float max; 	/**< Maximum value in the "C1_saliency_image_with_artifical_dirt" image. */
		float mean; 	/**< Mean value in the "C1_saliency_image_with_artifical_dirt" image. */
		float stdDev; 	/**< Standard deviation in the "C1_saliency_image_with_artifical_dirt" image. */

	};

	/**
	 * Determines the class of a carpet.
	 */
	struct CarpetClass
	{
		float dirtThreshold;	/**< Carpet-label. */
	};

	struct NumStruc
	{
		int correctnum; /**< Number of correct classified samples. */
		int totalnum;	/**< Total number of samples of this class. */
	};

	struct Statistics
	{
		int tp;
		int fp;
		int fn;
		int tn;
		int tpr;
		int fpr;
		int fnr;
		int tnr;
		void setZero() {tp=0; fp=0; fn=0; tn=0; tpr=0; fpr=0; fnr=0; tnr=0;};
	};


	/**
	 * Constructor.
	 */
	DirtDetection(ros::NodeHandle node_handle);

	/**
	 * Destructor.
	 */
	~DirtDetection();

	/**
	 * Create subscribers.
	 */
	void init();


	void resetMapsAndHistory();


	// dynamic reconfigure
	void dynamicReconfigureCallback(autopnp_dirt_detection::DirtDetectionConfig &config, uint32_t level);



	void floorPlanCallback(const nav_msgs::OccupancyGridConstPtr& map_msg);

//	/**
//	 * Function is called if color image topic is received.
//	 *
//	 * @param [in] color_image_msg	Color image message from camera.
//	 *
//	 */
//	void imageDisplayCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	/**
	 * Function is called if point cloud topic is received.
	 *
	 * @param [in] point_cloud2_rgb_msg	Point cloude message from camera.
	 *
	 */
	void dirtDetectionCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);

	void planeLabelingCallback(const sensor_msgs::PointCloud2ConstPtr& point_cloud2_rgb_msg);

	void databaseTest();

	void createOccupancyGridMapFromDirtDetections(nav_msgs::OccupancyGrid& detectionMap);

	/**
	 * Converts: "sensor_msgs::Image::ConstPtr" \f$ \rightarrow \f$ "cv::Mat".
	 *	@param [in] 	color_image_msg 		Color image message from camera.
	 *	@param [in] 	color_image_ptr			See cv_bridge message to cv::Mat converter manual.
	 *	@param [out] 	color_image 			Color image from the message, in OpenCV representation.
	 */
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);


	/**
	 * Converts: "sensor_msgs::PointCloud2" \f$ \rightarrow \f$ "pcl::PointCloud<pcl::PointXYZRGB>::Ptr".
	 *	@param [in] 	point_cloud2_rgb_msg 		Point cloud message from camera.
	 *	@param [out] 	point_cloud_XYZRG 			Point cloud representation in PCL.
	 */
	void convertPointCloudMessageToPointCloudPcl(const sensor_msgs::PointCloud2::ConstPtr& point_cloud2_rgb_msg, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &point_cloud_XYZRGB);


	/**
	 * Converts: "sensor_msgs::PointCloud2" \f$ \rightarrow \f$ "pcl::PointCloud<pcl::PointXYZRGB>::Ptr".
	 *
	 * The function detects a plane in the point cloud, creates a mask for the plane pixels and sets all pixels in
	 * "plane_color_image" to their color if they lie in the plane, else to black.
	 *
	 *	@param [in] 	input_cloud 				Point cloud for plane detection.
	 *	@param [out] 	plane_color_image 			Shows the true color of all pixel within the plane. The size of the image is determined with the help of the point cloud!
	 *	@param [out]	plane_mask					Mask to separate plane pixels. Plane pixels are white (255), all other pixels are black (0).
	 *	@return 		True if any plane could be found in the image.
	 */
	bool planeSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, const tf::StampedTransform& transform_map_camera, cv::Mat& grid_number_observations);

	/// remove perspective from image
	/// @param H Homography that maps points from the camera plane to the floor plane, i.e. pp = H*pc
	/// @param R Rotation matrix for transformation between floor plane and world coordinates, i.e. [xw,yw,zw] = R*[xp,yp,0]+t and [xp,yp,0] = R^T*[xw,yw,zw] - R^T*t
	/// @param t Translation vector. See Rotation matrix.
	/// @param cameraImagePlaneOffset Offset in the camera image plane. Conversion from floor plane to  [xc, yc]
	bool computeBirdsEyePerspective(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud, cv::Mat& plane_color_image, cv::Mat& plane_mask, pcl::ModelCoefficients& plane_model, cv::Mat& H, cv::Mat& R, cv::Mat& t, cv::Point2f& cameraImagePlaneOffset, cv::Mat& plane_color_image_warped, cv::Mat& plane_mask_warped);

	/// converts point pointCamera, that lies within the floor plane and is provided in coordinates of the original camera image, into map coordinates
	void transformPointFromCameraImageToWorld(const cv::Mat& pointCamera, const cv::Mat& H, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld);

	/// converts point pointPlane, that lies within the floor plane and is provided in coordinates of the warped camera image, into map coordinates
	void transformPointFromCameraWarpedToWorld(const cv::Mat& pointPlane, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Point3f& pointWorld);

	void transformPointFromWorldToCameraWarped(const cv::Point3f& pointWorld, const cv::Mat& R, const cv::Mat& t, const cv::Point2f& cameraImagePlaneOffset, const tf::StampedTransform& transformMapCamera, cv::Mat& pointPlane);

	void putDetectionIntoGrid(cv::Mat& grid, const labelImage::RegionPointTriple& detection);

	/**
	 * This function performs the saliency detection to spot dirt stains.
	 *
	 * @param [in] 	C1_image				!!!ONE CHANNEL!!!('C1') image used to perfom the salciency detection.
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 */
	void SaliencyDetection_C1(const cv::Mat& C1_image, cv::Mat& C1_saliency_image);


	/**
	 * This function performs a saliency detection for a  3 channel color image.
	 *
	 * The function proceeds as follows:
	 * 						1.) The 3 channel image is split into their 3 channels.
	 * 						2.) The saliency detection is performed for each channel.
	 * 						3.) The resulting images are add up.
	 *
	 * @param [in] 	C3_color_image			!!!THREE CHANNEL!!!('C3') image used to perform the saliency detection.
	 * @param [out]	C1_saliency_image		One channel image('C1') which results from the saliency detection.
	 * @param [in] 	mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 * @param [in]	gaussianBlurCycles		Determines the number of repetitions of the gaussian filter used to reduce the noise.
	 */
	void SaliencyDetection_C3(const cv::Mat& C3_color_image, cv::Mat& C1_saliency_image, const cv::Mat* mask = 0, int gaussianBlurCycles = 2);

	/**
	 * This function uses the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 *
	 *
	 * @param [in]		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [out] 	C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 * @param [in,out]	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 */
	void Image_Postprocessing_C1(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image);


	/**
	 *
	 * This function avoids the false detections if no dirt is on the floor.
	 * Furthermore, it returns an image ("C1_BlackWhite_image") in which all dirt pixels are white (255) and all other pixels are black (0).
	 * It also the "C1_saliency_image" to mark the dirt in the "C3_color_image".
	 *
	 * @param [in] 		C1_saliency_image		One channel('C1') saliency image used for postprocessing.
	 * @param [out]		C1_BlackWhite_image		One channel('C1') image in which all dirt pixels are white (255) and all other pixels are black (0).
	 * @param [in,out] 	C3_color_image			Three channel('C3') color which corresponds to the "C1_saliency_image".
	 * @param [in]		mask					Determines the area of interest. Pixel of interests are white (255), all other pixels are black (0).
	 *
	 */
	void Image_Postprocessing_C1_rmb(const cv::Mat& C1_saliency_image, cv::Mat& C1_BlackWhite_image, cv::Mat& C3_color_image, std::vector<cv::RotatedRect>& dirtDetections, const cv::Mat& mask = cv::Mat());


	/**
	 * This function is out of date and must not be used! The function is kept as backup copy.
	 *
	 * @param [in] color_image_msg		Color image message from camera.
	 */
//	void SaliencyDetection_C1_old_cv_code(const sensor_msgs::ImageConstPtr& color_image_msg);

	/**
	 * This function creates/calculates a carpet-classifier for the given carpets. In this case an opencv support vector machine (SVM)
	 * is used as carpet-classifier.
	 *
	 * @param [in]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in]  carp_class_vec	Vector containing the class specifier of the carpets.
	 * @param [out]	carpet_SVM		Returns an opencv support vector machine.
	 *
	 */
	void CreateCarpetClassiefierSVM(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec, CvSVM &carpet_SVM);


	/**
	 * This function creates/calculates a carpet-classifier for the given carpets. In this case an opencv tree model
	 * is used as carpet-classifier. PLEASE NOTE THAT THE OPENCV TREE LEARGNING ALGORITHMS DO NOT WORK PROPERLY!
	 *
	 * @param [in]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in]  carp_class_vec	Vector containing the class specifier of the carpets.
	 * @param [out]	carpet_Tree		Returns an opencv tree model.
	 *
	 */
	void CreateCarpetClassiefierRTree(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec, CvRTrees &carpet_Tree);

	/**
	 * This function creates/calculates a carpet-classifier for the given carpets. In this case an opencv gradient boosted tree model
	 * is used as carpet-classifier. PLEASE NOTE THAT THE OPENCV TREE LEARGNING ALGORITHMS DO NOT WORK PROPERLY!
	 *
	 * @param [in]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in]  carp_class_vec	Vector containing the class specifier of the carpets.
	 * @param [out]	carpet_GBTree	Returns an opencv gradient boosted tree model.
	 *
	 */
	void CreateCarpetClassiefierGBTree(const std::vector<CarpetFeatures>& carp_feat_vec, const std::vector<CarpetClass>& carp_class_vec,
			CvGBTrees &carpet_GBTree);


	/**
	 * This function illustrates how to use/implement openCV-SVM.
	 * The function has no other purpose than to illustrate how to use/implement openCV-SVM.
	 */
	void SVMExampleCode();


	/**
	 * This function illustrates how to use/implement openCV-SVM.
	 * The function has no other purpose than to illustrate how to use/implement openCV-SVM.
	 */
	//void SVMTestFunction();

	/**
	 * Reads the carpet features and the class of the carpet from a file and saves them to the corresponding vectors.
	 * Please note that the data are added to the given vectors, in other words, the vectors are not reset!
	 *
	 * @param [in,out]	carp_feat_vec	Vector which contains the features of the different carpets.
	 * @param [in,out]	carp_class_vec 	Vector which saves the class of each carpet.
	 * @param [in]		filepath		The path to the given file.
	 * @param [in] 		filename		Name of the file which contains the different carpet data.
	 *
	 */
	void ReadDataFromCarpetFile(std::vector<CarpetFeatures>& carp_feat_vec, std::vector<CarpetClass>& carp_class_vec, std::string filepath, std::string filename);

	/**
	 * Splits the carpets, given by the corresponding feature and class vector, into: \n
	 * 1.) carpets which are used to train the different machine learning algorithms and \n
	 * 2.) carpets which are used to test the different algorithms.
	 *
	 * @param [in]		NumTestSamples	Number of carpets which are used to test the algorithms.
	 * @param [in]		input_feat_vec	Vector containing the features of the carpets which have to be split.
	 * @param [in]		input_class_vec Vector containing the class specifier of the carpets which have to be split..
	 * @param [in,out]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms. Please note that the data are added to the already existing data in the vector!
	 * @param [in,out] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms. Please note that the data are added to the already existing data in the vector!
	 * @param [in,out]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms. Please note that the data are added to the already existing data in the vector!
	 * @param [in,out] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms. Please note that the data are added to the already existing data in the vector!
	 *
	 */
	void SplitIntoTrainAndTestSamples(	int NumTestSamples,
										std::vector<CarpetFeatures>& input_feat_vec, std::vector<CarpetClass>& input_class_vec,
										std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
										std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec);

	/**
	 * This function can be used to test a specific opencv support vector machine. The function, however, is only usable if only one or two features are used to classify
	 * a carpet. For more features it is necessary to adapt the function!
	 *
	 * @param [in]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms.
	 * @param [in] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms.
	 * @param [in]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms.
	 * @param [in] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms.
	 * @param [in]	carpet_SVM		Opencv support vector machine.
	 *
	 */
	void SVMEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvSVM &carpet_SVM, double ScaleMean, double ScaleStd);


	/**
	 * This function can be used to test a specific opencv random tree model. The function, however, is only usable if only one or two features are used to classify
	 * a carpet. For more features it is necessary to adapt the function!
	 *
	 * @param [in]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms.
	 * @param [in] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms.
	 * @param [in]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms.
	 * @param [in] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms.
	 * @param [in]	carpet_Tree		Opencv random tree model.
	 *
	 */
	void RTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvRTrees &carpet_Tree, double ScaleMean, double ScaleStd);

	/**
	 * This function can be used to test a specific opencv gradient boosted tree model. The function, however, is only usable if only one or two features are used to classify
	 * a carpet. For more features it is necessary to adapt the function!
	 *
	 * @param [in]	train_feat_vec	Vector containing the features of the carpets which are used to train the different algorithms.
	 * @param [in] 	train_class_vec	Vector containing the class specifier of the carpets which are used to train the different algorithms.
	 * @param [in]	test_feat_vec	Vector containing the features of the carpets which are used to test the different algorithms.
	 * @param [in] 	test_class_vec	Vector containing the class specifier of the carpets which are used to test the different algorithms.
	 * @param [in]	carpet_GBTree	Opencv gradient boosted tree model.
	 *
	 */
	void GBTreeEvaluation(	std::vector<CarpetFeatures>& train_feat_vec, std::vector<CarpetClass>& train_class_vec,
						std::vector<CarpetFeatures>& test_feat_vec, std::vector<CarpetClass>& test_class_vec,
						CvGBTrees &carpet_GBTree, double ScaleMean, double ScaleStd);

	/**
	 * This function scales the features to the interval [0,1]. It returns the scaled features and the
	 * scaling parameters.
	 *
	 * @param [in, out]	feat_vec 	Features which have to be scaled.
	 * @param [out]	maxMean			Scaling parameter for the mean values.
	 * @param [out]	maxStd			Scaling parameter for the standard deviation values.
	 *
	 */
	void ScaleSamples(std::vector<CarpetFeatures>& feat_vec,double & maxMean, double & maxStd);


};	//end-class

}; //end-namespace


#endif /* DIRT_DETECTION_H_ */
