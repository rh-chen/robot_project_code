/*
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.
*/

#ifndef _STEREOCAMERAMODEL_H
#define _STEREOCAMERAMODEL_H

#include "reconstruction3d_nodelet/CameraModel.h"

class StereoCameraModel
{
public:
	StereoCameraModel() : leftSuffix_("left"), rightSuffix_("right") {}
	StereoCameraModel(
			const std::string & name,
			const cv::Size & imageSize1,
			const cv::Mat & K1, const cv::Mat & D1, const cv::Mat & R1, const cv::Mat & P1,
			const cv::Size & imageSize2,
			const cv::Mat & K2, const cv::Mat & D2, const cv::Mat & R2, const cv::Mat & P2,
			const cv::Mat & R, const cv::Mat & T, const cv::Mat & E, const cv::Mat & F,
			const Transform & localTransform = Transform::getIdentity());

	// if R and T are not null, left and right camera models should be valid to be rectified.
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const cv::Mat & R = cv::Mat(),
			const cv::Mat & T = cv::Mat(),
			const cv::Mat & E = cv::Mat(),
			const cv::Mat & F = cv::Mat());
	// if extrinsics transform is not null, left and right camera models should be valid to be rectified.
	StereoCameraModel(
			const std::string & name,
			const CameraModel & leftCameraModel,
			const CameraModel & rightCameraModel,
			const Transform & extrinsics);

	//minimal
	StereoCameraModel(
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform::getIdentity(),
			const cv::Size & imageSize = cv::Size(0,0));
	//minimal to be saved
	StereoCameraModel(
			const std::string & name,
			double fx,
			double fy,
			double cx,
			double cy,
			double baseline,
			const Transform & localTransform = Transform::getIdentity(),
			const cv::Size & imageSize = cv::Size(0,0));
	virtual ~StereoCameraModel() {}

	bool isValidForProjection() const {return left_.isValidForProjection() && right_.isValidForProjection() && baseline() > 0.0;}
	bool isValidForRectification() const {return left_.isValidForRectification() && right_.isValidForRectification();}

	void initRectificationMap() {left_.initRectificationMap(); right_.initRectificationMap();}

	void setName(const std::string & name, const std::string & leftSuffix = "left", const std::string & rightSuffix = "right");
	const std::string & name() const {return name_;}

	// backward compatibility
	void setImageSize(const cv::Size & size) {left_.setImageSize(size); right_.setImageSize(size);}

	bool load(const std::string & directory, const std::string & cameraName, bool ignoreStereoTransform = true);
	bool save(const std::string & directory, bool ignoreStereoTransform = true) const;
	bool saveStereoTransform(const std::string & directory) const;

	double baseline() const {return right_.fx()!=0.0 && left_.fx() != 0.0 ? left_.Tx() / left_.fx() - right_.Tx()/right_.fx():0.0;}

	float computeDepth(float disparity) const;
	float computeDisparity(float depth) const; // m
	float computeDisparity(unsigned short depth) const; // mm

	const cv::Mat & R() const {return R_;} //extrinsic rotation matrix
	const cv::Mat & T() const {return T_;} //extrinsic translation matrix
	const cv::Mat & E() const {return E_;} //extrinsic essential matrix
	const cv::Mat & F() const {return F_;} //extrinsic fundamental matrix

	void scale(double scale);
	void roi(const cv::Rect & roi);

	void setLocalTransform(const Transform & transform) {left_.setLocalTransform(transform);}
	const Transform & localTransform() const {return left_.localTransform();}
	Transform stereoTransform() const;

	const CameraModel & left() const {return left_;}
	const CameraModel & right() const {return right_;}

	const std::string & getLeftSuffix() const {return leftSuffix_;}
	const std::string & getRightSuffix() const {return rightSuffix_;}

private:
	std::string leftSuffix_;
	std::string rightSuffix_;
	CameraModel left_;
	CameraModel right_;
	std::string name_;
	cv::Mat R_;
	cv::Mat T_;
	cv::Mat E_;
	cv::Mat F_;
};

#endif /* STEREOCAMERAMODEL_H_ */
