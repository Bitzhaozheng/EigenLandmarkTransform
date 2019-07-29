#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace std;

class eigenLandmarkTransform
{
public:
	eigenLandmarkTransform();
	~eigenLandmarkTransform();

	void SetSourcePoints(Eigen::Matrix3Xf source);
	void SetTargetPoints(Eigen::Matrix3Xf target);
	void ComputeTransform();
	Eigen::Matrix3f GetRotationMatrix();
	Eigen::Vector3f GetTranslationMatrix();
	Eigen::Matrix4f GetTransformMatrix4x4();

protected:
	Eigen::Matrix3Xf sourcePoints;
	Eigen::Matrix3Xf targetPoints;
	Eigen::Matrix3f rotation;
	Eigen::Vector3f translation;
	Eigen::Matrix4f transformMatrix;
};