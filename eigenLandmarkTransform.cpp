#include "eigenLandmarkTransform.h"

eigenLandmarkTransform::eigenLandmarkTransform()
{
	sourcePoints.setIdentity();
	targetPoints.setIdentity();
	rotation.setIdentity();
	translation.setIdentity();
	transformMatrix.setIdentity();
}

eigenLandmarkTransform::~eigenLandmarkTransform()
{
}

void eigenLandmarkTransform::SetSourcePoints(Eigen::Matrix3Xf source)
{
	this->sourcePoints = source;
}

void eigenLandmarkTransform::SetTargetPoints(Eigen::Matrix3Xf target)
{
	this->targetPoints = target;
}

void eigenLandmarkTransform::ComputeTransform()
{
	if (sourcePoints.cols() != targetPoints.cols()) {
		cerr << "Number of points must be equal!" << endl;
		return;
	}

	//Centroid
	Eigen::Vector3f sMean, tMean;
	sMean << this->sourcePoints.block(0, 0, 1, sourcePoints.cols()).mean(), 
		this->sourcePoints.block(1, 0, 1, sourcePoints.cols()).mean(), 
		this->sourcePoints.block(2, 0, 1, sourcePoints.cols()).mean();
	tMean << this->targetPoints.block(0, 0, 1, targetPoints.cols()).mean(),
		this->targetPoints.block(1, 0, 1, targetPoints.cols()).mean(),
		this->targetPoints.block(2, 0, 1, targetPoints.cols()).mean();

	Eigen::Matrix3Xf sq, tq;
	sq = Eigen::Matrix3Xf::Zero(3, sourcePoints.cols());
	tq = Eigen::Matrix3Xf::Zero(3, targetPoints.cols());
	for (int i = 0; i < sourcePoints.cols(); i++)
	{
		sq.block(0, i, 3, 1) = this->sourcePoints.block(0, i, 3, 1) - sMean;
		tq.block(0, i, 3, 1) = this->targetPoints.block(0, i, 3, 1) - tMean;
	}

	//Compute rotation matrix
	Eigen::Matrix3f w;
	w.setZero();
	for (int i = 0; i < sourcePoints.cols(); i++) 
	{
		w += tq.block(0, i, 3, 1) * sq.block(0, i, 3, 1).transpose();
	}
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(w, Eigen::ComputeThinU | Eigen::ComputeThinV);
	this->rotation = svd.matrixU() * svd.matrixV().transpose();
	Eigen::Matrix3f e;
	e = e.Identity();
	e(2, 2)= 1 / rotation.determinant();
	this->rotation = svd.matrixU() * e * svd.matrixV().transpose();

	//Translation
	this->translation = tMean - this->rotation * sMean;
	this->transformMatrix.block(0, 0, 3, 3) = this->rotation;
	this->transformMatrix.block(0, 3, 3, 1) = this->translation;
}

Eigen::Matrix3f eigenLandmarkTransform::GetRotationMatrix()
{
	return this->rotation;
}

Eigen::Vector3f eigenLandmarkTransform::GetTranslationMatrix()
{
	return this->translation;
}

Eigen::Matrix4f eigenLandmarkTransform::GetTransformMatrix4x4()
{
	return this->transformMatrix;
}


