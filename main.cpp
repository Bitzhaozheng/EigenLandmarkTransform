#include "eigenLandmarkTransform.h"

int main(int argc, char* argv[])
{
	//Create source and target points(4 points) (Translation: [1, 1, 1])
	Eigen::Matrix<float, 3, 4> SourcePoints;
	Eigen::Matrix<float, 3, 4> TargetPoints;
	SourcePoints << 2, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
	TargetPoints << 3, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13;

	//Generate random noise in [-1, 1]
	srand(4396);
	Eigen::Matrix<float, 3, 4> SourceNoise = Eigen::Matrix<float, 3, 4>::Random() / 10;
	Eigen::Matrix<float, 3, 4> TargetNoise = Eigen::Matrix<float, 3, 4>::Random() / 10;
	SourcePoints = SourcePoints + SourceNoise;
	TargetPoints = TargetPoints + TargetNoise;

	//Transform matrix
	eigenLandmarkTransform transform;
	transform.SetSourcePoints(SourcePoints);
	transform.SetTargetPoints(TargetPoints);
	transform.ComputeTransform();
	cout << "Rotation:\n" << transform.GetRotationMatrix() << endl;
	cout << "Translation:\n" << transform.GetTranslationMatrix() << endl;
	cout << "TransformMarix4x4:\n" << transform.GetTransformMatrix4x4() << endl;

	//Error (Mean Euclidean distance)
	Eigen::Matrix<float, 3, 4> direction;
	Eigen::VectorXd error;
	error.setZero(4);
	for (int i = 0; i < 4; i++)
	{
		direction.block(0, i, 3, 1) = TargetPoints.block(0, i, 3, 1) - transform.GetRotationMatrix() * SourcePoints.block(0, i, 3, 1) - transform.GetTranslationMatrix();
		error(i) = direction.block(0, i, 3, 1).norm();
		cout << "Error " << i << ": " << error(i) << endl;
	}
	cout << "Mean error: " << error.mean() << endl;
}
