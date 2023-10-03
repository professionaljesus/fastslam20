#include "utils.h"

using namespace std;
using namespace Eigen;

IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

Vector2d to_global(const Vector2d& z, const Vector3d& pose) 
{
	double theta = pose.z();

	Vector2d global_coord;

	global_coord << pose.x() + z.x()*cos(theta) - z.y()*sin(theta), pose.y() + z.x()*sin(theta) + z.y()*cos(theta);

	return global_coord;
}



//To poseations local polar system
Vector2d to_local(const Vector2d& lm_pos, const Vector3d& pose)
{

	double dx = lm_pos.x() - pose.x();
	double dy = lm_pos.y() - pose.y();
	double theta = pose.z();
	Vector2d local_coord;
	local_coord << dx*cos(-theta) - dy*sin(-theta), dx*sin(-theta) + dy*cos(-theta);

	return local_coord;
}


//https://www.wolframalpha.com/input/?i=jacobian+of+%28%28x-a%29*cos%28-c%29+-+%28y-b%29*sin%28-c%29%2C+%28x-a%29*sin%28-c%29+%2B+%28y-b%29*cos%28-c%29+%29+with+respect+to+x%2Cy
//Calculate measurement jacobian H
Matrix2d measurement_jacobian_landmark(const Vector2d& landmark_pos, const Vector3d& pose) {


	double theta = pose.z();
	Matrix2d jacobian;

	jacobian << cos(theta), sin(theta),
				-sin(theta), cos(theta);

	return jacobian;  
}

//https://www.wolframalpha.com/input/?i=jacobian+of+%28sqrt%28%28x+-+a%29%5E2+%2B+%28y+-+b%29%5E2%29%2C+arctan%28%28y+-+b+%29%2F%28x+-+a%29%29+-+c+%29+with+respect+to+a%2Cb%2Cc
Matrix<double, 2, 3> measurement_jacobian_pose(const Vector2d& landmark_pos, const Vector3d& pose) {

	double dx = landmark_pos.x() - pose.x();
	double dy = landmark_pos.y() - pose.y();
	double theta = pose.z();

	Matrix<double, 2, 3> jacobian;

	jacobian << -cos(theta), -sin(theta), dy*cos(theta) - dx*sin(theta),
				sin(theta), -cos(theta), -dx*cos(theta) - dy*sin(theta); 

	return jacobian;  
}

