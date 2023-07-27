#include "KineMod.h"

Eigen::Matrix4d KineMod::RotX(double theta)
{
	Eigen::Matrix4d rx;
	rx << 1, 0, 0, 0,
		0, cos(theta), -sin(theta), 0,
		0, sin(theta), cos(theta), 0,
		0, 0, 0, 1;
	return rx;
}


Eigen::Matrix4d KineMod::RotY(double theta)
{
	Eigen::Matrix4d ry;
	ry << cos(theta), 0, sin(theta), 0,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 1;
	return ry;
}


Eigen::Matrix4d KineMod::RotZ(double theta)
{
	Eigen::Matrix4d rz;
	rz << cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return rz;
}


Eigen::Matrix4d KineMod::Trans(double x, double y, double z)
{
	Eigen::Matrix4d t;
	t << 1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;
	return t;
}

