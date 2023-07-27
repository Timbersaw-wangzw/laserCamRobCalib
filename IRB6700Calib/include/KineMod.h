#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

using namespace std;
using namespace Eigen;

class KineMod
{
public:
	//绕单轴旋转的矩阵
	Eigen::Matrix4d RotX(double theta);

	Eigen::Matrix4d RotY(double theta);

	Eigen::Matrix4d RotZ(double theta);

	//沿轴平移矩阵
	Eigen::Matrix4d Trans(double x, double y, double z);

	 //计算从机器人起始坐标系到末端坐标系的转换矩阵
	virtual vector <Eigen::Matrix4d> CalculRTBegin2End() = 0;
};

