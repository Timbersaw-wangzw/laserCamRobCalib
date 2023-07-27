#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "KineMod.h"
using namespace std;
using namespace Eigen;
class DHKineMod:public KineMod
{
public:
	DHKineMod();
	~DHKineMod();

	DHKineMod(vector<vector<double>> DHParas, vector<vector<double>> jointAngles);
	//从机器人基坐标系到tool0坐标系的转换矩阵（MCPC运动学模型）
	vector <Eigen::Matrix4d> CalculRTBegin2End();

	vector<vector<Eigen::Matrix3d>> CalculRotBegin2I();

	//计算从机器人各个关节坐标系到末端坐标系的平移矩阵
	vector<vector<Eigen::Vector3d>> CalculTransI2End();

private:
	//当关节为转动副，相邻连杆的转换矩阵MCPC
	Eigen::Matrix4d R_RT(double alpha, double beta, double theta, double a, double d);

	vector<vector<double>> GetMCPCParas();

	vector<vector<double>> GetjointAngles();

	vector<vector<double>> m_MCPCParas;
	vector<vector<double>> m_jointAngles;
};

