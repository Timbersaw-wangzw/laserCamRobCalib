#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "KineMod.h"
using namespace std;
using namespace Eigen;
//6轴机器人MCPC运动学模型类
class MCPCKineMod:public KineMod
{

public:
	MCPCKineMod();
	~MCPCKineMod();

	MCPCKineMod(vector<vector<double>> MCPCParas, vector<vector<double>> jointAngles);
	//从机器人基坐标系到tool0坐标系的转换矩阵（MCPC运动学模型）
	vector <Eigen::Matrix4d> CalculRTBegin2End();

	//当关节为转动副，相邻连杆的转换矩阵MCPC
	Eigen::Matrix4d R_RT(double alpha, double beta, double theta,double x, double y);

	//当关节为移动副，相邻连杆的转换矩阵MCPC
	Eigen::Matrix4d T_RT(double alpha, double beta, double d, double x, double y);

	//两坐标系间的一般刚体变换(相对动坐标系)
	Eigen::Matrix4d RT(double alpha, double beta,double gama, double theta, double x, double y,double z);

private:
	vector<vector<double>> GetMCPCParas();

	vector<vector<double>> GetjointAngles();

	vector<vector<double>> m_MCPCParas;
	vector<vector<double>> m_jointAngles;
};



