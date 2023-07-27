#pragma once
#include "ErrorModel.h"
#include "GlobalData.h"
#include "MCPCKineMod.h"
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;


class MCPCErrorModel :public ErrorModel, MCPCKineMod
{
public:
	MCPCErrorModel();
	~MCPCErrorModel();

	//把MCPC参数转换成结构体的形式
	MCPCErrorModel(vector<vector<double>> kineParas, vector<vector<double>> jointAngles,
		vector<VectorXd> nominalPEVec,vector<VectorXd> measurePEVec,
		Matrix4d T);

	MatrixXd Calculjacobi();

	//计算从导轨r坐标系到tool0坐标系的运动学误差系数矩阵
	vector<MatrixXd> CalculM();

	//计算从机器人起始坐标系到各个关节坐标系的旋转矩阵
	vector<vector<Matrix3d>> CalculRotBegin2I();

	//计算从机器人各个关节坐标系到末端坐标系的平移矩阵
	vector<vector<Vector3d>> CalculTransI2End();

	//更新运动学参数
	void UpdataParasVec(VectorXd addedParasVec);
private:
	vector<_4MCPCParas> Get4ParasVec();
	vector<_6MCPCParas> Get6ParasVec();
	vector<vector<double>> GetjointAngles();
	vector<vector<double>> GetMCPCParas();

	vector<_4MCPCParas> m_4MCPCParasVec;
	vector<_6MCPCParas> m_6MCPCParasVec;
	vector<vector<double>> m_MCPCParasVec2;

	vector<vector<double>> m_jointAngles;
};

