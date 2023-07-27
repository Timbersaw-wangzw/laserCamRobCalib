#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "KineMod.h"
#include "GlobalData.h"

using namespace std;
using namespace Eigen;

//把XYZ欧拉角转换为旋转矩阵
vector<Matrix3d> Euler2Matrix3dVec(vector<Vector3d> eulerVec);

Matrix3d Euler2Matrix3d(Vector3d euler);

//把欧拉角和位置参数转化为位姿矩阵
//形参：位置在前，欧拉角在后
vector<Matrix4d> Conv2RTVec(vector<VectorXd> eulerPositionVec);

//形参：位置position在前，欧拉角euler在后
Matrix4d Conv2RT(VectorXd PE);

//把位姿矩阵转化为XYZ欧拉角和位置参数的向量，位置在前，欧拉角在后
vector<VectorXd> Conv2PEVec(vector<Matrix4d> rtVec);

//把位姿矩阵转化为XYZ欧拉角和位置参数的向量，位置在前，欧拉角在后
VectorXd Conv2PE(Matrix4d rt);

//把动态向量转化为_4MCPCParas，_6MCPCParas
_4MCPCParas Conv2_4MCPCParas(VectorXd vec);

_6MCPCParas Conv2_6MCPCParas(VectorXd vec);

//把容器中的容器中的数据转化为动态向量
template <typename T> Matrix<T, Dynamic, 1> Conv2VecX(vector<vector<T>> vec2);

//把动态向量转化为容器中的容器中的数据
//indexArray中包含每个小容器中的数据个数，根据该数组来分配容器中的数据
template <typename T> vector<vector<T>> Conv2Vec2(Matrix<T, Dynamic, 1> vecX, int *indexArray);

//计算位姿误差及模,每一行有8个数据，前4个是位置误差和模，后4个是欧拉角误差和模PE1-PE2
vector<VectorXd> CalculPEErrors(vector<VectorXd> PE1, vector<VectorXd> PE2);

//获取最大的数
double GetMaxValue(vector<double> valueVec);

//获取平均值
double GetMeanValue(vector<double> valueVec);

//获取均方根
double GetRMSValue(vector<double> valueVec);

//获取标准差
double GetSDValue(vector<double> valueVec);

//绕单轴旋转的矩阵
Eigen::Matrix4d RotX(double theta);

Eigen::Matrix4d RotY(double theta);

Eigen::Matrix4d RotZ(double theta);

//沿轴平移矩阵
Eigen::Matrix4d Trans(double x, double y, double z);


template<typename T>
inline Matrix<T, Dynamic, 1> Conv2VecX(vector<vector<T>> vec2)
{

	//计算数据量
	int sizeVec2 = 0;
	for (int i = 0; i < vec2.size(); ++i)
	{
		sizeVec2 = sizeVec2 + vec2[i].size();
	}
	Matrix<T, Dynamic, 1> nominalMCPCParas = Matrix<T, Dynamic, 1>::Zero(sizeVec2, 1);
	//Matrix<double, Dynamic, 1> nominalMCPCParas = Matrix<double, Dynamic, 1>::Zero(sizeVec2, 1);
	int indexPara = 0;
	for (int i = 0; i < vec2.size(); ++i)
	{
		for (int j = 0; j < vec2[i].size(); ++j)
		{
			nominalMCPCParas[indexPara] = vec2[i][j];
			indexPara += 1;
		}
	}
	return nominalMCPCParas;
}


template<typename T>
inline vector<vector<T>> Conv2Vec2(Matrix<T, Dynamic, 1> vecX, int *indexArray)
{
	int index = 0;
	vector <vector<T>>  compensatedMCPCParaVec;
	vector<T> compensatedMCPCParas;
	for (int i = 0; i < vecX.size(); ++i)
	{
		compensatedMCPCParas.push_back(vecX[i]);
		if (compensatedMCPCParas.size() >= indexArray[index])
		{
			index += 1;
			compensatedMCPCParaVec.push_back(compensatedMCPCParas);
			compensatedMCPCParas.clear();
		}
	}
	return compensatedMCPCParaVec;
}

