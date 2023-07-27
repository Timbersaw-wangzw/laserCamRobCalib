#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "DataConver.h"
#include "GlobalData.h"
using namespace std;
using namespace Eigen;
class ErrorModel
{
public:
	ErrorModel();
	~ErrorModel();

	//计算机器人末端测量点的位姿误差
	//T把测量数据转换到名义数据同一个参考坐标系
	VectorXd CalculErrors();

	//建立微分运动矢量转换关系矩阵6x6矩阵
	MatrixXd CalculPi(Matrix3d r, Vector3d t);

	//把三维向量转换为反对称矩阵
	Matrix3d Convert2AntiMatrix(Vector3d t);

	//建立雅克比矩阵
	virtual MatrixXd Calculjacobi()=0;

	//计算从机器人起始坐标系到各个关节坐标系的旋转矩阵
	virtual vector<vector<Matrix3d>> CalculRotBegin2I() = 0;

	//计算从机器人各个关节坐标系到末端坐标系的平移矩阵
	virtual vector<vector<Vector3d>> CalculTransI2End() = 0;

	//建立参数误差系数矩阵
	virtual vector<MatrixXd> CalculM()=0;

	//与增值相加，更新运动学参数,addedParasVec为运动学参数的增值
	virtual void UpdataParasVec(VectorXd addedParasVec) = 0;

	vector<VectorXd> m_nominalPE;
    vector<VectorXd> m_measurePE; 
	Matrix4d m_T;
};

