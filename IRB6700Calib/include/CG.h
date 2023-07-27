#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "MCPCErrorModel.h"
using namespace std;
using namespace Eigen;

//共轭梯度法
class CG 
{
public:
	CG();
	~CG();

	CG(VectorXd x0, double eps, MatrixXd H, VectorXd L,double b,
		double c,double geps);

	//共轭梯度法迭代
	VectorXd Itera();

	ErrorModel *CGMCPCErrorModel;

private:
	//目标函数
	double Func(VectorXd x);

	//目标函数的梯度
	VectorXd Grad(VectorXd x);

	//黄金分割法
	double GoldenSection(VectorXd x, VectorXd d);

private:
	VectorXd m_x0;//归一化后的目标函数的设计变量，取值一般在（-1，1）
	double m_eps = 0.001;//迭代终止条件
	int m_kmax = 500;//迭代次数

	//黄金分割法参数
	double m_b, m_c;//区间两端值m_b<m_c
	double m_geps;//迭代终止条件

	//对变量进行归一化,归一化矩阵
	MatrixXd m_H = MatrixXd::Identity(1,1);
	VectorXd m_L = VectorXd::Zero(1);

	MatrixXd m_J;
	VectorXd m_D;
};

