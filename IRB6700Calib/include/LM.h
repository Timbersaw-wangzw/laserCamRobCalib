#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "MCPCErrorModel.h"
#include "GlobalData.h"
#include "IdentiA.h"
using namespace std;
using namespace Eigen;

class LM:public IdentiA
{
public:
	LM();
	~LM();

	LM(VectorXd x0, double eps1, double eps2, int kmax, double tao, MatrixXd H, VectorXd L);

	//迭代计算
	double Itera(VectorXd &paramsError, int& k);

	//把各项正则项带入模型中，分别进行迭代计算
	double VariousRegulaItera(vector<VectorXd> &paramsErrors, vector<int>& ks);

	ErrorModel *LMMCPCErrorModel;

	RegularizationName m_Regular = non;//选择正则项类型
private:
	//获取矩阵对角线上的最大值
	double GetMaxDiagValue(const Eigen::MatrixXd& squareMat);

    //目标函数
    double Func(VectorXd x, MatrixXd J, VectorXd D);

	//目标函数的梯度
	VectorXd Grad(VectorXd x, MatrixXd J, VectorXd D);

	//计算下降率
	double CalcuReduRate(VectorXd x, VectorXd d, MatrixXd J, VectorXd D);

	double CalcuM(VectorXd x, VectorXd d, MatrixXd J, VectorXd D);

	//选择正则项导数种类
	VectorXd SelectRegulaDer(RegularizationName regulaName, VectorXd X);

private:
	VectorXd m_x0;//归一化后的目标函数的设计变量初值，取值一般在（-1，1）
	double m_eps1 = 0.001;//梯度精度
	double m_eps2 = 0.001;//迭代步长精度精度
	int m_kmax = 500;//迭代次数
	double m_tao=1;//计算阻尼因子的比例因子
	
	//对变量进行归一化,归一化矩阵
	MatrixXd m_H = MatrixXd::Identity(1, 1);
	VectorXd m_L = VectorXd::Zero(1);
};

