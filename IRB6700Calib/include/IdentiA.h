#pragma once
#include <Eigen/Eigen>
#include <vector>
using namespace std;
using namespace Eigen;
class IdentiA
{
public:
	//�ڼ����������Ĺ�ʽ�����������ĵ�����ʽ
	//L1�������
	VectorXd CalculL1(VectorXd X,double lambda,double v);

	//L2�������
	VectorXd CalculL2(VectorXd X, double lambda);

	//elasticNet������
	VectorXd CalculElasticNet(VectorXd X, double lambda1, double v,double lambda2);

	//log�������
	VectorXd CalculLog(VectorXd X, double lambda,double tao);

	//swish�������
	VectorXd CalculSwish(VectorXd X, double lambda, double beta);
};

