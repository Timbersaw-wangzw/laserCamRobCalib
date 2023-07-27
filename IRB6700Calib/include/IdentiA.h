#pragma once
#include <Eigen/Eigen>
#include <vector>
using namespace std;
using namespace Eigen;
class IdentiA
{
public:
	//在计算迭代方向的公式中添加正则项的导数公式
	//L1正则项导数
	VectorXd CalculL1(VectorXd X,double lambda,double v);

	//L2正则项导数
	VectorXd CalculL2(VectorXd X, double lambda);

	//elasticNet正则项
	VectorXd CalculElasticNet(VectorXd X, double lambda1, double v,double lambda2);

	//log正则项导数
	VectorXd CalculLog(VectorXd X, double lambda,double tao);

	//swish正则项导数
	VectorXd CalculSwish(VectorXd X, double lambda, double beta);
};

