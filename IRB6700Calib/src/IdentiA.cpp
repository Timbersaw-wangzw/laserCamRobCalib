#include "IdentiA.h"

VectorXd IdentiA::CalculL1(VectorXd X, double lambda, double v)
{
	VectorXd L1 = X;
	for (int i = 0; i < X.size(); i++)
	{
		L1[i] = lambda * (X[i] / sqrt(pow(X[i], 2) + v));
	}
	return L1;
}


VectorXd IdentiA::CalculL2(VectorXd X, double lambda)
{
	VectorXd L2 = lambda * X;
	return L2;
}


VectorXd IdentiA::CalculElasticNet(VectorXd X, double lambda1, double v, double lambda2)
{
	VectorXd L1 =X;
	for (int i = 0; i < X.size(); i++)
	{
		L1[i] = lambda1 * (X[i] / sqrt(pow(X[i], 2) + v));
	}
	VectorXd L2 = lambda2 * X;
    VectorXd elasNet = L1+L2;
	return elasNet;
}


VectorXd IdentiA::CalculLog(VectorXd X, double lambda, double tao)
{
	VectorXd log = X;
	for (int i = 0; i < X.size(); i++)
	{
		log[i] = lambda / (X[i] + tao);
	}
	return log;
}


VectorXd IdentiA::CalculSwish(VectorXd X, double lambda, double beta)
{
	VectorXd swish = X;//³õÊ¼»¯
	for (int i = 0; i < X.size(); i++)
	{
		swish[i] = lambda * (X[i] + X[i] * exp(-beta * X[i]) + beta * pow(X[i], 2)*exp(-beta * X[i])) / pow(1 + exp(-beta * X[i]), 3);
	}
	return swish;
}
