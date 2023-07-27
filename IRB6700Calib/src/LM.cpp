#include "LM.h"

LM::LM()
{
}

LM::~LM()
{
}

LM::LM(VectorXd x0, double eps1, double eps2, int kmax, double tao, MatrixXd H, VectorXd L):
	m_x0(x0), m_eps1(eps1), m_eps2(eps2), m_kmax(kmax),m_tao(tao), m_H(H), m_L(L)
{
}


//若优化参数更新，则用于计算损失函数，梯度，雅克比矩阵的运动学参数必要更新；
double LM::Itera(VectorXd &paramsError, int& k)
{
	//初始化参数
	VectorXd x = m_x0;
	double fx0, fx1;
	//记录迭代次数
	int k0 = 0;
	//设定v，miu的调整系数
	int v = 2;
	//更新运动学参数
	LMMCPCErrorModel->UpdataParasVec(m_H*x - m_L);
	MatrixXd J = LMMCPCErrorModel->Calculjacobi();//雅克比矩阵计算没问题
	//cout << m_J << endl;
	VectorXd D = LMMCPCErrorModel->CalculErrors();
	Eigen::MatrixXd AMat= (J*m_H).transpose()*(J*m_H);
	fx0 = Func(x, J, D);
	//构建梯度向量g
	Eigen::VectorXd g=Grad(x,J,D);
	//cout << g.transpose() << endl;
	//求解miu的初值
	double miu = m_tao * GetMaxDiagValue(AMat);
	bool found = g.norm() <= m_eps1;//根据论文，应该用无穷范数
	while (!found && k0 < m_kmax)
	{
		//迭代次数+1
		k0 += 1;
		//创建Hessian矩阵
		Eigen::MatrixXd unitMat = Eigen::MatrixXd::Identity(AMat.rows(), AMat.cols());
		Eigen::MatrixXd Hessian = AMat + miu * unitMat;//保证hessian矩阵非奇异
		//求解迭代方向	
		////添加正则项
		VectorXd regulaDer = SelectRegulaDer(m_Regular,x);
		Eigen::VectorXd d = (Hessian.inverse())*(-g + regulaDer);
		//判断步长是否过小（达到精度要求），若是则停止迭代
		double dNorm = d.norm();
		double xNorm = x.norm();
		double preE2 = m_eps2 * (xNorm + m_eps2);
		if (dNorm <= preE2)//若x变化过小（d的模过小）则退出迭代
		{
			found = true;
		}
		//否则继续更新X
		else
		{
			double gainRate = CalcuReduRate(x,d,J,D);
			if (gainRate > 0)
			{
				Eigen::VectorXd tmpNewX = x + d;
				//计算下降率
				//更新x，减小miu增大信赖域半径
				x = tmpNewX;
				///////////更新雅克比矩阵
				//更新运动学参数
				LMMCPCErrorModel->UpdataParasVec(m_H*x - m_L);
				//计算雅克比矩阵
				J = LMMCPCErrorModel->Calculjacobi();
				AMat = (J*m_H).transpose()*(J*m_H);
				g = Grad(x,	J,D);		
				//求解新的函数值F(xnew)
				fx1 = Func(x, J,D);
				if (fabs(fx1 - fx0) < m_eps1)
				{
					found = true;
				}
				double tmpVal = 1 - (2 * gainRate - 1)*(2 * gainRate - 1)*(2 * gainRate - 1);
				miu *= max(1.0 / 3.0, tmpVal);
				v = 2;
				fx0 = fx1;
			}
			else
			{
				//不更新x，增大miu减小信赖域半径
				x = x;
				miu = miu * v;
				v *= 2;
			}
		}
	}

	if (!found&&k0 >= m_kmax)
	{
		//寻优失败，迭代次数超过上限
		paramsError = x;
		k = k0;
		return 0;
	}
	else
	{
		//成功寻优
		paramsError = x;
		k = k0;
		return 1;
	}
}


double LM::VariousRegulaItera(vector<VectorXd>& paramsErrors, vector<int>& ks)
{
	VectorXd paramsError;
	int k=0;
	vector<RegularizationName> RegularNames = { Swish,L2,L1,ElasticNet,Log };
	for (auto v= RegularNames.begin();v!= RegularNames.end();v++)
	{
		m_Regular = *v;		
		if (Itera(paramsError, k)<1)
		{
			return 0;//寻优不成功
		}
		m_x0 = paramsError;//上一方法的结果作为下一方法的初值
		cout << paramsError.transpose() << endl;
		paramsErrors.push_back(paramsError);
		ks.push_back(k);
		cout << k << endl;
	}
	return 1;//寻优成功
}


double LM::GetMaxDiagValue(const Eigen::MatrixXd & squareMat)
{
	int rows = squareMat.rows();
	if (rows != squareMat.cols())
		return -1000.0;
	double maxDiag = squareMat(0, 0);
	for (int i = 0; i < rows; i++)
	{
		if (squareMat(i, i) > maxDiag)
			maxDiag = squareMat(i, i);
	}
	return maxDiag;
}


double LM::Func(VectorXd x, MatrixXd J,VectorXd D)
{	
	VectorXd f = J * (m_H*x - m_L) - D;
	double F = 0.5*f.transpose()*f;
	return F;
}


VectorXd LM::Grad(VectorXd x, MatrixXd J, VectorXd D)
{
	VectorXd f = J * (m_H*x - m_L) - D;
	VectorXd g = (J*m_H).transpose()*f;
	return g;
}


double LM::CalcuReduRate(VectorXd x, VectorXd d, MatrixXd J, VectorXd D)
{
	double f0 = Func(x,J,D);
	VectorXd d0 = VectorXd::Zero(d.size());
	//更新运动学参数
	LMMCPCErrorModel->UpdataParasVec(m_H*(x + d) - m_L);
	//计算雅克比矩阵
	MatrixXd newJ = LMMCPCErrorModel->Calculjacobi();
	double f1 = Func(x + d, newJ,D);
	//恢复运动学参数
	LMMCPCErrorModel->UpdataParasVec(m_H*(-x - d) - m_L);
	double m0 = CalcuM(x, d0, J,D);
	double m1 = CalcuM(x, d, J,D);
	double rate = (f0-f1) / (m0-m1);
	return rate;
}


double LM::CalcuM(VectorXd x, VectorXd d, MatrixXd J, VectorXd D)
{
	double m;
	VectorXd f = J * (m_H*x - m_L) - D;
	m = 0.5*Func(x,J,D) + d.transpose()*J.transpose()*f + 0.5*d.transpose()*J.transpose()*J*d;
	return m;
}


VectorXd LM::SelectRegulaDer(RegularizationName regulaName, VectorXd X)
{
	VectorXd x_ = X;
	double lambda;
	double v;
	double lambda1;
	double lambda2;
	double tao;
	double beta;
	switch (regulaName)
	{
	case non:
		return VectorXd::Zero(x_.size());
		break;
	case L1:		
		lambda=-32000;
		v=0.001;
		return CalculL1(x_, lambda, v);
		break;
	case L2:
		lambda =-50000 ;
		return CalculL2(x_, lambda);
		break;
	case ElasticNet:
		lambda1 = -10000;
		v = 0.001;
		lambda2 = -70000;
		return CalculElasticNet(x_, lambda1, v, lambda2);
		break;
	case Log:
		lambda = -0.02;
		tao = 0.001;
		return CalculLog(x_, lambda, tao);
		break;
	case Swish:
		lambda = -210000;
		beta = 1;
		return CalculSwish(x_, lambda, beta);
		break;
	default:
		cout << "输入正确的名称" << endl;
	}
}
