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


//���Ż��������£������ڼ�����ʧ�������ݶȣ��ſ˱Ⱦ�����˶�ѧ������Ҫ���£�
double LM::Itera(VectorXd &paramsError, int& k)
{
	//��ʼ������
	VectorXd x = m_x0;
	double fx0, fx1;
	//��¼��������
	int k0 = 0;
	//�趨v��miu�ĵ���ϵ��
	int v = 2;
	//�����˶�ѧ����
	LMMCPCErrorModel->UpdataParasVec(m_H*x - m_L);
	MatrixXd J = LMMCPCErrorModel->Calculjacobi();//�ſ˱Ⱦ������û����
	//cout << m_J << endl;
	VectorXd D = LMMCPCErrorModel->CalculErrors();
	Eigen::MatrixXd AMat= (J*m_H).transpose()*(J*m_H);
	fx0 = Func(x, J, D);
	//�����ݶ�����g
	Eigen::VectorXd g=Grad(x,J,D);
	//cout << g.transpose() << endl;
	//���miu�ĳ�ֵ
	double miu = m_tao * GetMaxDiagValue(AMat);
	bool found = g.norm() <= m_eps1;//�������ģ�Ӧ���������
	while (!found && k0 < m_kmax)
	{
		//��������+1
		k0 += 1;
		//����Hessian����
		Eigen::MatrixXd unitMat = Eigen::MatrixXd::Identity(AMat.rows(), AMat.cols());
		Eigen::MatrixXd Hessian = AMat + miu * unitMat;//��֤hessian���������
		//����������	
		////���������
		VectorXd regulaDer = SelectRegulaDer(m_Regular,x);
		Eigen::VectorXd d = (Hessian.inverse())*(-g + regulaDer);
		//�жϲ����Ƿ��С���ﵽ����Ҫ�󣩣�������ֹͣ����
		double dNorm = d.norm();
		double xNorm = x.norm();
		double preE2 = m_eps2 * (xNorm + m_eps2);
		if (dNorm <= preE2)//��x�仯��С��d��ģ��С�����˳�����
		{
			found = true;
		}
		//�����������X
		else
		{
			double gainRate = CalcuReduRate(x,d,J,D);
			if (gainRate > 0)
			{
				Eigen::VectorXd tmpNewX = x + d;
				//�����½���
				//����x����Сmiu����������뾶
				x = tmpNewX;
				///////////�����ſ˱Ⱦ���
				//�����˶�ѧ����
				LMMCPCErrorModel->UpdataParasVec(m_H*x - m_L);
				//�����ſ˱Ⱦ���
				J = LMMCPCErrorModel->Calculjacobi();
				AMat = (J*m_H).transpose()*(J*m_H);
				g = Grad(x,	J,D);		
				//����µĺ���ֵF(xnew)
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
				//������x������miu��С������뾶
				x = x;
				miu = miu * v;
				v *= 2;
			}
		}
	}

	if (!found&&k0 >= m_kmax)
	{
		//Ѱ��ʧ�ܣ�����������������
		paramsError = x;
		k = k0;
		return 0;
	}
	else
	{
		//�ɹ�Ѱ��
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
			return 0;//Ѱ�Ų��ɹ�
		}
		m_x0 = paramsError;//��һ�����Ľ����Ϊ��һ�����ĳ�ֵ
		cout << paramsError.transpose() << endl;
		paramsErrors.push_back(paramsError);
		ks.push_back(k);
		cout << k << endl;
	}
	return 1;//Ѱ�ųɹ�
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
	//�����˶�ѧ����
	LMMCPCErrorModel->UpdataParasVec(m_H*(x + d) - m_L);
	//�����ſ˱Ⱦ���
	MatrixXd newJ = LMMCPCErrorModel->Calculjacobi();
	double f1 = Func(x + d, newJ,D);
	//�ָ��˶�ѧ����
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
		cout << "������ȷ������" << endl;
	}
}
