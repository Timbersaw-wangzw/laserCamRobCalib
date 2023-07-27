#include "CG.h"

CG::CG()
{
}


CG::~CG()
{
}


CG::CG(VectorXd x0, double eps, MatrixXd H, VectorXd L, double b,
	double c, double geps):
	m_x0(x0),m_eps(eps),m_H(H),m_L(L),m_b(b),m_c(c),m_geps(geps)
{
	if (eps<=0|| geps<=0||b>=c)
	{
		cout << "参数错误" << endl;
	    //抛出异常TODO
	}
}


double CG::Func(VectorXd x)
{

	VectorXd f = m_J *(m_H*x -m_L)- m_D;
	double F = 0.5*f.transpose()*f;
	return F;
}


VectorXd CG::Grad(VectorXd x)
{
	VectorXd f = m_J * (m_H*x - m_L) - m_D;
	VectorXd g = (m_J*m_H).transpose()*f;
	return g;
}


VectorXd CG::Itera()
{
	MatrixXd m_J = CGMCPCErrorModel->Calculjacobi();//雅克比矩阵计算没问题
	VectorXd m_D = CGMCPCErrorModel->CalculErrors();
	VectorXd x = m_x0;//初始化变量
	VectorXd g0 = Grad(x);
	VectorXd g1;
	VectorXd d = -g0;
	int k = 0;
	while(g0.norm()>= m_eps&& k<m_kmax)
	{
		//进行精确一维搜索，找到最佳步长a，更新m_x
		//cout << "fvalue" << endl << Func(x) << endl;
		double a = GoldenSection(x,d);
		x = x + a*d;
		//cout << "m_x:"<<endl << m_x0.transpose() << endl;
		//cout << "fvalue" << endl << Func(x) << endl;
		//计算新的搜索方向
		g1 = Grad(x);
		k = k + 1;
		//cout << "m_g1:"<<endl << m_g1.transpose() << endl;
		double lambda = (g1.transpose()*g0).norm() / (g1.transpose()*g1).norm();
		if (lambda >0.1)
		{
			g0 = g1;
			d = -g0;
			continue;
		}
		double beta = (g1.transpose()*g1).norm() / (g0.transpose()*g0).norm();
		d = -g1 + beta * d;
		//cout << "m_d:" << endl << m_d.transpose() << endl;
		g0 = g1;
	}
	return x;
}


double CG::GoldenSection(VectorXd x, VectorXd d)
{
	//步1
	int k = 1;
	//把寻找最优的目标变量转化为寻找使函数值最小的步长
	double l = m_b + 0.382*(m_c - m_b);//计算步长区间的左值
	double r = m_b + 0.618*(m_c - m_b);//计算步长区间的右值
	while (1)
	{
		//步2
		VectorXd xl = x + l * d;//更新目标变量
		VectorXd xr = x + r * d;
		double fl = Func(xl);
		double fr = Func(xr);
		//步3
		if (fl > fr)
		{			
			if (m_c-l< m_geps)
			{
				return r;
			}
			m_b = l;
			m_c = m_c;
			l = r;
			r = m_b + 0.618*(m_c-m_b);
		}
		else
		{
			if (r-m_b< m_geps)
			{
				return l;
			}
			m_b = m_b;
			m_c = r;
			r = l;
			l = m_b + 0.382*(m_c-m_b);
		}
		k = k + 1;
	}
}


