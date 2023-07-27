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

	//��������
	double Itera(VectorXd &paramsError, int& k);

	//�Ѹ������������ģ���У��ֱ���е�������
	double VariousRegulaItera(vector<VectorXd> &paramsErrors, vector<int>& ks);

	ErrorModel *LMMCPCErrorModel;

	RegularizationName m_Regular = non;//ѡ������������
private:
	//��ȡ����Խ����ϵ����ֵ
	double GetMaxDiagValue(const Eigen::MatrixXd& squareMat);

    //Ŀ�꺯��
    double Func(VectorXd x, MatrixXd J, VectorXd D);

	//Ŀ�꺯�����ݶ�
	VectorXd Grad(VectorXd x, MatrixXd J, VectorXd D);

	//�����½���
	double CalcuReduRate(VectorXd x, VectorXd d, MatrixXd J, VectorXd D);

	double CalcuM(VectorXd x, VectorXd d, MatrixXd J, VectorXd D);

	//ѡ�������������
	VectorXd SelectRegulaDer(RegularizationName regulaName, VectorXd X);

private:
	VectorXd m_x0;//��һ�����Ŀ�꺯������Ʊ�����ֵ��ȡֵһ���ڣ�-1��1��
	double m_eps1 = 0.001;//�ݶȾ���
	double m_eps2 = 0.001;//�����������Ⱦ���
	int m_kmax = 500;//��������
	double m_tao=1;//�����������ӵı�������
	
	//�Ա������й�һ��,��һ������
	MatrixXd m_H = MatrixXd::Identity(1, 1);
	VectorXd m_L = VectorXd::Zero(1);
};

