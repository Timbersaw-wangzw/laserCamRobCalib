#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "MCPCErrorModel.h"
using namespace std;
using namespace Eigen;

//�����ݶȷ�
class CG 
{
public:
	CG();
	~CG();

	CG(VectorXd x0, double eps, MatrixXd H, VectorXd L,double b,
		double c,double geps);

	//�����ݶȷ�����
	VectorXd Itera();

	ErrorModel *CGMCPCErrorModel;

private:
	//Ŀ�꺯��
	double Func(VectorXd x);

	//Ŀ�꺯�����ݶ�
	VectorXd Grad(VectorXd x);

	//�ƽ�ָ
	double GoldenSection(VectorXd x, VectorXd d);

private:
	VectorXd m_x0;//��һ�����Ŀ�꺯������Ʊ�����ȡֵһ���ڣ�-1��1��
	double m_eps = 0.001;//������ֹ����
	int m_kmax = 500;//��������

	//�ƽ�ָ����
	double m_b, m_c;//��������ֵm_b<m_c
	double m_geps;//������ֹ����

	//�Ա������й�һ��,��һ������
	MatrixXd m_H = MatrixXd::Identity(1,1);
	VectorXd m_L = VectorXd::Zero(1);

	MatrixXd m_J;
	VectorXd m_D;
};

