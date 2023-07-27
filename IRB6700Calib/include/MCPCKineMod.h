#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "KineMod.h"
using namespace std;
using namespace Eigen;
//6�������MCPC�˶�ѧģ����
class MCPCKineMod:public KineMod
{

public:
	MCPCKineMod();
	~MCPCKineMod();

	MCPCKineMod(vector<vector<double>> MCPCParas, vector<vector<double>> jointAngles);
	//�ӻ����˻�����ϵ��tool0����ϵ��ת������MCPC�˶�ѧģ�ͣ�
	vector <Eigen::Matrix4d> CalculRTBegin2End();

	//���ؽ�Ϊת�������������˵�ת������MCPC
	Eigen::Matrix4d R_RT(double alpha, double beta, double theta,double x, double y);

	//���ؽ�Ϊ�ƶ������������˵�ת������MCPC
	Eigen::Matrix4d T_RT(double alpha, double beta, double d, double x, double y);

	//������ϵ���һ�����任(��Զ�����ϵ)
	Eigen::Matrix4d RT(double alpha, double beta,double gama, double theta, double x, double y,double z);

private:
	vector<vector<double>> GetMCPCParas();

	vector<vector<double>> GetjointAngles();

	vector<vector<double>> m_MCPCParas;
	vector<vector<double>> m_jointAngles;
};



