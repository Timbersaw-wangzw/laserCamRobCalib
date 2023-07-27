#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "KineMod.h"
using namespace std;
using namespace Eigen;
class DHKineMod:public KineMod
{
public:
	DHKineMod();
	~DHKineMod();

	DHKineMod(vector<vector<double>> DHParas, vector<vector<double>> jointAngles);
	//�ӻ����˻�����ϵ��tool0����ϵ��ת������MCPC�˶�ѧģ�ͣ�
	vector <Eigen::Matrix4d> CalculRTBegin2End();

	vector<vector<Eigen::Matrix3d>> CalculRotBegin2I();

	//����ӻ����˸����ؽ�����ϵ��ĩ������ϵ��ƽ�ƾ���
	vector<vector<Eigen::Vector3d>> CalculTransI2End();

private:
	//���ؽ�Ϊת�������������˵�ת������MCPC
	Eigen::Matrix4d R_RT(double alpha, double beta, double theta, double a, double d);

	vector<vector<double>> GetMCPCParas();

	vector<vector<double>> GetjointAngles();

	vector<vector<double>> m_MCPCParas;
	vector<vector<double>> m_jointAngles;
};

