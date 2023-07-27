#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;

using namespace std;
using namespace Eigen;

class KineMod
{
public:
	//�Ƶ�����ת�ľ���
	Eigen::Matrix4d RotX(double theta);

	Eigen::Matrix4d RotY(double theta);

	Eigen::Matrix4d RotZ(double theta);

	//����ƽ�ƾ���
	Eigen::Matrix4d Trans(double x, double y, double z);

	 //����ӻ�������ʼ����ϵ��ĩ������ϵ��ת������
	virtual vector <Eigen::Matrix4d> CalculRTBegin2End() = 0;
};

