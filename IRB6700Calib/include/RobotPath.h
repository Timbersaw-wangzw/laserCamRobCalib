#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

//�ڻ����˵�ÿ���ؽڽǹ�����Χ�ڣ��������6���ؽڽ�Ϊһ�������
vector<vector<double>> GeneJointAngles();

double Angle2Rad(double angle);

int GeneRand(int min,int max);
