#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "KineMod.h"
#include "GlobalData.h"

using namespace std;
using namespace Eigen;

//��XYZŷ����ת��Ϊ��ת����
vector<Matrix3d> Euler2Matrix3dVec(vector<Vector3d> eulerVec);

Matrix3d Euler2Matrix3d(Vector3d euler);

//��ŷ���Ǻ�λ�ò���ת��Ϊλ�˾���
//�βΣ�λ����ǰ��ŷ�����ں�
vector<Matrix4d> Conv2RTVec(vector<VectorXd> eulerPositionVec);

//�βΣ�λ��position��ǰ��ŷ����euler�ں�
Matrix4d Conv2RT(VectorXd PE);

//��λ�˾���ת��ΪXYZŷ���Ǻ�λ�ò�����������λ����ǰ��ŷ�����ں�
vector<VectorXd> Conv2PEVec(vector<Matrix4d> rtVec);

//��λ�˾���ת��ΪXYZŷ���Ǻ�λ�ò�����������λ����ǰ��ŷ�����ں�
VectorXd Conv2PE(Matrix4d rt);

//�Ѷ�̬����ת��Ϊ_4MCPCParas��_6MCPCParas
_4MCPCParas Conv2_4MCPCParas(VectorXd vec);

_6MCPCParas Conv2_6MCPCParas(VectorXd vec);

//�������е������е�����ת��Ϊ��̬����
template <typename T> Matrix<T, Dynamic, 1> Conv2VecX(vector<vector<T>> vec2);

//�Ѷ�̬����ת��Ϊ�����е������е�����
//indexArray�а���ÿ��С�����е����ݸ��������ݸ����������������е�����
template <typename T> vector<vector<T>> Conv2Vec2(Matrix<T, Dynamic, 1> vecX, int *indexArray);

//����λ����ģ,ÿһ����8�����ݣ�ǰ4����λ������ģ����4����ŷ��������ģPE1-PE2
vector<VectorXd> CalculPEErrors(vector<VectorXd> PE1, vector<VectorXd> PE2);

//��ȡ������
double GetMaxValue(vector<double> valueVec);

//��ȡƽ��ֵ
double GetMeanValue(vector<double> valueVec);

//��ȡ������
double GetRMSValue(vector<double> valueVec);

//��ȡ��׼��
double GetSDValue(vector<double> valueVec);

//�Ƶ�����ת�ľ���
Eigen::Matrix4d RotX(double theta);

Eigen::Matrix4d RotY(double theta);

Eigen::Matrix4d RotZ(double theta);

//����ƽ�ƾ���
Eigen::Matrix4d Trans(double x, double y, double z);


template<typename T>
inline Matrix<T, Dynamic, 1> Conv2VecX(vector<vector<T>> vec2)
{

	//����������
	int sizeVec2 = 0;
	for (int i = 0; i < vec2.size(); ++i)
	{
		sizeVec2 = sizeVec2 + vec2[i].size();
	}
	Matrix<T, Dynamic, 1> nominalMCPCParas = Matrix<T, Dynamic, 1>::Zero(sizeVec2, 1);
	//Matrix<double, Dynamic, 1> nominalMCPCParas = Matrix<double, Dynamic, 1>::Zero(sizeVec2, 1);
	int indexPara = 0;
	for (int i = 0; i < vec2.size(); ++i)
	{
		for (int j = 0; j < vec2[i].size(); ++j)
		{
			nominalMCPCParas[indexPara] = vec2[i][j];
			indexPara += 1;
		}
	}
	return nominalMCPCParas;
}


template<typename T>
inline vector<vector<T>> Conv2Vec2(Matrix<T, Dynamic, 1> vecX, int *indexArray)
{
	int index = 0;
	vector <vector<T>>  compensatedMCPCParaVec;
	vector<T> compensatedMCPCParas;
	for (int i = 0; i < vecX.size(); ++i)
	{
		compensatedMCPCParas.push_back(vecX[i]);
		if (compensatedMCPCParas.size() >= indexArray[index])
		{
			index += 1;
			compensatedMCPCParaVec.push_back(compensatedMCPCParas);
			compensatedMCPCParas.clear();
		}
	}
	return compensatedMCPCParaVec;
}

