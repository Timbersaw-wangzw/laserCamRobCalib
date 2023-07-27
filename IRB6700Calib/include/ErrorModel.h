#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "DataConver.h"
#include "GlobalData.h"
using namespace std;
using namespace Eigen;
class ErrorModel
{
public:
	ErrorModel();
	~ErrorModel();

	//���������ĩ�˲������λ�����
	//T�Ѳ�������ת������������ͬһ���ο�����ϵ
	VectorXd CalculErrors();

	//����΢���˶�ʸ��ת����ϵ����6x6����
	MatrixXd CalculPi(Matrix3d r, Vector3d t);

	//����ά����ת��Ϊ���Գƾ���
	Matrix3d Convert2AntiMatrix(Vector3d t);

	//�����ſ˱Ⱦ���
	virtual MatrixXd Calculjacobi()=0;

	//����ӻ�������ʼ����ϵ�������ؽ�����ϵ����ת����
	virtual vector<vector<Matrix3d>> CalculRotBegin2I() = 0;

	//����ӻ����˸����ؽ�����ϵ��ĩ������ϵ��ƽ�ƾ���
	virtual vector<vector<Vector3d>> CalculTransI2End() = 0;

	//�����������ϵ������
	virtual vector<MatrixXd> CalculM()=0;

	//����ֵ��ӣ������˶�ѧ����,addedParasVecΪ�˶�ѧ��������ֵ
	virtual void UpdataParasVec(VectorXd addedParasVec) = 0;

	vector<VectorXd> m_nominalPE;
    vector<VectorXd> m_measurePE; 
	Matrix4d m_T;
};

