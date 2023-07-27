#pragma once
#include "ErrorModel.h"
#include "GlobalData.h"
#include "MCPCKineMod.h"
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;


class MCPCErrorModel :public ErrorModel, MCPCKineMod
{
public:
	MCPCErrorModel();
	~MCPCErrorModel();

	//��MCPC����ת���ɽṹ�����ʽ
	MCPCErrorModel(vector<vector<double>> kineParas, vector<vector<double>> jointAngles,
		vector<VectorXd> nominalPEVec,vector<VectorXd> measurePEVec,
		Matrix4d T);

	MatrixXd Calculjacobi();

	//����ӵ���r����ϵ��tool0����ϵ���˶�ѧ���ϵ������
	vector<MatrixXd> CalculM();

	//����ӻ�������ʼ����ϵ�������ؽ�����ϵ����ת����
	vector<vector<Matrix3d>> CalculRotBegin2I();

	//����ӻ����˸����ؽ�����ϵ��ĩ������ϵ��ƽ�ƾ���
	vector<vector<Vector3d>> CalculTransI2End();

	//�����˶�ѧ����
	void UpdataParasVec(VectorXd addedParasVec);
private:
	vector<_4MCPCParas> Get4ParasVec();
	vector<_6MCPCParas> Get6ParasVec();
	vector<vector<double>> GetjointAngles();
	vector<vector<double>> GetMCPCParas();

	vector<_4MCPCParas> m_4MCPCParasVec;
	vector<_6MCPCParas> m_6MCPCParasVec;
	vector<vector<double>> m_MCPCParasVec2;

	vector<vector<double>> m_jointAngles;
};

