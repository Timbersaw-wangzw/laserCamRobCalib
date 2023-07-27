#include "MCPCErrorModel.h"
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
using namespace std;
using namespace Eigen;

 MCPCErrorModel::MCPCErrorModel()
{
}


MCPCErrorModel::~MCPCErrorModel()
{
}


MCPCErrorModel::MCPCErrorModel(vector<vector<double>> kineParas, vector<vector<double>> jointAngles,
	vector<VectorXd> nominalPEVec, vector<VectorXd> measurePEVec,Matrix4d T)
	:m_MCPCParasVec2(kineParas),m_jointAngles(jointAngles)
{	
	int i = 0;
	while(i<7)
	{
		_6MCPCParas temp6P;
		temp6P.alpha = kineParas[i][0];
		temp6P.beta = kineParas[i][1];
		temp6P.gamma = kineParas[i][2];
		temp6P.x = kineParas[i][3];
		temp6P.y = kineParas[i][4];
		temp6P.z = kineParas[i][5];
		m_6MCPCParasVec.push_back(temp6P);
		i = i + 6;
	}
	for (int j=1;j< kineParas.size()-1;++j)
	{
		_4MCPCParas temp4P;
		temp4P.alpha = kineParas[j][0];
		temp4P.beta = kineParas[j][1];
		temp4P.x = kineParas[j][2];
		temp4P.y = kineParas[j][3];
		m_4MCPCParasVec.push_back(temp4P);
	}	
	m_nominalPE = nominalPEVec;
	m_measurePE = measurePEVec;
	m_T=T;
}


MatrixXd MCPCErrorModel::Calculjacobi()
{
	vector<vector<Matrix3d>> rotR2IVec = CalculRotBegin2I();//坐标系i相对导轨的旋转矩阵
	vector<vector<Vector3d>> transI2T0Vec = CalculTransI2End();//tool0相对坐标系i位置矩阵
	vector<MatrixXd> M = CalculM();//从导轨r坐标系到tool0坐标系的运动学误差系数矩阵
	MatrixXd jacobis= MatrixXd::Zero(3* rotR2IVec.size(), 32);
	for (int j=0;j< rotR2IVec.size();++j)
	{
		MatrixXd jacobi = MatrixXd::Zero(6, 32);
		int indexJacobiCol = 0;
		for (int i = 0; i < rotR2IVec[0].size(); ++i)
		{
			MatrixXd Ji;
			MatrixXd Pi = CalculPi(rotR2IVec[j][i], transI2T0Vec[j][i]);//经验证，Pi计算正确
			Ji = Pi * M[i];
			jacobi.block(0, indexJacobiCol, Ji.rows(), Ji.cols())= Ji;
			indexJacobiCol = indexJacobiCol + Ji.cols();
		}
		jacobis.block(3*j,0,3,32)= jacobi.block(0,0,3,32);
	}
	return jacobis;
}


vector<MatrixXd> MCPCErrorModel::CalculM()
{
	vector<MatrixXd> M;
	vector < _6MCPCParas> p1;//坐标系1和t0处的运动学参数
	p1 = Get6ParasVec();
	vector<_4MCPCParas> p2;//坐标系2到6处的运动学参数
	p2 = Get4ParasVec();
	vector<MatrixXd> M1t0;//坐标系1和t0处的参数误差系数矩阵
	for (int i=0;i< p1.size();++i)
	{
		MatrixXd tempMi= MatrixXd::Identity(6,6);
		//检查无误，与论文公式一致
		tempMi << -p1[i].y*sin(p1[i].beta) - p1[i].z*cos(p1[i].beta)*sin(p1[i].gamma), p1[i].z*cos(p1[i].gamma), -p1[i].y, 1, 0, 0,
			-p1[i].z*cos(p1[i].beta)*cos(p1[i].gamma) + p1[i].x*sin(p1[i].beta), -p1[i].z*sin(p1[i].gamma), p1[i].x, 0, 1, 0,
			p1[i].y*cos(p1[i].beta)*cos(p1[i].gamma) + p1[i].x*cos(p1[i].beta)*sin(p1[i].gamma), -p1[i].x*cos(p1[i].gamma) + p1[i].y*sin(p1[i].gamma), 0, 0, 0, 1,
			cos(p1[i].beta)*cos(p1[i].gamma), sin(p1[i].gamma), 0, 0, 0, 0,
			-cos(p1[i].beta)*sin(p1[i].gamma), cos(p1[i].gamma), 0, 0, 0, 0,
			sin(p1[i].beta), 0, 1, 0, 0, 0;
		M1t0.push_back(tempMi);
	}
	M.push_back(M1t0[0]);

	for (int j=0;j< p2.size();++j)
	{
		MatrixXd tempMi=MatrixXd::Zero(6,4);
		//检查无误，与论文公式一致
		tempMi << -p2[j].y*sin(p2[j].beta), 0, 1, 0,
			p2[j].x*sin(p2[j].beta), 0, 0, 1,
			p2[j].y*cos(p2[j].beta), -p2[j].x, 0, 0,
			cos(p2[j].beta), 0, 0, 0,
			0, 1, 0, 0,
			sin(p2[j].beta), 0, 0, 0;
		M.push_back(tempMi);
	}
	M.push_back(M1t0[1]);
	return M;
}


vector<vector<Matrix3d>> MCPCErrorModel::CalculRotBegin2I()
{
	//获取MCPC参数、关节角参数
	vector < _6MCPCParas> p1;//坐标系1和t0处的运动学参数
	p1 = Get6ParasVec();
	vector<_4MCPCParas> p2;//坐标系2到6处的运动学参数
	p2 = Get4ParasVec();
	vector<vector<double>> jointAngles = GetjointAngles();//d,θ1,θ2,θ3,θ4,θ5,θ6,
	////从机器人起始坐标系到各个关节坐标系的旋转矩阵
	vector<vector <Eigen::Matrix3d>>rotBegin2Is;
	rotBegin2Is.clear();
	for (int j = 0; j < jointAngles.size(); ++j)
	{
		vector<Eigen::Matrix3d> rotBegin2I;
		rotBegin2I.clear();
		Eigen::Matrix4d tempRT = Matrix4d::Identity();
		//r->1轴坐标系变换，沿x轴移动
		tempRT = RT(p1[0].alpha, p1[0].beta, p1[0].gamma, 0, p1[0].x + jointAngles[j][0],p1[0].y, p1[0].z);
		rotBegin2I.push_back(tempRT.block(0, 0, 3, 3));
		//1->6->tool0坐标系转换，绕z轴转动
		for (int i = 0; i < p2.size(); ++i)
		{
			tempRT = tempRT * R_RT(p2[i].alpha, p2[i].beta, jointAngles[j][i+1], p2[i].x,p2[i].y);
			rotBegin2I.push_back(tempRT.block(0, 0, 3, 3));
		}
		tempRT = tempRT * RT(p1[1].alpha, p1[1].beta, p1[1].gamma,jointAngles[j][6], p1[1].x, p1[1].y, p1[1].z);
		rotBegin2I.push_back(tempRT.block(0, 0, 3, 3));
		rotBegin2Is.push_back(rotBegin2I);
	}
	return rotBegin2Is;
}


vector<vector<Vector3d>> MCPCErrorModel::CalculTransI2End()
{
	//获取MCPC参数、关节角参数
	vector < _6MCPCParas> p1;//坐标系1和t0处的运动学参数
	p1 = Get6ParasVec();
	vector<_4MCPCParas> p2;//坐标系2到6处的运动学参数
	p2 = Get4ParasVec();
	vector<vector<double>> jointAngles = GetjointAngles();//d,θ1,θ2,θ3,θ4,θ5,θ6,
	//从机器人各个关节坐标系i到末端坐标系的平移矩阵
	vector<vector<Vector3d>> transI2Ends;
	for (int j = 0; j < jointAngles.size(); ++j)
	{
		vector<Vector3d> transI2End;
		Matrix4d tempRT = Matrix4d::Zero();
		//tool0
		transI2End.push_back(tempRT.block(0, 3, 3, 1));
		//6
		tempRT = RT(p1[1].alpha, p1[1].beta, p1[1].gamma,jointAngles[j][6], p1[1].x, p1[1].y, p1[1].z);
		transI2End.push_back(tempRT.block(0, 3, 3, 1));
		//5,4,3,2,1
		int sizeJA = jointAngles[0].size();
		for (int i = 1; i < sizeJA - 1; ++i)
		{
			tempRT = R_RT(p2[sizeJA - i - 2].alpha, p2[sizeJA - i - 2].beta, jointAngles[j][sizeJA - i - 1],
				p2[sizeJA - i - 2].x, p2[sizeJA - i - 2].y)*tempRT;
			transI2End.push_back(tempRT.block(0, 3, 3, 1));
		}
		//在transI2End中，数据是按t0,6,5,4,3,2,1排列，为了与其它函数结果相适应，把该数据倒置排列
		vector<Vector3d> tempVec;
		for (int k = 0; k < transI2End.size(); ++k)
		{
			tempVec.push_back(transI2End[transI2End.size() - 1 - k]);
		}
		transI2End = tempVec;
		tempVec.clear();
		transI2Ends.push_back(transI2End);
	}
	return transI2Ends;
}


std::vector<_4MCPCParas> MCPCErrorModel::Get4ParasVec()
{
	return m_4MCPCParasVec;
}


std::vector<_6MCPCParas> MCPCErrorModel::Get6ParasVec()
{
	return m_6MCPCParasVec;
}


void MCPCErrorModel::UpdataParasVec(VectorXd addedParasVec)
{
	//先把VectorXd形式的参数转为_4MCPCParas,_6MCPCParas
	if (addedParasVec.size() != 32|| m_4MCPCParasVec.empty()|| m_6MCPCParasVec.empty())
	{
		exit(EXIT_FAILURE);
	}
	_4MCPCParas addedMCPCPara4;
	_6MCPCParas addedMCPCPara6;
	vector<_4MCPCParas> addedMCPCPara4Vec;
	vector<_6MCPCParas> addedMCPCPara6Vec;
	int indexArray[] = {6,4,4,4,4,4,6};
	int index = 0;
	for (int i=0;i< size(indexArray);++i)
	{		
		if (indexArray[i]==6)
		{
			//把addedParasVec中0~5,26~31索引值的数转化为_6MCPCParas
			addedMCPCPara6 = Conv2_6MCPCParas(addedParasVec.block(index, 0, indexArray[i], 1));
			addedMCPCPara6Vec.push_back(addedMCPCPara6);
		}
		else
		{
			//把6~25索引值的数转化为_4MCPCParas
			addedMCPCPara4 = Conv2_4MCPCParas(addedParasVec.block(index, 0, indexArray[i], 1));
			addedMCPCPara4Vec.push_back(addedMCPCPara4);
		}
		index = index + indexArray[i];
	}
	//对原来的运动学参数加上增值，更新运动学参数
	for (int i=0;i< addedMCPCPara6Vec.size();++i)
	{
		m_6MCPCParasVec[i] = addedMCPCPara6Vec[i] + m_6MCPCParasVec[i];
	}
	for (int i=0;i< addedMCPCPara4Vec.size();++i)
	{
		m_4MCPCParasVec[i] = addedMCPCPara4Vec[i] + m_4MCPCParasVec[i];
	}
}

vector<vector<double>> MCPCErrorModel::GetjointAngles()
{
	return m_jointAngles;
}

vector<vector<double>> MCPCErrorModel::GetMCPCParas()
{
	return m_MCPCParasVec2;
}



