#include "DHKineMod.h"

DHKineMod::DHKineMod()
{
}

DHKineMod::~DHKineMod()
{
}


DHKineMod::DHKineMod(vector<vector<double>> DHParas, vector<vector<double>> jointAngles):
	m_MCPCParas(DHParas), m_jointAngles(jointAngles)
{
}


vector<Eigen::Matrix4d> DHKineMod::CalculRTBegin2End()
{
	//获取MCPC参数、关节角参数
	vector<vector<double>> DHParas = GetMCPCParas();
	//参数正常应改为7组
	int DHParasSize = DHParas.size();
	if (DHParasSize != 6)
	{
		cout << "请输入正确数量的DH参数" << endl;
		exit(EXIT_FAILURE);
	}
	vector<vector<double>> jointAngles = GetjointAngles();
	vector <Eigen::Matrix4d>b2t0_RTs;
	for (int j = 0; j < jointAngles.size(); ++j)
	{
		Eigen::Matrix4d b2t0_RT= Eigen::MatrixXd::Identity(4,4);
		//1->6坐标系转换，绕z轴转动
		for (int i = 0; i < 6; i++)
		{
			b2t0_RT = b2t0_RT * R_RT(DHParas[i][0], DHParas[i][3], jointAngles[j][i], DHParas[i][1],
				DHParas[i][2]);
		}
		b2t0_RTs.push_back(b2t0_RT);
		cout << b2t0_RT << endl;
	}

	return b2t0_RTs;
}

vector<vector<Eigen::Matrix3d>> DHKineMod::CalculRotBegin2I()
{
	return vector<vector<Eigen::Matrix3d>>();
}

vector<vector<Eigen::Vector3d>> DHKineMod::CalculTransI2End()
{
	return vector<vector<Eigen::Vector3d>>();
}


Eigen::Matrix4d DHKineMod::R_RT(double alpha, double beta, double theta, double a, double d)
{
	Eigen::Matrix4d RT;
	RT = RotX(alpha)*Trans(a,0,0)*RotZ(beta+ theta)*Trans(0,0,d);
	return RT;
}


vector<vector<double>> DHKineMod::GetMCPCParas()
{
	return m_MCPCParas;
}


vector<vector<double>> DHKineMod::GetjointAngles()
{
	return m_jointAngles;
}
