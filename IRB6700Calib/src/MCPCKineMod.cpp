#include "MCPCKineMod.h"


MCPCKineMod::MCPCKineMod()
{
}


MCPCKineMod::~MCPCKineMod()
{
}


MCPCKineMod::MCPCKineMod(vector<vector<double>> MCPCParas, vector<vector<double>> jointAngles) :
	m_MCPCParas(MCPCParas), m_jointAngles(jointAngles)
{
}


vector <Eigen::Matrix4d> MCPCKineMod::CalculRTBegin2End()
{
	//��ȡMCPC�������ؽڽǲ���
	vector<vector<double>> MCPCParas = GetMCPCParas();
	//��������Ӧ��Ϊ7��
	int MCPCParasSize = MCPCParas.size();
	if (MCPCParasSize !=7)
	{
		cout << "��������ȷ������MCPC����" << endl;
		exit(EXIT_FAILURE);
	}
	vector<vector<double>> jointAngles = GetjointAngles();//d,��1,��2,��3,��4,��5,��6,
	Eigen::Matrix4d b2t0_RT;	
	vector <Eigen::Matrix4d>b2t0_RTs;	
	for (int j=0;j< jointAngles.size();++j)
	{
		//0->1����ϵ�任������z��ת��
		//b2t0_RT = R_RT(MCPCParas[0][0], MCPCParas[0][1], 0, MCPCParas[0][2], MCPCParas[0][3]);
		//r->1������ϵ�任����x���ƶ�
		b2t0_RT = RT(MCPCParas[0][0], MCPCParas[0][1], MCPCParas[0][2],0, MCPCParas[0][3] + jointAngles[j][0], MCPCParas[0][4],
			MCPCParas[0][5]);
		//1->6->tool0����ϵת������z��ת��
		for (int i = 1; i < 6; i++)
		{
			b2t0_RT = b2t0_RT * R_RT(MCPCParas[i][0], MCPCParas[i][1], jointAngles[j][i], MCPCParas[i][2],
				MCPCParas[i][3]);
		}
		b2t0_RT =b2t0_RT *RT(MCPCParas[MCPCParasSize - 1][0], MCPCParas[MCPCParasSize - 1][1], MCPCParas[MCPCParasSize - 1][2],
			jointAngles[j][6],MCPCParas[MCPCParasSize - 1][3], MCPCParas[MCPCParasSize - 1][4], MCPCParas[MCPCParasSize - 1][5]);

		b2t0_RTs.push_back(b2t0_RT);
		//cout << b2t0_RT << endl;
	}

	return b2t0_RTs;
}


Eigen::Matrix4d MCPCKineMod::R_RT(double alpha, double beta, double theta, double x, double y)
{
	Eigen::Matrix4d RT;
	RT = RotZ(theta)*RotX(alpha)*RotY(beta)*Trans(x,y,0);
	return RT;
}


Eigen::Matrix4d MCPCKineMod::T_RT(double alpha, double beta, double d, double x, double y)
{
	Eigen::Matrix4d RT;
	RT = Trans(0,0,d)*RotX(alpha)*RotY(beta)*Trans(x, y, 0);
	return RT;
}


Eigen::Matrix4d MCPCKineMod::RT(double alpha, double beta, double gama, double theta, double x, double y, double z)
{
	Eigen::Matrix4d RT;
	RT = RotZ(theta) * RotX(alpha) * RotY(beta) * RotZ(gama) * Trans(x, y, z);
	return RT;
}

vector<vector<double>> MCPCKineMod::GetMCPCParas()
{
	return m_MCPCParas;
}


vector<vector<double>> MCPCKineMod::GetjointAngles()
{
	return m_jointAngles;
}


