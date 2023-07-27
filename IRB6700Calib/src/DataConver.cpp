#include "DataConver.h"

vector<Matrix3d> Euler2Matrix3dVec(vector<Vector3d> eulerVec)
{
	vector<Matrix3d> rVec;
	for (int i = 0; i < eulerVec.size(); ++i)
	{
		Vector3d euler = eulerVec[i];
		Matrix4d r = RotZ(euler[2])*RotY(euler[1])*RotX(euler[0]);
		rVec.push_back(r.block(0, 0, 3, 3));
	}
	return rVec;
}


Matrix3d Euler2Matrix3d(Vector3d euler)
{
	Matrix4d r = RotZ(euler[2])*RotY(euler[1])*RotX(euler[0]);
	//Matrix4d r = RotX(euler[0])*RotY(euler[1])*RotZ(euler[2]);//右乘
	return r.block(0, 0, 3, 3);
}


vector<Matrix4d>  Conv2RTVec(vector<VectorXd> eulerPositionVec)
{
	//把位姿向量拆分出位置向量，姿态向量
	vector<Vector3d> eulerVec, positionVec;
	for (int i = 0; i < eulerPositionVec.size(); i++)
	{
		Vector3d euler, position;
		position = eulerPositionVec[i].block(0, 0, 3, 1);
		positionVec.push_back(position);
		euler = eulerPositionVec[i].block(3, 0, 3, 1);
		eulerVec.push_back(euler);
	}
	vector<Matrix3d> rVec = Euler2Matrix3dVec(eulerVec);
	vector<Matrix4d> rtVec;
	for (int i = 0; i < eulerVec.size(); ++i)
	{
		Matrix4d rt = Matrix4d::Identity();
		rt.block(0, 0, 3, 3) = rVec[i];
		rt.block(0, 3, 3, 1) = positionVec[i];
		rtVec.push_back(rt);
	}
	return rtVec;
}

Matrix4d Conv2RT(VectorXd PE)
{
	//把位姿向量拆分出位置向量，姿态向量
	Vector3d euler, position;
	position = PE.block(0, 0, 3, 1);
	euler = PE.block(3, 0, 3, 1);
	Matrix3d r = Euler2Matrix3d(euler);
	Matrix4d rt = Matrix4d::Identity();
	rt.block(0, 0, 3, 3) = r;
	rt.block(0, 3, 3, 1) = position;
	return rt;
}

vector<VectorXd> Conv2PEVec(vector<Matrix4d> rtVec)
{
	vector<VectorXd> EPVec;
	for (int i = 0; i < rtVec.size(); ++i)
	{
		VectorXd PE = VectorXd::Ones(6, 1);
		Matrix3d r = rtVec[i].block(0, 0, 3, 3);
		Vector3d eulerZYX = r.eulerAngles(2,1,0);//转化为绕运动轴的ZYX欧拉角，倒转顺序转化为绕固定轴的XYZ欧拉角
		Vector3d eulerRPY;
		eulerRPY << eulerZYX[2], eulerZYX[1], eulerZYX[0];//绕固定轴的XYZ欧拉角
		PE.block(0, 0, 3, 1) = rtVec[i].block(0, 3, 3, 1);
		PE.block(3, 0, 3, 1) = eulerRPY;
		EPVec.push_back(PE);
	}
	return EPVec;
}


VectorXd Conv2PE(Matrix4d rt)
{
	VectorXd PE = VectorXd::Ones(6, 1);
	Matrix3d r = rt.block(0, 0, 3, 3);
	Vector3d eulerZYX = r.eulerAngles(2,1,0);//转化为绕运动轴的ZYX欧拉角，倒转顺序转化为绕固定轴的XYZ欧拉角
	Vector3d eulerRPY;
	eulerRPY << eulerZYX[2], eulerZYX[1], eulerZYX[0];//绕固定轴的XYZ欧拉角
	PE.block(0, 0, 3, 1) = rt.block(0, 3, 3, 1);
	PE.block(3, 0, 3, 1) = eulerRPY;
	return PE;
}


_4MCPCParas Conv2_4MCPCParas(VectorXd vec)
{
	_4MCPCParas structParas;
	structParas.alpha = vec[0];
	structParas.beta = vec[1];
	structParas.x = vec[2];
	structParas.y = vec[3];
	return structParas;
}


_6MCPCParas Conv2_6MCPCParas(VectorXd vec)
{
	_6MCPCParas structParas;
	structParas.alpha = vec[0];
	structParas.beta = vec[1];
	structParas.gamma = vec[2];
	structParas.x = vec[3];
	structParas.y = vec[4];
	structParas.z = vec[5];
	return structParas;
}


vector<VectorXd> CalculPEErrors(vector<VectorXd> PE1, vector<VectorXd> PE2)
{
	vector<VectorXd> PEErrorVec;
	for (int i = 0; i < PE1.size(); ++i)
	{
		Matrix<double, 6, 1> temp6Vec = PE1[i] - PE2[i];
		Vector3d tempPVec = temp6Vec.block(0, 0, 3, 1);
		Vector3d tempEVec = temp6Vec.block(3, 0, 3, 1);
		VectorXd temp8Vec = VectorXd::Zero(8, 1);
		temp8Vec.block(0, 0, 3, 1) = tempPVec;
		temp8Vec[3] = tempPVec.norm();
		temp8Vec.block(4, 0, 3, 1) = tempEVec;
		temp8Vec[7] = tempEVec.norm();
		PEErrorVec.push_back(temp8Vec);
	}
	return PEErrorVec;
}


double GetMaxValue(vector<double> valueVec)
{
	double maxValue= valueVec[0];
	for (int i=0;i< valueVec.size();++i)
	{
		double tempValue= valueVec[i];
		if (maxValue<= tempValue)
		{
			maxValue = tempValue;
		}
	}
	return maxValue;
}


double GetMeanValue(vector<double> valueVec)
{
	double sumValue=0;
	for (int i = 0; i < valueVec.size(); ++i)
	{
		sumValue += valueVec[i];
	}
	return sumValue/ valueVec.size();
}


double GetRMSValue(vector<double> valueVec)
{
	double sumValue = 0;
	for (int i = 0; i < valueVec.size(); ++i)
	{
		sumValue += pow(valueVec[i],2);
	}
	double RMSValue=sqrt(sumValue/ valueVec.size());
	return RMSValue;
}

double GetSDValue(vector<double> valueVec)
{
	double sumValue = 0;
	for (int i = 0; i < valueVec.size(); ++i)
	{
		sumValue += valueVec[i];
	}
	double meanValue= sumValue / valueVec.size();
	double powSumValue = 0;
	for (int i = 0; i < valueVec.size(); ++i)
	{
		powSumValue += pow(valueVec[i]- meanValue,2);
	}
	return sqrt(powSumValue/ valueVec.size());
}


Eigen::Matrix4d RotX(double theta)
{
	Eigen::Matrix4d rx;
	rx << 1, 0, 0, 0,
		0, cos(theta), -sin(theta), 0,
		0, sin(theta), cos(theta), 0,
		0, 0, 0, 1;
	return rx;
}


Eigen::Matrix4d RotY(double theta)
{
	Eigen::Matrix4d ry;
	ry << cos(theta), 0, sin(theta), 0,
		0, 1, 0, 0,
		-sin(theta), 0, cos(theta), 0,
		0, 0, 0, 1;
	return ry;
}


Eigen::Matrix4d RotZ(double theta)
{
	Eigen::Matrix4d rz;
	rz << cos(theta), -sin(theta), 0, 0,
		sin(theta), cos(theta), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	return rz;
}


Eigen::Matrix4d Trans(double x, double y, double z)
{
	Eigen::Matrix4d t;
	t << 1, 0, 0, x,
		0, 1, 0, y,
		0, 0, 1, z,
		0, 0, 0, 1;
	return t;
}