#include "ErrorModel.h"

ErrorModel::ErrorModel()
{
}

ErrorModel::~ErrorModel()
{
}


//统一，位姿用6维向量表示
VectorXd ErrorModel::CalculErrors()
{
	VectorXd PEErrors = VectorXd::Identity(3* m_measurePE.size(),1);
	for (int i=0;i< m_measurePE.size();++i)
	{
		VectorXd PEError;//位姿误差position and euler
		Matrix4d measureRT;
		Matrix4d measureTempRT;
		measureRT = Conv2RT(m_measurePE[i]);
		measureTempRT = m_T * measureRT;
		//把计算得到的rt矩阵转化为位姿向量
		VectorXd measureTempPE = Conv2PE(measureTempRT);		
		//计算位姿误差
		PEError = measureTempPE.block(0,0,3,1) - m_nominalPE[i].block(0, 0, 3, 1);
		PEErrors.block(i*PEError.rows(), 0, PEError.rows(), PEError.cols())= PEError;
	}
	return PEErrors;
}


MatrixXd ErrorModel::CalculPi(Matrix3d r, Vector3d t)
{
	MatrixXd pi = MatrixXd::Zero(6, 6);
	pi.block(0, 0, 3, 3) = r;
	Matrix3d antiMatrix = Convert2AntiMatrix(t);
	pi.block(0, 3, 3, 3) = -r * antiMatrix;
	pi.block(3, 3, 3, 3) = r;
	return pi;
}


Matrix3d ErrorModel::Convert2AntiMatrix(Vector3d t)
{
	Matrix3d antiMatrix;
	antiMatrix << 0, -t[2], t[1],
		t[2], 0, -t[0],
		-t[1], t[0], 0;
	return antiMatrix;
}

