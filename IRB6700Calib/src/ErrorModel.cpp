#include "ErrorModel.h"

ErrorModel::ErrorModel()
{
}

ErrorModel::~ErrorModel()
{
}


//ͳһ��λ����6ά������ʾ
VectorXd ErrorModel::CalculErrors()
{
	VectorXd PEErrors = VectorXd::Identity(3* m_measurePE.size(),1);
	for (int i=0;i< m_measurePE.size();++i)
	{
		VectorXd PEError;//λ�����position and euler
		Matrix4d measureRT;
		Matrix4d measureTempRT;
		measureRT = Conv2RT(m_measurePE[i]);
		measureTempRT = m_T * measureRT;
		//�Ѽ���õ���rt����ת��Ϊλ������
		VectorXd measureTempPE = Conv2PE(measureTempRT);		
		//����λ�����
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

