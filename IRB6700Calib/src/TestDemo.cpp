#include "IO.h"
#include "MCPCKineMod.h"
#include "DHKineMod.h"
#include "KineMod.h"
#include "CG.h"
#include "LM.h"
#include "ErrorModel.h"
#include "MCPCErrorModel.h"
#include "RobotPath.h"
#include "DataConver.h"

using namespace std;
using namespace Eigen;

enum IteraFunc { ReguLM,EnsemReguLM};

//计算位置误差的范数
vector<double> CalcuPosErrorNorm(vector<vector<double>>compensatedMCPCParaVec, vector<vector<double>> jointAngles,
	vector<VectorXd> meaPEOnBaseVec,string filePath)
{
	/////////验证辨识参数的准确性
	//计算训练集经补偿后的误差
	KineMod* compenKineMod6700 = new MCPCKineMod(compensatedMCPCParaVec, jointAngles);
	vector<Matrix4d> compenRTVec = compenKineMod6700->CalculRTBegin2End();
	vector<VectorXd> compenPEVec = Conv2PEVec(compenRTVec);
	//计算补偿后的位姿误差及模,每一行有8个数据，前4个是位置误差和模，后4个是欧拉角误差和模
	vector<VectorXd> compenPEErrorVec = CalculPEErrors(compenPEVec, meaPEOnBaseVec);
	IO ioTest;
	ioTest.Export8DDatas(filePath, compenPEErrorVec);
	//把位置误差的范数提出来单独放入容器中
	vector<double> posErrorNormVec;
	for (auto v = compenPEErrorVec.begin(); v != compenPEErrorVec.end(); ++v)
	{
		VectorXd temp = *v;
		double positionErrorNorm = temp[3];
		posErrorNormVec.push_back(positionErrorNorm);
	}
	return posErrorNormVec;
}


void main()
{
	//qbs niu de 
	int i = 0;
	int trainingParasNum = 70;//训练参数个数
	Matrix4d T = Matrix4d::Identity();
	//tracker2base
	T << 0.486, -0.874, -0.002, -818,
		0.874, 0.486, -0.004, 4033.977,
		0.004, 0.001, 1, -350.669,
		0, 0, 0, 1;
	/////////////////输入理论运动学参数
	IO ioTest;
	//MCPC参数中，轴1和tool0的参数为六维，其它参数为四维
	vector<vector<double>> MCPCParas4D = ioTest.Import4DDatas("D:\\Data\\RobotCalib\\testUR10\\WT\\MCPCDatas4D.txt");
	vector<vector<double>> MCPCParas6D = ioTest.Import6DDatas("D:\\Data\\RobotCalib\\testUR10\\WT\\MCPCDatas6D.txt");
	//把4维和6维的数据组合到一个vector中
	vector<vector<double>> MCPCParas;
	MCPCParas.push_back(MCPCParas6D[0]);
	for (int i = 0; i < MCPCParas4D.size(); i++)
	{
		MCPCParas.push_back(MCPCParas4D[i]);
	}
	MCPCParas.push_back(MCPCParas6D[1]);
	//把容器中的数据转换到向量中保存
	VectorXd nominalMCPCParas= Conv2VecX(MCPCParas);
	vector<vector<double>> jointAngles = ioTest.Import7DDatas("D:\\Data\\RobotCalib\\testUR10\\WT\\theta80.txt");
	KineMod* KineMod6700 = new MCPCKineMod(MCPCParas, jointAngles);	
	vector<Matrix4d> nominalRTVec =KineMod6700->CalculRTBegin2End();
	vector<VectorXd> nominalPEVec = Conv2PEVec(nominalRTVec);
	//////////////读入TMac的位置值
	vector<vector<double>> tmacPositions = ioTest.Import3DDatas("D:\\Data\\RobotCalib\\testUR10\\WT\\pos80.txt");
	//////////////检查TMac的位置值的组数与关节角组数是否相同
	if (jointAngles.size()!= tmacPositions.size())
	{
		system("pause");
		exit(-1);
	}
	//把vector<double>转化为VectorXd形式
	vector<VectorXd> measurePEVec;
	for (int i = 0; i < tmacPositions.size(); ++i)
	{
		VectorXd measurePE= VectorXd::Ones(6);
		measurePE << tmacPositions[i][0], tmacPositions[i][1], tmacPositions[i][2], 0, 0, 0;
		measurePEVec.push_back(measurePE);
	}
	vector<VectorXd> measurePEOnBaseVec;
	for (int i = 0; i < measurePEVec.size(); ++i)
	{
		Matrix4d tempMat = Conv2RT(measurePEVec[i]);
		VectorXd tempVecX = Conv2PE((T.inverse())*tempMat);
		measurePEOnBaseVec.push_back(tempVecX);
	}
	//////////////前trainingParasNum个参数作为训练参数,剩下的参数作为验证集
	vector<vector<double>> trainingJointAngles;
	vector<VectorXd> trainingNominalPEVec;//在机器人base坐标系中的tool0的位姿
	vector<VectorXd> trainingMeasurePEVec;//在跟踪仪坐标系中的测量的靶标位姿
	vector<VectorXd> trainMeaPEOnBaseVec;//在机器人base坐标系中测量的靶标位姿
	for (int i = 0; i < trainingParasNum; i++)
	{
		trainingJointAngles.push_back(jointAngles[i]);
		trainingNominalPEVec.push_back(nominalPEVec[i]);
		trainingMeasurePEVec.push_back(measurePEVec[i]);
		trainMeaPEOnBaseVec.push_back(measurePEOnBaseVec[i]);
	}
	vector<vector<double>> validJointAngles;
	vector<VectorXd> validNominalPEVec;
	vector<VectorXd> validMeasurePEVec;
	vector<VectorXd> validMeasurePEOnBaseVec;
	for (int i= trainingParasNum;i< measurePEVec.size();++i)
	{
		validJointAngles.push_back(jointAngles[i]);
		validNominalPEVec.push_back(nominalPEVec[i]);
		validMeasurePEVec.push_back(measurePEVec[i]);
		validMeasurePEOnBaseVec.push_back(measurePEOnBaseVec[i]);
	}
	VectorXd x0 = 0.5*VectorXd::Ones(32,1);//经过归一化的初值

	//创建归一化矩阵H,L
	//X=H*x-L
	MatrixXd H= MatrixXd::Zero(32,32);//32x32
	VectorXd L= VectorXd::Zero(32,1);//32x1
	H.block(0, 0, 3, 3) = 0.02*Matrix3d::Identity();
	H.block(3, 3, 3, 3) = 10.0 * Matrix3d::Identity();
	H.block(26, 26, 3, 3) = 0.02*Matrix3d::Identity();
	H.block(29, 29, 3, 3) = 10.0 * Matrix3d::Identity();
	Matrix4d h = Matrix4d::Zero();
	h.block(0, 0, 2, 2) = 0.02*Matrix2d::Identity();
	h.block(2, 2, 2, 2) = 10.0*Matrix2d::Identity();

	VectorXd l0= VectorXd::Zero(6);
	l0 << 0.01, 0.01, 0.01, 5, 5, 5;
	L.block(0, 0, 6, 1) = l0;
	L.block(26, 0, 6, 1) = l0;
	VectorXd l1 = VectorXd::Zero(4);
	l1 << 0.01, 0.01, 5, 5;
	for (int i=0;i<5;++i)
	{
		H.block(6 + 4 * i, 6 + 4 * i, 4, 4) = h;
		L.block(6 + 4 * i, 0, 4, 1) = l1;
	}
	//不用归一化
	H = MatrixXd::Identity(32,32);
	L = VectorXd::Zero(32);
	x0 = VectorXd::Zero(32);
	LM LMIdenti6700(x0,1e-6, 1e-6,1000,1,H,L);
	//建立误差模型
    //准备误差模型的输入参数	
	LMIdenti6700.LMMCPCErrorModel=new MCPCErrorModel(MCPCParas, trainingJointAngles,
		trainingNominalPEVec, trainingMeasurePEVec, T.inverse());
    //.........................进行参数辨识
	VectorXd diffParas;
	IteraFunc funcName = EnsemReguLM;
	int sumK = 0;//算法的总迭代次数
	switch (funcName)
	{
	//////正则LM迭代
	case ReguLM:
	{
		VectorXd paramsError;
		int k;
		LMIdenti6700.m_Regular = L1;
		LMIdenti6700.Itera(paramsError,k);
		diffParas= H * paramsError - L;
		sumK = k;
		break;
	}
	case EnsemReguLM:
	{
		//////集成正则项法
		VectorXd weightTrainSample = VectorXd::Ones(trainingParasNum) / trainingParasNum;//每个训练样本的权重,初始化为1/n
		vector<int> trainIndexVec;//训练集中，补偿后误差偏离均值较大的训练集的索引
		vector<double> weightRegularModVec;
		vector<VectorXd> paramsErrors;
		vector<int> ks;
		//采用不同正则项带入到误差模型中进行参数辨识
		LMIdenti6700.VariousRegulaItera(paramsErrors, ks);
		for (auto i = 0; i < paramsErrors.size(); i++)
		{
			VectorXd  x = paramsErrors[i];//归一化后的参数
			sumK += ks[i];
			//运动学参数误差
			VectorXd ensemDiffParas = H * x - L;
			VectorXd compensatedKineParas = nominalMCPCParas + ensemDiffParas;
			//把向量转化为容器的容器中保存
			int indexArray[] = { 6,4,4,4,4,4,6 };
			vector <vector<double>>  compensatedMCPCParaVec = Conv2Vec2(compensatedKineParas, indexArray);
			//计算训练集经补偿后的误差
			vector<double> trainposErrorNormVec = CalcuPosErrorNorm(compensatedMCPCParaVec, trainingJointAngles,
				trainMeaPEOnBaseVec, "D:\\Data\\RobotCalib\\testUR10\\WT\\compenTrainingPEError.txt");
			double trainMeanError = GetMeanValue(trainposErrorNormVec);
			double trainSD = GetSDValue(trainposErrorNormVec);
			//检索训练集中，补偿后误差偏离均值较大的训练集的索引
			double errorRate = 0;//误差模型的错误率
			double lambda = 1;
			for (int i = 0; i < trainposErrorNormVec.size(); ++i)
			{
				double tempErr = abs(trainposErrorNormVec[i] - trainMeanError);
				if (tempErr > lambda*trainSD)
				{
					trainIndexVec.push_back(i);
					errorRate += weightTrainSample[i];
				}
			}
			double weightRegularMod = log(1 / errorRate);//误差模型的权重
			weightRegularModVec.push_back(weightRegularMod);
			//更新每个训练样本的权重
			if (1)
			{
				double tempWeight = 0;
				for (auto i = 0; i < trainIndexVec.size(); ++i)
				{
					tempWeight += weightTrainSample[trainIndexVec[i]];
				}
				double D = errorRate + errorRate * tempWeight;
				for (int i = 0; i < weightTrainSample.size(); ++i)
				{
					double tempErr = abs(trainposErrorNormVec[i] - trainMeanError);
					if (tempErr > lambda*trainSD)
					{
						weightTrainSample[i] = (weightTrainSample[i]/(weightTrainSample[i]+5))/ D;
					}
					else
					{
						weightTrainSample[i] = errorRate * weightTrainSample[i] / D;
					}
				}
			}
		}
		//由集成正则项模型预测的结果为	
		double sunweightRegularMod = 0;
		VectorXd tempNum = VectorXd::Zero(paramsErrors[0].size());
		//weightRegularModVec[0] = 1.5;
		for (auto i = 0; i < weightRegularModVec.size(); ++i)
		{
			tempNum += weightRegularModVec[i] * (H*paramsErrors[i] - L);
			sunweightRegularMod += weightRegularModVec[i];
			std::cout << weightRegularModVec[i] << endl;
		}
		diffParas = tempNum / sunweightRegularMod;
		break;
	}
	default:
		break;
	}

	////////验证辨识结果的精度
	VectorXd compensatedKineParas = nominalMCPCParas + diffParas;
	//把向量转化为容器的容器中保存
	int indexArray[] = { 6,4,4,4,4,4,6 };
	vector <vector<double>>  compensatedMCPCParaVec = Conv2Vec2(compensatedKineParas, indexArray);
	////计算原始误差
	vector<VectorXd> originalPEErrorVec = CalculPEErrors(nominalPEVec, measurePEOnBaseVec);
	ioTest.Export8DDatas("D:\\Data\\RobotCalib\\testUR10\\WT\\originalPEError.txt", originalPEErrorVec);
	////计算验证集补偿后的理论末端位姿和测量末端位置
	vector<double> positionErrorNormVec= CalcuPosErrorNorm(compensatedMCPCParaVec, validJointAngles,
		validMeasurePEOnBaseVec, "D:\\Data\\RobotCalib\\testUR10\\WT\\compenValidPEError.txt");
	double maxError = GetMaxValue(positionErrorNormVec);
	double meanError = GetMeanValue(positionErrorNormVec);
	double RMSError = GetRMSValue(positionErrorNormVec);
	double SD = GetSDValue(positionErrorNormVec);
	std::cout << "maxError: " << maxError << endl;
	std::cout << "meanError: " << meanError << endl;
	std::cout << "RMSError: " << RMSError << endl;
	std::cout << "SD: " << SD << endl;
	std::cout << "k: " << sumK << endl;
}