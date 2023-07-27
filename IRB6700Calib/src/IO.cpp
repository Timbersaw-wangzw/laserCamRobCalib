#include "IO.h"

IO::IO()
{
}


IO::~IO()
{
}


vector<vector<double>> IO::Import3DDatas(std::string filename)
{
	vector<vector<double>> datas3D;
	int num = 0;
	vector<double> tmpVectorData(3);
	FILE* mcpcFile = nullptr;
	errno_t err = fopen_s(&mcpcFile, filename.c_str(), "r");

	if (err != 0)
	{
		//��ʾ�ļ��򿪴���...
		exit(EXIT_FAILURE);
	}
	while (num != -1)
	{
		num = fscanf_s(mcpcFile, "%lf,%lf,%lf\n", &tmpVectorData[0], &tmpVectorData[1], &tmpVectorData[2]);
		if (num == 3)
		{
			datas3D.push_back(tmpVectorData);
		}
		else
		{
			continue;//���ĳ�����ݸ�����Ϊ4������������
		}
	}
	fclose(mcpcFile);
	return datas3D;
}


vector<vector<double>> IO::Import4DDatas(std::string filename)
{
	vector<vector<double>> datas4D;
	int num = 0;
	vector<double> tmpVectorData(4);
	FILE* mcpcFile = nullptr;
	errno_t err = fopen_s(&mcpcFile, filename.c_str(), "r");

	if (err != 0)
	{
		//��ʾ�ļ��򿪴���...
		exit(EXIT_FAILURE);
	}
	while (num != -1)
	{
		num = fscanf_s(mcpcFile, "%lf,%lf,%lf,%lf\n", &tmpVectorData[0], &tmpVectorData[1], &tmpVectorData[2],
			&tmpVectorData[3]);
		if (num == 4)
		{
			datas4D.push_back(tmpVectorData);
		}
		else
		{
			//if (!feof(mcpcFile))
			//{
			//	//��ʾ�ļ��ڲ����ݵ���ڷǷ���...
			//	cout << "���ݸ�������" << endl;
			//	exit(EXIT_FAILURE);
			//}
			continue;//���ĳ�����ݸ�����Ϊ4������������
		}
	}
	fclose(mcpcFile);
	return datas4D;
}


vector<vector<double>> IO::Import6DDatas(std::string filename)
{
	vector<vector<double>> datas6D;
	int num = 0;
	vector<double> tmpMatrixData(6);
	FILE* mcpcFile = nullptr;
	errno_t err = fopen_s(&mcpcFile, filename.c_str(), "r");

	if (err != 0)
	{
		//��ʾ�ļ��򿪴���...
		exit(EXIT_FAILURE);
	}
	while (num != -1)
	{
		num = fscanf_s(mcpcFile, "%lf,%lf,%lf,%lf,%lf,%lf\n", &tmpMatrixData[0], &tmpMatrixData[1], &tmpMatrixData[2],
			&tmpMatrixData[3], &tmpMatrixData[4], &tmpMatrixData[5]);
		if (num == 6)
		{
			datas6D.push_back(tmpMatrixData);
		}
		else
		{
			//if (!feof(mcpcFile))
			//{
			//	//��ʾ�ļ��ڲ����ݵ���ڷǷ���...
			//	cout << "���ݸ�������" << endl;
			//	exit(EXIT_FAILURE);
			//}
			continue;//���ĳ�����ݸ�����Ϊ6������������
		}
	}
	fclose(mcpcFile);
	return datas6D;
}

vector<vector<double>> IO::Import7DDatas(std::string filename)
{
	vector<vector<double>> datas7D;
	int num = 0;
	vector<double> tmpMatrixData(7);
	FILE* mcpcFile = nullptr;
	errno_t err = fopen_s(&mcpcFile, filename.c_str(), "r");

	if (err != 0)
	{
		//��ʾ�ļ��򿪴���...
		exit(EXIT_FAILURE);
	}
	while (num != -1)
	{
		num = fscanf_s(mcpcFile, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &tmpMatrixData[0], &tmpMatrixData[1], &tmpMatrixData[2],
			&tmpMatrixData[3], &tmpMatrixData[4], &tmpMatrixData[5], &tmpMatrixData[6]);
		if (num == 7)
		{
			datas7D.push_back(tmpMatrixData);
		}
		else
		{
			//if (!feof(mcpcFile))
			//{
			//	//��ʾ�ļ��ڲ����ݵ���ڷǷ���...
			//	cout << "���ݸ�������" << endl;
			//	exit(EXIT_FAILURE);
			//}
			continue;//���ĳ�����ݸ�����Ϊ6������������
		}
	}
	fclose(mcpcFile);
	return datas7D;
}


void IO::Export8DDatas(string filename, vector<VectorXd> dataVec)
{
	if (dataVec.empty())
	{
		cout << "����������" << endl;
		exit(EXIT_FAILURE);
	}
	FILE * pf=0;
	errno_t err = fopen_s(&pf, filename.c_str(), "w+");//������ļ��е�������д������
	if (err != 0)
	{
		//��ʾ�򿪻򴴽��ļ�����
		//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:�򿪻򴴽��ļ�����"));
		//return;
		cout << "�򿪻򴴽��ļ�����" << endl;
		exit(EXIT_FAILURE);
	}
	const int rows = dataVec.size();
	const int cols = dataVec[0].size();

	for (int row_index = 0; row_index < rows; ++row_index)
	{
		for (int col_index = 0; col_index < cols; ++col_index)
		{
			int a=-1;
			if (col_index== cols-1)
			{
				a = fprintf_s(pf, "%lf", dataVec[row_index][col_index]);
			}
			else 
			{
				a = fprintf_s(pf, "%lf,", dataVec[row_index][col_index]);
			}			
			if (a < 0)
			{
				//��ʾд���ļ�����...
				//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:д���ļ�����"));
				//return;
				cout << "д���ļ�����" << endl;
				exit(EXIT_FAILURE);
			}
		}
		int c = fprintf_s(pf, "\n");
		if (c < 0)
		{
			//��ʾд���ļ�����...
			//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:д���ļ�����"));
			//return;
			cout << "д���ļ�����" << endl;
			exit(EXIT_FAILURE);
		}
	}
	//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:д���ļ��ɹ���"));
	fclose(pf);
}
