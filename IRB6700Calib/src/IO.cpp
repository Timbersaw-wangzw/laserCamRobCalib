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
		//提示文件打开错误...
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
			continue;//如果某行数据个数不为4，则跳过该行
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
		//提示文件打开错误...
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
			//	//提示文件内部数据点存在非法点...
			//	cout << "数据个数错误" << endl;
			//	exit(EXIT_FAILURE);
			//}
			continue;//如果某行数据个数不为4，则跳过该行
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
		//提示文件打开错误...
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
			//	//提示文件内部数据点存在非法点...
			//	cout << "数据个数错误" << endl;
			//	exit(EXIT_FAILURE);
			//}
			continue;//如果某行数据个数不为6，则跳过该行
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
		//提示文件打开错误...
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
			//	//提示文件内部数据点存在非法点...
			//	cout << "数据个数错误" << endl;
			//	exit(EXIT_FAILURE);
			//}
			continue;//如果某行数据个数不为6，则跳过该行
		}
	}
	fclose(mcpcFile);
	return datas7D;
}


void IO::Export8DDatas(string filename, vector<VectorXd> dataVec)
{
	if (dataVec.empty())
	{
		cout << "无数据数据" << endl;
		exit(EXIT_FAILURE);
	}
	FILE * pf=0;
	errno_t err = fopen_s(&pf, filename.c_str(), "w+");//先清除文件中的数据再写入数据
	if (err != 0)
	{
		//提示打开或创建文件错误
		//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:打开或创建文件错误！"));
		//return;
		cout << "打开或创建文件错误" << endl;
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
				//提示写入文件错误...
				//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:写入文件错误！"));
				//return;
				cout << "写入文件错误" << endl;
				exit(EXIT_FAILURE);
			}
		}
		int c = fprintf_s(pf, "\n");
		if (c < 0)
		{
			//提示写入文件错误...
			//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:写入文件错误！"));
			//return;
			cout << "写入文件错误" << endl;
			exit(EXIT_FAILURE);
		}
	}
	//GlobalParams::mw->ui->InfoTxt->append(QString::fromLocal8Bit("Wxxxx:写入文件成功！"));
	fclose(pf);
}
