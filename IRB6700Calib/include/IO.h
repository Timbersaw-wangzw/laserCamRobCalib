#pragma once
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
using namespace std;
using namespace Eigen;
class IO
{
public:
	IO();
	~IO();
	vector<vector<double>> Import3DDatas(std::string filename);
	vector<vector<double>> Import4DDatas(std::string filename);
	vector<vector<double>> Import6DDatas(std::string filename);
	vector<vector<double>> Import7DDatas(std::string filename);

	void Export8DDatas(string filename, vector<Eigen::VectorXd> dataVec);
};

