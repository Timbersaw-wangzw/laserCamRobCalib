#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

//在机器人的每个关节角工作范围内，随机生成6个关节角为一组的数据
vector<vector<double>> GeneJointAngles();

double Angle2Rad(double angle);

int GeneRand(int min,int max);
