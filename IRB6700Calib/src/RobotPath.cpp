#include "RobotPath.h"
#include <cstdlib> 

vector<vector<double>> GeneJointAngles()
{
	vector<vector<double>> JointAngleVec;
	unsigned seed = 1;
	srand(seed);
	int i = 0;
	while (i<50)
	{
		vector<double>JointAngles;
		double j0 = 0;
		double j1 = Angle2Rad(GeneRand(-170,170));
		double j2 = Angle2Rad(GeneRand(-65, 85));
		double j3 = Angle2Rad(GeneRand(-180, 70));
		double j4 = Angle2Rad(GeneRand(-300, 300));
		double j5 = Angle2Rad(GeneRand(-130, 130));
		double j6 = Angle2Rad(GeneRand(-360, 360));
		JointAngles = { j0,j1,j2,j3 ,j4 ,j5 ,j6 };
		JointAngleVec.push_back(JointAngles);
		cout << j0 << "," << j1 << "," << j2 << "," << j3 << "," << j4 << "," << j5 << "," << j6 <<","<< endl;
		i = i + 1;
	}	
	return JointAngleVec;
}

double Angle2Rad(double angle)
{
	return angle*3.1415926/180.0;
}

int GeneRand(int min, int max)
{
	return rand()%(max-min+1)+min;
}
