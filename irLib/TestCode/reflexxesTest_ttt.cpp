#include <iostream>
#include <fstream>
#include <conio.h>
#include <string>
#include <time.h>
#include <cstdlib>
#include <math.h>

#include <irUtils\Diagnostic.h>
#include <irDyn\SerialOpenChain.h>
#include <irTrajectoryGeneration\OTGAPI.h>

#include <Reflexxes\Core.h>

#include "efortRobot.h"

using namespace std;
using namespace Eigen;
using namespace irLib::irDyn;
using namespace irLib::irMath;
using namespace irLib::irTG;

std::string filepath_ys = "C:/Users/crazy/Desktop/Time optimization/sCurveTest/";
std::string filepath = "D:/jkkim/Documents/projects/otgtest/";

void saveReal1DArray2txt(Real* input, unsigned int size, std::string filename);
void saveRealArray2txt(Real** input, unsigned int size1, unsigned int size2, std::string filename);
void saveProfile(Profile* profile, std::string filename);
//Real makeRandLU(Real lower, Real upper);
void saveReal(Real in, std::string filename);

int main()
{
	// 라이브러리용 코드는 공유해도 main cpp 파일은 각자 하나씩 만들어서 쓰는게 github 코드 공유하는데 좋음
	// 일단 똑같은 내용으로 ttt 만들어 놨으니 '빌드시 제외' 옵션 이용해서 하면 됨

	// random input 으로 되어 있는데, 구현 안되어 있는 부분도 있고, 특정 조건에서 decision box를 들어와야 하는 조건 등으로 인해 error 뜰 수 있음

	srand((unsigned int)time(NULL));
	SerialOpenChainPtr robot = SerialOpenChainPtr(new efortRobot);
	unsigned int dof = robot->getNumOfJoint();
	dof = 1;

	// variables for step 1 / step 2 test
	//Real maxVel, maxAcc, maxJer, curPos, curVel, curAcc, tarPos, tarVel, curTime;

	//int jointIdx = 0;
	//maxVel = robot->getJointPtr(jointIdx)->getLimitVelUpper();
	//maxAcc = robot->getJointPtr(jointIdx)->getLimitAccUpper();
	//maxJer = robot->getJointPtr(jointIdx)->getLimitJerkUpper();
	//curPos = makeRandLU(robot->getJointPtr(jointIdx)->getLimitPosLower(), robot->getJointPtr(jointIdx)->getLimitPosUpper());
	//curVel = makeRandLU(robot->getJointPtr(jointIdx)->getLimitVelLower(), robot->getJointPtr(jointIdx)->getLimitVelUpper());
	////curAcc = makeRandLU(/*robot->getJointPtr(jointIdx)->getLimitAccLower()*/0.0, robot->getJointPtr(jointIdx)->getLimitAccUpper());
	//curAcc = makeRandLU(robot->getJointPtr(jointIdx)->getLimitAccLower(), robot->getJointPtr(jointIdx)->getLimitAccUpper());
	//tarPos = makeRandLU(robot->getJointPtr(jointIdx)->getLimitPosLower(), robot->getJointPtr(jointIdx)->getLimitPosUpper());
	//tarVel = makeRandLU(robot->getJointPtr(jointIdx)->getLimitVelLower(), robot->getJointPtr(jointIdx)->getLimitVelUpper());
	//curTime = 0.0;

	//curVel = -335;
	//maxVel = 985;
	//maxAcc = 972;
	//maxJer = 324;
	//curPos = -499;
	//curAcc = 152;
	//tarPos = -90;
	//tarVel = -347;
	//curTime = 0.0;

	/////////////////////////////////////////////////////////////////
	//////////////// test step 1 with random initial ////////////////

	//TreeSCurveStep1 * decisionTreeStep1 = makeDecisionTreeSCurveStep1(maxVel, maxAcc, maxJer, curPos, curVel, curAcc, tarPos, tarVel, curTime);
	////calculateMinimumTimeAndInoperTimeSCurve(decisionTreeStep1);
	////cout << "minimum time: " << decisionTreeStep1->_tmin << endl;
	////cout << "decision box 1 result: " << Decision1_SCurveStep1(decisionTreeStep1) << endl; // test decision box...

	//calculateMinimumTimeAndInoperTimeSCurve(decisionTreeStep1);
	//cout << endl << "minimum time: " << decisionTreeStep1->_tmin << endl;
	//deleteDecisionTreeSCurveStep1(decisionTreeStep1);

	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////



	/////////////////////////////////////////////////////////////////
	//////////////// test step 2 with random initial ////////////////

	//Real syncTime = 3.0; //decisionTreeStep1->_tmin
	//TreeSCurveStep2 * decisionTreeStep2 = makeDecisionTreeSCurveStep2(maxVel, maxAcc, maxJer, curPos, curVel, curAcc, tarPos, tarVel, curTime, syncTime);
	//Profile * step2Profile = new Profile;
	//makeFinalProfilesSCurve(decisionTreeStep2, step2Profile);
	//cout << step2Profile->PositionProfile[0]._a0 << endl;
	//// position/velocity/acceleration/jerk 모두 3차 polynomial. _ai: i차 계수
	//cout << "decision box 5 result: " << Decision5_SCurveStep2(decisionTreeStep2) << endl;
	//deleteDecisionTreeSCurveStep2(decisionTreeStep2);
	//delete step2Profile;

	/////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////



	////////////////////////////////////////////////////////////////////////
	//////////////// test whole process with random initial ////////////////

	Real* maxPosition = new Real[dof];
	Real* maxVelocity = new Real[dof];
	Real* maxAcceleration = new Real[dof];
	Real* maxJerk = new Real[dof];
	Real* currentPosition = new Real[dof];
	Real* currentVelocity = new Real[dof];
	Real* currentAcceleration = new Real[dof];
	Real* targetPosition = new Real[dof];
	Real* targetVelocity = new Real[dof];
	Real* currTime = new Real[dof];

	for (unsigned int i = 0; i < dof; i++)
	{
		maxPosition[i] = robot->getJointPtr(i)->getLimitPosUpper();
		maxVelocity[i] = robot->getJointPtr(i)->getLimitVelUpper();
		maxAcceleration[i] = robot->getJointPtr(i)->getLimitAccUpper();
		maxJerk[i] = robot->getJointPtr(i)->getLimitJerkUpper();

		currentPosition[i] = makeRandLU(-maxPosition[i], maxPosition[i]);
		currentVelocity[i] = makeRandLU(-0.8*maxVelocity[i], 0.8 * maxVelocity[i]);
		currentAcceleration[i] = makeRandLU(0.0, maxAcceleration[i]);

		targetPosition[i] = makeRandLU(-maxPosition[i], maxPosition[i]);
		targetVelocity[i] = makeRandLU(-maxVelocity[i], maxVelocity[i]);
		
		currTime[i] = 0.0;
	}

	// 랜덤 인풋으로 실험할 때 오류나는 부분 저장해 놨다가 찾아서 디버깅해야 하니까 
	// 아래처럼 콘솔창에 출력하든지 txt 파일로 저장하든지 둘 중 하나 해야함

	for (unsigned int i = 0; i < dof; i++)
	{
		cout << "=== " << i << " ===" << endl;
		cout << "max position: " << maxPosition[i] << endl;
		cout << "max velocity: " << maxVelocity[i] << endl;
		cout << "max acceleration: " << maxAcceleration[i] << endl;
		cout << "max jerk: " << maxJerk[i] << endl;
		cout << endl;
		cout << "cur position: " << currentPosition[i] << endl;
		cout << "cur velocity: " << currentVelocity[i] << endl;
		cout << "cur acceleration: " << currentAcceleration[i] << endl;
		cout << endl;
		cout << "trgt position: " << targetPosition[i] << endl;
		cout << "trgt velocity: " << targetVelocity[i] << endl;
		cout << endl;
	}

	//saveReal1DArray2txt(maxPosition, dof, filepath + "aaa.txt");

	maxPosition[0] = 3.05433;
	maxVelocity[0] = 1.74533;
	maxAcceleration[0] = 8.72655;
	maxJerk[0] = 3000;
	currentPosition[0] = 2.29892;
	currentVelocity[0] = 0.206199;
	currentAcceleration[0] = 3.96184;
	targetPosition[0] = -2.6934;
	targetVelocity[0] = 0.05736;

	OTGSCurve * otgSCurve = new OTGSCurve(dof);
	otgSCurve->setInputParameters(maxVelocity, maxAcceleration, maxJerk, currentPosition, currentVelocity, currentAcceleration, targetPosition, targetVelocity);

	unsigned int numOfdata = 5000;
	Real** posTraj = new Real*[dof];
	Real** velTraj = new Real*[dof];
	Real** accTraj = new Real*[dof];
	Real** jerTraj = new Real*[dof];
	for (unsigned int i = 0; i < dof; i++)
	{
		posTraj[i] = new Real[numOfdata];
		velTraj[i] = new Real[numOfdata];
		accTraj[i] = new Real[numOfdata];
		jerTraj[i] = new Real[numOfdata];
	}
	
	otgSCurve->calculateJointTrajectory(posTraj, velTraj, accTraj, jerTraj, numOfdata);

	saveRealArray2txt(posTraj, dof, numOfdata, filepath + "position.txt");
	saveRealArray2txt(velTraj, dof, numOfdata, filepath + "velocity.txt");
	saveRealArray2txt(accTraj, dof, numOfdata, filepath + "acceleration.txt");
	saveRealArray2txt(jerTraj, dof, numOfdata, filepath + "jerk.txt");

	delete otgSCurve;
	for (unsigned int i = 0; i < dof; i++)
	{
		delete[] posTraj[i];
		delete[] velTraj[i];
		delete[] accTraj[i];
		delete[] jerTraj[i];
	}
	delete[] posTraj;
	delete[] velTraj;
	delete[] accTraj;
	delete[] jerTraj;


	delete[] maxVelocity;
	delete[] maxAcceleration;
	delete[] maxJerk;
	delete[] currentPosition;
	delete[] currentVelocity;
	delete[] currentAcceleration;
	delete[] targetPosition;
	delete[] targetVelocity;
	delete[] currTime;

	////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////


	cout << "Program complete" << endl;
	_getch();
	return 0;
}

void saveReal1DArray2txt(Real* input, unsigned int size, std::string filename)
{
	std::ofstream fout;
	fout.open(filename);

	for (unsigned int i = 0; i < size; i++)
	{
		fout << input[i] << endl;
	}
	fout.close();
}

void saveRealArray2txt(Real** input, unsigned int size1, unsigned int size2, std::string filename)
{
	std::ofstream fout;
	fout.open(filename);

	for (unsigned int i = 0; i < size2; i++)
	{
		for (unsigned int j = 0; j < size1; j++)
			fout << input[j][i] << '\t';
		fout << std::endl;
	}

	fout.close();
}

void saveProfile(Profile * profile, std::string filename)
{
	std::ofstream fout;
	// position
	fout.open(filename + "position.txt");
	for (unsigned int i = 0; i < profile->numOfPolynomial; i++)
	{
		fout << profile->PositionProfile[i]._a0 << '\t';
		fout << profile->PositionProfile[i]._a1 << '\t';
		fout << profile->PositionProfile[i]._a2 << '\t';
		fout << profile->PositionProfile[i]._a3 << endl;
	}
	fout.close();

	// velocity
	fout.open(filename + "velocity.txt");
	for (unsigned int i = 0; i < profile->numOfPolynomial; i++)
	{
		fout << profile->VelocityProfile[i]._a0 << '\t';
		fout << profile->VelocityProfile[i]._a1 << '\t';
		fout << profile->VelocityProfile[i]._a2 << '\t';
		fout << profile->VelocityProfile[i]._a3 << endl;
	}
	fout.close();

	// acceleration
	fout.open(filename + "acceleration.txt");
	for (unsigned int i = 0; i < profile->numOfPolynomial; i++)
	{
		fout << profile->AccelerationProfile[i]._a0 << '\t';
		fout << profile->AccelerationProfile[i]._a1 << '\t';
		fout << profile->AccelerationProfile[i]._a2 << '\t';
		fout << profile->AccelerationProfile[i]._a3 << endl;
	}
	fout.close();

	// jerk
	fout.open(filename + "jerk.txt");
	for (unsigned int i = 0; i < profile->numOfPolynomial; i++)
	{
		fout << profile->JerkProfile[i]._a0 << '\t';
		fout << profile->JerkProfile[i]._a1 << '\t';
		fout << profile->JerkProfile[i]._a2 << '\t';
		fout << profile->JerkProfile[i]._a3 << endl;
	}
	fout.close();


	// time
	fout.open(filename + "polytime.txt");
	for (unsigned int i = 0; i < profile->numOfPolynomial - 1; i++)
	{
		fout << profile->PolynomialTime[i] << endl;
	}
	fout.close();

}

//Real makeRandLU(Real lower, Real upper)
//{
//	return lower + (upper - lower)*((double)rand() / 32767.0);
//}

void saveReal(Real in, std::string filename)
{
	std::ofstream fout;
	fout.open(filename);
	fout << in << endl;
	fout.close();
}
