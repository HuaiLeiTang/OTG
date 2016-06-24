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

#include "efortRobot.h"

using namespace std;
using namespace Eigen;
using namespace irLib::irDyn;
using namespace irLib::irMath;
using namespace irLib::irTG;

//std::string filepath = "C:/Users/crazy/Desktop/Time optimization/Trapazodal test/";
void saveRealArray2txt(Real** input, unsigned int size1, unsigned int size2, std::string filename);


int main()
{

	SerialOpenChainPtr robot = SerialOpenChainPtr(new efortRobot);
	int dof = robot->getNumOfJoint();

	Real maxPosition[6], maxVelocity[6], maxAcceleration[6];
	Real currentPosition[6], currentVelocity[6];
	Real waypointPosition1[6], waypointVelocity1[6];
	Real waypointPosition2[6], waypointVelocity2[6];
	Real targetPosition[6], targetVelocity[6];

	srand((unsigned int)time(NULL));
	for (int i = 0; i < dof; i++)
	{
		maxPosition[i] = robot->getJointPtr(i)->getLimitPosUpper();
		maxVelocity[i] = robot->getJointPtr(i)->getLimitVelUpper();
		maxAcceleration[i] = robot->getJointPtr(i)->getLimitAccUpper();

		currentPosition[i] = makeRandLU(-maxPosition[i], maxPosition[i]);
		currentVelocity[i] = makeRandLU(-maxVelocity[i], maxVelocity[i]);

		waypointPosition1[i] = makeRandLU(-maxPosition[i], maxPosition[i]);
		waypointVelocity1[i] = makeRandLU(-maxVelocity[i], maxVelocity[i]);

		waypointPosition2[i] = makeRandLU(-maxPosition[i], maxPosition[i]);
		waypointVelocity2[i] = makeRandLU(-maxVelocity[i], maxVelocity[i]);

		targetPosition[i] = makeRandLU(-maxPosition[i], maxPosition[i]);
		targetVelocity[i] = makeRandLU(-maxVelocity[i], maxVelocity[i]);
	}

	//// experiment 1
	//currentPosition[0] = -0.92328; currentPosition[1] = 0.910493; currentPosition[2] = -0.823;
	//currentPosition[3] = -2.86317; currentPosition[4] = 2.07993; currentPosition[5] = 1.01994;
	//currentVelocity[0] = -1.51416; currentVelocity[1] = -1.01242; currentVelocity[2] = 0.0287098;
	//currentVelocity[3] = 1.33229; currentVelocity[4] = -4.4649; currentVelocity[5] = -4.8516;

	//targetPosition[0] = -0.94882; targetPosition[1] = -0.0319749; targetPosition[2] = 0.589221;
	//targetPosition[3] = 1.00623; targetPosition[4] = 1.91957; targetPosition[5] = -5.47782;
	//targetVelocity[0] = 1.71284; targetVelocity[1] = 0.992985; targetVelocity[2] = 0.318343;
	//targetVelocity[3] = -3.95732; targetVelocity[4] = -3.79142; targetVelocity[5] = -6.75558;

	// experiment 2
	//currentPosition[0] = -2.33155; currentPosition[1] = 1.14645; currentPosition[2] = 0.107345;
	//currentPosition[3] = -1.42751; currentPosition[4] = -0.472073; currentPosition[5] = 6.13937;
	//currentVelocity[0] = 0.602479; currentVelocity[1] = -1.08119; currentVelocity[2] = -0.586947;
	//currentVelocity[3] = -3.49669; currentVelocity[4] = 1.7994; currentVelocity[5] = -6.54184;

	//waypointPosition1[0] = 0.683348; waypointPosition1[1] = 0.712508; waypointPosition1[2] = 0.526582;
	//waypointPosition1[3] = -0.794914; waypointPosition1[4] = 0.278498; waypointPosition1[5] = 4.89604;
	//waypointVelocity1[0] = 1.53025; waypointVelocity1[1] = 1.00509; waypointVelocity1[2] = -0.401265;
	//waypointVelocity1[3] = 4.03949; waypointVelocity1[4] = 1.75646; waypointVelocity1[5] = -6.09983;
	//
	//targetPosition[0] = -1.99151; targetPosition[1] = 1.22698; targetPosition[2] = 1.13314;
	//targetPosition[3] = 2.83402; targetPosition[4] = 1.619; targetPosition[5] = -1.83182;
	//targetVelocity[0] = -0.425959; targetVelocity[1] = 1.23911; targetVelocity[2] = 0.121178;
	//targetVelocity[3] = 3.87916; targetVelocity[4] = 4.33175; targetVelocity[5] = 4.83753;

	cout << "* currentPosition : ";
	for (int i = 0; i < dof; i++)
		cout << currentPosition[i] << '\t';
	cout << endl;
	cout << "* currentVelocity : ";
	for (int i = 0; i < dof; i++)
		cout << currentVelocity[i] << '\t';
	cout << endl;
	cout << "* waypointPosition1 : ";
	for (int i = 0; i < dof; i++)
		cout << waypointPosition1[i] << '\t';
	cout << endl;
	cout << "* waypointVelocity1 : ";
	for (int i = 0; i < dof; i++)
		cout << waypointVelocity1[i] << '\t';
	cout << endl;
	cout << "* waypointPosition2 : ";
	for (int i = 0; i < dof; i++)
		cout << waypointPosition2[i] << '\t';
	cout << endl;
	cout << "* waypointVelocity2 : ";
	for (int i = 0; i < dof; i++)
		cout << waypointVelocity2[i] << '\t';
	cout << endl;
	cout << "* targetPosition : ";
	for (int i = 0; i < dof; i++)
		cout << targetPosition[i] << '\t';
	cout << endl;
	cout << "* targetVelocity : ";
	for (int i = 0; i < dof; i++)
		cout << targetVelocity[i] << '\t';
	cout << endl;

	OTGTrapTrajAPI API(dof);
	API.setKinematicConstraints(maxVelocity, maxAcceleration);
	API.addPoint(currentPosition);
	API.addPoint(waypointPosition1);
	API.addPoint(waypointPosition2);
	API.addPoint(targetPosition);

	//API.addPoint(currentPosition, currentVelocity);
	//API.addPoint(waypointPosition1, waypointVelocity1);
	//API.addPoint(waypointPosition2, waypointVelocity2);
	//API.addPoint(targetPosition, targetVelocity);

	unsigned int numOfdata = 1000;
	Real** posTraj = new Real*[dof];
	Real** velTraj = new Real*[dof];
	Real** accTraj = new Real*[dof];
	for (int i = 0; i < dof; i++)
	{
		posTraj[i] = new Real[numOfdata];
		velTraj[i] = new Real[numOfdata];
		accTraj[i] = new Real[numOfdata];
	}

	API.calculateJointTrajectory(posTraj, velTraj, accTraj, numOfdata);

	//saveRealArray2txt(posTraj, dof, numOfdata, filepath + "position.txt");
	//saveRealArray2txt(velTraj, dof, numOfdata, filepath + "velocity.txt");
	//saveRealArray2txt(accTraj, dof, numOfdata, filepath + "acceleration.txt");

	for (int i = 0; i < dof; i++)
	{
		delete[] posTraj[i];
		delete[] velTraj[i];
		delete[] accTraj[i];
	}

	cout << "Program Complete" << endl;
	_getch();
	return 0;
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