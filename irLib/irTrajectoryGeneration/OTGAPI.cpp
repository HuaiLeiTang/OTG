#include "OTGAPI.h"

using namespace std;
using namespace irLib::irMath;


namespace irLib
{
	namespace irTG
	{
		/*
		* Trapazodal trajectory API class (OTGTrapTrajAPI)
		*/

		OTGTrapTrajAPI::OTGTrapTrajAPI(unsigned int dof) : _dof(dof)
		{
			_maxVelocity = new Real[_dof];
			_maxAcceleration = new Real[_dof];
			_numOfAddPoint = 0;
		}

		OTGTrapTrajAPI::~OTGTrapTrajAPI()
		{
			delete[] _maxVelocity;
			delete[] _maxAcceleration;
		}

		void OTGTrapTrajAPI::setKinematicConstraints(Real* maxVelocity, Real* maxAcceleration)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_maxVelocity[i] = maxVelocity[i];
				_maxAcceleration[i] = maxAcceleration[i];
				LOGIF(_maxVelocity[i] > 0, "maximum velocity must be larger than zero");
				LOGIF(_maxAcceleration[i] > 0, "maximum acceleration must be larger than zero");
			}
		}

		void OTGTrapTrajAPI::addPoint(Real* position, Real* velocity)
		{
			if (_numOfAddPoint == 1)
			{
				if (velocity == NULL)
					_TrapTrajs[0]->setTargetPosVelInfo(position);
				else
					_TrapTrajs[0]->setTargetPosVelInfo(position, velocity);

				_numOfAddPoint++;
				return;
			}

			OTGTrapTrajPtr TrapTraj = OTGTrapTrajPtr(new OTGTrapTraj(_dof));
			TrapTraj->setKinematicConstraints(_maxVelocity, _maxAcceleration);

			if (_numOfAddPoint == 0)
			{
				if (velocity == NULL)
					TrapTraj->setCurrentPosVelInfo(position);
				else
					TrapTraj->setCurrentPosVelInfo(position, velocity);
			}
			else
			{
				if (velocity == NULL)
					TrapTraj->setTargetPosVelInfo(position);
				else
					TrapTraj->setTargetPosVelInfo(position, velocity);

				if (_TrapTrajs[_TrapTrajs.size() - 1]->gettargetVelSetSwi())
					TrapTraj->setCurrentPosVelInfo(_TrapTrajs[_TrapTrajs.size() - 1]->getTargetposition(), _TrapTrajs[_TrapTrajs.size() - 1]->getTargetvelocity());
				else
				{
					Real* vel = new Real[_dof];
					Real* startpos = _TrapTrajs[_TrapTrajs.size() - 1]->getCurrentposition();
					Real* waypos = _TrapTrajs[_TrapTrajs.size() - 1]->getTargetposition();

					for (unsigned int i = 0; i < _dof; i++)
					{
						if (waypos[i] > startpos[i] && waypos[i] < position[i])
							vel[i] = _maxVelocity[i];
						else if (waypos[i] < startpos[i] && waypos[i] > position[i])
							vel[i] = -(_maxVelocity[i]);
						else
							vel[i] = 0;
					}

					_TrapTrajs[_TrapTrajs.size() - 1]->setTargetVelInfo(vel);
					TrapTraj->setCurrentPosVelInfo(_TrapTrajs[_TrapTrajs.size() - 1]->getTargetposition(), vel);
					delete[] vel;
				}
			}

			_TrapTrajs.push_back(TrapTraj);
			_numOfAddPoint++;
		}

		void OTGTrapTrajAPI::printAllPointInfo() const
		{
			for (unsigned int i = 0; i < _TrapTrajs.size(); i++)
			{
				if (i == 0)
					std::cout << "[Start point position & velocity info]" << std::endl;
				else if (i == _TrapTrajs.size() - 1)
					std::cout << "[" << i << "-th way point position & velocity info]" << std::endl;
				else
					std::cout << "[End point position & velocity info]" << std::endl;

				std::cout << "* current position : ";
				for (unsigned int j = 0; j < _dof; j++)
					std::cout << (_TrapTrajs[i]->getCurrentposition())[j] << '\t';
				std::cout << std::endl;
				std::cout << "* current velocity : ";
				for (unsigned int j = 0; j < _dof; j++)
					std::cout << (_TrapTrajs[i]->getCurrentvelocity())[j] << '\t';
				std::cout << std::endl;
				std::cout << "* target position : ";
				for (unsigned int j = 0; j < _dof; j++)
					std::cout << (_TrapTrajs[i]->getTargetposition())[j] << '\t';
				std::cout << std::endl;
				std::cout << "* target velocity : ";
				for (unsigned int j = 0; j < _dof; j++)
					std::cout << (_TrapTrajs[i]->getTargetvelocity())[j] << '\t';
				std::cout << std::endl;

				std::cout << std::endl;
			}
		}

		void OTGTrapTrajAPI::calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, unsigned int numOfdata)
		{
			unsigned int numOfTraj = 0, s = _TrapTrajs.size();
			unsigned int sizeOfTraj = numOfdata / s;
			Real t = 0;

			for (unsigned int i = 0; i < s; i++)
			{
				if (i == (s - 1))
					sizeOfTraj = numOfdata - sizeOfTraj * (s - 1);

				Real** posTraj_tmp = new Real*[_dof];
				Real** velTraj_tmp = new Real*[_dof];
				Real** accTraj_tmp = new Real*[_dof];

				for (unsigned int j = 0; j < _dof; j++)
				{
					posTraj_tmp[j] = new Real[sizeOfTraj];
					velTraj_tmp[j] = new Real[sizeOfTraj];
					accTraj_tmp[j] = new Real[sizeOfTraj];
				}

				_TrapTrajs[i]->settcurr(t);
				_TrapTrajs[i]->calculateJointTrajectory(posTraj_tmp, velTraj_tmp, accTraj_tmp, sizeOfTraj);
				t += _TrapTrajs[i]->gettsync();

				for (unsigned int k = 0; k < _dof; k++)
				{
					for (unsigned int l = 0; l < sizeOfTraj; l++)
					{
						posTraj[k][numOfTraj + l] = posTraj_tmp[k][l];
						velTraj[k][numOfTraj + l] = velTraj_tmp[k][l];
						accTraj[k][numOfTraj + l] = accTraj_tmp[k][l];
					}
				}

				numOfTraj += sizeOfTraj;

				for (unsigned int j = 0; j < _dof; j++)
				{
					delete[] posTraj_tmp[j];
					delete[] velTraj_tmp[j];
					delete[] accTraj_tmp[j];
				}
			}
		}

		/*
		* Trapazodal trajectory class (OTGTrapTraj)
		*/

		OTGTrapTraj::OTGTrapTraj(unsigned int dof) : _dof(dof), _tcurr(0.0), _targetVelSetSwi(true)
		{
			_profile = new Profile[_dof];
			for (unsigned int i = 0; i < _dof; i++)
				_profile[i].numOfPolynomial = 0;

			_maxVelocity = new Real[_dof];
			_maxAcceleration = new Real[_dof];

			_currentPosition = new Real[_dof];
			_currentVelocity = new Real[_dof];
			_targetPosition = new Real[_dof];
			_targetVelocity = new Real[_dof];
		}

		OTGTrapTraj::~OTGTrapTraj()
		{
			delete[] _profile;
			delete[] _maxVelocity;
			delete[] _maxAcceleration;

			delete[] _currentPosition;
			delete[] _currentVelocity;
			delete[] _targetPosition;
			delete[] _targetVelocity;
		}

		void OTGTrapTraj::setInputParameters(Real * maxVelocity, Real * maxAcceleration, Real * currentPosition, Real * currentVelocity, Real * targetPosition, Real * targetVelocity)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_maxVelocity[i] = maxVelocity[i];
				_maxAcceleration[i] = maxAcceleration[i];
				_currentPosition[i] = currentPosition[i];
				_currentVelocity[i] = currentVelocity[i];
				_targetPosition[i] = targetPosition[i];
				_targetVelocity[i] = targetVelocity[i];
				LOGIF(_maxVelocity[i] > 0, "maximum velocity must be larger than zero");
				LOGIF(_maxAcceleration[i] > 0, "maximum acceleration must be larger than zero");
				LOGIF(std::abs(_targetVelocity[i]) <= _maxVelocity[i], "absolute value of target velocity must be smaller than maximum velocity");
			}
		}

		void OTGTrapTraj::setKinematicConstraints(Real* maxVelocity, Real* maxAcceleration)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_maxVelocity[i] = maxVelocity[i];
				_maxAcceleration[i] = maxAcceleration[i];
				LOGIF(_maxVelocity[i] > 0, "maximum velocity must be larger than zero");
				LOGIF(_maxAcceleration[i] > 0, "maximum acceleration must be larger than zero");
			}
		}

		void OTGTrapTraj::setCurrentPosVelInfo(Real* currentPosition, Real* currentVelocity)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_currentPosition[i] = currentPosition[i];
				_currentVelocity[i] = currentVelocity[i];
			}
		}

		void OTGTrapTraj::setCurrentPosVelInfo(Real* currentPosition)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_currentPosition[i] = currentPosition[i];
				_currentVelocity[i] = 0;
			}
		}

		void OTGTrapTraj::setTargetPosVelInfo(Real* targetPosition, Real* targetVelocity)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_targetPosition[i] = targetPosition[i];
				_targetVelocity[i] = targetVelocity[i];
				LOGIF(std::abs(_targetVelocity[i]) <= _maxVelocity[i], "absolute value of target velocity must be smaller than maximum velocity");
			}
			_targetVelSetSwi = true;
		}

		void OTGTrapTraj::setTargetPosVelInfo(Real* targetPosition)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_targetPosition[i] = targetPosition[i];
				_targetVelocity[i] = 0;
			}
			_targetVelSetSwi = false;
		}

		void OTGTrapTraj::setTargetVelInfo(Real* targetVelocity)
		{
			for (unsigned int i = 0; i < _dof; i++)
			{
				_targetVelocity[i] = targetVelocity[i];
			}
		}

		void OTGTrapTraj::calculateSychronizeTime()
		{
			Real * tmin = new Real[_dof];
			Real * t1begin = new Real[_dof];
			Real * t1end = new Real[_dof];
			bool * inoperExist = new bool[_dof];

			for (unsigned int i = 0; i < _dof; i++)
			{
				TreeTrapTrajStep1* decisionTreeStep1 = makeDecisionTreeTrapTrajStep1(_maxVelocity[i], _maxAcceleration[i], _currentPosition[i], _currentVelocity[i],
					_targetPosition[i], _targetVelocity[i], 0);
				calculateMinimumTimeAndInoperTimeTrapTraj(decisionTreeStep1);

				tmin[i] = decisionTreeStep1->_tmin;
				t1begin[i] = decisionTreeStep1->_t1begin;
				t1end[i] = decisionTreeStep1->_t1end;
				inoperExist[i] = decisionTreeStep1->_inoperTimeExist;

				deleteDecisionTreeTrapTrajStep1(decisionTreeStep1);
			}

			// calculate _tsync
			_tsync = tmin[0];
			for (unsigned int i = 0; i < _dof; i++)
				if (tmin[i] > _tsync)
					_tsync = tmin[i];

			bool tsyncUpdated = false;
			Real tmpeps = 1e-4;
			while (true)
			{
				for (unsigned int i = 0; i < _dof; i++)
				{
					if (inoperExist[i])
					{
						if (RealBigger(_tsync, t1begin[i], tmpeps) && RealLess(_tsync, t1end[i], tmpeps))
						{
							_tsync = t1end[i];
							inoperExist[i] = false;
							tsyncUpdated = true;
							break;
						}
					}
				}

				if (!tsyncUpdated)
					break;
				tsyncUpdated = false;
			}

			delete[] tmin;
			delete[] t1begin;
			delete[] t1end;
			delete[] inoperExist;
		}

		void OTGTrapTraj::calculateFinalProfiles()
		{
			//Real _tcurr = 0.0;
			//cout << "_tcurr : " << _tcurr << endl;
			//cout << "_tsync : " << _tsync << endl;

			for (unsigned int i = 0; i < _dof; i++)
				_profile[i].numOfPolynomial = 0;

			for (unsigned int i = 0; i < _dof; i++)
			{
				TreeTrapTrajStep2* decisionTreeStep2 = makeDecisionTreeTrapTrajStep2(_maxVelocity[i], _maxAcceleration[i], _currentPosition[i], _currentVelocity[i],
					_targetPosition[i], _targetVelocity[i], _tcurr, _tsync + _tcurr);
				makeFinalProfilesTrapTraj(decisionTreeStep2, &(_profile[i]));
				deleteDecisionTreeTrapTrajStep2(decisionTreeStep2);
			}
		}

		void OTGTrapTraj::calculateJointTrajectory(Real** posTraj, Real dt)
		{
			unsigned int numOfdata = unsigned int(_tsync / dt) + 1;
			// TO DO
			return;
		}

		void OTGTrapTraj::calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, Real dt)
		{
			unsigned int numOfdata = unsigned int(_tsync / dt) + 1;
			// TO DO
			return;
		}

		void OTGTrapTraj::calculateJointTrajectory(Real** posTraj, unsigned int numOfdata)
		{
			calculateSychronizeTime();
			calculateFinalProfiles();



			Real t = 0.0;
			Real dt = _tsync / (Real)(numOfdata - 1);

			for (unsigned int idxdof = 0; idxdof < _dof; idxdof++)
			{
				for (unsigned int idxdata = 0; idxdata < numOfdata; idxdata++)
				{
					for (unsigned int idxpoly = 0; idxpoly < _profile[idxdof].numOfPolynomial; idxpoly++)
					{
						if (t <= _profile[idxdof].PolynomialTime[idxpoly])
						{
							posTraj[idxdof][idxdata] = _profile[idxdof].PositionProfile[idxpoly].CalculateValue(t);
							break;
						}
					}
					t += dt;
				}
			}
			return;
		}

		void OTGTrapTraj::calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, unsigned int numOfdata)
		{
			calculateSychronizeTime();
			calculateFinalProfiles();

			//cout << "_tcurr : " << _tcurr << endl;
			//cout << "_tsync : " << _tsync << endl;

			Real t = _tcurr;
			Real dt = _tsync / (Real)(numOfdata - 1);

			for (unsigned int idxdof = 0; idxdof < _dof; idxdof++)
			{
				t = _tcurr;
				for (unsigned int idxdata = 0; idxdata < numOfdata; idxdata++)
				{
					for (unsigned int idxpoly = 0; idxpoly < _profile[idxdof].numOfPolynomial; idxpoly++)
					{
						if (t <= _profile[idxdof].PolynomialTime[idxpoly])
						{
							posTraj[idxdof][idxdata] = _profile[idxdof].PositionProfile[idxpoly].CalculateValue(t);
							velTraj[idxdof][idxdata] = _profile[idxdof].VelocityProfile[idxpoly].CalculateValue(t);
							accTraj[idxdof][idxdata] = _profile[idxdof].AccelerationProfile[idxpoly].CalculateValue(t);
							break;
						}
					}
					t += dt;
				}
			}
			return;
		}


		/*
		* S-curve class (OTGCurve)
		*/

		OTGSCurve::OTGSCurve(unsigned int dof) : _dof(dof)
		{
			_profile = new Profile[_dof];
			for (unsigned int i = 0; i < _dof; i++)
				_profile[i].numOfPolynomial = 0;
		}

		void OTGSCurve::setInputParameters(Real* maxVelocity, Real* maxAcceleration, Real* maxJerk, Real* currentPosition,
			Real* currentVelocity, Real* currentAcceleration, Real* targetPosition, Real* targetVelocity)
		{
			_maxVelocity = maxVelocity;
			_maxAcceleration = maxAcceleration;
			_maxJerk = maxJerk;
			_currentPosition = currentPosition;
			_currentVelocity = currentVelocity;
			_currentAcceleration = currentAcceleration;
			_targetPosition = targetPosition;
			_targetVelocity = targetVelocity;

			for (unsigned int i = 0; i < _dof; i++)
			{
				// feasibility check
				LOGIF(_maxVelocity[i] > 0, "maximum velocity must be larger than zero");
				LOGIF(_maxAcceleration[i] > 0, "maximum acceleration must be larger than zero");
				LOGIF(_maxJerk[i] > 0, "maximum jerk must be larger than zero");
				LOGIF(std::abs(_targetVelocity[i]) <= _maxVelocity[i], "absolute value of target velocity must be smaller than maximum velocity");
			}
		}

		void OTGSCurve::calculateSychronizeTime()
		{
			Real * tmin = new Real[_dof];
			Real * t1begin = new Real[_dof];
			Real * t1end = new Real[_dof];
			bool * inoperExist = new bool[_dof];

			for (unsigned int i = 0; i < _dof; i++)
			{
				TreeSCurveStep1* decisionTreeStep1 = makeDecisionTreeSCurveStep1(_maxVelocity[i], _maxAcceleration[i], _maxJerk[i], _currentPosition[i], _currentVelocity[i],
					_currentAcceleration[i], _targetPosition[i], _targetVelocity[i], 0);
				calculateMinimumTimeAndInoperTimeSCurve(decisionTreeStep1);

				tmin[i] = decisionTreeStep1->_tmin;
				t1begin[i] = decisionTreeStep1->_t1begin;
				t1end[i] = decisionTreeStep1->_t1end;
				inoperExist[i] = decisionTreeStep1->_inoperTimeExist;

				deleteDecisionTreeSCurveStep1(decisionTreeStep1);
			}

			cout << "minimum time for each dof" << endl;
			for (unsigned int i = 0; i < _dof; i++)
				cout << tmin[i] << '\t';
			cout << endl;

			// calculate _tsync
			_tsync = tmin[0];
			for (unsigned int i = 0; i < _dof; i++)
				if (tmin[i] > _tsync)
					_tsync = tmin[i];

			bool tsyncUpdated = false;
			Real tmpeps = 1e-4;
			while (true)
			{
				for (unsigned int i = 0; i < _dof; i++)
				{
					if (inoperExist[i])
					{
						if (RealBigger(_tsync, t1begin[i], tmpeps) && RealLess(_tsync, t1end[i], tmpeps))
						{
							_tsync = t1end[i];
							inoperExist[i] = false;
							tsyncUpdated = true;
							break;
						}
					}
				}

				if (!tsyncUpdated)
					break;
				tsyncUpdated = false;
			}

			delete[] tmin;
			delete[] t1begin;
			delete[] t1end;
			delete[] inoperExist;
		}

		void OTGSCurve::calculateFinalProfiles()
		{
			Real _tcurr = 0.0;
			for (unsigned int i = 0; i < _dof; i++)
				_profile[i].numOfPolynomial = 0;

			for (unsigned int i = 0; i < _dof; i++)
			{
				TreeSCurveStep2* decisionTreeStep2 = makeDecisionTreeSCurveStep2(_maxVelocity[i], _maxAcceleration[i], _maxJerk[i], _currentPosition[i], _currentVelocity[i],
					_currentAcceleration[i], _targetPosition[i], _targetVelocity[i], _tcurr, _tsync);
				makeFinalProfilesSCurve(decisionTreeStep2, &(_profile[i]));
				deleteDecisionTreeSCurveStep2(decisionTreeStep2);
			}
		}

		void OTGSCurve::calculateJointTrajectory(Real** posTraj, Real dt)
		{
			unsigned int numOfdata = unsigned int(_tsync / dt) + 1;
			// TO DO
			return;
		}

		void OTGSCurve::calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, Real dt)
		{
			unsigned int numOfdata = unsigned int(_tsync / dt) + 1;
			// TO DO
			return;
		}

		void OTGSCurve::calculateJointTrajectory(Real** posTraj, unsigned int numOfdata)
		{
			calculateSychronizeTime();
			calculateFinalProfiles();

			Real t = 0.0;
			Real dt = _tsync / (Real)(numOfdata - 1);

			for (unsigned int idxdof = 0; idxdof < _dof; idxdof++)
			{
				for (unsigned int idxdata = 0; idxdata < numOfdata; idxdata++)
				{
					for (unsigned int idxpoly = 0; idxpoly < _profile[idxdof].numOfPolynomial; idxpoly++)
					{
						if (t <= _profile[idxdof].PolynomialTime[idxpoly])
						{
							posTraj[idxdof][idxdata] = _profile[idxdof].PositionProfile[idxpoly].CalculateValue(t);
							break;
						}
					}
					t += dt;
				}
			}
			return;
		}

		void OTGSCurve::calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, Real** jerkTraj, unsigned int numOfdata)
		{
			calculateSychronizeTime();
			cout << endl << "t sync: " << _tsync << endl;
			calculateFinalProfiles();

			Real t = 0.0;
			Real dt = _tsync / (Real)(numOfdata - 1);

			for (unsigned int idxdof = 0; idxdof < _dof; idxdof++)
			{
				t = 0.0;
				for (unsigned int idxdata = 0; idxdata < numOfdata; idxdata++)
				{
					for (unsigned int idxpoly = 0; idxpoly < _profile[idxdof].numOfPolynomial; idxpoly++)
					{
						if (t <= _profile[idxdof].PolynomialTime[idxpoly])
						{
							posTraj[idxdof][idxdata] = _profile[idxdof].PositionProfile[idxpoly].CalculateValue(t);
							velTraj[idxdof][idxdata] = _profile[idxdof].VelocityProfile[idxpoly].CalculateValue(t);
							accTraj[idxdof][idxdata] = _profile[idxdof].AccelerationProfile[idxpoly].CalculateValue(t);
							jerkTraj[idxdof][idxdata] = _profile[idxdof].JerkProfile[idxpoly].CalculateValue(t);
							break;
						}
					}
					t += dt;
				}
			}
			return;
		}



	}
}