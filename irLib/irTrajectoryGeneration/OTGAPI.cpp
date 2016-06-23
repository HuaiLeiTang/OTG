#include "OTGAPI.h"

namespace irLib
{
	namespace irTG
	{
		/*
		 * Trapazodal trajectory class functions
		*/

		OTGTrapTraj::OTGTrapTraj(unsigned int dof) : _dof(dof)
		{
			_profile = new Profile[_dof];
			for (unsigned int i = 0; i < _dof; i++)
				_profile[i].numOfPolynomial = 0;
		}

		OTGTrapTraj::~OTGTrapTraj(){}

		void OTGTrapTraj::setInputParameters(Real * maxVelocity, Real * maxAcceleration, Real * currentPosition, Real * currentVelocity, Real * targetPosition, Real * targetVelocity)
		{
			_maxVelocity = maxVelocity;
			_maxAcceleration = maxAcceleration;
			_currentPosition = currentPosition;
			_currentVelocity = currentVelocity;
			_targetPosition = targetPosition;
			_targetVelocity = targetVelocity;

			for (unsigned int i = 0; i < _dof; i++)
			{
				// feasibility check
				LOGIF(_maxVelocity[i] > 0, "maximum velocity must be larger than zero");
				LOGIF(_maxAcceleration[i] > 0, "maximum acceleration must be larger than zero");
				LOGIF(std::abs(_targetVelocity[i]) <= _maxVelocity[i], "absolute value of target velocity must be smaller than maximum velocity");
			}
		}

		void OTGTrapTraj::calculateSychronizeTime()
		{
			Real * tmin		= new Real[_dof];
			Real * t1begin	= new Real[_dof];
			Real * t1end	= new Real[_dof];
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
			Real _tcurr = 0.0;
			for (unsigned int i = 0; i < _dof; i++)
				_profile[i].numOfPolynomial = 0;
				
			for (unsigned int i = 0; i < _dof; i++)
			{
				TreeTrapTrajStep2* decisionTreeStep2 = makeDecisionTreeTrapTrajStep2(_maxVelocity[i], _maxAcceleration[i], _currentPosition[i], _currentVelocity[i],
					_targetPosition[i], _targetVelocity[i], _tcurr, _tsync);
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

			Real t = 0.0;
			Real dt = _tsync / (Real)(numOfdata-1);

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
							break;
						}
					}
					t += dt;
				}
			}
			return;
		}

		/*
		* S-curve trajectory functions
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