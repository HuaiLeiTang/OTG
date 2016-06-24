#pragma once

#include <iostream>
#include <irUtils\Diagnostic.h>
#include <irMath\Constant.h>
#include <irMath\Common.h>

#include "OTGProfile.h"
#include "OTGTrapTrajStep1.h"
#include "OTGTrapTrajStep2.h"
#include "OTGSCurveStep1.h"
#include "OTGSCurveStep2.h"

namespace irLib
{
	namespace irTG
	{
		class OTGTrapTraj;
		class OTGTrapTrajAPI;

		class OTGSCurve;

		typedef std::shared_ptr<OTGTrapTraj> OTGTrapTrajPtr;

		class OTGTrapTrajAPI
		{
		public:
			OTGTrapTrajAPI(unsigned int dof);
			~OTGTrapTrajAPI();
			void setKinematicConstraints(Real* _maxVelocity, Real* maxAcceleration);
			void addPoint(Real* position, Real* velocity = NULL);
			void printAllPointInfo() const;

			void calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, unsigned int numOfdata);

		private:
			unsigned int _dof;
			unsigned int _numOfAddPoint;

			Real* _maxVelocity;
			Real* _maxAcceleration;

			std::vector<OTGTrapTrajPtr> _TrapTrajs;
		};

		class OTGTrapTraj
		{
		public:
			OTGTrapTraj(unsigned int dof);
			~OTGTrapTraj();

			void setKinematicConstraints(Real* maxVelocity, Real* maxAcceleration);
			void setCurrentPosVelInfo(Real* currentPosition);
			void setCurrentPosVelInfo(Real* currentPosition, Real* currentVelocity);
			void setTargetPosVelInfo(Real* targetPosition, Real* targetVelocity);
			void setTargetVelInfo(Real* targetVelocity);
			void setTargetPosVelInfo(Real* targetPosition);
			void setInputParameters(Real* maxVelocity, Real* maxAcceleration, Real* currentPosition,
				Real* currentVelocity, Real* targetPosition, Real* targetVelocity);
			void settcurr(Real tcurr) { _tcurr = tcurr; }

			/*
			* Step 1 : Calculate the synchronization time
			* input variable : maxVelocity, maxAcceleration, currentPosition, currentVelocity,
			*					targetPosition, targetVelocity
			* output variable : tsync (save at _tsync)
			*/
			void calculateSychronizeTime();

			/*
			* Step 2 : Calculate final profiles of each joint
			* input variable : maxVelocity, maxAcceleration, currentPosition, currentVelocity,
			*					targetPosition, targetVelocity, tsync
			* output variable : final profile(trajectory) of each joint
			*/
			void calculateFinalProfiles();

			/*
			* Step 3 : Get the joint trajectories of each joint with respect to the time from each profiles
			* input variable : profiles
			* output variable : joint trajectories of each joint with respect to the time
			*/
			void calculateJointTrajectory(Real** posTraj, Real dt);
			void calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, Real dt);
			void calculateJointTrajectory(Real** posTraj, unsigned int numOfdata);
			void calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, unsigned int numOfdata);

			const Real gettsync() const { return _tsync; }
			const bool gettargetVelSetSwi() const { return _targetVelSetSwi; }
			Real* getTargetposition() const { return _targetPosition; }
			Real* getTargetvelocity() const { return _targetVelocity; }
			Real* getCurrentposition() const { return _currentPosition; }
			Real* getCurrentvelocity() const { return _currentVelocity; }

		private:
			unsigned int _dof;
			Profile* _profile;

			Real* _maxVelocity;
			Real* _maxAcceleration;

			Real* _currentPosition;
			Real* _currentVelocity;
			Real* _targetPosition;
			Real* _targetVelocity;

			Real _tsync;
			Real _tcurr;

			bool _targetVelSetSwi;
		};




		class OTGSCurve
		{
		private:
			unsigned int _dof;
			Profile* _profile;

			Real* _maxVelocity;
			Real* _maxAcceleration;
			Real* _maxJerk;

			Real* _currentPosition;
			Real* _currentVelocity;
			Real* _currentAcceleration;

			Real* _targetPosition;
			Real* _targetVelocity;

			Real _tsync;
		public:
			OTGSCurve(unsigned int dof);
			~OTGSCurve() {}

			void setInputParameters(Real* maxVelocity, Real* maxAcceleration, Real* maxJerk, Real* currentPosition,
				Real* currentVelocity, Real* currentAcceleration, Real* targetPosition, Real* targetVelocity);

			/*
			* Step 1 : Calculate the synchronization time
			* input variable : maxVelocity, maxAcceleration, currentPosition, currentVelocity,
			*					targetPosition, targetVelocity
			* output variable : tsync (save at _tsync)
			*/
			void calculateSychronizeTime();

			/*
			* Step 2 : Calculate final profiles of each joint
			* input variable : maxVelocity, maxAcceleration, currentPosition, currentVelocity,
			*					targetPosition, targetVelocity, tsync
			* output variable : final profile(trajectory) of each joint
			*/
			void calculateFinalProfiles();

			/*
			* Step 3 : Get the joint trajectories of each joint with respect to the time from each profiles
			* input variable : profiles
			* output variable : joint trajectories of each joint with respect to the time
			*/
			void calculateJointTrajectory(Real** posTraj, Real dt);
			void calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, Real dt);
			void calculateJointTrajectory(Real** posTraj, unsigned int numOfdata);
			void calculateJointTrajectory(Real** posTraj, Real** velTraj, Real** accTraj, Real** jerkTraj, unsigned int numOfdata);

			const Real gettsync() const { return _tsync; }
		};

	}

}