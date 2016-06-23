#pragma once


#include <irUtils\Diagnostic.h>
#include <irMath\Constant.h>
#include "OTGOperator.h"

using namespace std;
using namespace irLib::irMath;

namespace irLib
{
	namespace irTG
	{
		typedef struct _treeTrapTrajStep1
		{
			Real _maxVelocity;
			Real _maxAcceleration;

			Real _currentPosition;
			Real _currentVelocity;
			Real _targetPosition;
			Real _targetVelocity;

			Real _tcurr;			// current time
			Real _tmin;				// minimum reachable time
			Real _t1begin;			// begin time of inopertative interval if it exists
			Real _t1end;			// end time of inopertative interval if it exists
			bool _inoperTimeExist;	// bool: inoperative time interval exist

		} TreeTrapTrajStep1;


		TreeTrapTrajStep1* makeDecisionTreeTrapTrajStep1(const Real maxVelocity, const Real maxAcceleration, const Real currentPosition, const Real currentVelocity,
			const Real targetPosition, const Real targetVelocity, const Real tcurr);
		void deleteDecisionTreeTrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);

		// main function..
		void calculateMinimumTimeAndInoperTimeTrapTraj(TreeTrapTrajStep1* decisionTreeStep1);
		
		// cases which are divided by comparing values of Vm, Vt, V0, and zero
		void Vm_Vt_V0_Zero_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		void Vm_V0_Vt_Zero_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		void Vm_V0_Zero_Vt_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);

		// calculate minimum time and save at _tmin
		void calculateTime_PosLinNegLin_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);		// reversed v-shape
		void calculateTime_PosLinHldNegLin_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);		// trapezoidal shape
		void calculateTime_NegLinPosLin_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);		// v-shape
		void calculateTime_NegLinHldPosLin_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);		// reversed trapezoidal shape

		// assistant function
		void FromVToVmax_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		void ReverseSign_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);

		// decision boolean function (comments are inside function definition, .cpp file)
		bool Decision1_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		bool Decision2_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		bool Decision3_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		bool Decision4_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		bool Decision5_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
		bool Decision6_TrapTrajStep1(TreeTrapTrajStep1* decisionTreeStep1);
	}
}