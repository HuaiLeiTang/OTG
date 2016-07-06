#pragma once

#include <iostream>
#include <irUtils\Diagnostic.h>
#include <irMath\Constant.h>
#include <irMath\Common.h>
#include "OTGOperator.h"
#include "irMath\NonlinearOptimization.h"

using namespace std;
using namespace irLib::irMath;

namespace irLib
{
	namespace irTG
	{
		enum ProfileSCurveStep1
		{
			profile_notAssigned = 0,
			profile_PosTriNegTri,
			profile_PosTriZeroNegTri,
			profile_PosTrapNegTri,
			profile_PosTrapZeroNegTri,
			profile_PosTrapNegTrap,
			profile_PosTrapZeroNegTrap,
			profile_PosTriNegTrap,
			profile_PosTriZeroNegTrap,
			profile_NegTriPosTri,
			profile_NegTriZeroPosTri,
			profile_NegTrapPosTri,
			profile_NegTrapZeroPosTri,
			profile_NegTrapPosTrap,
			profile_NegTrapZeroPosTrap,
			profile_NegTriPosTrap,
			profile_NegTriZeroPosTrap,
			// neg도 있어야 inoperative 거기서 구분할 수 있을 듯...
		};

		typedef struct _treeSCurveStep1
		{
			Real _maxVelocity;
			Real _maxAcceleration;
			Real _maxJerk;

			Real _currentPosition;
			Real _currentVelocity;
			Real _currentAcceleration;

			Real _targetPosition;
			Real _targetVelocity;

			Real _tcurr;								// current time
			Real _tcurrNotChanging;						// current time never changed after first setting (for the function 'FromZeroToA_SCurveStep1': can be removed if not neddend)
			Real _tmin;									// minimum reachable time
			bool _bReversed;

			ProfileSCurveStep1	_minProfile;		// min time profile for tmin
			ProfileSCurveStep1	_inopProfileBegin;	// inoperative time profile for t1begin
			ProfileSCurveStep1	_inopProfileEnd;		// inoperative time profile for t1bend
			Real _t1begin;								// begin time of inopertative interval if it exists
			Real _t1end;								// end time of inopertative interval if it exists
			bool _inoperTimeExist;						// bool: inoperative time interval exist
		} TreeSCurveStep1;

		TreeSCurveStep1* makeDecisionTreeSCurveStep1(const Real maxVelocity, const Real maxAcceleration, const Real maxJerk,
			const Real currentPosition, const Real currentVelocity, const Real currentAcceleration,
			const Real targetPosition, const Real targetVelocity, const Real tcurr);
		void deleteDecisionTreeSCurveStep1(TreeSCurveStep1* decisionTreeStep1);

		// main function..
		void calculateMinimumTimeAndInoperTimeSCurve(TreeSCurveStep1* decisionTreeStep1);


		// main fuction for minimum time ( called inside the function 'calculateMinimumTimeAndInoperTimeSCurve' )
		void calculateMiniumumTimeSCurve(TreeSCurveStep1* decisionTreeStep1);

		// calculate minimum time and save at _tmin
		void calculateTime_PosTriNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // numerical solution (case 1)
		void calculateTime_PosTriZeroNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 2)
		void calculateTime_PosTrapNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 3)
		void calculateTime_PosTrapZeroNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 4)
		void calculateTime_PosTrapNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 5)
		void calculateTime_PosTrapZeroNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 6)
		void calculateTime_PosTriNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 7)
		void calculateTime_PosTriZeroNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime); // closed-form solution (case 8)
		

		// main function for inoperative time interval ( called inside the function 'calculateMinimumTimeAndInoperTimeSCurve' )
		void calculateInoperTimeSCurve(TreeSCurveStep1* decisionTreeStep1);

		// 4 cases for inoperative time interval
		void SetProfile1_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		void SetProfile2_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		void SetProfile3_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		void SetProfile4_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);

		// calculate time by profiles were set
		void calcInopTimeProfile_SCurveStep1(TreeSCurveStep1* decisionTreeStep1);

		// assistant function
		void FromAToZero_SCurveStep1(TreeSCurveStep1* dtStep1);
		void FromZeroToA_SCurveStep1(TreeSCurveStep1* dtStep1); // restore function of 'FromAToZero_SCurveStep1'
		void ReverseSign_SCurveStep1(TreeSCurveStep1* dtStep1);

		// decision boolean function (comments are inside function definition, .cpp file)
		bool Decision1_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision2_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision3_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision4_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision5_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision6_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision7_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision8_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision9_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision10_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision11_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision12_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision13_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision14_SCurveStep1(TreeSCurveStep1* dtStep1);
		// AT THE PAPER FROM 1 TO 14
		bool Decision15_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision16_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision17_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision18_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision19_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision20_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision21_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision22_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision23_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision24_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision25_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision26_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision27_SCurveStep1(TreeSCurveStep1* dtStep1);
		// implemented at 160702 - from 28 to 43
		bool Decision28_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision29_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision30_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision31_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision32_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision33_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision34_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision35_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision36_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision37_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision38_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision39_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision40_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision41_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision42_SCurveStep1(TreeSCurveStep1* dtStep1);
		bool Decision43_SCurveStep1(TreeSCurveStep1* dtStep1);


		// decision for inoperative time interval
		bool Decision0_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision1_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision2_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision3_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision4_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision5_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision6_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);
		bool Decision7_SCurveStep1InopTime(TreeSCurveStep1* dtStep1);



		// functions for numerical process
		void PosTriNegTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data); // case 1
		//void PosTrapNegTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data); // case 3

		void substitutefcn(TreeSCurveStep1* decisionTreeStep1, Real& a0, Real& vt, Real& v0, Real& pt, Real& p0, Real& jmax, Real& amax, Real& vmax);

	}
}