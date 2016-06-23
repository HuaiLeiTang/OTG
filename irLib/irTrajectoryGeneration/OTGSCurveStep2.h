#pragma once

#include <irUtils\Diagnostic.h>
#include <irMath\Constant.h>
#include <irMath\Common.h>
#include "OTGOperator.h"
#include "OTGProfile.h"
#include "irMath\NonlinearOptimization.h"

namespace irLib
{
	namespace irTG
	{
		typedef struct _treeSCurveStep2
		{
			Real _maxVelocity;
			Real _maxAcceleration;
			Real _maxJerk;

			Real _currentPosition;
			Real _currentVelocity;
			Real _currentAcceleration;

			Real _targetPosition;
			Real _targetVelocity;

			Real _tcurr;	// current time
			Real _tsync;	// synchronized time calculated in step1	
		} TreeSCurveStep2;

		enum SCurveStep2_Decision
		{
			SCurveStep2_Decision_1 = 1, SCurveStep2_Decision_2 = 2, SCurveStep2_Decision_3 = 3,
		};

		TreeSCurveStep2* makeDecisionTreeSCurveStep2(const Real maxVelocity, const Real maxAcceleration, const Real maxJerk,
			const Real currentPosition, const Real currentVelocity, const Real currentAcceleration,
			const Real targetPosition, const Real targetVelocity, const Real tcurr, const Real tsync);
		void deleteDecisionTreeSCurveStep2(TreeSCurveStep2* decisionTreeStep2);

		// main function..
		void makeFinalProfilesSCurve(TreeSCurveStep2* decisionTreeStep2, Profile* profile);

		// assistant function
		void ReverseSign_SCurveStep2(TreeSCurveStep2* dtStep2);

		// decision boolean function (comments are inside function definition, .cpp file)
		bool Decision1_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision2_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision3_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision4_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision5_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision6_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision7_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision8_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision9_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision10_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision11_SCurveStep2(TreeSCurveStep2* dtStep2);
		bool Decision12_SCurveStep2(TreeSCurveStep2* dtStep2);

		// Profile calculation function
		// 수정해야되는 부분 : case 1, 8의 경우 numerical 하게 풀기때문에 이상한 해가 나왔을 경우 다시 풀어주는 루틴 추가해줘야함
		void calculateProfile_PosTriZeroNegTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 1) numerical solution 
		void calculateProfile_PosTrapZeroNegTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 2) closed-form solution 
		void calculateProfile_PosTrapZeroNegTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 3) closed-form solution 
		void calculateProfile_PosTriZeroNegTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 4) closed-form solution
		void calculateProfile_PosTrapZeroPosTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 5) closed-form solution
		void calculateProfile_PosTrapZeroPosTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 6) closed-form solution
		void calculateProfile_PosTriZeroPosTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 7) closed-form solution
		void calculateProfile_PosTriZeroPosTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/); // (case 8) numerical solution 


		// functions for numerical process
		void PosTriZeroNegTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data); // case 1
		void PosTriZeroPosTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data); // case 8

		void substitutefcn(TreeSCurveStep2* decisionTreeStep2, Real& tsyn, Real& a0, Real& vt, Real& v0, Real& pt, Real& p0, Real& jmax, Real& amax, Real& vmax);

		void calcProfile_Slash(TreeSCurveStep2* decisionTreeStep2, Profile* profile, const Real nAcc, const Real t);
		void calcProfile_BackSlash(TreeSCurveStep2* decisionTreeStep2, Profile* profile, const Real nAcc, const Real t);
		void calcProfile_Rectangular(TreeSCurveStep2* decisionTreeStep2, Profile* profile, const Real nAcc, const Real t);
		void calcProfile_Aftertsync(TreeSCurveStep2* decisionTreeStep2, Profile* profile);
	}
}