#pragma once

#include "OTGProfile.h"

namespace irLib
{
	namespace irTG
	{
		typedef struct _treeTrapTrajStep2
		{
			Real _maxVelocity;
			Real _maxAcceleration;

			Real _currentPosition;
			Real _currentVelocity;
			Real _targetPosition;
			Real _targetVelocity;

			Real _tcurr;			// current time
			Real _tsync;			// synchronized time calculated in step1
		} TreeTrapTrajStep2;

		enum TrapTrajStep2_Decision
		{
			TrapTrajStep2_Decision_1 = 1, TrapTrajStep2_Decision_2 = 2, TrapTrajStep2_Decision_3 = 3, TrapTrajStep2_Decision_4 = 4,
			TrapTrajStep2_Decision_5 = 5, TrapTrajStep2_Decision_6 = 6, TrapTrajStep2_Decision_7 = 7, TrapTrajStep2_Decision_8 = 8,
			TrapTrajStep2_Decision_9 = 9, TrapTrajStep2_Decision_10 = 10, TrapTrajStep2_Decision_11 = 11, TrapTrajStep2_Decision_12 = 12,
		};
		
		TreeTrapTrajStep2* makeDecisionTreeTrapTrajStep2(const Real maxVelocity, const Real maxAcceleration, const Real currentPosition, const Real currentVelocity,
			const Real targetPosition, const Real targetVelocity, const Real tcurr, const Real tsync);
		void deleteDecisionTreeTrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);

		// Main function
		void makeFinalProfilesTrapTraj(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile);

		// Profile calculation function
		void calculateProfile_PosLinHldNegLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse);
		void calculateProfile_PosLinHldPosLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse);
		void calculateProfile_NegLinHldPosLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse);
		void calculateProfile_NegLinHldNegLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse);

		// Assistant function
		void FromVToVmax_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse);
		void FromVToZero_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse);
		void ReverseSign_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, bool* reverse);

		// Decision boolean function
		bool Decision1_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision2_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision3_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision4_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision5_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision6_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision7_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision8_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision9_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision10_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision11_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
		bool Decision12_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2);
	}
}