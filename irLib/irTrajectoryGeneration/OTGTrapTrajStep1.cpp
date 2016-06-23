#include "OTGTrapTrajStep1.h"

namespace irLib
{
	namespace irTG
	{
		TreeTrapTrajStep1 * makeDecisionTreeTrapTrajStep1(const Real maxVelocity, const Real maxAcceleration, const Real currentPosition, const Real currentVelocity, const Real targetPosition, const Real targetVelocity, const Real tcurr)
		{
			TreeTrapTrajStep1* tree = new TreeTrapTrajStep1;
			tree->_maxVelocity = maxVelocity;
			tree->_maxAcceleration = maxAcceleration;
			tree->_currentPosition = currentPosition;
			tree->_currentVelocity = currentVelocity;
			tree->_targetPosition = targetPosition;
			tree->_targetVelocity = targetVelocity;
			tree->_tcurr = tcurr;

			tree->_tmin = RealMax;
			tree->_t1begin = RealMax;
			tree->_t1end = RealMax;
			tree->_inoperTimeExist = false;

			return tree;
		}

		void deleteDecisionTreeTrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			delete decisionTreeStep1;
		}

		void calculateMinimumTimeAndInoperTimeTrapTraj(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// if V0 is negative, reverse all component and then calculate same as positive V0 case
			if (decisionTreeStep1->_currentVelocity < 0)
				ReverseSign_TrapTrajStep1(decisionTreeStep1);

			// if V0 is larger than Vmax, force V to Vmax with -accmax
			if (decisionTreeStep1->_currentVelocity >= decisionTreeStep1->_maxVelocity)
				FromVToVmax_TrapTrajStep1(decisionTreeStep1);


			if (decisionTreeStep1->_targetVelocity >= decisionTreeStep1->_currentVelocity)
				Vm_Vt_V0_Zero_TrapTrajStep1(decisionTreeStep1); // case: Vmax >= Vtarget >= V0 >= 0
			else if (decisionTreeStep1->_targetVelocity >= 0)
				Vm_V0_Vt_Zero_TrapTrajStep1(decisionTreeStep1); // case: Vmax >= V0 >= Vtarget >= 0
			else
				Vm_V0_Zero_Vt_TrapTrajStep1(decisionTreeStep1); // case: Vmax >= V0 >= 0 >= Vtarget
		}
		

		void Vm_Vt_V0_Zero_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			if (Decision1_TrapTrajStep1(decisionTreeStep1))
			{
				if (Decision3_TrapTrajStep1(decisionTreeStep1))
					calculateTime_PosLinHldNegLin_TrapTrajStep1(decisionTreeStep1);
				else
					calculateTime_PosLinNegLin_TrapTrajStep1(decisionTreeStep1);

				// calculate inoperative time interval
				if (!Decision5_TrapTrajStep1(decisionTreeStep1))
				{
					decisionTreeStep1->_inoperTimeExist = true;
					decisionTreeStep1->_t1begin = decisionTreeStep1->_tcurr + (decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity - sqrt(2) * sqrt(decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_currentPosition - decisionTreeStep1->_targetPosition))) / decisionTreeStep1->_maxAcceleration;
					decisionTreeStep1->_t1end = decisionTreeStep1->_tcurr + (decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity + sqrt(2)*sqrt(decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_currentPosition - decisionTreeStep1->_targetPosition))) / decisionTreeStep1->_maxAcceleration;
				}
			}
			else
			{
				if (Decision4_TrapTrajStep1(decisionTreeStep1))
					calculateTime_NegLinPosLin_TrapTrajStep1(decisionTreeStep1);
				else
					calculateTime_NegLinHldPosLin_TrapTrajStep1(decisionTreeStep1);
			}
		}

		void Vm_V0_Vt_Zero_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			if (Decision2_TrapTrajStep1(decisionTreeStep1))
			{
				if (Decision4_TrapTrajStep1(decisionTreeStep1))
					calculateTime_NegLinPosLin_TrapTrajStep1(decisionTreeStep1);
				else
					calculateTime_NegLinHldPosLin_TrapTrajStep1(decisionTreeStep1);
			}
			else
			{
				if (Decision3_TrapTrajStep1(decisionTreeStep1))
					calculateTime_PosLinHldNegLin_TrapTrajStep1(decisionTreeStep1);
				else
					calculateTime_PosLinNegLin_TrapTrajStep1(decisionTreeStep1);

				// calculate inoperative time interval
				if (!Decision5_TrapTrajStep1(decisionTreeStep1))
				{
					decisionTreeStep1->_inoperTimeExist = true;
					decisionTreeStep1->_t1begin = decisionTreeStep1->_tcurr + (decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity - sqrt(2) * sqrt(decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_currentPosition - decisionTreeStep1->_targetPosition))) / decisionTreeStep1->_maxAcceleration;
					decisionTreeStep1->_t1end = decisionTreeStep1->_tcurr + (decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity + sqrt(2)*sqrt(decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_currentPosition - decisionTreeStep1->_targetPosition))) / decisionTreeStep1->_maxAcceleration;
				}
			}
		}

		void Vm_V0_Zero_Vt_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			if (Decision6_TrapTrajStep1(decisionTreeStep1))
				if (Decision4_TrapTrajStep1(decisionTreeStep1))
					calculateTime_NegLinPosLin_TrapTrajStep1(decisionTreeStep1);
				else
					calculateTime_NegLinHldPosLin_TrapTrajStep1(decisionTreeStep1);
			else
				if (Decision3_TrapTrajStep1(decisionTreeStep1))
					calculateTime_PosLinHldNegLin_TrapTrajStep1(decisionTreeStep1);
				else
					calculateTime_PosLinNegLin_TrapTrajStep1(decisionTreeStep1);
		}


		void calculateTime_PosLinNegLin_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			decisionTreeStep1->_tmin = decisionTreeStep1->_tcurr +
				(sqrt(2) * sqrt(decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_targetPosition - decisionTreeStep1->_currentPosition)) - decisionTreeStep1->_currentVelocity - decisionTreeStep1->_targetVelocity) / decisionTreeStep1->_maxAcceleration;
		}	 
		void calculateTime_PosLinHldNegLin_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			decisionTreeStep1->_tmin = decisionTreeStep1->_tcurr +
				((decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity - 2 * decisionTreeStep1->_maxVelocity*decisionTreeStep1->_currentVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_targetPosition - decisionTreeStep1->_currentPosition)) + 2 * decisionTreeStep1->_maxVelocity*decisionTreeStep1->_maxVelocity - 2 * decisionTreeStep1->_maxVelocity*decisionTreeStep1->_targetVelocity) / (2 * decisionTreeStep1->_maxAcceleration*decisionTreeStep1->_maxVelocity);
		}	 
		void calculateTime_NegLinPosLin_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			decisionTreeStep1->_tmin = decisionTreeStep1->_tcurr +
				(decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity + sqrt(2)*sqrt(decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_currentPosition - decisionTreeStep1->_targetPosition))) / decisionTreeStep1->_maxAcceleration;
		}
		void calculateTime_NegLinHldPosLin_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			decisionTreeStep1->_tmin = decisionTreeStep1->_tcurr +
				((decisionTreeStep1->_currentVelocity*decisionTreeStep1->_currentVelocity + decisionTreeStep1->_targetVelocity*decisionTreeStep1->_targetVelocity + 2 * decisionTreeStep1->_currentVelocity*decisionTreeStep1->_maxVelocity + 2 * decisionTreeStep1->_maxAcceleration*(decisionTreeStep1->_currentPosition - decisionTreeStep1->_targetPosition)) + 2 * decisionTreeStep1->_maxVelocity*decisionTreeStep1->_maxVelocity + 2 * decisionTreeStep1->_maxVelocity*decisionTreeStep1->_targetVelocity) / (2 * decisionTreeStep1->_maxAcceleration*decisionTreeStep1->_maxVelocity);
		}


		void FromVToVmax_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			decisionTreeStep1->_tcurr += (decisionTreeStep1->_currentVelocity - decisionTreeStep1->_maxVelocity) / decisionTreeStep1->_maxAcceleration;
			decisionTreeStep1->_currentPosition += calcDP_ConstAccWithVf(decisionTreeStep1->_currentVelocity, decisionTreeStep1->_maxVelocity, -decisionTreeStep1->_maxAcceleration);
			decisionTreeStep1->_currentVelocity = decisionTreeStep1->_maxVelocity;
		}

		void ReverseSign_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			decisionTreeStep1->_currentPosition *= -1;
			decisionTreeStep1->_targetPosition *= -1;
			decisionTreeStep1->_currentVelocity *= -1;
			decisionTreeStep1->_targetVelocity *= -1;
		}


		bool Decision1_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// called when Vt >= V
			// if V->Vtarget, is P <= Ptarget
			if (decisionTreeStep1->_currentPosition + calcDP_ConstAccWithVf(decisionTreeStep1->_currentVelocity, decisionTreeStep1->_targetVelocity, decisionTreeStep1->_maxAcceleration) <= decisionTreeStep1->_targetPosition)
				return true;
			else
				return false;
		}

		bool Decision2_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// called when Vt <= V
			// if V->Vtarget, is P >= Ptarget
			if (decisionTreeStep1->_currentPosition + calcDP_ConstAccWithVf(decisionTreeStep1->_currentVelocity, decisionTreeStep1->_targetVelocity, -decisionTreeStep1->_maxAcceleration) >= decisionTreeStep1->_targetPosition)
				return true;
			else
				return false;
		}

		bool Decision3_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// called when both V and Vt <= Vmax
			// if V->Vmax->Vtarget, is P <= Ptarget
			if (decisionTreeStep1->_currentPosition + calcDP_reverseVShapeVel(decisionTreeStep1->_currentVelocity, decisionTreeStep1->_maxVelocity, decisionTreeStep1->_targetVelocity, decisionTreeStep1->_maxAcceleration) <= decisionTreeStep1->_targetPosition)
				return true;
			else
				return false;
		}

		bool Decision4_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// called when both V and Vt >= -Vmax
			// if V->-Vmax->Vtarget, is P <= Ptarget
			if (decisionTreeStep1->_currentPosition + calcDP_VShapeVel(decisionTreeStep1->_currentVelocity, -decisionTreeStep1->_maxVelocity, decisionTreeStep1->_targetVelocity, decisionTreeStep1->_maxAcceleration) <= decisionTreeStep1->_targetPosition)
				return true;
			else
				return false;
		}

		bool Decision5_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// called when both V and Vt >= 0
			// if V->0->Vtarget, is P <= Ptarget
			// in order to calculate inoperative time interval
			if (decisionTreeStep1->_currentPosition + calcDP_VShapeVel(decisionTreeStep1->_currentVelocity, 0.0, decisionTreeStep1->_targetVelocity, decisionTreeStep1->_maxAcceleration) <= decisionTreeStep1->_targetPosition)
				return true;
			else
				return false;
		}

		bool Decision6_TrapTrajStep1(TreeTrapTrajStep1 * decisionTreeStep1)
		{
			// called when V >= 0 and Vt <= 0: penetrate zero
			// if V->0->Vtarget, is P >= Ptarget
			if (decisionTreeStep1->_currentPosition + calcDP_ConstAccWithVf(decisionTreeStep1->_currentVelocity, decisionTreeStep1->_targetVelocity, -decisionTreeStep1->_maxAcceleration) >= decisionTreeStep1->_targetPosition)
				return true;
			else
				return false;
		}

	}
}