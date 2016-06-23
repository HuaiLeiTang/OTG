#include "OTGTrapTrajStep2.h"

namespace irLib
{
	namespace irTG
	{
		TreeTrapTrajStep2* makeDecisionTreeTrapTrajStep2(const Real maxVelocity, const Real maxAcceleration, const Real currentPosition, const Real currentVelocity,
			const Real targetPosition, const Real targetVelocity, const Real tcurr, const Real tsync)
		{
			TreeTrapTrajStep2* tree = new TreeTrapTrajStep2;
			tree->_maxVelocity = maxVelocity;
			tree->_maxAcceleration = maxAcceleration;
			tree->_currentPosition = currentPosition;
			tree->_currentVelocity = currentVelocity;
			tree->_targetPosition = targetPosition;
			tree->_targetVelocity = targetVelocity;
			tree->_tcurr = tcurr;
			tree->_tsync = tsync;
			return tree;
		}

		void deleteDecisionTreeTrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			delete decisionTreeStep2;
		}

		void makeFinalProfilesTrapTraj(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile)
		{
			bool reverse = false;
			int Step2_Decision_Swi = 1;
			while (true)
			{
				if (Step2_Decision_Swi == TrapTrajStep2_Decision_1)
				{
					if (Decision1_TrapTrajStep2(decisionTreeStep2))
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_2;
					}
					else
					{
						ReverseSign_TrapTrajStep2(decisionTreeStep2, &reverse);
						Step2_Decision_Swi = TrapTrajStep2_Decision_2;
					}

				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_2)
				{
					if (Decision2_TrapTrajStep2(decisionTreeStep2))
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_3;
					}
					else
					{
						FromVToVmax_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						Step2_Decision_Swi = TrapTrajStep2_Decision_3;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_3)
				{
					if (Decision3_TrapTrajStep2(decisionTreeStep2))
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_4;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_7;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_4)
				{
					if (Decision4_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_PosLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_5;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_5)
				{
					if (Decision5_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_PosLinHldPosLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_6;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_6)
				{
					if (Decision6_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_NegLinHldPosLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						FromVToZero_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						ReverseSign_TrapTrajStep2(decisionTreeStep2, &reverse);
						calculateProfile_PosLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_7)
				{
					if (Decision7_TrapTrajStep2(decisionTreeStep2))
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_8;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_10;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_8)
				{
					if (Decision8_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_PosLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_9;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_9)
				{
					if (Decision9_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_NegLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_6;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_10)
				{
					if (Decision10_TrapTrajStep2(decisionTreeStep2))
					{
						FromVToZero_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						ReverseSign_TrapTrajStep2(decisionTreeStep2, &reverse);
						Step2_Decision_Swi = TrapTrajStep2_Decision_11;
					}
					else
					{
						Step2_Decision_Swi = TrapTrajStep2_Decision_12;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_11)
				{
					if (Decision11_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_PosLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						calculateProfile_PosLinHldPosLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
				}

				if (Step2_Decision_Swi == TrapTrajStep2_Decision_12)
				{
					if (Decision12_TrapTrajStep2(decisionTreeStep2))
					{
						calculateProfile_PosLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
					else
					{
						calculateProfile_NegLinHldNegLin_TrapTrajStep2(decisionTreeStep2, profile, reverse);
						return;
					}
				}
			}
		}

		// Profile calculation function
		void calculateProfile_PosLinHldNegLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse)
		{
			Real Vhold = 0.0;
			Real delta_t1 = 0.0, delta_t2 = 0.0, delta_t3 = 0.0;
			Real TimeDifference = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr;

			Vhold = 0.5 * (TimeDifference * decisionTreeStep2->_maxAcceleration + decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity -
				std::sqrt(std::pow(decisionTreeStep2->_maxAcceleration,2)*std::pow(TimeDifference,2) - std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity),2) +
					2.0 * decisionTreeStep2->_maxAcceleration * (2.0*(decisionTreeStep2->_currentPosition - decisionTreeStep2->_targetPosition) + 
						TimeDifference*(decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity))));

			// Compute first polynomial for the time interval t1
			delta_t1 = (Vhold - decisionTreeStep2->_currentVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), (decisionTreeStep2->_tcurr));
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, -(decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), (decisionTreeStep2->_tcurr));
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, decisionTreeStep2->_maxAcceleration, decisionTreeStep2->_currentVelocity, decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, decisionTreeStep2->_maxAcceleration, decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t1;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity + Vhold) * delta_t1 / 2.0;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t1;

			// Compute second polynomial for the time interval t2
			delta_t2 = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr - (Vhold - decisionTreeStep2->_targetVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, 0.0, decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, decisionTreeStep2->_currentVelocity, decisionTreeStep2->_currentPosition, decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, decisionTreeStep2->_currentVelocity, decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, 0.0, decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t2;
			profile->numOfPolynomial++;
			
			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * delta_t2;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t2;

			// Compute third polynomial for the time interval t3
			delta_t3 = (decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t3;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_targetVelocity + decisionTreeStep2->_currentVelocity) * delta_t3 / 2.0;
			decisionTreeStep2->_currentVelocity -= delta_t3 * decisionTreeStep2->_maxAcceleration;
			decisionTreeStep2->_tcurr += delta_t3;

			// Compute polynomial parameters after sychronization time
			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + RealMax;
			profile->numOfPolynomial++;

			return;
		}

		void calculateProfile_PosLinHldPosLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse)
		{
			Real Vhold = 0.0;
			Real delta_t1 = 0.0, delta_t2 = 0.0, delta_t3 = 0.0;
			Real TimeDifference = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr;

			Vhold = (0.5 * (2 * decisionTreeStep2->_maxAcceleration * (decisionTreeStep2->_targetPosition - decisionTreeStep2->_currentPosition) + std::pow(decisionTreeStep2->_currentVelocity, 2) -  std::pow(decisionTreeStep2->_targetVelocity, 2))
				/ (decisionTreeStep2->_maxAcceleration * TimeDifference + decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity));

			// Compute first polynomial for the time interval t1
			delta_t1 = (Vhold - decisionTreeStep2->_currentVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * (decisionTreeStep2->_maxAcceleration)), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * (decisionTreeStep2->_maxAcceleration)), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t1;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity + Vhold) * delta_t1 / 2.0;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t1;

			// Compute first polynomial for the time interval t2
			delta_t2 = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr - (decisionTreeStep2->_targetVelocity - Vhold) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t2;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * delta_t2;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t2;

			// Compute first polynomial for the time interval t3
			delta_t3 = (decisionTreeStep2->_targetVelocity - Vhold) / decisionTreeStep2->_maxAcceleration;
			
			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, -(0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t3;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_targetVelocity + decisionTreeStep2->_currentVelocity) * delta_t3 / 2.0;
			decisionTreeStep2->_currentVelocity += delta_t3 * decisionTreeStep2->_maxAcceleration;
			decisionTreeStep2->_tcurr += delta_t3;

			// Compute polynomial parameters after sychronization time
			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + RealMax;
			profile->numOfPolynomial++;

			return;
		}

		void calculateProfile_NegLinHldPosLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse)
		{
			Real Vhold = 0.0;
			Real delta_t1 = 0.0, delta_t2 = 0.0, delta_t3 = 0.0;
			Real TimeDifference = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr;

			Vhold = 0.5 * (decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity - TimeDifference * decisionTreeStep2->_maxAcceleration + 
				std::sqrt(std::pow(decisionTreeStep2->_maxAcceleration, 2) * std::pow(TimeDifference, 2) - std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity), 2) - 
					2.0 * decisionTreeStep2->_maxAcceleration * (2.0 * (decisionTreeStep2->_currentPosition - decisionTreeStep2->_targetPosition) + TimeDifference * (decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity))));

			// Compute first polynomial for the time interval t1
			delta_t1 = (decisionTreeStep2->_currentVelocity - Vhold) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t1;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity + Vhold) * delta_t1 / 2.0;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t1;
			
			// Compute first polynomial for the time interval t2
			delta_t2 = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr - (decisionTreeStep2->_targetVelocity - Vhold) / decisionTreeStep2->_maxAcceleration;
			
			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t2;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * delta_t2;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t2;
			
			// Compute first polynomial for the time interval t3
			delta_t3 = (decisionTreeStep2->_targetVelocity - decisionTreeStep2->_currentVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t3;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_targetVelocity + decisionTreeStep2->_currentVelocity) * delta_t3 / 2.0;
			decisionTreeStep2->_currentVelocity += delta_t3 * decisionTreeStep2->_maxAcceleration;
			decisionTreeStep2->_tcurr += delta_t3;

			// Compute polynomial parameters after sychronization time
			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + RealMax;
			profile->numOfPolynomial++;

			return;
		}

		void calculateProfile_NegLinHldNegLin_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse)
		{
			Real Vhold = 0.0;
			Real delta_t1 = 0.0, delta_t2 = 0.0, delta_t3 = 0.0;
			Real TimeDifference = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr;

			Vhold = (0.5 * (2.0 * decisionTreeStep2->_maxAcceleration * (decisionTreeStep2->_targetPosition - decisionTreeStep2->_currentPosition) - 
				std::pow(decisionTreeStep2->_currentVelocity, 2) + std::pow(decisionTreeStep2->_targetVelocity, 2)) / 
				(decisionTreeStep2->_maxAcceleration * TimeDifference - decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity));

			// Compute first polynomial for the time interval t1
			delta_t1 = (decisionTreeStep2->_currentVelocity - Vhold) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t1;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity + Vhold) * delta_t1 / 2.0;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t1;

			// Compute first polynomial for the time interval t2
			delta_t2 = decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr - (Vhold - decisionTreeStep2->_targetVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t2;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * delta_t2;
			decisionTreeStep2->_currentVelocity = Vhold;
			decisionTreeStep2->_tcurr += delta_t2;

			// Compute first polynomial for the time interval t3
			delta_t3 = (decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_maxAcceleration), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + delta_t3;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity) * delta_t3 / 2.0;
			decisionTreeStep2->_currentVelocity -= delta_t3 * decisionTreeStep2->_maxAcceleration;
			decisionTreeStep2->_tcurr += delta_t3;

			// Compute polynomial parameters after sychronization time
			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.0), (0.0), (0.0), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + RealMax;
			profile->numOfPolynomial++;

			return;
		}

		// Assistant function
		void FromVToVmax_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse)
		{
			Real executeTime = (decisionTreeStep2->_currentVelocity - decisionTreeStep2->_maxVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * (decisionTreeStep2->_maxAcceleration)), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, (decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * (decisionTreeStep2->_maxAcceleration)), decisionTreeStep2->_currentVelocity, decisionTreeStep2->_currentPosition, decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_currentVelocity, decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = (decisionTreeStep2->_tcurr) + executeTime;
			profile->numOfPolynomial++;

			decisionTreeStep2->_tcurr += executeTime;
			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity + decisionTreeStep2->_maxVelocity) * executeTime / 2.0;
			decisionTreeStep2->_currentVelocity = decisionTreeStep2->_maxVelocity;

			return;
		}

		void FromVToZero_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, Profile* profile, const bool reverse)
		{
			Real executeTime = (decisionTreeStep2->_currentVelocity) / decisionTreeStep2->_maxAcceleration;

			if (reverse)
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (0.5 * (decisionTreeStep2->_maxAcceleration)), -(decisionTreeStep2->_currentVelocity), -(decisionTreeStep2->_currentPosition), decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, (decisionTreeStep2->_maxAcceleration), -(decisionTreeStep2->_currentVelocity), decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, (decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}
			else
			{
				profile->PositionProfile[profile->numOfPolynomial].setCoefficient(0.0, (-0.5 * (decisionTreeStep2->_maxAcceleration)), decisionTreeStep2->_currentVelocity, decisionTreeStep2->_currentPosition, decisionTreeStep2->_tcurr);
				profile->VelocityProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_currentVelocity, decisionTreeStep2->_tcurr);
				profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient(0.0, 0.0, 0.0, -(decisionTreeStep2->_maxAcceleration), decisionTreeStep2->_tcurr);
			}

			profile->PolynomialTime[profile->numOfPolynomial] = (decisionTreeStep2->_tcurr) + executeTime;
			profile->numOfPolynomial++;

			decisionTreeStep2->_tcurr += executeTime;
			decisionTreeStep2->_currentPosition += (decisionTreeStep2->_currentVelocity) * executeTime / 2.0;
			decisionTreeStep2->_currentVelocity = 0.0;

			return;
		}

		void ReverseSign_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2, bool* reverse)
		{
			decisionTreeStep2->_currentPosition = -(decisionTreeStep2->_currentPosition);
			decisionTreeStep2->_currentVelocity = -(decisionTreeStep2->_currentVelocity);
			decisionTreeStep2->_targetPosition = -(decisionTreeStep2->_targetPosition);
			decisionTreeStep2->_targetVelocity = -(decisionTreeStep2->_targetVelocity);
			*reverse = !(*reverse);
		}

		// Decision boolean function
		bool Decision1_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// V >= 0
			return(decisionTreeStep2->_currentVelocity >= 0.0);
		}

		bool Decision2_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// V <= Vmax
			return(decisionTreeStep2->_currentVelocity <= decisionTreeStep2->_maxVelocity);
		}

		bool Decision3_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// V <= Vt
			return(decisionTreeStep2->_currentVelocity <= decisionTreeStep2->_targetVelocity);
		}

		bool Decision4_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->Vt->hold until t = tsync, is  P <= Ptarget
			Real p = decisionTreeStep2->_currentPosition + (decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr) * decisionTreeStep2->_targetVelocity -
				std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity), 2) / (2 * decisionTreeStep2->_maxAcceleration);
			return(p <= decisionTreeStep2->_targetPosition);
		}

		bool Decision5_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->hold->Vt until t = tsync, is P <= Ptarget
			Real p = decisionTreeStep2->_currentPosition + (decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr) * decisionTreeStep2->_currentVelocity +
				std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity), 2) / (2 * decisionTreeStep2->_maxAcceleration);
			return(p <= decisionTreeStep2->_targetPosition);
			return true;
		}

		bool Decision6_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->0->Vt, is P <= Ptarget or t > tsync
			Real p = decisionTreeStep2->_currentPosition + std::pow(decisionTreeStep2->_currentVelocity, 2) / (2 * decisionTreeStep2->_maxAcceleration) +
				std::pow(decisionTreeStep2->_targetVelocity, 2) / (2 * decisionTreeStep2->_maxAcceleration);
			Real t = (decisionTreeStep2->_currentVelocity + decisionTreeStep2->_targetVelocity) / decisionTreeStep2->_maxAcceleration;
			return(p <= decisionTreeStep2->_targetPosition || t > (decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr));
		}

		bool Decision7_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// Vt >= 0
			return(decisionTreeStep2->_targetVelocity >= 0.0);
		}

		bool Decision8_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->hold->Vt until t = tsync, is P <= Ptarget
			Real p = decisionTreeStep2->_currentPosition + (decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr) * decisionTreeStep2->_currentVelocity -
				std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity), 2) / (2 * decisionTreeStep2->_maxAcceleration);
			return(p <= decisionTreeStep2->_targetPosition);
		}

		bool Decision9_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->Vt->hold until t = tsync, is P <= Ptarget
			Real p = decisionTreeStep2->_currentPosition + (decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr) * decisionTreeStep2->_targetVelocity +
				std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity), 2) / (2 * decisionTreeStep2->_maxAcceleration);
			return(p <= decisionTreeStep2->_targetPosition);
		}

		bool Decision10_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->0->Vt, is P >= Ptarget
			Real p = decisionTreeStep2->_currentPosition + std::pow(decisionTreeStep2->_currentVelocity, 2) / (2 * decisionTreeStep2->_maxAcceleration) -
				std::pow(decisionTreeStep2->_targetVelocity, 2) / (2 * decisionTreeStep2->_maxAcceleration);
			return(p >= decisionTreeStep2->_targetPosition);
		}

		bool Decision11_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->Vt->hold until t = tsync, is P <= Ptarget
			return(Decision4_TrapTrajStep2(decisionTreeStep2));
		}

		bool Decision12_TrapTrajStep2(TreeTrapTrajStep2* decisionTreeStep2)
		{
			// if V->hold->0->Vt until t = tsync, is P <= Ptarget
			Real p = decisionTreeStep2->_currentPosition + (decisionTreeStep2->_tsync - decisionTreeStep2->_tcurr) * decisionTreeStep2->_currentVelocity -
				std::pow((decisionTreeStep2->_currentVelocity - decisionTreeStep2->_targetVelocity), 2) / (2 * decisionTreeStep2->_maxAcceleration);
			return(p <= decisionTreeStep2->_targetPosition);
		}
	}

}