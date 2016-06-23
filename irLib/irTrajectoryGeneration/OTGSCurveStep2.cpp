#include "OTGSCurveStep2.h"
#include <iostream>

using namespace std;
using namespace irLib::irMath;

namespace irLib
{
	namespace irTG
	{
		TreeSCurveStep2 * makeDecisionTreeSCurveStep2(const Real maxVelocity, const Real maxAcceleration, const Real maxJerk, const Real currentPosition, const Real currentVelocity, const Real currentAcceleration, const Real targetPosition, const Real targetVelocity, const Real tcurr, const Real tsync)
		{
			TreeSCurveStep2* tree = new TreeSCurveStep2;
			tree->_maxVelocity = maxVelocity;
			tree->_maxAcceleration = maxAcceleration;
			tree->_maxJerk = maxJerk;
			tree->_currentPosition = currentPosition;
			tree->_currentVelocity = currentVelocity;
			tree->_currentAcceleration = currentAcceleration;
			tree->_targetPosition = targetPosition;
			tree->_targetVelocity = targetVelocity;

			tree->_tcurr = tcurr;
			tree->_tsync = tsync;

			return tree;
		}

		void deleteDecisionTreeSCurveStep2(TreeSCurveStep2 * decisionTreeStep2)
		{
			delete decisionTreeStep2;
		}

		void makeFinalProfilesSCurve(TreeSCurveStep2 * decisionTreeStep2, Profile* profile)
		{

			// decision tree start!
		//decisionBox_01:
			if (Decision1_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_02;
			}
			else
			{
				goto decisionBox_xx;
			}
		decisionBox_02:
			if (Decision2_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_03;
			}
			else
			{
				goto decisionBox_10;
			}
		decisionBox_03:
			if (Decision3_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_04;
			}
			else
			{
				goto decisionBox_07;
			}
		decisionBox_04:
			if (Decision4_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_05;
			}
			else
			{
				goto decisionBox_xx;
			}
		decisionBox_05:
			if (Decision5_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroNegTri");
				calculateProfile_PosTrapZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_06;
			}
		decisionBox_06:
			if (Decision6_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroNegTrap");
				calculateProfile_PosTrapZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" PosTrapZeroNegTri");
				calculateProfile_PosTrapZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_07:
			if (Decision7_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_08;
			}
			else
			{
				goto decisionBox_xx;
			}
		decisionBox_08:
			if (Decision8_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTriZeroNegTri");
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_09;
			}
		decisionBox_09:
			if (Decision9_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_05;
			}
			else
			{
				LOG(" PosTriZeroNegTri");
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_10:
			if (Decision10_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_11;
			}
			else
			{
				goto decisionBox_12;
			}
		decisionBox_11:
			if (Decision11_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_xx;
			}
			else
			{
				goto decisionBox_xx;
			}
		decisionBox_12:
			if (Decision12_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_xx;
			}
			else
			{
				goto decisionBox_xx;
			}
		decisionBox_xx:
			LOG(" not implemented yet....");
			return;

		}

		bool Decision1_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << "decision box: 01";
			if (dtStep2->_currentAcceleration >= 0)
				return true;
			return false;
		}

		bool Decision2_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 02";
			if (dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk) <= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision3_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 03";
			if (dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk) <= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision4_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 04";
			Real taftertrap = dtStep2->_tcurr + calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, dtStep2->_targetVelocity);
			if (dtStep2->_currentPosition + calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, dtStep2->_targetVelocity)
				+ calcDP_constVel(dtStep2->_tsync - taftertrap, dtStep2->_targetVelocity) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision5_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 05";
			Real vaftertrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vaftertrap >= dtStep2->_targetVelocity, "wrong1 - Decision5_SCurveStep2");
			Real taftertrap = dtStep2->_tcurr + calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vaftertrap);
			if (taftertrap + calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk) >= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision6_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 06";
			Real vaftertrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vaftertrap >= dtStep2->_targetVelocity, "wrong1 - Decision6_SCurveStep2");
			Real taftertrap = dtStep2->_tcurr + calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vaftertrap);
			Real hldTime = dtStep2->_tsync - taftertrap - calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			//cout << endl << dtStep2->_currentPosition + calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vaftertrap)
			//	+ calcDP_constVel(hldTime, vaftertrap)
			//	+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vaftertrap) << endl;
			if (dtStep2->_currentPosition + calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vaftertrap)
				+ calcDP_constVel(hldTime, vaftertrap)
				+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vaftertrap) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision7_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 07";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, dtStep2->_targetVelocity);
			Real taftertri = dtStep2->_tcurr + calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_currentPosition + calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(dtStep2->_tsync - taftertri, dtStep2->_targetVelocity) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision8_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 08";
			Real vaftertri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vaftertri >= dtStep2->_targetVelocity, "wrong1 - Decision8_SCurveStep2");
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vaftertri, dtStep2->_targetVelocity);
			if (dtStep2->_tcurr + calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk) >= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision9_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 09";
			Real vaftertri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vaftertri >= dtStep2->_targetVelocity, "wrong1 - Decision9_SCurveStep2");
			Real taftertri = dtStep2->_tcurr + calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vaftertri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - taftertri - calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_currentPosition + calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vaftertri)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk, vaftertri) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision10_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 10";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterbackslash >= dtStep2->_targetVelocity, "wrong1 - Decision10_SCurveStep2");
			LOGIF(vafterbackslash <= dtStep2->_maxVelocity, "wrong2 - Decision10_SCurveStep2");
			if (vafterbackslash + calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk) <= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision11_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 11";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterbackslash >= dtStep2->_targetVelocity, "wrong1 - Decision11_SCurveStep2");
			LOGIF(vafterbackslash <= dtStep2->_maxVelocity, "wrong2 - Decision11_SCurveStep2");
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity);
			Real tafterbackslashandvshape = dtStep2->_tcurr + dtStep2->_currentAcceleration / dtStep2->_maxJerk + calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_currentPosition + calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk, vafterbackslash)
				+ calcDP_constVel(dtStep2->_tsync - tafterbackslashandvshape, dtStep2->_targetVelocity) >= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision12_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 12";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterbackslash >= dtStep2->_targetVelocity, "wrong1 - Decision12_SCurveStep2");
			LOGIF(vafterbackslash <= dtStep2->_maxVelocity, "wrong2 - Decision12_SCurveStep2");
			Real tafterbackslashandnegtrap = dtStep2->_tcurr + dtStep2->_currentAcceleration / dtStep2->_maxJerk + calcDT_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity);
			if (dtStep2->_currentPosition + calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity)
				+ calcDP_constVel(dtStep2->_tsync - tafterbackslashandnegtrap, dtStep2->_targetVelocity) >= dtStep2->_targetPosition)
				return true;
			return false;
		}


		// Profile calculation function
		void calculateProfile_PosTriZeroNegTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 1) numerical solution
		{
			// calculate time interval
			NewtonRaphson solver(2, 2);
			solver.settolerance(1E-10);
			solver.setfunction(PosTriZeroNegTriFcn);
			solver.setfdata(decisionTreeStep2);

			VectorX X(2), result(2); // ap1 , ap2
			Real ap1, ap2, t12, t23, t34, t45, t56;

			X(0) = (decisionTreeStep2->_maxAcceleration + decisionTreeStep2->_currentAcceleration) / 2;
			X(1) = -decisionTreeStep2->_maxAcceleration / 2;

			srand((unsigned int)time(NULL));

			int count = 0;

			while (1)
			{
				count++;
				if (count > 10)
				{
					X(0) = makeRandLU(decisionTreeStep2->_currentAcceleration, decisionTreeStep2->_maxAcceleration);
					X(1) = makeRandLU(-decisionTreeStep2->_maxAcceleration, -X(0));
					count = 0;
				}

				solver.solve(X);
				result = solver.getResultX();

				ap1 = result(0); ap2 = result(1);
				t12 = (ap1 - decisionTreeStep2->_currentAcceleration) / decisionTreeStep2->_maxJerk;
				t23 = ap1 / decisionTreeStep2->_maxJerk;
				t45 = -ap2 / decisionTreeStep2->_maxJerk;
				t56 = t45;
				t34 = decisionTreeStep2->_tsync - t12 - t23 - t45 - t56;

				if (ap1 > decisionTreeStep2->_currentAcceleration && ap1 < decisionTreeStep2->_maxAcceleration && ap2 < 0 && ap2 > -decisionTreeStep2->_maxAcceleration && t34 > 0)
					break;

				if (t34 < 0 && ap1 > decisionTreeStep2->_currentAcceleration && ap1 < decisionTreeStep2->_maxAcceleration && ap2 < 0 && ap2 > -decisionTreeStep2->_maxAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep2->_currentAcceleration) / 2;
					X(1) = X(1) / 2;
				}
				else if (t34 < 0 && ap1 > decisionTreeStep2->_maxAcceleration && ap2 < -decisionTreeStep2->_maxAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep2->_currentAcceleration) / 2;
					X(1) = X(1) / 2;
				}
				else if (t34 < 0 && ap1 > decisionTreeStep2->_maxAcceleration && ap2 < 0 && ap2 > -decisionTreeStep2->_maxAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep2->_currentAcceleration) / 2;
				}
				else if (t34 < 0 && ap1 > decisionTreeStep2->_currentAcceleration && ap1 < decisionTreeStep2->_maxAcceleration && ap2 < -decisionTreeStep2->_maxAcceleration)
				{
					X(1) = X(1) / 2;
				}
				else if (t34 > 0 && ap1 > decisionTreeStep2->_maxAcceleration && ap2 < -decisionTreeStep2->_maxAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep2->_currentAcceleration) / 2;
					X(1) = X(1) / 2;
				}
				else if (t34 > 0 && ap1 > decisionTreeStep2->_maxAcceleration && ap2 < 0 && ap2 > -decisionTreeStep2->_maxAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep2->_currentAcceleration) / 2;
				}
				else if (t34 > 0 && ap1 > decisionTreeStep2->_currentAcceleration && ap1 < decisionTreeStep2->_maxAcceleration && ap2 < -decisionTreeStep2->_maxAcceleration)
				{
					X(1) = X(1) / 2;
				}
				else if (t34 > 0 && ap1 > decisionTreeStep2->_currentAcceleration && ap1 < decisionTreeStep2->_maxAcceleration && ap2 > 0)
				{
					X(1) = (X(1) - decisionTreeStep2->_maxAcceleration) / 2;
				}
			}

			calcProfile_Slash(decisionTreeStep2, profile, ap1, t12); // time interval t12
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t23); // time interval t23
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_BackSlash(decisionTreeStep2, profile, ap2, t45); // time interval t45
			calcProfile_Slash(decisionTreeStep2, profile, 0, t56); // time interval t56
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTrapZeroNegTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 2) closed-form solution
		{
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep2, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real a, b, c, d, e;
			a = 12;
			b = -24 * amax;
			c = 12 * a0*a0 + 12 * amax * amax + 24 * jmax*vt - 24 * amax*jmax*tsyn -
				24 * a0*amax - 24 * jmax*v0;
			d = 0;
			e = 3 * std::pow(a0, 4) - 8 * std::pow(a0, 3) * amax + 6 * a0*a0 * amax * amax + 12 * jmax*jmax * v0*v0 + 12 * jmax*jmax * vt*vt -
				12 * a0*a0 * jmax*v0 - 12 * amax * amax * jmax*v0 + 12 * a0*a0 * jmax*vt + 12 * amax * amax * jmax*vt -
				24 * jmax*jmax * v0*vt - 24 * amax*jmax*jmax * p0 + 24 * amax*jmax*jmax * pt + 24 * a0*amax*jmax*v0 -
				24 * a0*amax*jmax*vt - 24 * amax*jmax*jmax * tsyn*vt;
			
			ComplexNumber x[4];
			calcQuarticAlgebraicEqn(a, b, c, d, e, x);

			Real ep = 1E-8, sol = -RealMax;
			for (int i = 0; i < 4; i++)
			{
				if (abs(x[i]._iN) < ep)
				{
					if (x[i]._rN < 0 && x[i]._rN > -amax)
					{
						if (x[i]._rN > sol)
							sol = x[i]._rN;
					}
				}
			}

			Real ap2, t12, t23, t34, t45, t56, t67;
			ap2 = sol;
			t23 = (a0 * a0 - 2 * amax * amax + 2 * ap2 * ap2 - 2 * jmax*v0 + 2 * jmax*vt) / (2 * amax*jmax);
			t45 = (-a0 * a0 + 2 * a0*amax - 2 * amax * amax + 4 * amax*ap2 + 2 * jmax*tsyn*amax - 2 * ap2 * ap2 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t12 = (decisionTreeStep2->_maxAcceleration - decisionTreeStep2->_currentAcceleration) / decisionTreeStep2->_maxJerk;
			t34 = decisionTreeStep2->_maxAcceleration / decisionTreeStep2->_maxJerk;
			t56 = -ap2 / decisionTreeStep2->_maxJerk;
			t67 = t56;
			
			calcProfile_Slash(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t12); // time interval t12
			calcProfile_Rectangular(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t23); // time interval t23
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t45); // time interval t45
			calcProfile_BackSlash(decisionTreeStep2, profile, ap2, t56); // time interval t56
			calcProfile_Slash(decisionTreeStep2, profile, 0, t67); // time interval t67
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTrapZeroNegTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 3) closed-form solution
		{
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep2, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			// calculate time interval
			Real t12, t23, t34, t45, t56, t67, t78;
			t67 = -(2 * jmax*vt - 2 * jmax*v0 - 2 * a0*amax + a0 * a0 + 6 * amax *amax + 2 * sqrt((a0 * a0 * a0 * amax) / 3 -
				2 * a0*std::pow(amax, 3) - (std::pow(a0, 4)) / 4 + std::pow(amax, 4) + a0 * a0 * amax * amax - jmax * jmax * v0 * v0 - jmax * jmax * vt * vt +
				a0 * a0 * jmax*v0 - a0 * a0 * jmax*vt + 2 * jmax * jmax * v0*vt + amax * amax * jmax * jmax * tsyn * tsyn +
				4 * amax*jmax*jmax * p0 - 4 * amax*jmax*jmax * pt - 2 * amax*amax*amax * jmax*tsyn - 2 * a0*amax*jmax*v0 +
				2 * a0*amax*jmax*vt + 2 * a0*amax *amax * jmax*tsyn - a0 *a0 * amax*jmax*tsyn + 2 * amax*jmax*jmax * tsyn*v0 +
				2 * amax*jmax*jmax * tsyn*vt) - 2 * amax*jmax*tsyn) / (4 * amax*jmax);
			t23 = (a0 * a0 - 2 * jmax*v0 + 2 * jmax*vt + 2 * amax*jmax*t67) / (2 * amax*jmax);
			t12 = (amax - a0) / jmax;
			t34 = amax / jmax;
			t56 = t34; t78 = t56;
			t45 = tsyn - t12 - t23 - t34 - t56 - t67 - t78;

			calcProfile_Slash(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t12); // time interval t12
			calcProfile_Rectangular(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t23); // time interval t23
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t45); // time interval t45
			calcProfile_BackSlash(decisionTreeStep2, profile, -decisionTreeStep2->_maxAcceleration, t56); // time interval t56
			calcProfile_Rectangular(decisionTreeStep2, profile, -decisionTreeStep2->_maxAcceleration, t67); // time interval t67
			calcProfile_Slash(decisionTreeStep2, profile, 0, t78); // time interval t78
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTriZeroNegTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 4) closed-form solution
		{
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep2, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real a, b, c, d, e;
			a = 12;
			b = 24 * amax;
			c = -12 * a0 * a0 + 12 * amax * amax - 24 * jmax*vt - 24 * a0*amax - 24 * amax*jmax*tsyn + 24 * jmax*v0;
			d = 0;
			e = 4 * std::pow(a0, 3) * amax + 3 * std::pow(a0, 4) - 6 * a0 * a0 * amax * amax + 12 * jmax * jmax * v0 * v0 + 12 * jmax * jmax * vt * vt - 12 * a0 * a0 * jmax*v0 +
				12 * amax * amax * jmax*v0 + 12 * a0 * a0 * jmax*vt - 12 * amax * amax * jmax*vt - 24 * jmax * jmax * v0*vt - 24 * amax*jmax * jmax * p0 +
				24 * amax*jmax * jmax * pt + 12 * a0 * a0 * amax*jmax*tsyn - 24 * amax*jmax * jmax * tsyn*v0;

			ComplexNumber x[4];
			calcQuarticAlgebraicEqn(a, b, c, d, e, x);

			Real ep = 1E-8, sol = RealMax;
			for (int i = 0; i < 4; i++)
			{
				if (abs(x[i]._iN) < ep)
				{
					if (x[i]._rN > 0 && x[i]._rN < amax)
					{
						if (x[i]._rN < sol)
							sol = x[i]._rN;
					}
				}
			}

			Real ap1, t12, t23, t34, t45, t56, t67;
			ap1 = sol;
			t56 = -(a0 * a0 + 2 * amax * amax - 2 * ap1 * ap1 - 2 * jmax*v0 + 2 * jmax*vt) / (2 * amax*jmax);
			t34 = (a0 * a0 + 2 * a0*amax - 2 * amax * amax - 4 * amax*ap1 + 2 * jmax*tsyn*amax - 2 * ap1 * ap1 - 2 * jmax*v0 + 2 * jmax*vt) / (2 * amax*jmax);
			t12 = (ap1 - a0) / jmax;
			t23 = ap1 / jmax;
			t45 = amax / jmax;
			t67 = t45;

			// Compute polynomial for the time interval t12
			calcProfile_Slash(decisionTreeStep2, profile, ap1, t12); // time interval t12
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t23); // time interval t23
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_BackSlash(decisionTreeStep2, profile, -decisionTreeStep2->_maxAcceleration, t45); // time interval t45
			calcProfile_Rectangular(decisionTreeStep2, profile, -decisionTreeStep2->_maxAcceleration, t56); // time interval t56
			calcProfile_Slash(decisionTreeStep2, profile, 0, t67); // time interval t67
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTrapZeroPosTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 5) closed-form solution
		{
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep2, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real a, b;
			a = -(24 * a0*std::pow(amax, 3) - 8 * std::pow(a0, 3) * amax + 3 * std::pow(a0, 4) - 24 * std::pow(amax, 4) - 6 * a0 * a0 * amax * amax +
				12 * jmax * jmax * v0 * v0 + 12 * jmax * jmax * vt * vt - 12 * a0 * a0 * jmax*v0 + 12 * amax * amax * jmax*v0 +
				12 * a0 * a0 * jmax*vt - 12 * amax * amax * jmax*vt - 24 * jmax * jmax * v0*vt -
				24 * amax*jmax * jmax * p0 + 24 * amax*jmax * jmax * pt + 24 * std::pow(amax, 3) * jmax*tsyn +
				24 * a0*amax*jmax*v0 - 24 * a0*amax*jmax*vt - 24 * amax*jmax * jmax * tsyn*vt);
			b = 24 * amax * amax * jmax * jmax * tsyn + 24 * amax*jmax * jmax * v0 -
				24 * std::pow(amax, 3) * jmax + 24 * a0*amax * amax * jmax - 12 * a0 * a0 * amax*jmax - 24 * amax*jmax * jmax * vt;

			Real t12, t23, t34, t45, t56, t67, t78;
			t67 = a / b;
			t23 = -(-a0 * a0 + 4 * amax * amax + 2 * jmax*t67*amax + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t45 = (-a0 * a0 + 2 * a0*amax - 4 * amax * amax + 2 * jmax*tsyn*amax + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t12 = (amax - a0) / jmax;
			t34 = amax / jmax;
			t56 = t34;
			t78 = t56;

			calcProfile_Slash(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t12); // time interval t12
			calcProfile_Rectangular(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t23); // time interval t23
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t45); // time interval t45
			calcProfile_Slash(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t56); // time interval t56
			calcProfile_Rectangular(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t67); // time interval t67
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t78); // time interval t78
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTrapZeroPosTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 6) closed-form solution
		{
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep2, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real a, b, c, d, e;
			a = 12;
			b = -24 * amax;
			c = -12 * a0 * a0 - 12 * amax * amax + 24 * jmax*v0 - 24 * jmax*vt + 24 * amax*jmax*tsyn + 24 * a0*amax;
			d = 0;
			e = 3 * std::pow(a0, 4) - 8 * std::pow(a0, 3) * amax + 6 * a0 * a0 * amax * amax + 12 * jmax * jmax * v0 * v0 + 12 * jmax * jmax * vt * vt -
				12 * a0 * a0 * jmax*v0 - 12 * amax * amax * jmax*v0 + 12 * a0 * a0 * jmax*vt + 12 * amax * amax * jmax*vt -
				24 * jmax * jmax * v0*vt - 24 * amax*jmax * jmax * p0 + 24 * amax*jmax * jmax * pt + 24 * a0*amax*jmax*v0 -
				24 * a0*amax*jmax*vt - 24 * amax*jmax * jmax * tsyn*vt;

			ComplexNumber x[4];
			calcQuarticAlgebraicEqn(a, b, c, d, e, x);

			Real ep = 1E-8, sol = RealMax;
			for (int i = 0; i < 4; i++)
			{
				if (abs(x[i]._iN) < ep)
				{
					if (x[i]._rN > 0 && x[i]._rN < amax)
					{
						if (x[i]._rN < sol)
							sol = x[i]._rN;
					}
				}
			}

			Real ap1, t12, t23, t34, t45, t56, t67;
			ap1 = sol;
			t23 = -(-a0 * a0 + 2 * amax * amax + 2 * ap1 * ap1 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t45 = (-a0 * a0 + 2 * a0*amax - 2 * amax * amax - 4 * amax*ap1 + 2 * jmax*tsyn*amax + 2 * ap1 * ap1 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t12 = (amax - a0) / jmax;
			t34 = amax / jmax;
			t56 = ap1 / jmax;
			t67 = t56;

			calcProfile_Slash(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t12); // time interval t12
			calcProfile_Rectangular(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t23); // time interval t23
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t45); // time interval t45
			calcProfile_Slash(decisionTreeStep2, profile, ap1, t56); // time interval t56
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t67); // time interval t67
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTriZeroPosTrap_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 7) closed-form solution
		{
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep2, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real a, b, c, d, e;
			a = 12;
			b = -24 * amax;
			c = -12 * a0 * a0 - 12 * amax * amax + 24 * jmax*v0 + 24 * a0*amax +
				24 * amax*jmax*tsyn - 24 * jmax*vt;
			d = 0;
			e = 3 * std::pow(a0, 4) - 4 * std::pow(a0, 3) * amax + 6 * a0 * a0 * amax * amax + 12 * jmax * jmax * v0 * v0 +
				12 * jmax * jmax * vt * vt - 12 * a0 * a0 * jmax*v0 - 12 * amax * amax * jmax*v0 +
				12 * a0 * a0 * jmax*vt + 12 * amax * amax * jmax*vt - 24 * jmax * jmax * v0*vt + 24 * amax*jmax * jmax * p0 -
				24 * amax*jmax * jmax * pt - 12 * a0 * a0 * amax*jmax*tsyn + 24 * amax*jmax * jmax * tsyn*v0;

			ComplexNumber x[4];
			calcQuarticAlgebraicEqn(a, b, c, d, e, x);

			Real ep = 1E-8, sol = RealMax;
			for (int i = 0; i < 4; i++)
			{
				if (abs(x[i]._iN) < ep)
				{
					if (x[i]._rN > 0 && x[i]._rN < amax)
					{
						if (x[i]._rN < sol)
							sol = x[i]._rN;
					}
				}
			}

			Real ap1, t12, t23, t34, t45, t56, t67;
			ap1 = sol;
			t34 = (-a0 * a0 + 2 * a0*amax - 2 * amax * amax - 4 * amax*ap1 + 2 * jmax*tsyn*amax + 2 * ap1 * ap1 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t56 = -(-a0 * a0 + 2 * amax * amax + 2 * ap1 * ap1 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * amax*jmax);
			t12 = (ap1 - a0) / jmax;
			t23 = ap1 / jmax;
			t45 = amax / jmax;
			t67 = t45;

			calcProfile_Slash(decisionTreeStep2, profile, ap1, t12); // time interval t12
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t23); // time interval t23
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_Slash(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t45); // time interval t45
			calcProfile_Rectangular(decisionTreeStep2, profile, decisionTreeStep2->_maxAcceleration, t56); // time interval t56
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t67); // time interval t67
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}

		void calculateProfile_PosTriZeroPosTri_SCurveStep2(TreeSCurveStep2* decisionTreeStep2, Profile* profile/*, const bool reverse*/) // (case 8) numerical solution 
		{
			// calculate time interval
			NewtonRaphson solver(2, 2);
			solver.settolerance(1E-10);
			solver.setfunction(PosTriZeroPosTriFcn);
			solver.setfdata(decisionTreeStep2);

			VectorX X(2), result(2); // ap1 , ap2
			Real ap1, ap2, t12, t23, t34, t45, t56;

			X(0) = (decisionTreeStep2->_maxAcceleration + decisionTreeStep2->_currentAcceleration) / 2;
			X(1) = decisionTreeStep2->_maxAcceleration / 2;

			srand((unsigned int)time(NULL));

			while (1)
			{
				solver.solve(X);
				result = solver.getResultX();

				ap1 = result(0); ap2 = result(1);
				t12 = (ap1 - decisionTreeStep2->_currentAcceleration) / decisionTreeStep2->_maxJerk;
				t23 = ap1 / decisionTreeStep2->_maxJerk;
				t45 = ap2 / decisionTreeStep2->_maxJerk;
				t56 = t45;
				t34 = decisionTreeStep2->_tsync - t12 - t23 - t45 - t56;

				if (ap1 > decisionTreeStep2->_currentAcceleration && ap1 < decisionTreeStep2->_maxAcceleration && ap2 > 0 && ap2 < decisionTreeStep2->_maxAcceleration && t34 > 0)
					break;

				X(0) = makeRandLU(decisionTreeStep2->_currentAcceleration, decisionTreeStep2->_maxAcceleration);
				X(1) = makeRandLU(0, decisionTreeStep2->_maxAcceleration);
			}

			calcProfile_Slash(decisionTreeStep2, profile, ap1, t12); // time interval t12
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t23); // time interval t23
			calcProfile_Rectangular(decisionTreeStep2, profile, 0, t34); // time interval t34
			calcProfile_Slash(decisionTreeStep2, profile, ap2, t45); // time interval t45
			calcProfile_BackSlash(decisionTreeStep2, profile, 0, t56); // time interval t56
			calcProfile_Aftertsync(decisionTreeStep2, profile); // after sychronization time
		}


		void PosTriZeroNegTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data) // case 1
		{
			TreeSCurveStep2* tree = reinterpret_cast<TreeSCurveStep2*>(f_data);
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(tree, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap1 = x(0), ap2 = x(1);
			f(0) = -(a0*a0 - 2 * ap1*ap1 + 2 * ap2*ap2 - 2 * jmax*v0 + 2 * jmax*vt) / (2 * jmax);
			f(1) = (6 * a0*ap1*ap1 + 6 * jmax*jmax * p0 - 6 * jmax*jmax * pt - a0*a0*a0 - 6 * ap1*ap1*ap1 +
				6 * ap2*ap2*ap2 + 6 * jmax*jmax * tsyn*v0 - 3 * a0*a0 * jmax*tsyn + 6 * ap1*ap1 * jmax*tsyn) / (6 * jmax*jmax);

			Real det = (2 * ap1*ap2*(2 * a0 - 3 * ap1 + 3 * ap2 + 2 * jmax*tsyn)) / (jmax *jmax * jmax);
			ig(0, 0) = ((3 * ap2*ap2) / (jmax*jmax)) / det;
			ig(1, 1) = ((2 * ap1) / jmax) / det;
			ig(0, 1) = ((2 * ap2) / jmax) / det;
			ig(1, 0) = -((ap1*(2 * a0 - 3 * ap1 + 2 * jmax*tsyn)) / (jmax*jmax)) / det;
		}

		void PosTriZeroPosTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data) // case 8
		{
			TreeSCurveStep2* tree = reinterpret_cast<TreeSCurveStep2*>(f_data);
			Real tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(tree, tsyn, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap1 = x(0), ap2 = x(1);
			f(0) = (-a0 * a0 + 2 * ap1 * ap1 + 2 * ap2 * ap2 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * jmax);
			f(1) = (6 * a0*ap1 * ap1 + 6 * jmax * jmax * p0 - 6 * jmax * jmax * pt - std::pow(a0, 3) - 6 * std::pow(ap1, 3) + 6 * std::pow(ap2, 3) +
				6 * jmax * jmax * tsyn*v0 - 3 * a0 * a0 * jmax*tsyn + 6 * ap1 * ap1 * jmax*tsyn) / (6 * jmax * jmax);

			Real det = -(2 * ap1*ap2*(2 * a0 - 3 * ap1 - 3 * ap2 + 2 * jmax*tsyn)) / std::pow(jmax, 3);
			ig(0, 0) = (3 * ap2 * ap2) / (jmax * jmax) / det;
			ig(1, 1) = (2 * ap1 / jmax) / det;
			ig(0, 1) = -((2 * ap2) / jmax) / det;
			ig(1, 0) = -((ap1*(2 * a0 - 3 * ap1 + 2 * jmax*tsyn)) / (jmax * jmax)) / det;
		}

		void substitutefcn(TreeSCurveStep2* decisionTreeStep2, Real& tsyn, Real& a0, Real& vt, Real& v0, Real& pt, Real& p0, Real& jmax, Real& amax, Real& vmax)
		{
			tsyn = decisionTreeStep2->_tsync;
			a0 = decisionTreeStep2->_currentAcceleration;
			vt = decisionTreeStep2->_targetVelocity;
			v0 = decisionTreeStep2->_currentVelocity;
			pt = decisionTreeStep2->_targetPosition;
			p0 = decisionTreeStep2->_currentPosition;
			jmax = decisionTreeStep2->_maxJerk;
			amax = decisionTreeStep2->_maxAcceleration;
			vmax = decisionTreeStep2->_maxVelocity;
		}

		void calcProfile_Slash(TreeSCurveStep2* decisionTreeStep2, Profile* profile, const Real nAcc, const Real t)
		{
			profile->PositionProfile[profile->numOfPolynomial].setCoefficient((1.0 / 6.0 * decisionTreeStep2->_maxJerk), (0.5 * decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), (decisionTreeStep2->_tcurr));
			profile->VelocityProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.5 * decisionTreeStep2->_maxJerk), (decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_tcurr));
			profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (decisionTreeStep2->_maxJerk), (decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_tcurr));
			profile->JerkProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (decisionTreeStep2->_maxJerk), (decisionTreeStep2->_tcurr));

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + t;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * t + 0.5 * decisionTreeStep2->_currentAcceleration * std::pow(t, 2) + 1.0 / 6.0 * decisionTreeStep2->_maxJerk * std::pow(t, 3);
			decisionTreeStep2->_currentVelocity += 0.5 * (decisionTreeStep2->_currentAcceleration + nAcc) * t;
			decisionTreeStep2->_currentAcceleration = nAcc;
			decisionTreeStep2->_tcurr += t;

		}

		void calcProfile_BackSlash(TreeSCurveStep2* decisionTreeStep2, Profile* profile, const Real nAcc, const Real t)
		{
			profile->PositionProfile[profile->numOfPolynomial].setCoefficient((-1.0 / 6.0 * decisionTreeStep2->_maxJerk), (0.5 * decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), (decisionTreeStep2->_tcurr));
			profile->VelocityProfile[profile->numOfPolynomial].setCoefficient((0.0), (-0.5 * decisionTreeStep2->_maxJerk), (decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_tcurr));
			profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (-decisionTreeStep2->_maxJerk), (decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_tcurr));
			profile->JerkProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (-decisionTreeStep2->_maxJerk), (decisionTreeStep2->_tcurr));

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + t;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * t + 0.5 * decisionTreeStep2->_currentAcceleration * std::pow(t, 2) - 1.0 / 6.0 * decisionTreeStep2->_maxJerk * std::pow(t, 3);
			decisionTreeStep2->_currentVelocity += 0.5 * (decisionTreeStep2->_currentAcceleration + nAcc) * t;
			decisionTreeStep2->_currentAcceleration = nAcc;
			decisionTreeStep2->_tcurr += t;
		}

		void calcProfile_Rectangular(TreeSCurveStep2* decisionTreeStep2, Profile* profile, const Real nAcc, const Real t)
		{
			profile->PositionProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.5 * decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), (decisionTreeStep2->_tcurr));
			profile->VelocityProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_tcurr));
			profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (decisionTreeStep2->_currentAcceleration), (decisionTreeStep2->_tcurr));
			profile->JerkProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (0.0), (decisionTreeStep2->_tcurr));

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + t;
			profile->numOfPolynomial++;

			decisionTreeStep2->_currentPosition += decisionTreeStep2->_currentVelocity * t + 0.5 * decisionTreeStep2->_currentAcceleration * std::pow(t, 2);
			decisionTreeStep2->_currentVelocity += decisionTreeStep2->_currentAcceleration * t;
			decisionTreeStep2->_currentAcceleration = nAcc;
			decisionTreeStep2->_tcurr += t;
		}

		void calcProfile_Aftertsync(TreeSCurveStep2* decisionTreeStep2, Profile* profile)
		{
			profile->PositionProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_currentPosition), (decisionTreeStep2->_tcurr));
			profile->VelocityProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (decisionTreeStep2->_currentVelocity), (decisionTreeStep2->_tcurr));
			profile->AccelerationProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (0.0), (decisionTreeStep2->_tcurr));
			profile->JerkProfile[profile->numOfPolynomial].setCoefficient((0.0), (0.0), (0.0), (0.0), (decisionTreeStep2->_tcurr));

			profile->PolynomialTime[profile->numOfPolynomial] = decisionTreeStep2->_tcurr + RealMax;
			profile->numOfPolynomial++;
		}

	}
}