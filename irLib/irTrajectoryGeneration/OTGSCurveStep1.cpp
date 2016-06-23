#include "OTGSCurveStep1.h"

namespace irLib
{
	namespace irTG
	{

		TreeSCurveStep1 * makeDecisionTreeSCurveStep1(const Real maxVelocity, const Real maxAcceleration, const Real maxJerk, const Real currentPosition, const Real currentVelocity, const Real currentAcceleration, const Real targetPosition, const Real targetVelocity, const Real tcurr)
		{
			TreeSCurveStep1* tree = new TreeSCurveStep1;
			tree->_maxVelocity = maxVelocity;
			tree->_maxAcceleration = maxAcceleration;
			tree->_maxJerk = maxJerk;
			tree->_currentPosition = currentPosition;
			tree->_currentVelocity = currentVelocity;
			tree->_currentAcceleration = currentAcceleration;
			tree->_targetPosition = targetPosition;
			tree->_targetVelocity = targetVelocity;
			tree->_tcurr = tcurr;
			tree->_tcurrNotChanging = tcurr;

			tree->_tmin = RealMax;

			tree->_inopProfileBegin = InoperTimeProfileSCurve::notAssigned;
			tree->_inopProfileEnd = InoperTimeProfileSCurve::notAssigned;
			tree->_t1begin = RealMax;
			tree->_t1end = RealMax;
			tree->_inoperTimeExist = false;

			return tree;
		}

		void deleteDecisionTreeSCurveStep1(TreeSCurveStep1 * decisionTreeStep1)
		{
			delete decisionTreeStep1;
		}

		void calculateMinimumTimeAndInoperTimeSCurve(TreeSCurveStep1 * decisionTreeStep1)
		{
			LOGIF(abs(decisionTreeStep1->_currentAcceleration) <= decisionTreeStep1->_maxAcceleration, "current acceleration must be smaller than max acceleration");

			// decision tree start!
		//decisionBox_01:
			if (Decision1_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_02;
			}
			else
			{
				//ReverseSign_SCurveStep1(decisionTreeStep1);
				//goto decisionBox_02;
				goto decisionBox_xx;
			}

		decisionBox_02:
			if (Decision2_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_03;
			}
			else
			{
				goto decisionBox_12;
			}

		decisionBox_03:
			if (Decision3_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_04;
			}
			else
			{
				goto decisionBox_08;
			}

		decisionBox_04:
			if (Decision4_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_05;
			}
			else
			{
				goto decisionBox_24;
			}

		decisionBox_05:
			if (Decision5_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_06;
			}
			else
			{
				goto decisionBox_07;
			}

		decisionBox_06:
			if (Decision6_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTrapZeroNegTri");
				calculateTime_PosTrapZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				//FromAToZero_SCurveStep1(decisionTreeStep1);
				//setInopProfile_PosTrapZeroNegTri_SCurveStep1(decisionTreeStep1);
				//calcInopTime_SCurveStep1(decisionTreeStep1);
				return;
			}
			else
			{
				LOG(" PosTrapNegTri");
				calculateTime_PosTrapNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_07:
			if (Decision7_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_17;
			}
			else
			{
				LOG(" PosTrapNegTri");
				calculateTime_PosTrapNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_08:
			if (Decision8_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_09;
			}
			else
			{
				goto decisionBox_xx;
			}

		decisionBox_09:
			if (Decision9_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_10;
			}
			else
			{
				goto decisionBox_11;
			}

		decisionBox_10:
			if (Decision10_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_05;
			}
			else
			{
				LOG(" PosTriNegTri");
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_11:
			if (Decision11_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTriZeroNegTri");
				calculateTime_PosTriZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}
			else
			{
				LOG(" PosTriNegTri");
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_12:
			if (Decision12_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_13;
			}
			else
			{
				goto decisionBox_14;
			}

		decisionBox_13:
			if (Decision13_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_xx;
			}
			else
			{
				goto decisionBox_xx;
			}

		decisionBox_14:
			if (Decision14_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_20;
			}
			else
			{
				goto decisionBox_16;
			}

		decisionBox_15:
			if (Decision15_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				// maybe restoring functions are net needed... depends on the algorithm of calculating inoperative time
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				// restoring -end-
				return;
			}
			else
			{
				goto decisionBox_19;
			}

		decisionBox_16:
			if (Decision16_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_18;
			}
			else
			{
				goto decisionBox_15;
			}

		decisionBox_17:
			if (Decision17_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTrapZeroNegTrap");
				calculateTime_PosTrapZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}
			else
			{
				LOG(" PosTrapNegTrap");
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_18:
			if (Decision18_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapZeroPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}

		decisionBox_19:
			if (Decision19_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapZeroPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}

		decisionBox_20:
			if (Decision20_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_21;
			}
			else
			{
				goto decisionBox_22;
			}

		decisionBox_21:
			if (Decision21_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTriNegTrap");
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}
			else
			{
				LOG(" PosTriZeroNegTrap");
				calculateTime_PosTriZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_22:
			if (Decision22_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_23;
			}
			else
			{
				LOG(" PosTriNegTrap");
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_23:
			if (Decision23_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTrapNegTrap");
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}
			else
			{
				LOG(" PosTrapZeroNegTrap");
				calculateTime_PosTrapZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				return;
			}

		decisionBox_24:
			if (Decision24_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_25;
			}
			else
			{
				goto decisionBox_26;
			}

		decisionBox_25:
			if (Decision25_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriZeroPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}

		decisionBox_26:
			if (Decision26_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_27;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}

		decisionBox_27:
			if (Decision27_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapZeroPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				return;
			}

		decisionBox_xx:
			LOG(" not implemented yet....");
			return;

		}




		void FromAToZero_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			LOGIF(dtStep1->_currentAcceleration >= 0.0, "curAcc must be larger than zero - 'FromAToZero_SCurveStep1");
			dtStep1->_tcurr += dtStep1->_currentAcceleration / dtStep1->_maxJerk;
			dtStep1->_currentPosition += calcDP_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity);
			dtStep1->_currentVelocity += calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk);
			dtStep1->_currentAcceleration = 0.0;
		}

		void FromZeroToA_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			// restore function of 'FromAToZero_SCurveStep1'
			LOGIF(dtStep1->_tcurr - dtStep1->_tcurrNotChanging >= 0.0, "delta time must be larger than zero - 'FromZeroToA_SCurveStep1'");
			dtStep1->_currentAcceleration = dtStep1->_maxJerk * (dtStep1->_tcurr - dtStep1->_tcurrNotChanging);
			dtStep1->_currentVelocity -= calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk);
			dtStep1->_currentPosition -= calcDP_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity);
			dtStep1->_tcurr = dtStep1->_tcurrNotChanging;
		}

		void ReverseSign_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			dtStep1->_currentPosition *= -1;
			dtStep1->_currentVelocity *= -1;
			dtStep1->_currentAcceleration *= -1;
			dtStep1->_targetPosition *= -1;
			dtStep1->_targetVelocity *= -1;
		}

		bool Decision1_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << "decision box: 01";
			if (dtStep1->_currentAcceleration >= 0)
				return true;
			return false;
		}

		bool Decision2_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 02";
			// called when curAcc >= 0
			LOGIF(dtStep1->_currentVelocity + calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk) <= dtStep1->_maxVelocity, "if a->0, velocity cannot exceed maxVel..., try smaller input of curVel");
			if (dtStep1->_currentVelocity + calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk) <= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision3_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 03";
			if (dtStep1->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) <= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision4_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 04";
			if (dtStep1->_currentPosition + calcDP_TrapezoidAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision5_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 05";
			//cout << "decision5" << endl;
			//cout << dtStep1->_maxVelocity << '\t' << calcDV_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) << endl;
			//cout << dtStep1->_maxVelocity + calcDV_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) << '\t' << dtStep1->_targetVelocity << endl;
			//cout << "======" << endl;
			if (dtStep1->_maxVelocity + calcDV_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) <= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision6_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 06";
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_TrapezoidAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision7_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 07";
			Real vaftertrap = dtStep1->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk);
			if (dtStep1->_currentPosition + calcDP_TrapezoidAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, vaftertrap)
				+ calcDP_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, vaftertrap) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision8_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 08";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, ahigh, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision9_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 09";
			if (dtStep1->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) <= dtStep1->_maxVelocity)
				return true;
			return false;
		}

		bool Decision10_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 10";
			Real vaftertri = dtStep1->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk);
			LOGIF(vaftertri >= dtStep1->_targetVelocity, "wrong1 - Decision10_SCurveStep1");
			LOGIF(vaftertri <= dtStep1->_maxVelocity, "wrong2 - Decision10_SCurveStep1");
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep1->_maxJerk, vaftertri, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep1->_maxJerk, vaftertri) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision11_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 11";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, ahigh, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision12_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 12";
			Real vafterbackslash = dtStep1->_currentVelocity + calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk);
			LOGIF(vafterbackslash >= dtStep1->_targetVelocity, "wrong1 - Decision12_SCurveStep1");
			LOGIF(vafterbackslash <= dtStep1->_maxVelocity, "wrong2 - Decision12_SCurveStep1");
			if (vafterbackslash + calcDV_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) <= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision13_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 13";
			Real vafterbackslash = dtStep1->_currentVelocity + calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk);
			LOGIF(vafterbackslash >= dtStep1->_targetVelocity, "wrong1 - Decision13_SCurveStep1");
			LOGIF(vafterbackslash <= dtStep1->_maxVelocity, "wrong2 - Decision13_SCurveStep1");
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep1->_maxJerk, vafterbackslash, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep1->_maxJerk, vafterbackslash) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision14_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 14";
			Real vafterbackslash = dtStep1->_currentVelocity + calcDV_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk);
			LOGIF(vafterbackslash >= dtStep1->_targetVelocity, "wrong1 - Decision14_SCurveStep1");
			LOGIF(vafterbackslash <= dtStep1->_maxVelocity, "wrong2 - Decision14_SCurveStep1");
			if (dtStep1->_currentPosition + calcDP_BackSlashAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, vafterbackslash, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision15_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 15";
			Real vaftertrap = dtStep1->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk);
			if (dtStep1->_currentPosition + calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, vaftertrap)
				+ calcDP_reverseVShapeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, vaftertrap) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision16_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 16";
			if (-dtStep1->_maxVelocity + calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision17_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 17";
			if (dtStep1->_currentPosition + calcDP_TrapezoidAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision18_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 18";
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision19_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 19";
			if (dtStep1->_currentPosition + calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision20_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 20";
			if (dtStep1->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= dtStep1->_maxVelocity)
				return true;
			return false;
		}

		bool Decision21_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 21";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity);
			if (dtStep1->_currentPosition + calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, ahigh, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity) >= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision22_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 22";
			Real vafterreversev = dtStep1->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk);
			if (dtStep1->_currentPosition + calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, vafterreversev, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;

			return false;
		}

		bool Decision23_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 23";
			if (dtStep1->_currentPosition + calcDP_TrapezoidAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity) >= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision24_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 24";
			if (dtStep1->_currentVelocity + calcDV_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) <= -dtStep1->_maxVelocity)
				return true;
			return false;
		}

		bool Decision25_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 25";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			if (dtStep1->_currentPosition + calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity) >= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision26_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 26"; 
			Real vaftervshape = dtStep1->_currentVelocity + calcDV_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk);
			if (dtStep1->_currentPosition + calcDP_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, vaftervshape, dtStep1->_targetVelocity) >= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision27_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 27";
			if (dtStep1->_currentPosition + calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity) >= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool inopTimeDecision1_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			// called when curAcc = 0 (after the function FromAToZero call)
			// 일단 curVel 도 양수라고 생각
			// 0에서 -amax 찍고 다시 0으로 올라왔을때 V 가 0 아래로 내려 가는지...
			// V가 0보다 크면 true, 아니면 false!
			if (dtStep1->_currentVelocity + calcDV_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= 0.0)
				return true;
			return false;
		}

		bool inopTimeDecision2_SCurveStep1(TreeSCurveStep1 * dtStep1) 
		{
			// v 가 0 일 때, a 가 0->maxAcc->0 의 reverse V shape 일때 v target을 넘는지: 넘으면 true
			if (0.0 + calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		// minimum time calc fcns
		void calculateTime_PosTriNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // numerical solution (case 1)
		{
			NewtonRaphson solver(2, 2);
			solver.settolerance(1E-10);
			solver.setfunction(PosTriNegTriFcn);
			solver.setfdata(decisionTreeStep1);

			VectorX X(2), result(2);
			Real ap1, ap2, e01, e12, e23, e34;
			
			X(0) = (decisionTreeStep1->_currentAcceleration + decisionTreeStep1->_maxAcceleration) / 2; // ap1
			X(1) = -decisionTreeStep1->_maxAcceleration / 2; // ap2

			srand((unsigned int)time(NULL));
			int count = 0;

			while (1)
			{
				count++;
				if (count > 10)
				{
					X(0) = makeRandLU(decisionTreeStep1->_currentAcceleration, decisionTreeStep1->_maxAcceleration);
					X(1) = makeRandLU(-decisionTreeStep1->_maxAcceleration, 0);
					count = 0;
				}

				solver.solve(X);
				result = solver.getResultX();

				if (result(0) > decisionTreeStep1->_currentAcceleration && result(0) <= decisionTreeStep1->_maxAcceleration && result(1) < 0 && result(1) >= -(decisionTreeStep1->_maxAcceleration))
					break;
					
				if (result(0) > decisionTreeStep1->_maxAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep1->_currentAcceleration) / 2;
				}
				else if (result(0) <= decisionTreeStep1->_currentAcceleration)
				{
					X(0) = (X(0) + decisionTreeStep1->_maxAcceleration) / 2;
				}

				if (X(1) < -decisionTreeStep1->_maxAcceleration)
				{
					X(1) = (X(1)) / 2;
				}
				else if (result(1) >= 0)
				{
					X(1) = (X(1) + (-decisionTreeStep1->_maxAcceleration)) / 2;
				}
			}

			ap1 = result(0), ap2 = result(1);
			e01 = (ap1 - decisionTreeStep1->_currentAcceleration) / decisionTreeStep1->_maxJerk;
			e12 = ap1 / decisionTreeStep1->_maxJerk;
			e23 = -ap2 / decisionTreeStep1->_maxJerk;
			e34 = e23;

			saveTime = e01 + e12 + e23 + e34;
		}

		void calculateTime_PosTriZeroNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 2)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap1, ap2, e23;
			ap1 = 1 / std::sqrt(2) * std::sqrt(a0*a0 + 2 * jmax*(vmax - v0));
			ap2 = -1 / std::sqrt(2) * std::sqrt(-a0*a0 + 2 * ap1*ap1 + 2 * jmax*v0 - 2 * jmax*vt);
			e23 = (6 * a0 * a0 * ap1 - 3 * a0 * a0 * ap2 + 6 * ap1 * ap1 * ap2 - 6 * jmax * jmax * p0 + 6 * jmax * jmax * pt -
				2 * a0 * a0 * a0 - 6 * ap1 * ap1 * ap1 + 6 * a0*jmax*v0 - 12 * ap1*jmax*v0 +
				6 * ap2 * jmax*v0 + 6 * ap2 * jmax*vt) / (6 * jmax * jmax * v0 - 3 * a0 * a0 * jmax + 6 * ap1 * ap1 * jmax);
			
			Real e01, e12, e34, e45;
			e01 = (ap1 - a0) / jmax; 
			e12 = ap1 / jmax;
			e34 = -ap2 / jmax;
			e45 = e34;

			saveTime = e01 + e12 + e23 + e34 + e45;
		}

		void calculateTime_PosTrapNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 3)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap2, e01, e12, e23, e34, e45;

			Real a, b, c, d, e;
			a = 12;
			b = -24 * amax;
			c = 12 * amax * amax + 24 * jmax*vt;
			d = -48 * amax*jmax*vt;
			e = 8 * std::pow(a0, 3) * amax - 3 * std::pow(a0, 4) - 6 * a0 * a0 * amax * amax - 12 * jmax * jmax * v0 * v0 + 12 * jmax * jmax * vt * vt +
				12 * a0 * a0 * jmax*v0 + 12 * amax * amax * jmax*v0 + 12 * amax * amax * jmax*vt + 24 * amax*jmax * jmax * p0 -
				24 * amax*jmax * jmax * pt - 24 * a0*amax*jmax*v0;

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

			ap2 = sol;
			e12 = (a0 * a0 - 2 * amax * amax + 2 * ap2 * ap2 - 2 * jmax*v0 + 2 * jmax*vt) / (2 * amax*jmax);
			e01 = (amax - a0) / jmax;
			e23 = amax / jmax;
			e34 = -ap2 / jmax;
			e45 = e34;

			saveTime = e01 + e12 + e23 + e34 + e45;
		}

		void calculateTime_PosTrapZeroNegTri_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 4)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap2, e12, e34;
			e12 = (-a0 * a0 + 2 * amax * amax + 2 * jmax*v0 - 2 * jmax*vmax) / (-2 * jmax*amax);
			ap2 = -1 / std::sqrt(2) * std::sqrt(-a0 * a0 + 2 * amax * amax + 2 * e12*jmax*amax + 2 * jmax*v0 - 2 * jmax*vt);
			e34 = (3 * a0 * a0 * ap2 - 6 * a0 * a0 * amax - 6 * amax * amax * ap2 + 6 * jmax * jmax * p0 - 6 * jmax * jmax * pt +
				2 * a0 * a0 * a0 + 6 * amax * amax * amax + 6 * e12*jmax * jmax * v0 + 3 * amax*e12 * e12 * jmax * jmax -
				6 * a0*jmax*v0 + 12 * amax*jmax*v0 - 6 * ap2*jmax*v0 - 6 * ap2*jmax*vt - 3 * a0 * a0 * e12*jmax
				+ 9 * amax * amax * e12*jmax - 6 * amax*ap2*e12*jmax) / (-6 * jmax * jmax * v0 + 3 * a0 * a0 *jmax - 6 * amax * amax *jmax - 6 * amax*e12*jmax* jmax);

			Real e01, e23, e45, e56;
			e01 = (amax - a0) / jmax;
			e23 = amax / jmax;
			e45 = -ap2 / jmax;
			e56 = e45;

			saveTime = e01 + e12 + e23 + e34 + e45 + e56;
		}

		void calculateTime_PosTrapNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 5)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real e12, e45;
			e45 = -(2 * jmax*vt + 3 * amax * amax - std::sqrt((std::pow(a0, 4)) / 2 - (4 * std::pow(a0, 3) * amax) / 3 +
				std::pow(amax, 4) + a0*a0 * amax*amax + 2 * jmax*jmax * v0*v0 + 2 * jmax*jmax * vt*vt - 2 * a0*a0 * jmax*v0 -
				2 * amax*amax * jmax*v0 - 2 * amax * amax * jmax*vt - 4 * amax*jmax*jmax * p0 + 4 * amax*jmax * jmax * pt + 4 * a0*amax*jmax*v0)) / (2 * amax*jmax);
			e12 = (a0 * a0 - 2 * jmax*v0 + 2 * jmax*vt + 2 * amax*e45*jmax) / (2 * amax*jmax);

			Real e01, e23, e34, e56;
			e01 = (amax - a0) / jmax;
			e23 = amax / jmax;
			e34 = e23;
			e56 = e23;

			saveTime = e01 + e12 + e23 + e34 + e45 + e56;
		}

		void calculateTime_PosTrapZeroNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 6)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real e01, e12, e23, e34, e45, e56, e67;
			e12 = -(-a0 * a0 + 2 * amax * amax + 2 * jmax*v0 - 2 * jmax*vmax) / (2 * amax*jmax);
			e56 = -(amax * amax - jmax*vmax + jmax*vt) / (amax*jmax);
			e34 = -(8 * std::pow(a0, 3) * amax - 3 * std::pow(a0, 4) - 6 * a0*a0 * amax*amax - 12 * jmax*jmax * v0*v0 + 24 * jmax*jmax * vmax*vmax -
				12 * jmax*jmax * vt*vt + 12 * a0*a0 * jmax*v0 + 12 * amax*amax * jmax*v0 + 24 * amax*amax * jmax*vmax +
				12 * amax*amax * jmax*vt + 24 * amax*jmax*jmax * p0 - 24 * amax*jmax*jmax * pt - 24 * a0*amax*jmax*v0) / (24 * amax*jmax*jmax * vmax);

			e01 = (amax - a0) / jmax;
			e23 = amax / jmax;
			e45 = e23;
			e67 = e23;

			saveTime = e01 + e12 + e23 + e34 + e45 + e56 + e67;
		}

		void calculateTime_PosTriNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 7)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap1, e01, e12, e23, e34, e45;

			Real a, b, c, d, e;
			a = 12;
			b = 24 * amax;
			c = -12 * a0*a0 + 12 * amax*amax + 24 * jmax*v0;
			d = -24 * a0*a0 * amax + 48 * amax*jmax*v0;
			e = 8 * std::pow(a0, 3) * amax + 3 * std::pow(a0, 4) - 6 * a0*a0 * amax*amax + 12 * jmax*jmax * v0*v0 - 12 * jmax*jmax * vt*vt
				- 12 * a0 * a0 * jmax*v0 + 12 * amax * amax * jmax*v0 + 12 * amax * amax * jmax*vt + 24 * amax*jmax * jmax * p0
				- 24 * amax*jmax*jmax * pt - 24 * a0*amax*jmax*v0;

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

			ap1 = sol;
			e34 = -(a0 * a0 + 2 * amax * amax - 2 * ap1 * ap1 - 2 * jmax*v0 + 2 * jmax*vt) / (2 * amax*jmax);

			e01 = (ap1 - a0) / jmax;
			e12 = ap1 / jmax;
			e23 = amax / jmax;
			e45 = e23;

			saveTime = e01 + e12 + e23 + e34 + e45;
		}
		
		void calculateTime_PosTriZeroNegTrap_SCurveStep1(TreeSCurveStep1* decisionTreeStep1, Real& saveTime) // closed-form solution (case 8)
		{
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(decisionTreeStep1, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap1, e01, e12, e23, e34, e45, e56;
			ap1 = std::sqrt(0.5*a0*a0 + vmax*jmax - v0*jmax);

			e01 = (ap1 - a0) / jmax;
			e12 = ap1 / jmax;
			e34 = amax / jmax;
			e56 = e34;

			Real v01, v12, v34, v45, v56;
			v01 = 0.5*(a0 + ap1)*e01;
			v12 = 0.5*ap1*e12;
			v34 = -0.5*amax*e34;
			v56 = v34;

			e45 = (v0 - vt + v01 + v12 + v34 + v56) / amax;

			v45 = -amax*e45;

			Real xi01, xi12, /*xi23, */xi34, xi45, xi56;
			xi01 = v0*e01 + 0.5*a0*e01*e01 + 1.0 / 6.0*jmax*e01*e01*e01;
			xi12 = (v0 + v01)*e12 + 0.5*ap1*e12*e12 - 1.0 / 6.0*jmax*e12*e12*e12;
			xi34 = (v0 + v01 + v12)*e34 - 1.0 / 6.0 *jmax*e34*e34*e34;
			xi45 = (vt - v45 - v56)*e45 - 0.5*amax*e45*e45;
			xi56 = (vt - v56)*e56 - 0.5*amax*e56*e56 + 1.0 / 6.0*jmax*e56*e56*e56;

			e23 = (pt - p0 - xi01 - xi12 - xi34 - xi45 - xi56) / vmax;

			saveTime = e01 + e12 + e23 + e34 + e45 + e56;
		}

		//void calcInopTime_SCurveStep1(TreeSCurveStep1 * decisionTreeStep1)
		//{
		//	switch (decisionTreeStep1->_inopProfileBegin)
		//	{
		//	case InoperTimeProfileSCurve::inop_NegTriPosTri:
		//		break;
		//	case InoperTimeProfileSCurve::inop_NegTriPosTrap:
		//		break;
		//	case InoperTimeProfileSCurve::inop_NegTrapPosTri:
		//		break;
		//	case InoperTimeProfileSCurve::inop_NegTrapPosTrap:
		//		ReverseSign_SCurveStep1(decisionTreeStep1);
		//		calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_t1begin);
		//		break;
		//	default:
		//		LOG("not assigned - no inoperative time exists");
		//	}

		//	switch (decisionTreeStep1->_inopProfileEnd)
		//	{
		//	case InoperTimeProfileSCurve::inop_NegTriPosTri:
		//		break;
		//	case InoperTimeProfileSCurve::inop_NegTriPosTrap:
		//		break;
		//	case InoperTimeProfileSCurve::inop_NegTrapPosTri:
		//		break;
		//	case InoperTimeProfileSCurve::inop_NegTrapPosTrap:
		//		ReverseSign_SCurveStep1(decisionTreeStep1);
		//		calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_t1end);
		//		break;
		//	default:
		//		LOG("not assigned - no inoperative time exists");
		//	}
		//}

		//void setInopProfile_PosTrapZeroNegTri_SCurveStep1(TreeSCurveStep1 * decisionTreeStep1)
		//{
		//	// tri 가 붙으면 tri 도 되고 trap 도 될 수 있을듯...??
		//	if (inopTimeDecision1_SCurveStep1(decisionTreeStep1))
		//	{
		//		// 여기 잘못함...
		//		//// NegTrap (v goes to zero) will be assigned
		//		//if (inopTimeDecision2_SCurveStep1(decisionTreeStep1))
		//		//{
		//		//	decisionTreeStep1->_inopProfileBegin = InoperTimeProfileSCurve::inop_NegTrapPosTri;
		//		//}
		//		//else
		//		//{
		//		//	decisionTreeStep1->_inopProfileBegin = InoperTimeProfileSCurve::inop_NegTrapPosTrap;
		//		//	decisionTreeStep1->_inopProfileEnd = InoperTimeProfileSCurve::inop_NegTrapPosTrap;
		//		//}
		//	}
		//	else
		//	{
		//		// NegTri (v goes to zero) will be assigned
		//		if (inopTimeDecision2_SCurveStep1(decisionTreeStep1))
		//		{
		//			decisionTreeStep1->_inopProfileBegin = InoperTimeProfileSCurve::inop_NegTriPosTri;
		//		}
		//		else
		//		{
		//			decisionTreeStep1->_inopProfileBegin = InoperTimeProfileSCurve::inop_NegTriPosTrap;
		//		}
		//	}
		//}

		// functions for numerical process
		void PosTriNegTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data) // case 1
		{
			TreeSCurveStep1* tree = reinterpret_cast<TreeSCurveStep1*>(f_data);
			Real a0, vt, v0, pt, p0, jmax, amax, vmax;
			substitutefcn(tree, a0, vt, v0, pt, p0, jmax, amax, vmax);

			Real ap1 = x(0), ap2 = x(1);
			f(0) = vt - v0 - (ap1*ap1 / jmax - 0.5*a0*a0 / jmax - ap2*ap2 / jmax);
			f(1) = pt - p0 - (-2 * vt*ap2 / jmax - ap2*ap2*ap2 / (jmax*jmax) + 2 * v0*ap1 / jmax - v0*a0 / jmax + a0*a0*a0 / (3 * jmax*jmax) +
				ap1*ap1*ap1 / (jmax*jmax) - a0*a0*ap1 / (jmax*jmax));

			Real det = -(2 * (a0*a0 * ap2 - 3 * ap1*ap1 * ap2 + 3 * ap1*ap2*ap2 + 2 * jmax*vt*ap1 - 2 * jmax*v0*ap2)) / (jmax*jmax*jmax);
			ig(0, 0) = ((2 * vt) / jmax + (3 * ap2 * ap2) / (jmax*jmax)) / det;
			ig(0, 1) = -((2 * ap2) / jmax) / det;
			ig(1, 0) = -(a0 * a0 / (jmax *jmax) - (2 * v0) / jmax - (3 * ap1 * ap1) / (jmax*jmax)) / det;
			ig(1, 1) = (-(2 * ap1) / jmax) / det;
		}

		//void PosTrapNegTriFcn(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data) // case 3
		//{
		//	TreeSCurveStep1* tree = reinterpret_cast<TreeSCurveStep1*>(f_data);
		//	Real a0, vt, v0, pt, p0, jmax, amax, vmax;
		//	substitutefcn(tree, a0, vt, v0, pt, p0, jmax, amax, vmax);

		//	Real ap2 = x(0), e12 = x(1);
		//	f(0) = (-a0 *a0 + 2 * amax * amax + 2 * e12*jmax*amax - 2 * ap2 * ap2 + 2 * jmax*v0 - 2 * jmax*vt) / (2 * jmax);
		//	f(1) = -(6 * a0 * a0 * amax - 3 * a0 * a0 * ap2 + 6 * amax * amax * ap2 - 6 * jmax * jmax * p0 + 6 * jmax * jmax * pt - 2 * a0 * a0 * a0 -
		//		6 * amax * amax * amax - 6 * e12*jmax *jmax * v0 - 3 * amax*e12 *e12 * jmax * jmax + 6 * a0*jmax*v0 - 12 * amax*jmax*v0 + 6 * ap2*jmax*v0 +
		//		6 * ap2*jmax*vt + 3 * a0 * a0 * e12*jmax - 9 * amax * amax * e12*jmax + 6 * amax*ap2*e12*jmax) / (6 * jmax * jmax);

		//	Real det = (2 * a0 * a0 * ap2 - a0 * a0 * amax + 4 * amax*ap2*ap2 - 6 * amax*amax * ap2 + 2 * amax*amax*amax + 2 * amax*jmax*v0 -
		//		4 * ap2*jmax*v0 + 2 * amax*jmax*vt + 2 * amax*amax * e12*jmax - 4 * amax*ap2*e12*jmax) / (2 * jmax*jmax);
		//	ig(0, 0) = ((2 * jmax*v0 - 2 * amax*ap2 - a0 * a0 + 3 * amax * amax + 2 * amax*e12*jmax) / (2 * jmax)) / det;
		//	ig(1, 1) = (-(2 * ap2) / jmax) / det;
		//	ig(0, 1) = -amax / det;
		//	ig(1, 0) = (-3 * a0 * a0 + 6 * amax * amax + 6 * e12*jmax*amax + 6 * jmax*v0 + 6 * jmax*vt) / (6 * jmax * jmax) / det;
		//}


		void substitutefcn(TreeSCurveStep1* decisionTreeStep1, Real& a0, Real& vt, Real& v0, Real& pt, Real& p0, Real& jmax, Real& amax, Real& vmax)
		{
			a0 = decisionTreeStep1->_currentAcceleration;
			vt = decisionTreeStep1->_targetVelocity;
			v0 = decisionTreeStep1->_currentVelocity;
			pt = decisionTreeStep1->_targetPosition;
			p0 = decisionTreeStep1->_currentPosition;
			jmax = decisionTreeStep1->_maxJerk;
			amax = decisionTreeStep1->_maxAcceleration;
			vmax = decisionTreeStep1->_maxVelocity;
		}
	}
}