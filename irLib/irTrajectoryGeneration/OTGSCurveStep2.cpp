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
			
			tree->_bReversed = false;

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
				// 일단 임시
				ReverseSign_SCurveStep2(decisionTreeStep2);
				decisionTreeStep2->_bReversed = true;
				goto decisionBox_02;
				//goto decisionBox_xx;
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
				goto decisionBox_18;
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
				goto decisionBox_13;
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
				goto decisionBox_37;
			}
			else
			{
				goto decisionBox_27;
			}
		decisionBox_12:
			if (Decision12_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_47;
			}
			else
			{
				goto decisionBox_41;
			}
		decisionBox_13:
			if (Decision13_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTriZeroPosTri");
				calculateProfile_PosTriZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_14;
			}
		decisionBox_14:
			if (Decision14_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_15;
			}
		decisionBox_15:
			if (Decision15_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_16;
			}
		decisionBox_16:
			if (Decision16_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroPosTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_17;
			}
		decisionBox_17:
			if (Decision17_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroPosTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroPosTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_18:
			if (Decision18_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_19;
			}
			else
			{
				goto decisionBox_25;
			}
		decisionBox_19:
			if (Decision19_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_20;
			}
			else
			{
				goto decisionBox_16;
			}
		decisionBox_20:
			if (Decision20_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_21;
			}
			else
			{
				goto decisionBox_23;
			}
		decisionBox_21:
			if (Decision21_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_22;
			}
			else
			{
				LOG(" PosTriZeroPosTrap");
				calculateProfile_PosTriZeroPosTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_22:
			if (Decision22_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroPosTri");
				calculateProfile_PosTrapZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" PosTrapZeroPosTrap");
				calculateProfile_PosTrapZeroPosTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_23:
			if (Decision23_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_24;
			}
			else
			{
				LOG(" PosTriZeroPosTrap");
				calculateProfile_PosTriZeroPosTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_24:
			if (Decision24_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroPosTri");
				calculateProfile_PosTrapZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" PosTriZeroPosTri");
				calculateProfile_PosTriZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_25:
			if (Decision25_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_26;
			}
			else
			{
				goto decisionBox_14;
			}
		decisionBox_26:
			if (Decision26_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroPosTri");
				calculateProfile_PosTrapZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" PosTriZeroPosTri");
				calculateProfile_PosTriZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_27:
			if (Decision27_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_28;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroNegTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_28:
			if (Decision28_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_29;
			}
			else
			{
				goto decisionBox_33;
			}
		decisionBox_29:
			if (Decision29_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_30;
			}
			else
			{
				goto decisionBox_32;
			}
		decisionBox_30:
			if (Decision30_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTriZeroNegTri");
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_31;
			}
		decisionBox_31:
			if (Decision31_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroNegTri");
				calculateProfile_PosTrapZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" PosTriZeroNegTri");
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_32:
			if (Decision32_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroNegTrap");
				calculateProfile_PosTrapZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_31;
			}
		decisionBox_33:
			if (Decision33_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_34;
			}
			else
			{
				goto decisionBox_36;
			}
		decisionBox_34:
			if (Decision34_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTriZeroNegTri");
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_35;
			}
		decisionBox_35:
			if (Decision35_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTriZeroNegTrap");
				calculateProfile_PosTriZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" PosTriZeroNegTri");
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_36:
			if (Decision36_SCurveStep2(decisionTreeStep2))
			{
				LOG(" PosTrapZeroNegTrap");
				calculateProfile_PosTrapZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_35;
			}
		decisionBox_37:
			if (Decision37_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_38;
			}
			else
			{
				goto decisionBox_40;
			}
		decisionBox_38:
			if (Decision38_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_39;
			}
		decisionBox_39:
			if (Decision39_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_40:
			if (Decision40_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_39;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroPosTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_41:
			if (Decision41_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_33;
			}
			else
			{
				goto decisionBox_42;
			}
		decisionBox_42:
			if (Decision42_SCurveStep2(decisionTreeStep2))
			{
				goto decisionBox_43;
			}
			else
			{
				goto decisionBox_45;
			}
		decisionBox_43:
			if (Decision43_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroNegTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroPosTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_44;
			}
		decisionBox_44:
			if (Decision44_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroNegTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroNegTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_45:
			if (Decision45_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTriZeroNegTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTriZeroPosTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_46;
			}
		decisionBox_46:
			if (Decision46_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroNegTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroPosTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroNegTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroPosTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_47:
			if (Decision47_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				goto decisionBox_48;
			}
		decisionBox_48:
			if (Decision48_SCurveStep2(decisionTreeStep2))
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroPosTri");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroNegTri_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
			else
			{
				LOG(" to do!!!!!!!!!!!!!! - 앞부분 저장 ");
				LOG(" NegTrapZeroPosTrap");
				FromAToZero_SCurveStep2(decisionTreeStep2);
				calculateProfile_PosTrapZeroNegTrap_SCurveStep2(decisionTreeStep2, profile);
				return;
			}
		decisionBox_xx:
			LOG(" not implemented yet....");
			return;

		}

		void FromAToZero_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			LOGIF(dtStep2->_currentAcceleration >= 0.0, "curAcc must be larger than zero - 'FromAToZero_SCurveStep2");
			dtStep2->_tcurr += dtStep2->_currentAcceleration / dtStep2->_maxJerk;
			dtStep2->_currentPosition += calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity);
			dtStep2->_currentVelocity += calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			dtStep2->_currentAcceleration = 0.0;
		}

		void ReverseSign_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			dtStep2->_currentPosition *= -1;
			dtStep2->_currentVelocity *= -1;
			dtStep2->_currentAcceleration *= -1;
			dtStep2->_targetPosition *= -1;
			dtStep2->_targetVelocity *= -1;
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
			Real tafterpostrap = dtStep2->_tcurr + calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, dtStep2->_targetVelocity);
			if (dtStep2->_currentPosition + calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, dtStep2->_targetVelocity)
				+ calcDP_constVel(dtStep2->_tsync - tafterpostrap, dtStep2->_targetVelocity) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision5_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 05";
			Real vafterpostrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterpostrap >= dtStep2->_targetVelocity, "wrong1 - Decision5_SCurveStep2");
			Real tafterpostrap = dtStep2->_tcurr + calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap);
			if (tafterpostrap + calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk) >= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision6_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 06";
			Real vafterpostrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterpostrap >= dtStep2->_targetVelocity, "wrong1 - Decision6_SCurveStep2");
			Real tafterpostrap = dtStep2->_tcurr + calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap);
			Real hldTime = dtStep2->_tsync - tafterpostrap - calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision6_SCurveStep2");
			if (dtStep2->_currentPosition + calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap)
				+ calcDP_constVel(hldTime, vafterpostrap)
				+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostrap) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision7_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 07";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, dtStep2->_targetVelocity);
			Real tafterpostri = dtStep2->_tcurr + calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_currentPosition + calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(dtStep2->_tsync - tafterpostri, dtStep2->_targetVelocity) <= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision8_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 08";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterpostri >= dtStep2->_targetVelocity, "wrong1 - Decision8_SCurveStep2");
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			if (dtStep2->_tcurr + calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk) >= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision9_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 09";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterpostri >= dtStep2->_targetVelocity, "wrong1 - Decision9_SCurveStep2");
			Real tafterpostri = dtStep2->_tcurr + calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - tafterpostri - calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision9_SCurveStep2");
			if (dtStep2->_currentPosition + calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk, vafterpostri) <= dtStep2->_targetPosition)
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


		bool Decision13_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 13";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterbackslash <= dtStep2->_targetVelocity, "wrong1 - Decision13_SCurveStep2");
			Real tafterbackslash = dtStep2->_tcurr + calcDT_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision13_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterbackslash)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk, vafterbackslash)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision14_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 14";
			Real vafternegtri = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtri);
			if (dtStep2->_tcurr
				+ calcDT_VShapeAcc(dtStep2->_currentAcceleration, alow, 0.0, dtStep2->_maxJerk)
				+ calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision15_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 15";
			Real vafternegtri = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtri);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_VShapeAcc(dtStep2->_currentAcceleration, alow, 0.0, dtStep2->_maxJerk)
				- calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision15_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_VShapeAcc(dtStep2->_currentAcceleration, alow, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafternegtri)
				+ calcDP_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision16_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 16";
			Real vafternegtri = dtStep2->_currentVelocity + calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_tcurr
				+ calcDT_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDT_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision17_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 17";
			Real vafternegtri = dtStep2->_currentVelocity + calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision17_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafternegtri)
				+ calcDP_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision18_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 18";
			//Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			//LOGIF(vafterbackslash <= dtStep2->_targetVelocity, "wrong1 - Decision18_SCurveStep2");
			if (dtStep2->_currentVelocity
				+ calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				<= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision19_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 19";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision19_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterbackslash)
				+ calcDP_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision20_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 20";
			if (dtStep2->_currentVelocity
				+ calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				<= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision21_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 21";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision21_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_TrapezoidAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision22_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 22";
			Real vafterpostrap = dtStep2->_targetVelocity - dtStep2->_currentVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision22_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap)
				+ calcDP_constVel(hldTime, vafterpostrap)
				+ calcDP_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostrap)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision23_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 23";
			Real vafterpostri = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostri);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk)
				- calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision23_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision24_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 24";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision24_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk, vafterpostri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision25_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 25 (same as 13)";
			return Decision13_SCurveStep2(dtStep2);
		}

		bool Decision26_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 26";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision26_SCurveStep2");
			if (dtStep2->_targetPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk, vafterpostri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision27_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 27";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(vafterbackslash >= dtStep2->_targetVelocity, "wrong1 - Decision27_SCurveStep2");
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision27_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterbackslash)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk, vafterbackslash)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision28_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 28";
			if (dtStep2->_currentVelocity
				+ calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				<= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision29_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 29";
			Real vafterpostrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_tcurr
				+ calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap)
				+ calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision30_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 30";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			if (dtStep2->_tcurr
				+ calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision31_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 31";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_maxAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision31_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk, vafterpostri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision32_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 32";
			Real vafterpostrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr
				- calcDT_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap)
				- calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision32_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_TrapezoidAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostrap)
				+ calcDP_constVel(hldTime, vafterpostrap)
				+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostrap)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision33_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 33";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_tcurr
				+ calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDT_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision34_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 34";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity,
				dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk));
			if (dtStep2->_tcurr
				+ calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk)
				+ calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision35_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 35";
			Real vafterpostri = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafterpostri);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk)
				- calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision35_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, ahigh, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision36_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 36";
			Real vafterpostri = dtStep2->_currentVelocity + calcDV_revesreVShpaeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision36_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep2->_currentAcceleration, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterpostri)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterpostri, dtStep2->_targetVelocity)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision37_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 37";
			Real vafternegtrap = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_tcurr
				+ calcDT_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				+ calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision38_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 38";
			Real vafternegtri = dtStep2->_currentVelocity + calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity);
			if (dtStep2->_tcurr
				+ calcDT_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDT_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision39_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 39";
			Real vafternegtri = dtStep2->_currentVelocity + calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision39_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafternegtri)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep2->_maxJerk, vafternegtri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision40_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 40";
			Real vafternegtrap = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr
				- calcDT_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				- calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision40_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				+ calcDP_constVel(hldTime, vafternegtrap)
				+ calcDP_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtrap)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision41_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 41";
			Real vafterbackslash = dtStep2->_currentVelocity + calcDV_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision41_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_BackSlashAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafterbackslash)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafterbackslash, dtStep2->_targetVelocity)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision42_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 42";
			if (dtStep2->_currentVelocity
				+ calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				+ calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				<= dtStep2->_targetVelocity)
				return true;
			return false;
		}

		bool Decision43_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 43";
			Real vafternegtri = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(dtStep2->_currentAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtri);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_VShapeAcc(dtStep2->_currentAcceleration, alow, 0.0, dtStep2->_maxJerk)
				- calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision43_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_VShapeAcc(dtStep2->_currentAcceleration, alow, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafternegtri)
				+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision44_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 44";
			Real vafternegtri = dtStep2->_currentVelocity + calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision44_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafternegtri)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep2->_maxJerk, vafternegtri)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision45_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 45";
			Real vafternegtri = dtStep2->_currentVelocity + calcDV_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr - calcDT_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				- calcDT_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision45_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_VShapeAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity)
				+ calcDP_constVel(hldTime, vafternegtri)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtri, dtStep2->_targetVelocity)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision46_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 46";
			Real vafternegtrap = dtStep2->_targetVelocity - calcDV_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr
				- calcDT_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				- calcDT_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision46_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				+ calcDP_constVel(hldTime, vafternegtrap)
				+ calcDP_VShapeAcc(0.0, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtrap)
				<= dtStep2->_targetPosition)
				return true;
			return false;
		}

		bool Decision47_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 47";
			Real vafternegtrap = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			if (dtStep2->_tcurr
				+ calcDT_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				+ calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk)
				>= dtStep2->_tsync)
				return true;
			return false;
		}

		bool Decision48_SCurveStep2(TreeSCurveStep2 * dtStep2)
		{
			cout << " - 48";
			Real vafternegtrap = dtStep2->_targetVelocity - calcDV_revesreVShpaeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			Real hldTime = dtStep2->_tsync - dtStep2->_tcurr
				- calcDT_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				- calcDT_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk);
			LOGIF(hldTime >= 0.0, "holding time must be positive - Decision48_SCurveStep2");
			if (dtStep2->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep2->_currentAcceleration, -dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, dtStep2->_currentVelocity, vafternegtrap)
				+ calcDP_constVel(hldTime, vafternegtrap)
				+ calcDP_reverseVShapeAcc(0.0, dtStep2->_maxAcceleration, 0.0, dtStep2->_maxJerk, vafternegtrap)
				<= dtStep2->_targetPosition)
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