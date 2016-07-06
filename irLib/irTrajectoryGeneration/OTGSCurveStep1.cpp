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
			tree->_bReversed = false;

			tree->_minProfile = ProfileSCurveStep1::profile_notAssigned;
			tree->_inopProfileBegin = ProfileSCurveStep1::profile_notAssigned;
			tree->_inopProfileEnd = ProfileSCurveStep1::profile_notAssigned;
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

			calculateMiniumumTimeSCurve(decisionTreeStep1);
			//calculateInoperTimeSCurve(decisionTreeStep1);

			return;
		}




		void calculateInoperTimeSCurve(TreeSCurveStep1 * decisionTreeStep1)
		{
			// check necessary conditions for existence of inop. time
			if (!Decision0_SCurveStep1InopTime(decisionTreeStep1))
				return;

			// 여기는 검증 필요함!
			if (decisionTreeStep1->_currentAcceleration < 0.0)
				ReverseSign_SCurveStep1(decisionTreeStep1);

			// 1,2 구분 없고, 3,4 구분 없을 수도 있음
			if (decisionTreeStep1->_currentVelocity >= 0.0)
			{
				// min time 구하고 아래 같은 조건 필요할 수도 있음...
				// case 1/2 를 만들 수 있는 minimum profile 일 때만 inoperative time interval 찾을 수 있도록!
				// 이 조건 넣는다면, min time 구할 때 reverse 하고 구했었던건 어떻게 처리할지 생각하기..
				// 아래 세개 프로파일 말고도 많이 있을 수 있음 (일단 세개만)
				if (decisionTreeStep1->_minProfile != profile_PosTriNegTri		 &&
					decisionTreeStep1->_minProfile != profile_PosTriZeroNegTri	 &&
					decisionTreeStep1->_minProfile != profile_PosTrapNegTri		 &&
					decisionTreeStep1->_minProfile != profile_PosTrapZeroNegTri	 &&
					decisionTreeStep1->_minProfile != profile_PosTrapNegTrap	 &&
					decisionTreeStep1->_minProfile != profile_PosTrapZeroNegTrap &&
					decisionTreeStep1->_minProfile != profile_PosTriNegTrap		 &&
					decisionTreeStep1->_minProfile != profile_PosTriZeroNegTrap)
					return;


				if (decisionTreeStep1->_currentVelocity <= decisionTreeStep1->_targetVelocity)
				{
					SetProfile1_SCurveStep1InopTime(decisionTreeStep1);
				}
				else
				{
					SetProfile2_SCurveStep1InopTime(decisionTreeStep1);
				}
			}
			else
			{
				if (decisionTreeStep1->_minProfile != profile_NegTriPosTri		 &&
					decisionTreeStep1->_minProfile != profile_NegTriZeroPosTri	 &&
					decisionTreeStep1->_minProfile != profile_NegTrapPosTri		 &&
					decisionTreeStep1->_minProfile != profile_NegTrapZeroPosTri	 &&
					decisionTreeStep1->_minProfile != profile_NegTrapPosTrap	 &&
					decisionTreeStep1->_minProfile != profile_NegTrapZeroPosTrap &&
					decisionTreeStep1->_minProfile != profile_NegTriPosTrap		 &&
					decisionTreeStep1->_minProfile != profile_NegTriZeroPosTrap)
					return;

				// a 를 0으로 내릴 때 v 가 0 위로 넘어가면 inoperative time interval 없을 듯... 
				if (RealBigger(decisionTreeStep1->_currentVelocity + calcDV_BackSlashAcc(decisionTreeStep1->_currentAcceleration, 0.0, decisionTreeStep1->_maxJerk), 0.0, 1E-6))
					return;


				if (decisionTreeStep1->_currentVelocity <= decisionTreeStep1->_targetVelocity)
				{
					SetProfile3_SCurveStep1InopTime(decisionTreeStep1);
				}
				else
				{
					SetProfile4_SCurveStep1InopTime(decisionTreeStep1);
				}
			}


			if (decisionTreeStep1->_inoperTimeExist)
			{
				calcInopTimeProfile_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_t1begin += decisionTreeStep1->_tcurrNotChanging;
				decisionTreeStep1->_t1end += decisionTreeStep1->_tcurrNotChanging;
			}
			
		}

		void calcInopTimeProfile_SCurveStep1(TreeSCurveStep1 * decisionTreeStep1)
		{
			if (decisionTreeStep1->_inopProfileBegin == decisionTreeStep1->_inopProfileEnd)
			{
				// 같으면 같은 프로파일이니까 여러번돌려서 솔루션 찾고 큰거 작은거 나눠서 뱉어야 함...
			}
			else
			{
				// begin 이랑 end 랑 따로 스위치문 돌려서 시간 구하기

				//switch (begin)
				//{
				//case inop_NegTriPosTri:
				//	break;
				//default:
				//	break;
				//}


				//switch (begin)
				//{
				//case inop_NegTriPosTri:
				//	break;
				//default:
				//	break;
				//}
			}
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

		bool Decision28_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 28 (same as 24)";
			return (Decision24_SCurveStep1(dtStep1));
		}

		bool Decision29_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 29";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			if (-dtStep1->_maxVelocity
				+ calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk)
				<= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision30_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 30";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision31_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 31";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision32_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 32";
			if (dtStep1->_currentVelocity
				+ calcDV_revesreVShpaeAcc(dtStep1->_currentAcceleration, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk)
				>= dtStep1->_maxVelocity)
				return true;
			return false;
		}

		bool Decision33_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 33";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity);
			if (dtStep1->_maxVelocity
				+ calcDV_VShapeAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk)
				<= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision34_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 34";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity);
			Real alow = calcALow_VShapeAcc(0.0, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, ahigh, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_VShapeAcc(0.0, alow, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision35_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 35";
			Real ahigh = calcAHigh_reverseVShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, dtStep1->_maxVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_reverseVShapeAcc(dtStep1->_currentAcceleration, ahigh, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_NegTrapezoidAcc(0.0, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_maxVelocity, dtStep1->_targetVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision36_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 36";
			if (dtStep1->_currentVelocity
				+ calcDV_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk)
				<= -dtStep1->_maxVelocity)
				return true;
			return false;
		}

		bool Decision37_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 37";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			if (-dtStep1->_maxVelocity
				+ calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk)
				<= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision38_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 38";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision39_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 39";
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision40_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 40";
			Real vafterv = dtStep1->_currentVelocity + calcDV_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk);
			LOGIF(vafterv >= -dtStep1->_maxVelocity, "v after v-shape must be larger than -maxVel");
			LOGIF(vafterv <= dtStep1->_targetVelocity, "v after v-shape must be smaller than targetVel");
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, vafterv, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, vafterv)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision41_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 41";
			if (-dtStep1->_maxVelocity
				+ calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk)
				<= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision42_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 42";
			if (dtStep1->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision43_SCurveStep1(TreeSCurveStep1 * dtStep1)
		{
			cout << " - 43";
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, -dtStep1->_maxVelocity)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, -dtStep1->_maxVelocity)
				<= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision0_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			// necessary conditions for existing of inoperative time interval
			// if inoperatie time interval must not exists, return false;
			if (RealLess(dtStep1->_currentVelocity * dtStep1->_targetVelocity, 0.0, 1E-6) ||
				RealLess(dtStep1->_currentVelocity * (dtStep1->_targetPosition - dtStep1->_currentPosition), 0.0, 1E-6))
				return false;
			return true;
		}

		bool Decision1_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			if (dtStep1->_currentVelocity + calcDV_VShapeAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= 0.0)
				return true;
			return false;
		}

		bool Decision2_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, 0.0);
			if (dtStep1->_currentVelocity + calcDV_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk)
				+ calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision3_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, 0.0);
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, 0.0, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition + calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, 0.0) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision4_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			Real alow = calcALow_VShapeAcc(dtStep1->_currentAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, 0.0);
			if (dtStep1->_currentPosition
				+ calcDP_VShapeAcc(dtStep1->_currentAcceleration, alow, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, 0.0, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision5_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			if (0.0 + calcDV_revesreVShpaeAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk) >= dtStep1->_targetVelocity)
				return true;
			return false;
		}

		bool Decision6_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			Real ahigh = calcAHigh_reverseVShapeAcc(0.0, 0.0, dtStep1->_maxJerk, 0.0, dtStep1->_targetVelocity);
			if (dtStep1->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, 0.0)
				+ calcDP_reverseVShapeAcc(0.0, ahigh, 0.0, dtStep1->_maxJerk, 0.0) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		bool Decision7_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			if (dtStep1->_currentPosition
				+ calcDP_NegTrapezoidAcc(dtStep1->_currentAcceleration, -dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, dtStep1->_currentVelocity, 0.0)
				+ calcDP_TrapezoidAcc(0.0, dtStep1->_maxAcceleration, 0.0, dtStep1->_maxJerk, 0.0, dtStep1->_targetVelocity) <= dtStep1->_targetPosition)
				return true;
			return false;
		}

		void SetProfile1_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			// maxVel > tarVel > curVel > 0
			if (Decision1_SCurveStep1InopTime(dtStep1))
			{
				if (Decision5_SCurveStep1InopTime(dtStep1))
				{
					if (Decision6_SCurveStep1InopTime(dtStep1))
					{
						dtStep1->_inoperTimeExist = true;
						dtStep1->_inopProfileBegin = profile_NegTrapPosTri;
					}
				}
				else
				{
					if (Decision7_SCurveStep1InopTime(dtStep1))
					{
						dtStep1->_inoperTimeExist = true;
						dtStep1->_inopProfileBegin = profile_NegTrapPosTrap;
					}
				}
			}
			else
			{
				if (Decision2_SCurveStep1InopTime(dtStep1))
				{
					if (Decision3_SCurveStep1InopTime(dtStep1))
					{
						dtStep1->_inoperTimeExist = true;
						dtStep1->_inopProfileBegin = profile_NegTriPosTri;
					}
				}
				else
				{
					if (Decision4_SCurveStep1InopTime(dtStep1))
					{
						dtStep1->_inoperTimeExist = true;
						dtStep1->_inopProfileBegin = profile_NegTriPosTrap;
					}
				}
			}
		}

		void SetProfile2_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			// maxVel > curVel > tarVel > 0
		}

		void SetProfile3_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			// 0 > tarVel > curVel > -maxVel
		}

		void SetProfile4_SCurveStep1InopTime(TreeSCurveStep1 * dtStep1)
		{
			// 0 > curVel > tarVel > -maxVel
		}


		void calculateMiniumumTimeSCurve(TreeSCurveStep1 * decisionTreeStep1)
		{

			// decision tree start!
			//decisionBox_01:
			if (Decision1_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_02;
			}
			else
			{
				ReverseSign_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_bReversed = true;
				goto decisionBox_02;
				//goto decisionBox_xx;
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
				decisionTreeStep1->_minProfile = profile_PosTrapZeroNegTri;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTrapNegTri");
				calculateTime_PosTrapNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTrapNegTri;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_PosTrapNegTri;
				goto decisionSuccess;
			}

		decisionBox_08:
			if (Decision8_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_09;
			}
			else
			{
				goto decisionBox_28;
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
				decisionTreeStep1->_minProfile = profile_PosTriNegTri;
				goto decisionSuccess;
			}

		decisionBox_11:
			if (Decision11_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTriZeroNegTri");
				calculateTime_PosTriZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriZeroNegTri;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTriNegTri");
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriNegTri;
				goto decisionSuccess;
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
				goto decisionBox_32;
			}
			else
			{
				goto decisionBox_36;
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
				decisionTreeStep1->_minProfile = profile_NegTrapPosTri;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_PosTrapZeroNegTrap;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTrapNegTrap");
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTrapNegTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapPosTri;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapZeroPosTri;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapZeroPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_PosTriNegTrap;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTriZeroNegTrap");
				calculateTime_PosTriZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriZeroNegTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_PosTriNegTrap;
				goto decisionSuccess;
			}

		decisionBox_23:
			if (Decision23_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTrapNegTrap");
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTrapNegTrap;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTrapZeroNegTrap");
				calculateTime_PosTrapZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTrapZeroNegTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTriZeroPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTriPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTriPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapZeroPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapPosTrap;
				goto decisionSuccess;
			}

		decisionBox_28:
			if (Decision28_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_29;
			}
			else
			{
				goto decisionBox_26;
			}

		decisionBox_29:
			if (Decision29_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_30;
			}
			else
			{
				goto decisionBox_31;
			}

		decisionBox_30:
			if (Decision30_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriPosTrap;
				goto decisionSuccess;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriZeroPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriZeroPosTrap;
				goto decisionSuccess;
			}

		decisionBox_31:
			if (Decision31_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriPosTri;
				goto decisionSuccess;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriZeroPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriZeroPosTri;
				goto decisionSuccess;
			}

		decisionBox_32:
			if (Decision32_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_33;
			}
			else
			{
				goto decisionBox_22;
			}

		decisionBox_33:
			if (Decision33_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_34;
			}
			else
			{
				goto decisionBox_35;
			}

		decisionBox_34:
			if (Decision34_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTriZeroNegTri");
				calculateTime_PosTriZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriZeroNegTri;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTriNegTri");
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriNegTri;
				goto decisionSuccess;
			}

		decisionBox_35:
			if (Decision35_SCurveStep1(decisionTreeStep1))
			{
				LOG(" PosTriZeroNegTrap");
				calculateTime_PosTriZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriZeroNegTrap;
				goto decisionSuccess;
			}
			else
			{
				LOG(" PosTriNegTrap");
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				decisionTreeStep1->_minProfile = profile_PosTriNegTrap;
				goto decisionSuccess;
			}

		decisionBox_36:
			if (Decision36_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_37;
			}
			else
			{
				goto decisionBox_40;
			}

		decisionBox_37:
			if (Decision37_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_38;
			}
			else
			{
				goto decisionBox_39;
			}

		decisionBox_38:
			if (Decision38_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriPosTrap;
				goto decisionSuccess;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriZeroPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriZeroNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriZeroPosTrap;
				goto decisionSuccess;
			}

		decisionBox_39:
			if (Decision39_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriPosTri;
				goto decisionSuccess;
			}
			else
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriZeroPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriZeroNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriZeroPosTri;
				goto decisionSuccess;
			}

		decisionBox_40:
			if (Decision40_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTriPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTriNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTriPosTri;
				goto decisionSuccess;
			}
			else
			{
				goto decisionBox_41;
			}

		decisionBox_41:
			if (Decision41_SCurveStep1(decisionTreeStep1))
			{
				goto decisionBox_42;
			}
			else
			{
				goto decisionBox_43;
			}

		decisionBox_42:
			if (Decision42_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapPosTrap");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapNegTrap_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTrapPosTrap;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapZeroPosTrap;
				goto decisionSuccess;
			}

		decisionBox_43:
			if (Decision43_SCurveStep1(decisionTreeStep1))
			{
				LOG("before call calculate minimum time function, move a to zero needed!!!");
				LOG(" NegTrapPosTri");
				FromAToZero_SCurveStep1(decisionTreeStep1);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				calculateTime_PosTrapNegTri_SCurveStep1(decisionTreeStep1, decisionTreeStep1->_tmin);
				ReverseSign_SCurveStep1(decisionTreeStep1);
				FromZeroToA_SCurveStep1(decisionTreeStep1);
				decisionTreeStep1->_minProfile = profile_NegTrapPosTri;
				goto decisionSuccess;
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
				decisionTreeStep1->_minProfile = profile_NegTrapZeroPosTri;
				goto decisionSuccess;
			}

		decisionBox_xx:
			LOG(" not implemented yet....");
			return;

		decisionSuccess:
			// 계산은 나중에... t1begin, t1end 프로파일 다 결정하고 나서!
			decisionTreeStep1->_tmin += decisionTreeStep1->_tcurrNotChanging;
			if (decisionTreeStep1->_bReversed)
				ReverseSign_SCurveStep1(decisionTreeStep1);
			return;

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