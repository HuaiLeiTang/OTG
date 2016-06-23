#include "OTGOperator.h"


namespace irLib
{
	namespace irTG
	{
		Real calcDP_constVel(Real dt, Real vel)
		{
			return vel*dt;
		}
		// calc delta position following by shape of velocity profile
		Real calcDP_VShapeVel(Real vi, Real vlow, Real vf, Real amax)
		{
			return 0.5*(vi*vi + vf*vf - 2 * vlow*vlow) / amax;
		}


		Real calcDP_reverseVShapeVel(Real vi, Real vhigh, Real vf, Real amax)
		{
			return 0.5 * (2 * vhigh*vhigh - vi*vi - vf*vf) / amax;
		}


		// calc delta position, delta velocity functions following by shape of acceleration profile
		Real calcDV_ConstAcc(Real dt, Real acc)
		{
			return acc*dt;
		}

		Real calcDP_ConstAcc(Real dt, Real acc, Real vi)
		{
			return (vi*dt + 0.5 * acc * dt* dt);
		}

		Real calcDP_ConstAccWithVf(Real vi, Real vf, Real acc)
		{
			return  0.5*(vf*vf - vi*vi) / acc;
		}

		Real calcDV_SlashAcc(Real ai, Real af, Real jmax)
		{
			// af > ai
			return 0.5*(af*af - ai*ai) / jmax;
		}

		Real calcDP_SlashAcc(Real ai, Real af, Real jmax, Real vi)
		{
			// af > ai
			Real tmpt = (af - ai) / jmax;
			return (jmax*tmpt*tmpt*tmpt / 6 + ai*tmpt*tmpt / 2 + vi*tmpt);
		}


		Real calcDV_BackSlashAcc(Real ai, Real af, Real jmax)
		{
			// ai > af
			return 0.5*(ai*ai - af*af) / jmax;
		}

		Real calcDP_BackSlashAcc(Real ai, Real af, Real jmax, Real vi)
		{
			// ai > af
			Real tmpt = (ai - af) / jmax;
			return (-jmax*tmpt*tmpt*tmpt / 6 + ai*tmpt*tmpt / 2 + vi*tmpt);
		}

		Real calcDT_BackSlashAcc(Real ai, Real af, Real jmax)
		{
			return (ai - af) / jmax;
		}

		Real calcDV_revesreVShpaeAcc(Real ai, Real ahigh, Real af, Real jmax)
		{
			// ahigh > ai, af
			return calcDV_SlashAcc(ai, ahigh, jmax) + calcDV_BackSlashAcc(ahigh, af, jmax);
		}

		Real calcDP_reverseVShapeAcc(Real ai, Real ahigh, Real af, Real jmax, Real vi)
		{
			// ahigh > ai, af
			return calcDP_SlashAcc(ai, ahigh, jmax, vi) + calcDP_BackSlashAcc(ahigh, af, jmax, vi + calcDV_SlashAcc(ai, ahigh, jmax));
		}

		Real calcDT_reverseVShapeAcc(Real ai, Real ahigh, Real af, Real jmax)
		{
			return (2 * ahigh - ai - af) / jmax;
		}

		Real calcAHigh_reverseVShapeAcc(Real ai, Real af, Real jmax, Real vi, Real vf)
		{
			// ahigh > ai, af
			Real tmpval = 0.5 * (ai*ai + af*af + 2 * jmax * (vf - vi));
			LOGIF(tmpval >= 0, "this value must be larger than zero - 'calcAHigh_reverseVShapeAcc'");
			tmpval = sqrt(tmpval);
			if ((-tmpval < ai) || (-tmpval < af))
			{
				return tmpval;
			}
			else
			{
				if (RealEqual(vi + calcDV_revesreVShpaeAcc(ai, tmpval, af, jmax), vf, EPS_COMPARE))
				{
					return tmpval;
				}
				else
				{
					LOGIF(RealEqual(vi + calcDV_revesreVShpaeAcc(ai, -tmpval, af, jmax), vf, EPS_COMPARE), "something wrong...... - 'calcAHigh_reverseVShapeAcc'");
					return -tmpval;
				}
			}
		}

		Real calcDV_VShapeAcc(Real ai, Real alow, Real af, Real jmax)
		{
			// alow < ai, af
			return calcDV_BackSlashAcc(ai, alow, jmax) + calcDV_SlashAcc(alow, af, jmax);
		}

		Real calcDP_VShapeAcc(Real ai, Real alow, Real af, Real jmax, Real vi)
		{
			// alow < ai, af
			return calcDP_BackSlashAcc(ai, alow, jmax, vi) + calcDP_SlashAcc(alow, af, jmax, vi + calcDV_BackSlashAcc(ai, alow, jmax));
		}

		Real calcDT_VShapeAcc(Real ai, Real alow, Real af, Real jmax)
		{
			return (ai + af - 2 * alow) / jmax;
		}

		Real calcALow_VShapeAcc(Real ai, Real af, Real jmax, Real vi, Real vf)
		{
			// alow < ai, af
			Real tmpval = 0.5 * (ai*ai + af*af - 2 * jmax*(vf - vi));
			LOGIF(tmpval >= 0, "this value must be larger than zero - 'calcALow_VShapeAcc'");
			tmpval = sqrt(tmpval);
			if ((tmpval > ai) || (tmpval > af))
			{
				return -tmpval;
			}
			else
			{
				if (RealEqual(vi + calcDV_VShapeAcc(ai, tmpval, af, jmax), vf, EPS_COMPARE))
				{
					return tmpval;
				}
				else
				{
					LOGIF(RealEqual(vi + calcDV_VShapeAcc(ai, -tmpval, af, jmax), vf, EPS_COMPARE), "something wrong...... - 'calcALow_VShapeAcc'");
					return -tmpval;
				}
			}
		}

		Real calcDP_TrapezoidAcc(Real ai, Real ahigh, Real af, Real jmax, Real vi, Real vf)
		{
			// ahigh > ai, af
			Real dv1 = calcDV_SlashAcc(ai, ahigh, jmax);
			Real hlddv = (vf - vi - dv1 - calcDV_BackSlashAcc(ahigh, af, jmax));
			LOGIF(hlddv >= 0, "holding delta velocity must be larger than zero");
			Real hldTime = hlddv / ahigh;
			//cout << dv1 << '\t' << hlddv << '\t' << calcDV_BackSlashAcc(ahigh, af, jmax) << '\t' << hldTime << endl;
			return calcDP_SlashAcc(ai, ahigh, jmax, vi) + calcDP_ConstAcc(hldTime, ahigh, vi + dv1) + calcDP_BackSlashAcc(ahigh, af, jmax, vi + dv1 + hlddv);
		}

		Real calcDT_TrapezoidAcc(Real ai, Real ahigh, Real af, Real jmax, Real vi, Real vf)
		{
			Real hlddv = (vf - vi - calcDV_SlashAcc(ai, ahigh, jmax) - calcDV_BackSlashAcc(ahigh, af, jmax));
			LOGIF(hlddv >= 0, "holding delta velocity must be larger than zero");
			return (ahigh - ai) / jmax + hlddv / ahigh + (ahigh - af) / jmax;
		}

		Real calcDP_NegTrapezoidAcc(Real ai, Real alow, Real af, Real jmax, Real vi, Real vf)
		{
			// alow < ai, af
			Real dv1 = calcDV_BackSlashAcc(ai, alow, jmax);
			Real hlddv = (vf - vi - dv1 - calcDV_SlashAcc(alow, af, jmax));
			LOGIF(hlddv <= 0, "holding delta velocity must be smaller than zero");
			Real hldTime = hlddv / alow;
			//cout << dv1 << '\t' << hlddv << '\t' << calcDV_SlashAcc(alow, af, jmax) << '\t' << hldTime << endl;
			return calcDP_BackSlashAcc(ai, alow, jmax, vi) + calcDP_ConstAcc(hldTime, alow, vi + dv1) + calcDP_SlashAcc(alow, af, jmax, vi + dv1 + hlddv);
		}

		Real calcDT_NegTrapezoidAcc(Real ai, Real alow, Real af, Real jmax, Real vi, Real vf)
		{
			Real hlddv = (vf - vi - calcDV_BackSlashAcc(ai, alow, jmax) - calcDV_SlashAcc(alow, af, jmax));
			LOGIF(hlddv <= 0, "holding delta velocity must be smaller than zero");
			return (ai - alow) / jmax + hlddv / alow + (af - alow) / jmax;
		}

		Real makeRandLU(Real lower, Real upper)
		{
			return lower + (upper - lower)*((double)rand() / 32767.0);
		}

		void calcQuarticAlgebraicEqn(Real a, Real b, Real c, Real d, Real e, ComplexNumber* x)
		{
			Real p, q, det0, det1;

			p = (8 * a*c - 3 * b*b) / (8 * a*a);
			q = (b*b*b - 4 * a*b*c + 8 * a*a*d) / (8 * a*a*a);
			det0 = c*c - 3 * b*d + 12 * a*e;
			det1 = 2 * c*c*c - 9 * b*c*d + 27 * b*b*e + 27 * a*d*d - 72 * a*c*e;

			Real det = det1*det1 - 4 * det0*det0*det0;
			ComplexNumber Q, S;

			if (det >= 0)
			{
				Q._rN = (det1 + std::sqrt(std::abs(det))) / 2;
			}
			else
			{
				Q._rN = det1 / 2;
				Q._iN = std::sqrt(std::abs(det)) / 2;
			}
			Q.cuberoot();
			S = 1.0 / 2.0 * ComplexNumber::sqrt(-2.0 / 3.0*p + 1.0 / 3.0 / a*(Q + det0 / Q));

			x[0] = -(b / 4 / a) - S + 0.5*ComplexNumber::sqrt(-4 * S*S - 2 * p + q / S);
			x[1] = -(b / 4 / a) - S - 0.5*ComplexNumber::sqrt(-4 * S*S - 2 * p + q / S);
			x[2] = -(b / 4 / a) + S + 0.5*ComplexNumber::sqrt(-4 * S*S - 2 * p - q / S);
			x[3] = -(b / 4 / a) + S - 0.5*ComplexNumber::sqrt(-4 * S*S - 2 * p - q / S);
		}

	}
}