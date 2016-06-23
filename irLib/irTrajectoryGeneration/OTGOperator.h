#pragma once


#include <irUtils\Diagnostic.h>
#include <irMath\Constant.h>
#include <irMath\Common.h>
#include <time.h>

//#include "OTGProfile.h"
//#include "OTGSCurveStep2.h"

#ifndef EPS_COMPARE
#define EPS_COMPARE 1E-4
#endif

using namespace std;
using namespace irLib::irMath;

namespace irLib
{
	namespace irTG
	{
		class ComplexNumber;

		// calc delta position following by shape of velocity profile
		Real calcDP_constVel(Real dt, Real vel);

		Real calcDP_VShapeVel(Real vi, Real vlow, Real vf, Real amax);
		Real calcDP_reverseVShapeVel(Real vi, Real vhigh, Real vf, Real amax);

		// calc delta position, delta velocity functions following by shape of acceleration profile
		Real calcDV_ConstAcc(Real dt, Real acc);
		Real calcDP_ConstAcc(Real dt, Real acc, Real vi);
		Real calcDP_ConstAccWithVf(Real vi, Real vf, Real acc);

		Real calcDV_SlashAcc(Real ai, Real af, Real jmax);
		Real calcDP_SlashAcc(Real ai, Real af, Real jmax, Real vi);

		Real calcDV_BackSlashAcc(Real ai, Real af, Real jmax);
		Real calcDP_BackSlashAcc(Real ai, Real af, Real jmax, Real vi);

		Real calcDV_revesreVShpaeAcc(Real ai, Real ahigh, Real af, Real jmax);
		Real calcDP_reverseVShapeAcc(Real ai, Real ahigh, Real af, Real jmax, Real vi);
		Real calcDT_reverseVShapeAcc(Real ai, Real ahigh, Real af, Real jmax);
		Real calcAHigh_reverseVShapeAcc(Real ai, Real af, Real jmax, Real vi, Real vf);

		Real calcDV_VShapeAcc(Real ai, Real alow, Real af, Real jmax);
		Real calcDP_VShapeAcc(Real ai, Real alow, Real af, Real jmax, Real vi);
		Real calcDT_VShapeAcc(Real ai, Real alow, Real af, Real jmax);
		Real calcALow_VShapeAcc(Real ai, Real af, Real jmax, Real vi, Real vf);

		Real calcDP_TrapezoidAcc(Real ai, Real ahigh, Real af, Real jmax, Real vi, Real vf);
		Real calcDT_TrapezoidAcc(Real ai, Real ahigh, Real af, Real jmax, Real vi, Real vf);

		Real calcDP_NegTrapezoidAcc(Real ai, Real alow, Real af, Real jmax, Real vi, Real vf);
		Real calcDT_NegTrapezoidAcc(Real ai, Real alow, Real af, Real jmax, Real vi, Real vf);

		Real makeRandLU(Real lower, Real upper);

		void calcQuarticAlgebraicEqn(Real a, Real b, Real c, Real d, Real e, ComplexNumber* x);

		class ComplexNumber
		{
		public:
			Real _rN; // real number
			Real _iN; // imagine number

		public:
			ComplexNumber(Real rN = 0, Real iN = 0) : _rN(rN), _iN(iN) {}

			friend ComplexNumber operator+(const ComplexNumber& cn, const Real num)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn._rN + num;
				tmpCN._iN = cn._iN;
				return tmpCN;
			}

			friend ComplexNumber operator+(const Real num, const ComplexNumber& cn)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn._rN + num;
				tmpCN._iN = cn._iN;
				return tmpCN;
			}

			friend ComplexNumber operator+(const ComplexNumber& cn1, const ComplexNumber& cn2)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn1._rN + cn2._rN;
				tmpCN._iN = cn1._iN + cn2._iN;
				return tmpCN;
			}

			friend ComplexNumber operator-(const ComplexNumber& cn, const Real num)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn._rN - num;
				tmpCN._iN = cn._iN;
				return tmpCN;
			}

			friend ComplexNumber operator-(const Real num, const ComplexNumber& cn)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = num - cn._rN;
				tmpCN._iN = cn._iN;
				return tmpCN;
			}

			friend ComplexNumber operator-(const ComplexNumber& cn1, const ComplexNumber& cn2)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn1._rN - cn2._rN;
				tmpCN._iN = cn1._iN - cn2._iN;
				return tmpCN;
			}

			friend ComplexNumber operator*(const ComplexNumber& cn, const Real num)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn._rN * num;
				tmpCN._iN = cn._iN * num;
				return tmpCN;
			}

			friend ComplexNumber operator*(const Real num, const ComplexNumber& cn)
			{
				ComplexNumber tmpCN;
				tmpCN._rN = cn._rN * num;
				tmpCN._iN = cn._iN * num;
				return tmpCN;
			}

			friend ComplexNumber operator*(const ComplexNumber& cn1, const ComplexNumber& cn2)
			{
				ComplexNumber tmpCN;
				Real a, b, c, d;
				a = cn1._rN; b = cn1._iN;
				c = cn2._rN; d = cn2._iN;
				tmpCN._rN = a*c - b*d;
				tmpCN._iN = a*d + b*c;
				return tmpCN;
			}

			friend ComplexNumber operator/(const Real num, const ComplexNumber& cn)
			{
				Real a, b;
				a = cn._rN; b = cn._iN;
				ComplexNumber tmpCN(a, b);
				tmpCN._rN = num*a / (a*a + b*b);
				tmpCN._iN = -num*b / (a*a + b*b);
				return tmpCN;
			}

			friend ComplexNumber operator/(const ComplexNumber& cn, const Real num)
			{
				ComplexNumber tmpCN(cn._rN, cn._iN);
				tmpCN._rN /= num;
				tmpCN._iN /= num;
				return tmpCN;
			}

			friend ostream& operator<<(ostream& os, const ComplexNumber& cn)
			{
				if (cn._iN == 0)
				{
					os << cn._rN << "+0.0i";
				}
				else if (cn._iN > 0)
				{
					os << cn._rN << " + " << cn._iN << "i";
				}
				else
				{
					os << cn._rN << " - " << std::abs(cn._iN) << "i";
				}
				return os;
			}

			void sqrt()
			{
				Real r, w, th;
				r = std::sqrt(_rN*_rN + _iN*_iN);
				w = std::sqrt(r);
				th = atan(_iN / _rN);
				_rN = w*cos(th / 2);
				_iN = w*sin(th / 2);
			}

			static ComplexNumber sqrt(const ComplexNumber& cn)
			{
				ComplexNumber tmpCN;
				Real r, w, th;
				Real ep = 1E-8;
				if (abs(cn._rN) > ep && abs(cn._iN) > ep)
				{
					r = std::sqrt(cn._rN*cn._rN + cn._iN*cn._iN);
					w = std::sqrt(r);
					th = atan(cn._iN / cn._rN);
					tmpCN._rN = w*cos(th / 2);
					tmpCN._iN = w*sin(th / 2);
				}
				else if (abs(cn._iN) < ep) // pure real
				{
					if (cn._rN >= 0)
						tmpCN._rN = std::sqrt(cn._rN);
					else
						tmpCN._iN = std::sqrt(abs(cn._rN));
				}
				else if (abs(cn._rN) < ep) // pure imaginary
				{
					tmpCN._rN = std::sqrt(std::abs(cn._iN)) / std::sqrt(2);
					tmpCN._iN = tmpCN._rN;
					if (cn._iN < 0)
						tmpCN._rN = -tmpCN._rN;
				}
				else // zero
				{
					tmpCN._rN = 0;
					tmpCN._iN = 0;
				}

				return tmpCN;
			}

			void cuberoot()
			{
				int n = 3;
				Real r, w, th;
				Real ep = 1E-8;
				if (abs(_rN) > ep && abs(_iN) > ep)
				{
					r = std::sqrt(_rN*_rN + _iN*_iN);
					w = std::pow(r, 1.0 / (double)(n));
					th = atan(_iN / _rN);
					_rN = w*cos(th / n);
					_iN = w*sin(th / n);
				}
				else if (abs(_iN) < ep) // pure real
				{
					if (_rN >= 0)
						_rN = std::pow(_rN, 1.0 / 3.0);
					else
					{
						_rN = std::pow(std::abs(_rN), 1.0 / 3.0) * 0.5;
						_iN = std::pow(std::abs(_rN), 1.0 / 3.0) * 0.866025403784439;
					}
				}
				else if (abs(_rN) < ep) // pure imaginary
				{
					if (_iN > 0)
					{
						_rN = std::pow(_rN, 1.0 / 3.0) * 0.866025403784439;
						_iN = std::pow(_rN, 1.0 / 3.0) * 0.5;
					}
					else
						_iN = std::pow(std::abs(_iN), 1.0 / 3.0);
				}
				else // zero
				{
					_rN = 0;
					_iN = 0;
				}
			}

			void root(const int n)
			{
				Real r, w, th;
				r = std::sqrt(_rN*_rN + _iN*_iN);
				w = std::pow(r, 1.0 / (double)(n));
				th = atan(_iN / _rN);
				_rN = w*cos(th / n);
				_iN = w*sin(th / n);
			}
		};

	}
}