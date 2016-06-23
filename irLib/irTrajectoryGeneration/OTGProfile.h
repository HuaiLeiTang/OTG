#pragma once

#include <irUtils\Diagnostic.h>
#include <irMath\Constant.h>

using namespace std;
using namespace irLib::irMath;

#define NUM_OF_POLYNOMIAL 15

namespace irLib
{
	namespace irTG
	{
		class Polynomial
		{
		public:
			Real _a0;
			Real _a1;
			Real _a2;
			Real _a3;
			Real _deltaT;
			unsigned int _degree;
		public:
			void setCoefficient(const Real a3, const Real a2, const Real a1, const Real a0, const Real deltaT)
			{ 
				_a3 = a3;
				_a2 = a2;
				_a1 = a1;
				_a0 = a0;
				_deltaT = deltaT;
			}
			Real CalculateValue(const Real &t) const 
			{ 
				return(_a3*(t - _deltaT)*(t - _deltaT)*(t - _deltaT) + _a2*(t - _deltaT)*(t - _deltaT) + _a1*(t - _deltaT) + _a0);
			}
		};

		typedef struct MotionProfile
		{
			Polynomial PositionProfile[NUM_OF_POLYNOMIAL];
			Polynomial VelocityProfile[NUM_OF_POLYNOMIAL];
			Polynomial AccelerationProfile[NUM_OF_POLYNOMIAL];
			Polynomial JerkProfile[NUM_OF_POLYNOMIAL];
			Real PolynomialTime[NUM_OF_POLYNOMIAL];
			unsigned int numOfPolynomial;
		} Profile;



	}
}
