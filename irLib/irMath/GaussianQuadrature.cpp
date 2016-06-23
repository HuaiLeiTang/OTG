#include "GaussianQuadrature.h"

#include <irUtils\Diagnostic.h>

namespace irLib
{
	namespace irMath
	{
		GaussianQuadrature::GaussianQuadrature(unsigned int num_of_points, Real initialTime, Real finalTime)
			: _N(num_of_points), _t0(initialTime), _tf(finalTime)
		{
			setNumOfPoints(_N);
			//setTimeInterval(initialTime, finalTime);
		}

		void GaussianQuadrature::_calcCoeffs()
		{
			//
			//	Press, W.H. (2007).Numerical recipes 3rd edition, pp.184 : The art of scientific computing.Cambridge university press.
			//

			Real z1, z, pp, p3, p2, p1;
			//	The roots are symmetric in the interval, so we only have to find half of them.
			int m = (_N + 1) / 2;
			for (int i = 0; i < m; i++)
			{
				//	Loop over the desired roots.
				z = cos(3.141592654*(i + 0.75) / (_N + 0.5));
				//	Starting with this approximation to the ith root, we enter the main loop of refinement by Newton¡¯s method.
				do {
					p1 = 1.0;
					p2 = 0.0;
					for (unsigned int j = 0; j<_N; j++)
					{
						//	Loop up the recurrence relation to get the
						//	Legendre polynomial evaluated at z.
						p3 = p2;
						p2 = p1;
						p1 = ((2.0*j + 1.0)*z*p2 - j*p3) / (j + 1);
					}
					//p1 is now the desired Legendre polynomial.We next compute pp, its derivative,
					//	by a standard relation involving also p2, the polynomial of one lower order.
					pp = _N*(z*p1 - p2) / (z*z - 1.0);
					z1 = z;
					z = z1 - p1 / pp;
					//Newton¡¯s method.
				} while (abs(z - z1) > RealEps);
				//Scale the root to the desired interval,
				_x[i] = -z;
				_t[i] = (_tf - _t0) / 2 * _x[i] + (_tf + _t0) / 2;
				//and put in its symmetric counterpart.
				_x[_N - 1 - i] = z;
				_t[_N - 1 - i] = (_tf - _t0) / 2 * _x[_N - 1 - i] + (_tf + _t0) / 2;
				//Compute the weight
				//_w[i] = 2.0 / ((1.0 - z*z)*pp*pp);
				_w[i] = (_tf - _t0) / ((1.0 - z*z)*pp*pp);
				//and its symmetric counterpart.
				_w[_N - 1 - i] = _w[i];
			}
		}

		void GaussianQuadrature::setNumOfPoints(unsigned int num_of_points)
		{
			_N = num_of_points;
			_t.resize(_N);
			_x.resize(_N);
			_w.resize(_N);
			_calcCoeffs();
		}

		void GaussianQuadrature::setTimeInterval(Real initialTime, Real finalTime)
		{
			LOGIF(finalTime > initialTime, "final time must larger than initial time");

			_w *= (finalTime - initialTime) / (_tf - _t0);

			_t0 = initialTime;
			_tf = finalTime;
			_t = (_tf - _t0) / 2 * _x + (_tf + _t0) / 2 * VectorX::Ones(_N);
		}

		const Real GaussianQuadrature::evalIntegration(const VectorX & functionVal) const
		{
			return  _w.dot(functionVal);
		}
	}
}