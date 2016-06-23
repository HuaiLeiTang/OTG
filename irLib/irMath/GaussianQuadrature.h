/*!
*	\file	GaussianQuadrature.h
*	\date	2016.01.23
*	\author	Keunjun Choi(ckj.monikaru@gmail.com)
*	\brief	GaussianQuadrature class header file
*/

#pragma once

#include <vector>
#include "Constant.h"

namespace irLib
{
	namespace irMath
	{

		class GaussianQuadrature;

		class GaussianQuadrature
		{
		public:
			const enum Schem { LG, LGR, LGL };

		private:

			// grid number
			unsigned int _N;
			// initial time & final time
			Real		_t0, _tf;
			// sampling points from t0 to tf.
			VectorX		_t;
			// sampling points in [-1, 1].
			VectorX		_x;
			// weight for integration.
			VectorX		_w;

		public:

			GaussianQuadrature(unsigned int num_of_points = 0, Real initialTime = 0, Real finalTime = 1);

			void	_calcCoeffs();

			void	setNumOfPoints(unsigned int num_of_points);
			void	setTimeInterval(Real initialTime, Real finalTime);

			const VectorX&	getQueryPoints() const { return _t; }
			const VectorX&	getWeights() const { return _w; }

			const Real	evalIntegration(const VectorX& functionVal) const;
		};
	}
}