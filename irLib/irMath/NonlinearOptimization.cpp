#include "NonlinearOptimization.h"

#include "Common.h"

#include <nlopt.hpp>
#include <iostream>
#include <time.h>

using namespace std;
using namespace nlopt;

namespace irLib
{
	namespace irMath
	{
		double objective(unsigned n, const double* x, double* grad, void *f_data)
		{
			NonlinearOptimization* ptr = reinterpret_cast<NonlinearOptimization*>(f_data);
			VectorX xValue(n);
			for (unsigned int i = 0; i < n; i++)
			{
				xValue(i) = x[i];
			}
			if (grad)
			{
				MatrixX jacobian = (*ptr->getObjectiveFunction()).Jacobian(xValue);
				for (unsigned int j = 0; j < n; j++)
				{
					grad[j] = jacobian(0, j);
				}
			}
			return (*ptr->getObjectiveFunction())(xValue)(0);
		}

		void constraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
		{
			pair<bool, NonlinearOptimization*>* data = reinterpret_cast<pair<bool, NonlinearOptimization*>*>(f_data);
			FunctionPtr constraintFunction;
			if (!data->first)
			{
				constraintFunction = data->second->getEqualityConstraint();
			}
			else
			{
				constraintFunction = data->second->getInequalityConstraint();
			}
			VectorX xValue(n);
			for (unsigned int i = 0; i < n; i++)
			{
				xValue(i) = x[i];
			}
			if (grad)
			{
				MatrixX jacobian = (*constraintFunction).Jacobian(xValue);
				for (unsigned int i = 0; i < m; i++)
				{
					for (unsigned int j = 0; j < n; j++)
					{
						grad[i*n + j] = jacobian(i, j);
					}
				}
			}
			VectorX fval = (*constraintFunction).func(xValue);
			for (unsigned int i = 0; i < m; i++)
			{
				result[i] = fval(i);
			}
		}

		void NonlinearOptimization::solve(const VectorX& initialX, const VectorX& lower, const VectorX& upper)
		{
			if (_xN == -1)		_xN = initialX.size();
			if (_eqN == -1)		_eqN = _eqConstraint->func(initialX).size();
			if (_ineqN == -1)	_ineqN = _ineqConstraint->func(initialX).size();

			if (_eqN != 0) optimizer = opt(nlopt::LD_SLSQP, _xN);
			else optimizer = opt(nlopt::LD_MMA, _xN);

			optimizer.set_min_objective(objective, this);
			if (_eqN != 0)
			{
				optimizer.add_equality_mconstraint(constraint, new pair<bool, NonlinearOptimization*>(false, this), vector<Real>(_eqN, 1e-5));
			}
			if (_ineqN != 0)
			{
				optimizer.add_inequality_mconstraint(constraint, new pair<bool, NonlinearOptimization*>(true, this), vector<Real>(_ineqN, 1e-5));
			}

			Real startTime = clock();

			optimizer.set_maxtime(1.5); // 원래는 없었음
			optimizer.set_maxeval(500);
			optimizer.set_xtol_rel(1e-4);
			optimizer.set_ftol_rel(1e-4);

			if (lower.size() != 0)
			{
				vector<Real> lower_bounds;
				for (int i = 0; i < _xN; i++)
				{
					lower_bounds.push_back(lower(i));
				}
				optimizer.set_lower_bounds(lower_bounds);
			}
			if (upper.size() != 0)
			{
				vector<Real> upper_bounds;
				for (int i = 0; i < _xN; i++)
				{
					upper_bounds.push_back(upper(i));
				}
				optimizer.set_upper_bounds(upper_bounds);
			}

			std::vector<Real> x(_xN);
			for (int i = 0; i < _xN; i++)
			{
				x[i] = initialX(i);
			}

			/////////////////////////////
			//optimizer.optimize(x, resultFunc);

			//resultX = VectorX::Zero(_xN);
			//for (int i = 0; i < _xN; i++)
			//{
			//	resultX(i) = x[i];
			//}
			/////////////////////////////

			VectorX minX = initialX;
			Real min = RealMax;
			for (unsigned int i = 0; i < 10; i++) // 3 -> 10
			{
				Real past = resultFunc;
				bool flag = false;
				try
				{
					optimizer.optimize(x, resultFunc);
				}
				catch (exception&)
				{
					//cout << "[OPTIMIZATION] FORCE STOP" << endl;
				}

				VectorX x_tmp(_xN);
				for (int i = 0; i < _xN; i++)
				{
					x_tmp(i) = x[i];
				}
				resultFunc = _objectFunc->func(x_tmp)(0);

				if (min > resultFunc && RealLessEqual(_ineqConstraint->func(x_tmp), 1e-3))
				{
					minX = x_tmp;
					min = resultFunc;
				}

				if (RealLess(Abs(past - resultFunc), resultFunc / (Real)1.0e+3))
				{
					if (RealEqual(min, RealMax))
					{
						minX = x_tmp;
						min = resultFunc;
					}
					break;
				}
			}

			if (!RealLessEqual(_ineqConstraint->func(minX), 1e-3))
			{
				min = RealMax;
			}

			//cout << "[OPTIMIZATION] Computation Time : " << clock() - startTime << "ms" << endl;

			/////////////////////////////// 만약에 정답이 없는 경우 처리 ///////////////////////////////
			resultX = minX;
			resultFunc = min;
		}
	}
}