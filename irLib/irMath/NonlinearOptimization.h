#pragma once

#include "Function.h"
#include <NLopt\nlopt.hpp>

namespace irLib
{
	namespace irMath
	{
		class NewtonRaphson;
		class NonlinearOptimization;

		typedef void(*fcn)(const VectorX& x, VectorX& f, MatrixX& ig, void* f_data);

		class NewtonRaphson
		{
		private:
			int _xN;
			int _fN;
			void* _f_data;

			Real _tol;
			fcn _fcn;
			VectorX resultX;
		public:
			NewtonRaphson(int xN, int fN, Real tol = 1E-10) : _xN(xN), _fN(fN), _tol(tol)
			{
				resultX.resize(xN);
				resultX.setZero();
			}
			void setxN(const int xN) { _xN = xN; }
			void setfN(const int fN) { _fN = fN; }
			void setfunction(fcn fcnt) { _fcn = fcnt; }
			void setfdata(void* f_data) { _f_data = f_data; }
			void settolerance(const Real tol) { _tol = tol; }

			void solve(const VectorX& initX, int maxIter = 1000)
			{
				int iter = 0;
				VectorX f(_fN);
				MatrixX g(_fN, _xN);
				VectorX tmp(_fN);
				resultX = initX;

				while (iter < maxIter)
				{
					_fcn(resultX, f, g, _f_data);
					tmp.setZero();
					for (int i = 0; i < _fN; i++)
					{
						for (int j = 0; j < _xN; j++)
						{
							tmp(i) += g(i, j) * f(j);
						}
					}
					resultX -= tmp;

					if (f.norm() < _tol)
						break;
					iter++;
				}
			}
			const VectorX& getResultX() const { return resultX; }
		};

		class NonlinearOptimization
		{
		private:
			int _xN; ///< number of parameters
			int _eqN; ///< number of equality constraint
			int _ineqN; ///< number of inequality constrain

			FunctionPtr _objectFunc;
			FunctionPtr _eqConstraint;
			FunctionPtr _ineqConstraint;

			nlopt::opt optimizer;

		public:
			VectorX resultX;
			Real resultFunc;

		public:
			NonlinearOptimization() : _xN(-1), _eqN(-1), _ineqN(-1),
				_objectFunc(FunctionPtr(new EmptyFunction())), _eqConstraint(FunctionPtr(new EmptyFunction())), _ineqConstraint(FunctionPtr(new EmptyFunction())) {}
			NonlinearOptimization(int xN, int eqN, int ineqN) : _xN(xN), _eqN(eqN), _ineqN(ineqN),
				_objectFunc(FunctionPtr(new EmptyFunction())), _eqConstraint(FunctionPtr(new EmptyFunction())), _ineqConstraint(FunctionPtr(new EmptyFunction())) {}

			void forceStop() { optimizer.force_stop(); }

			void setXN(int xN) { _xN = xN; }
			void setEqN(int eqN) { _eqN = eqN; }
			void setIneqN(int ineqN) { _ineqN = ineqN; }

			void setObjectiveFunction(const FunctionPtr& objectFunc) { _objectFunc = objectFunc; }
			void setEqualityConstraint(const FunctionPtr& eqConstraint) { _eqConstraint = eqConstraint; }
			void setInequalityConstraint(const FunctionPtr& ineqConstraint) { _ineqConstraint = ineqConstraint; }

			FunctionPtr& getObjectiveFunction() { return _objectFunc; }
			FunctionPtr& getEqualityConstraint() { return _eqConstraint; }
			FunctionPtr& getInequalityConstraint() { return _ineqConstraint; }

			void solve(const VectorX& initialX, const VectorX& lower, const VectorX& upper);

		};
	}
}