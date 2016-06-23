/*!
*	\file	Function.h
*	\date	2016.01.28
*	\author	Wooyoung Kim(wykim1989@gmail.com)
*	\brief	Functions
*/

#pragma once

#include "Constant.h"
#include <memory>
#include <vector>

namespace irLib
{
	namespace irMath
	{
		class Function;
		typedef std::shared_ptr< Function > FunctionPtr;

		class Function
		{
		public:
			Function(const Real& eps = (1e-5))
			{
				_eps = eps;
			}

			VectorX operator()(const VectorX& x) const
			{
				return func(x);
			}

			virtual VectorX func(const VectorX& x) const = 0;
			virtual MatrixX Jacobian(const VectorX& x) const;
			virtual MatrixX InverseJacobian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian(const VectorX& x) const;
			virtual FunctionPtr MultiplyConst(const Real& w) const;

		private:
			Real _eps;
		};

		class EmptyFunction : public Function
		{
		public:
			EmptyFunction() {}

			VectorX operator()(const VectorX& x) const
			{
				return func(x);
			}

			VectorX func(const VectorX& x) const { return VectorX(); }
			MatrixX Jacobian(const VectorX& x) const { return MatrixX(); }
			std::vector< MatrixX > Hessian(const VectorX& x) const { return std::vector< MatrixX >(0); }
			FunctionPtr MultiplyConst(const Real& w) const { return FunctionPtr(new EmptyFunction); }
		};

		class MultiplyConstFunction : public Function
		{
		public:
			MultiplyConstFunction() : _w(1.0) {}

			VectorX operator()(const VectorX& x) const
			{
				return func(x);
			}

			virtual VectorX func(const VectorX& x) const;
			virtual MatrixX Jacobian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian(const VectorX& x) const;
			virtual FunctionPtr MultiplyConst(const Real& w) const;

			void setBaseFunction(const std::shared_ptr<const Function>& baseFunction);
			void setWeight(const Real w);

		private:
			std::shared_ptr<const Function> _baseFunction;
			Real _w;
		};

		class AugmentedFunction : public Function
		{
		public:
			AugmentedFunction() {}

			virtual VectorX func(const VectorX& x) const;
			virtual MatrixX Jacobian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian(const VectorX& x) const;

			void addFunction(const FunctionPtr& function);

		private:
			std::vector<FunctionPtr> _functionList;

		};

		class AffineFunction : public Function
		{
		public:
			AffineFunction() {}
			AffineFunction(const MatrixX& A, const VectorX& b) : _A(A), _b(b) {}

			virtual VectorX func(const VectorX& x) const;
			virtual MatrixX Jacobian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian(const VectorX& x) const;
			virtual FunctionPtr MultiplyConst(const Real& w) const;

			void setA(const MatrixX& A) { _A = A; }
			void setb(const VectorX& b) { _b = b; }

			const MatrixX& getA() const { return _A; }
			const VectorX& getb() const { return _b; }

		private:
			MatrixX _A;
			VectorX _b;
		};

		class QuadraticFunction : public Function
		{
		public:
			QuadraticFunction() {}

			virtual VectorX func(const VectorX& x) const;
			virtual MatrixX Jacobian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian(const VectorX& x) const;
			virtual FunctionPtr MultiplyConst(const Real& w) const;

			void setA(const MatrixX& A) { _A = A; }
			void setb(const VectorX& b) { _b = b; }
			void setc(const Real c) { _c(0) = c; }

		private:
			MatrixX _A;
			VectorX _b;
			Eigen::Matrix<Real, 1, 1> _c;
		};
	}
}