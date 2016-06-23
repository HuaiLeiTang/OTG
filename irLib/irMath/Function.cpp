#include "Function.h"
#include "irUtils\Diagnostic.h"

#include <iostream>

using namespace std;

namespace irLib
{
	namespace irMath
	{
		MatrixX Function::Jacobian(const VectorX & x) const
		{
			int n = x.size();

			VectorX e(n);
			e.setZero();
			e(0) = 1.0;

			VectorX Fp = func(x + _eps*e);
			VectorX Fn = func(x - _eps*e);

			int m = Fp.size();

			MatrixX J(m, n);
			J.col(0) = (Fp - Fn) / (2 * _eps);

			for (int i = 1; i < n; i++)
			{
				e(i - 1) = 0.0;
				e(i) = 1.0;

				Fp = func(x + _eps*e);
				Fn = func(x - _eps*e);

				J.col(i) = (Fp - Fn) / (2 * _eps);
			}

			return J;
		}

		MatrixX Function::InverseJacobian(const VectorX& x) const
		{
			MatrixX invj = Jacobian(x);
			int m = invj.cols();
			int n = invj.rows();
			if (m != n)
			{
				LOG("Jacobian is not square. couldn't do inverse.");
				return MatrixX();
			}
			else
			{
				return invj.inverse();
			}
		}

		std::vector<MatrixX> Function::Hessian(const VectorX & x) const
		{
			Real eps_square_inv = 1.0 / (_eps*_eps);

			int n = x.size();

			VectorX ei(n);
			VectorX ej(n);
			ei.setZero();
			ej.setZero();

			VectorX F = func(x);
			int m = F.size();

			MatrixX Fval1(m, n);
			MatrixX Fval2(m, n);
			MatrixX Hii(m, n);
			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				Fval1.col(i) = func(x + _eps*ei);
				Fval2.col(i) = func(x - _eps*ei);
				ei(i) = 0.0;
			}

			vector< MatrixX > H;
			for (int i = 0; i < m; i++)
			{
				H.push_back(MatrixX(n, n));
				for (int j = 0; j < n; j++)
				{
					Hii(i, j) = (Fval1(i, j) + Fval2(i, j) - F(i) - F(i)) * eps_square_inv;
				}
			}

			VectorX F1(m);
			VectorX F2(m);
			VectorX F3(m);
			VectorX F4(m);
			VectorX Hij(m);

			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				for (int j = 0; j < n; j++)
				{
					if (i == j)
					{
						for (int k = 0; k < m; k++)
						{
							H[k](i, i) = Hii(k, i);
						}
					}
					else if (i < j)
					{
						ej(j) = 1.0;

						F1 = func(x + _eps*ei + _eps*ej);
						F2 = func(x - _eps*ei - _eps*ej);
						F3 = func(x - _eps*ei + _eps*ej);
						F4 = func(x + _eps*ei - _eps*ej);
						Hij = 0.25 * (F1 + F2 - F3 - F4) * eps_square_inv;

						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = Hij(k);
						}

						ej(j) = 0.0;
					}
					else
					{
						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = H[k](j, i);
						}
					}
				}
				ei(i) = 0.0;
			}

			return H;
		}

		FunctionPtr Function::MultiplyConst(const Real& w) const
		{
			FunctionPtr p_wf;
			p_wf = FunctionPtr(new MultiplyConstFunction);
			std::static_pointer_cast<MultiplyConstFunction>(p_wf)->setBaseFunction(std::shared_ptr<const Function>(this));
			std::static_pointer_cast<MultiplyConstFunction>(p_wf)->setWeight(w);
			return p_wf;
		}

		VectorX MultiplyConstFunction::func(const VectorX & x) const
		{
			return _w * _baseFunction->func(x);
		}

		MatrixX MultiplyConstFunction::Jacobian(const VectorX & x) const
		{
			return _w * _baseFunction->Jacobian(x);
		}

		std::vector<MatrixX> MultiplyConstFunction::Hessian(const VectorX & x) const
		{
			std::vector<MatrixX>& Hess = _baseFunction->Hessian(x);
			for (unsigned int i = 0; i < Hess.size(); i++)
				Hess[i] *= _w;
			return Hess;
		}

		FunctionPtr MultiplyConstFunction::MultiplyConst(const Real& w) const
		{
			FunctionPtr p_wf;
			p_wf = FunctionPtr(new MultiplyConstFunction);
			std::static_pointer_cast<MultiplyConstFunction>(p_wf)->_baseFunction = _baseFunction;
			std::static_pointer_cast<MultiplyConstFunction>(p_wf)->_w = _w * w;
			return p_wf;
		}

		void MultiplyConstFunction::setBaseFunction(const std::shared_ptr<const Function>& baseFunction)
		{
			_baseFunction = baseFunction;
		}

		void MultiplyConstFunction::setWeight(const Real w)
		{
			_w = w;
		}

		VectorX AugmentedFunction::func(const VectorX& x) const
		{
			vector<VectorX> fvalList(_functionList.size());
			unsigned int dimension = 0;
			for (unsigned int i = 0; i < _functionList.size(); i++)
			{
				fvalList[i] = _functionList[i]->func(x);
				dimension += fvalList[i].size();
			}
			VectorX f(dimension);
			for (unsigned int i = 0, idx = 0; i < _functionList.size(); i++)
			{
				f.block(idx, 0, fvalList[i].size(), 1) = fvalList[i];
				idx += fvalList[i].size();
			}
			return f;
		}

		MatrixX AugmentedFunction::Jacobian(const VectorX& x) const
		{
			vector<MatrixX> jacobianList(_functionList.size());
			unsigned int dimension = 0;
			for (unsigned int i = 0; i < _functionList.size(); i++)
			{
				jacobianList[i] = _functionList[i]->Jacobian(x);
				dimension += jacobianList[i].rows();
			}
			MatrixX jacobian(dimension, x.size());
			for (unsigned int i = 0, idx = 0; i < _functionList.size(); i++)
			{
				jacobian.block(idx, 0, jacobianList[i].rows(), x.size()) = jacobianList[i];
				idx += jacobianList[i].rows();
			}
			return jacobian;
		}

		std::vector< MatrixX > AugmentedFunction::Hessian(const VectorX& x) const
		{
			std::vector< MatrixX > hessian, hessianItem;
			for (unsigned int i = 0; i < _functionList.size(); i++)
			{
				hessianItem = _functionList[i]->Hessian(x);
				for (unsigned int j = 0; j < hessianItem.size(); j++)
				{
					hessian.push_back(hessianItem[j]);
				}
			}
			return hessian;
		}

		void AugmentedFunction::addFunction(const FunctionPtr& function)
		{
			_functionList.push_back(function);
		}

		VectorX AffineFunction::func(const VectorX & x) const
		{
			return _A*x + _b;
		}

		MatrixX AffineFunction::Jacobian(const VectorX & x) const
		{
			return _A;
		}

		std::vector<MatrixX> AffineFunction::Hessian(const VectorX & x) const
		{
			std::vector<MatrixX> Hess(_b.size());
			for (int i = 0; i < _b.size(); i++)
				Hess[i] = MatrixX::Zero(x.size(), x.size());
			return Hess;
		}

		FunctionPtr AffineFunction::MultiplyConst(const Real& w) const
		{
			FunctionPtr p_wf;
			p_wf = FunctionPtr(new AffineFunction);
			std::static_pointer_cast<AffineFunction>(p_wf)->_A = _A*w;
			std::static_pointer_cast<AffineFunction>(p_wf)->_b = _b*w;
			return p_wf;
		}

		VectorX QuadraticFunction::func(const VectorX & x) const
		{
			return x.transpose()*_A*x + _b.transpose()*x + _c;
		}

		MatrixX QuadraticFunction::Jacobian(const VectorX & x) const
		{
			return (_A + _A.transpose())*x + _b;
		}

		std::vector<MatrixX> QuadraticFunction::Hessian(const VectorX & x) const
		{
			std::vector<MatrixX> Hess(1);
			Hess[0] = _A + _A.transpose();
			return Hess;
		}

		FunctionPtr QuadraticFunction::MultiplyConst(const Real& w) const
		{
			FunctionPtr p_wf;
			p_wf = FunctionPtr(new QuadraticFunction);
			std::static_pointer_cast<QuadraticFunction>(p_wf)->_A = _A*w;
			std::static_pointer_cast<QuadraticFunction>(p_wf)->_b = _b*w;
			std::static_pointer_cast<QuadraticFunction>(p_wf)->_c = _c*w;
			return p_wf;
		}
	}
}