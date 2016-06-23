#include "Interpolation.h"

#include <cmath>
#include <irUtils/Diagnostic.h>

namespace irLib
{
	namespace irMath
	{
		BSpline<-1, -1, -1> BSplineInterpolation(const MatrixX& viaPoints, int order, const Real& xi, const Real& xf)
		{
			///< Choice of associated parameters to via points.
			VectorX x_vp = ParameterAllocation(viaPoints, xi, xf);

			return BSplineInterpolation(viaPoints, order, x_vp);
		}

		BSpline<-1, -1, -1> BSplineInterpolation(const MatrixX& viaPoints, int order, const VectorX& x_vp)
		{
			int N_cp = viaPoints.cols(); ///< number of control points = number of via points

										 ///< Choice of associated parameters to via points.
			LOGIF(N_cp == x_vp.rows(), "BSplineInterpolation error : The number of viaPoints is different from the number of corresponding parameter.")

				///< Choice of knot sequence.
				VectorX knot = KnotAllocation(x_vp, order, N_cp);

			/**
			*  Solving the linear equation.
			*  Find the control points
			*/
			MatrixX Id(N_cp, N_cp); ///< Identity matrix
			Id.setIdentity();
			BSpline<-1, -1, -1> basis_fcn(knot, Id); ///< Basis functions
			MatrixX N_T = basis_fcn(x_vp); ///< Basis function values at parameters corresponding to via points.
			N_T(N_T.rows() - 1, N_T.cols() - 1) = 1; // Don't like it :(
			MatrixX cp = viaPoints * N_T.inverse();

			BSpline<-1, -1, -1> interpolate_bsp(knot, cp);

			return interpolate_bsp;
		}

		BSpline<-1, -1, -1> BSplineFitting(const MatrixX & givenPoints, int order, int N_ctrlPnt, const Real & xi, const Real & xf)
		{
			int Dim = givenPoints.rows(); ///< dimension
			int N_gPnt = givenPoints.cols(); ///< number of given points
			int N_knot = N_ctrlPnt + order; ///< number of knots


											///< Choice of associated parameters to given points.
			VectorX x_gp = ParameterAllocation(givenPoints, xi, xf);

			///< Choice of knot sequence.
			VectorX knot = KnotAllocation(x_gp, order, N_ctrlPnt);

			/**
			*	Solving the linear equation.
			*	Find the control points
			*/
			MatrixX Id(N_ctrlPnt, N_ctrlPnt); ///< Identity matrix
			Id.setIdentity();
			BSpline<-1, -1, -1> basis_fcn(knot, Id); ///< Basis functions
			MatrixX Nall_T = basis_fcn(x_gp); ///< Basis function values at parameters corresponding to given points.
			Nall_T(Nall_T.rows() - 1, Nall_T.cols() - 1) = 1; // Don't like it :(
			MatrixX N_T = Nall_T.block(1, 1, N_ctrlPnt - 2, N_gPnt - 2);
			MatrixX Q_T = givenPoints.block(0, 1, Dim, N_gPnt - 2)
				- givenPoints.col(0) * Nall_T.block(0, 1, 1, N_gPnt - 2)
				- givenPoints.col(N_gPnt - 1) * Nall_T.block(N_ctrlPnt - 1, 1, 1, N_gPnt - 2);

			MatrixX cp(Dim, N_ctrlPnt);
			cp.col(0) = givenPoints.col(0);
			cp.col(N_ctrlPnt - 1) = givenPoints.col(N_gPnt - 1);
			cp.block(0, 1, Dim, N_ctrlPnt - 2) = Q_T * N_T.transpose() *(N_T*N_T.transpose()).inverse();

			BSpline<-1, -1, -1> fitting_bsp(knot, cp);
			return fitting_bsp;
		}

		VectorX ParameterAllocation(const MatrixX & points, const Real & xi, const Real & xf)
		{
			/**
			*	Centripetal parametrization.
			*/
			int N_p = points.cols(); ///< number of points

			VectorX dist_p(N_p - 1); ///< distance between via points
			Real dist_all = 0;
			for (int i = 0; i < N_p - 1; i++) {
				dist_p(i) = sqrt((points.col(i + 1) - points.col(i)).norm());
				dist_all += dist_p(i);
			}

			VectorX x_p(N_p); ///< parameter values corresponding to the points
			for (int i = 0; i < N_p; i++)
			{
				if (i == 0)				x_p(i) = xi;
				else if (i == N_p - 1)	x_p(i) = xf;
				else					x_p(i) = x_p(i - 1) + (xf - xi) * dist_p(i - 1) / dist_all;
			}

			return x_p;
		}

		VectorX KnotAllocation(const VectorX & x_arr, int order, int N_cp)
		{
			/**
			*	Apply de Boor(1978) algorithm.
			*/
			int N_x = x_arr.rows(); ///< number of parameter points
			int N_knot = N_cp + order; ///< number of knots

			Real xi = x_arr(0);
			Real xf = x_arr(N_x - 1);

			VectorX knot(N_knot);
			for (int i = 0; i < order; i++)
				knot(i) = xi;

			Real d = Real(N_x - 1) / Real(N_cp - order + 1);
			for (int i = order; i < N_cp; i++)
			{
				Real step_real = Real(i + 1 - order) * d;
				int step_int = int(step_real);
				Real alpha = step_real - step_int;
				knot(i) = (1 - alpha) * x_arr(step_int) + alpha * x_arr(step_int + 1);
			}

			for (int i = N_cp; i < N_cp + order; i++)
				knot(i) = xf;

			return knot;
		}
	}

}