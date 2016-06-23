#include "GCMMAOptimization.h"

#include <iostream>
#include <fstream>
#include <time.h>
#include <memory>

using namespace std;

namespace irLib
{
	namespace irTG
	{
		void ObjFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const Real scaleObjFunc, const VectorX& scaleX)
		{
			fval = f->func(params);
		}

		void ObjJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const Real scaleObjFunc, const VectorX& scaleX)
		{
			dfdx = f->Jacobian(params);
		}

		void InequalFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const VectorX& scaleIneqFunc, const VectorX& scaleX)
		{
			fval = f->func(params);
		}

		void InequalJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const VectorX& scaleIneqFunc, const VectorX& scaleX)
		{
			dfdx = f->Jacobian(params);
		}

		void ScaleObjFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const Real scaleObjFunc, const VectorX& scaleX)
		{
			int xN = params.size();
			VectorX tmpVec(xN);
			for (int j = 0; j < xN; j++)
				tmpVec(j) = params(j) / scaleX(j);
			fval = f->func(tmpVec);
			fval(0) *= scaleObjFunc;
		}

		void ScaleObjJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const Real scaleObjFunc, const VectorX& scaleX)
		{
			int xN = params.size();
			VectorX tmpVec(xN);
			for (int j = 0; j < xN; j++)
				tmpVec(j) = params(j) / scaleX(j);
			dfdx = f->Jacobian(tmpVec);
			for (int j = 0; j < xN; j++)
				dfdx(0, j) *= scaleObjFunc / scaleX(j);
		}

		void ScaleInequalFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const VectorX& scaleIneqFunc, const VectorX& scaleX)
		{
			int xN = params.size();
			VectorX tmpVec(xN);
			for (int j = 0; j < xN; j++)
				tmpVec(j) = params(j) / scaleX(j);
			fval = f->func(tmpVec);
			for (int i = 0; i < fval.size(); i++)
				fval(i) *= scaleIneqFunc(i);
		}

		void ScaleInequalJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const VectorX& scaleIneqFunc, const VectorX& scaleX)
		{
			int xN = params.size();
			VectorX tmpVec(xN);
			for (int j = 0; j < xN; j++)
				tmpVec(j) = params(j) / scaleX(j);
			dfdx = f->Jacobian(tmpVec);
			for (int i = 0; i < dfdx.rows(); i++)
				for (int j = 0; j < xN; j++)
					dfdx(i, j) *= scaleIneqFunc(i) / scaleX(j);
		}

		GCMMAOptimization::GCMMAOptimization(const int xN, const int ineqN, bool st_scale) : _xN(xN), _ineqN(ineqN), Strategy_Scale(st_scale), _objectFunc(NULL), _ineqConstraint(NULL)
		{
			initialize(xN, ineqN);

			Strategy_Scale = true;

			if (!Strategy_Scale)
			{
				_objectf = &ObjFcnEval;
				_objectJ = &ObjJacobianEval;
				_ineqf = &InequalFcnEval;
				_ineqJ = &InequalJacobianEval;
			}
			else
			{
				_objectf = &ScaleObjFcnEval;
				_objectJ = &ScaleObjJacobianEval;
				_ineqf = &ScaleInequalFcnEval;
				_ineqJ = &ScaleInequalJacobianEval;
			}
		}

		//void GCMMAOptimization::saveMatrixX2txt(MatrixX in, std::string filename)
		//{
		//	std::ofstream fout;
		//	fout.open(filename);

		//	//for (int i = 0; i < in.cols(); i++)
		//	//{
		//	//	for (int j = 0; j < in.rows(); j++)
		//	//		fout << in(j, i) << '\t';
		//	//	fout << std::endl;
		//	//}
		//	for (int i = 0; i < in.rows(); i++)
		//	{
		//		for (int j = 0; j < in.cols(); j++)
		//		{
		//			fout << in(i, j) << '\t';
		//		}
		//		fout << std::endl;
		//	}

		//	fout.close();
		//}

		//void GCMMAOptimization::saveVectorX2txt(VectorX in, std::string filename)
		//{
		//	std::ofstream fout;
		//	fout.open(filename);

		//	for (int i = 0; i < in.size(); i++)
		//	{
		//		fout << in(i) << endl;
		//	}

		//	fout.close();
		//}

		void displayGCMMAResult(GCMMAReturnFlag retFlag)
		{
			cout << "return reslut: ";
			switch (retFlag)
			{
			case Success_tolFunc:
				cout << "optimization succeeded by satisfying 'tolFunc' condition" << endl;
				break;
			case Success_tolX:
				cout << "optimization succeeded by satisfying 'tolX' condition" << endl;
				break;
			case quasiSuccess_subProbFailure:
				cout << "sub problem failed, but former outer loop values are regarded as solution" << endl;
				break;
			case Failure_exceedInequality:
				cout << "optimization failed because exceeded inequality constraints" << endl;
				break;
			case Failure_exceedMaxIterOL:
				cout << "optimization terminated because exceeded max iteration number (outer loop)" << endl;
				break;
			default:
				cout << "appropriate return flag is not assigned" << endl;
				break;
			}
		}

		void GCMMAOptimization::initialize(int xN, int ineqN)
		{
			setXN(xN);
			setIneqN(ineqN);

			setParameters(0.1, 0.25, 0.5, 0.8, 1.2);
			//setParameters(0.1, 0.2, 0.5, 0.9, 1.1);

			setCoefficients(1.0, VectorX(_ineqN).setZero(), VectorX(_ineqN).setConstant(20000.0), VectorX(_ineqN).setOnes());
			//setCoefficients(1.0, VectorX(_ineqN).setZero(), VectorX(_ineqN).setConstant(80000.0), VectorX(_ineqN).setOnes());

			_tolX = 1E-4;
			_tolFunc = 1E-4;
			_maxIterOL = 1000;
			_maxIterIL = 1000;

			//_minX = -VectorX(_xN).setConstant(RealMax);
			//_maxX = -VectorX(_xN).setConstant(RealMax);
		}

		void GCMMAOptimization::setParameters(const Real & albefa, const Real & move0, const Real & asyinit, const Real & asydecr, const Real & asyincr)
		{
			_ALBEFA = albefa;
			_MOVE0 = move0;
			_ASYINIT = asyinit;
			_ASYDECR = asydecr;
			_ASYINCR = asyincr;
		}

		void GCMMAOptimization::setCoefficients(const Real & a0, const VectorX & ai, const VectorX & ci, const VectorX & di)
		{
			_a0 = a0;
			_ai = ai;
			_ci = ci;
			_di = di;
		}

		void GCMMAOptimization::setMinMax(const VectorX & minX, const VectorX & maxX)
		{
			_minX = minX;
			_maxX = maxX;
		}

		void GCMMAOptimization::restoreResultScale(void)
		{
			if (Strategy_Scale)
			{
				resultFunc /= _scaleObjFunc;
				for (int j = 0; j < _xN; j++)
					resultX(j) /= _scaleX(j);
			}
		}

		void GCMMAOptimization::calScaleFactors(VectorX& x)
		{
			VectorX fvali = _objectFunc->func(x);
			VectorX InequalVal = _ineqConstraint->func(x);

			_scaleObjFunc = 1 / std::abs(fvali(0)) * 100;

			_scaleIneqFunc.setOnes(_ineqN);

			for (int i = 0; i < _ineqN; i++)
				_scaleIneqFunc(i) /= abs(InequalVal(i));

			_scaleX.resize(_xN);
			_scaleX.setConstant(10);

			for (int i = 0; i < _xN; i++)
			{
				_minX(i) *= _scaleX(i);
				_maxX(i) *= _scaleX(i);
				x(i) *= _scaleX(i);
			}
		}

		GCMMAReturnFlag GCMMAOptimization::solve(const VectorX & initialX)
		{
			GCMMAReturnFlag ret; // return variable

								 // initialize
								 // variables for outer loop

								 // m1: values of 1 loop before, m2: values of 2 loops before
			VectorX xk(_xN), xkm1(_xN), xkm2(_xN);
			xk = initialX; xkm1 = xk; xkm2 = xk;

			//cout << "initialX" << endl << initialX << endl << endl;
			allocOLvar();
			allocILvar();
			allocSUBvar();

			// function evaluation variables
			VectorX f0val(1);			MatrixX df0dx(1, _xN), df0dxp(1, _xN), df0dxm(1, _xN); // originally, Real and VectorX respectively.
			VectorX fival(_ineqN);		MatrixX dfidx(_ineqN, _xN), dfidxp(_ineqN, _xN), dfidxm(_ineqN, _xN);


	#ifdef STRATEGY_01
			VectorX f0valm1(1), fivalm1(_ineqN);
			MatrixX df0dxm1(1, _xN), dfidxm1(_ineqN, _xN);
	#endif

	#ifdef STRATEGY_02
			_rescur = RealMax;
			_resm1 = RealMax;
			_resm2 = RealMax;
			_muVal = 0;
	#endif
			if (Strategy_Scale)
				calScaleFactors(xk);

			_objectf(_objectFunc, xk, f0val, _scaleObjFunc, _scaleX);
			_ineqf(_ineqConstraint, xk, fival, _scaleIneqFunc, _scaleX);
			_objectJ(_objectFunc, xk, df0dx, _scaleObjFunc, _scaleX);
			_ineqJ(_ineqConstraint, xk, dfidx, _scaleIneqFunc, _scaleX);


			// variables for inner loop
			VectorX xknu(_xN);
			// function evaluation variables
			VectorX f0valknu(1), f0tvalknu(1), fivalknu(_ineqN), fitvalknu(_ineqN);

			int iterOL = 0, iterIL; // iter for outer/inner loop
			while (iterOL < _maxIterOL) // outer loop
			{
				//cout << "=== outer iter num: " << iterOL << endl;


				//cout << xk << endl << endl;
				//cout << xkm1 << endl << endl;
				//cout << xkm2 << endl << endl;

				calcLowUpp(iterOL, xk, xkm1, xkm2);
				//cout << "_olupp" << endl << _olupp << endl << endl;
				//cout << "_ollow" << endl << _ollow << endl << endl;
				calcAlphaBeta(xk);
				//cout << f0val << endl << endl;
				//cout << df0dx << endl << endl;
				//cout << fival << endl << endl;
				//cout << dfidx << endl << endl;
				calcPlusMinusMatrix(df0dx, df0dxp, df0dxm);
				calcPlusMinusMatrix(dfidx, dfidxp, dfidxm);

				//std::cout << _ollow << std::endl << std::endl;
				//std::cout << _olupp << std::endl << std::endl;
				//std::cout << _olalpha << std::endl << std::endl;
				//std::cout << _olbeta << std::endl << std::endl;
				//saveVectorX2txt123(_ollow, filedd);
				//saveVectorX2txt123(_olupp, filedd);
				//saveVectorX2txt123(_olalpha, filedd);
				//saveVectorX2txt123(_olbeta, filedd);


				//cout << f0val << endl;
				//cout << fival << endl;
				//cout << df0dx << endl << endl;
				//cout << dfidx << endl << endl;

				//saveVectorX2txt123(f0val, filedd);
				//saveVectorX2txt123(fival, filedd);
				//saveMatrixX2txt123(df0dx, filedd);
				//saveMatrixX2txt123(dfidx, filedd);

	#ifdef STRATEGY_01
				calcSigma(iterOL, xk, xkm1, xkm2);
				calcInitialRho_st01(iterOL, xk, xkm1, df0dx, df0dxm1, dfidx, dfidxm1);
	#else
				calcInitialRho(df0dx, dfidx);
	#endif

				//cout << "======" << endl;
				//cout << iterOL << endl;
				//cout << _ilrho0 << endl;
				//cout << _ilrhoi << endl << endl;

				//saveRealX2txt123(_ilrho0, filedd);
				//saveVectorX2txt123(_ilrhoi, filedd);

				//cout << "xk" << endl << xk << endl << endl;
				//if ((iterOL % 2 == 0) && (iterOL > 1))
				//   _maxIterIL = 2;
				//else
				//   _maxIterIL = (int)1E3;
				//if (iterOL % 2)
				//	_maxIterIL = 2;
				//else
				//	_maxIterIL = (int)1E3;

				iterIL = 0;
				while (iterIL < _maxIterIL) // inner loop
				{
					//cout << "====== inner iter num: " << iterIL << endl;


					calcPQR(df0dxp, df0dxm, dfidxp, dfidxm, xk, f0val, fival);

					//cout << _ilp0 << endl << endl;
					//cout << _ilpi << endl << endl;
					//cout << _ilq0 << endl << endl;
					//cout << _ilqi << endl << endl;
					//cout << _ilr0 << endl << endl;
					//cout << _ilri << endl << endl;

					//saveVectorX2txt123(_ilp0, filedd);
					//saveMatrixX2txt123(_ilpi, filedd);
					//saveVectorX2txt123(_ilq0, filedd);
					//saveMatrixX2txt123(_ilqi, filedd);
					//saveRealX2txt123(_ilr0, filedd);
					//saveVectorX2txt123(_ilri, filedd);


					ret = solveSubProblem(xknu);

					if (ret == subProblemFailure)
					{
						// save solution as former outer loop x
						resultX = xk;
						resultFunc = f0val(0);
						if (Strategy_Scale)
							restoreResultScale();
						//return quasiSuccess_subProbFailure;
						return checkSolution(fival, quasiSuccess_subProbFailure);
					}

					//cout << xknu << endl << endl;
					//saveVectorX2txt123(xknu, filedd);


					/////////////////////////////////////////////////
					//for (int i = 0; i < 3; i++)
					//{
					//	for (int j = 0; j < 6; j++)
					//	{
					//		tmpcp(i, j + 3) = xknu(i * 6 + j);
					//	}
					//}
					//int datanum = 2000;
					//Real stepsize = (2.0 - 0.0) / 2000;
					//Real t = 0.0;
					//BSpline<-1, -1, -1> tmpqSpline(tmpknot, tmpcp);
					//BSpline<-1, -1, -1> tmpqdotSpline = tmpqSpline.derivative();
					//BSpline<-1, -1, -1> tmpqddotSpline = tmpqdotSpline.derivative();
					//MatrixX qTraj(6, datanum), qdotTraj(6, datanum), qddotTraj(6, datanum);

					//for (int i = 0; i < datanum; i++)
					//{
					//	qTraj.col(i) = tmpqSpline(t);
					//	t += stepsize;
					//}
					//t = 0.0;
					//for (int i = 0; i < datanum; i++)
					//{
					//	qdotTraj.col(i) = tmpqdotSpline(t);
					//	t += stepsize;
					//}
					//t = 0.0;
					//for (int i = 0; i < datanum; i++)
					//{
					//	qddotTraj.col(i) = tmpqddotSpline(t);
					//	t += stepsize;
					//}

					//std::string filename = "C:/Users/crazy/Desktop/Time optimization/nloptMMA test/exp/";

					//saveMatrixX2txt(qTraj, filename + "q.txt");
					//saveMatrixX2txt(qdotTraj, filename + "qdot.txt");
					//saveMatrixX2txt(qddotTraj, filename + "qddot.txt");
					/////////////////////////////////////////////////


					if (testILSuccess(xknu, f0valknu, fivalknu, f0tvalknu, fitvalknu))
					{
						if (RealBigger(f0valknu(0), f0val(0), 1E-5))
						{
							// save solution as former outer loop x if the value of objective function increased
							resultX = xk;
							resultFunc = f0val(0);
							if (Strategy_Scale)
								restoreResultScale();
							return checkSolution(fival, quasiSuccess_subProbFailure);
						}
						break; // xknu is the optimal solution
					}
					//cout << "............" << f0valknu(0) << endl;

							   //cout << f0valknu << endl << endl;
							   //cout << fivalknu << endl << endl;
							   //cout << f0tvalknu << endl << endl;
							   //cout << fitvalknu << endl << endl;

							   //saveVectorX2txt123(f0valknu, filedd);
							   //saveVectorX2txt123(fivalknu, filedd);
							   //saveVectorX2txt123(f0tvalknu, filedd);
							   //saveVectorX2txt123(fitvalknu, filedd);

							   //cout << rho0 << endl << endl;
							   //cout << rhoi << endl << endl;

							   // update rho0/rhoi
					updateRho0i(xknu, xk, f0valknu, fivalknu, f0tvalknu, fitvalknu);

					//cout << _ilrho0 << endl << endl;
					//cout << _ilrhoi << endl << endl;

					//saveRealX2txt123(_ilrho0, filedd);
					//saveVectorX2txt123(_ilrhoi, filedd);

					iterIL++;
				}



				/////////////////////////////////////////////////
				//for (int i = 0; i < 3; i++)
				//{
				//	for (int j = 0; j < 6; j++)
				//	{
				//		tmpcp(i, j + 3) = xknu(i * 6 + j);
				//	}
				//}
				//int datanum = 2000;
				//Real stepsize = (2.0 - 0.0) / 2000;
				//Real t = 0.0;
				//BSpline<-1, -1, -1> tmpqSpline(tmpknot, tmpcp);
				//BSpline<-1, -1, -1> tmpqdotSpline = tmpqSpline.derivative();
				//BSpline<-1, -1, -1> tmpqddotSpline = tmpqdotSpline.derivative();
				//MatrixX qTraj(6, datanum), qdotTraj(6, datanum), qddotTraj(6, datanum);

				//for (int i = 0; i < datanum; i++)
				//{
				//	qTraj.col(i) = tmpqSpline(t);
				//	t += stepsize;
				//}
				//t = 0.0;
				//for (int i = 0; i < datanum; i++)
				//{
				//	qdotTraj.col(i) = tmpqdotSpline(t);
				//	t += stepsize;
				//}
				//t = 0.0;
				//for (int i = 0; i < datanum; i++)
				//{
				//	qddotTraj.col(i) = tmpqddotSpline(t);
				//	t += stepsize;
				//}

				//std::string filename = "C:/Users/crazy/Desktop/Time optimization/nloptMMA test/exp/";

				//saveMatrixX2txt(qTraj, filename + "q.txt");
				//saveMatrixX2txt(qdotTraj, filename + "qdot.txt");
				//saveMatrixX2txt(qddotTraj, filename + "qddot.txt");
				/////////////////////////////////////////////////

				//cout << iterIL << endl;
				//cout << xknu << endl;

				// terminate condition
				//cout << "abs(f0valm1(0) - f0valknu(0)) : " << abs(f0valm1(0) - f0valknu(0)) << endl;
				//cout << "(xkm1 - xknu).norm() : " << (xkm1 - xknu).norm() << endl;

				//cout << f0val(0) << endl << endl;

				//cout << f0val(0) - f0valknu(0) << '\t' << (xkm1 - xknu).norm() << endl;
				//cout << f0val << '\t' << f0valknu(0) << endl;

				if (abs(f0val(0) - f0valknu(0)) < _tolFunc)
				{
					resultX = xknu;
					resultFunc = f0valknu(0);
					if (Strategy_Scale)
						restoreResultScale();
					//return Success_tolFunc;
					return checkSolution(fivalknu, Success_tolFunc);
				}
				if ((xkm1 - xknu).norm() < _tolX)
				{
					resultX = xknu;
					resultFunc = f0valknu(0);
					if (Strategy_Scale)
						restoreResultScale();
					//return Success_tolX;
					return checkSolution(fivalknu, Success_tolX);
				}
				// update
				xkm2 = xkm1;
				xkm1 = xk;
				xk = xknu;
				_ollowm1 = _ollow;
				_oluppm1 = _olupp;

				f0val = f0valknu;
				fival = fivalknu;

				_objectJ(_objectFunc, xk, df0dx, _scaleObjFunc, _scaleX);
				_ineqJ(_ineqConstraint, xk, dfidx, _scaleIneqFunc, _scaleX);

	#ifdef STRATEGY_01
				f0valm1 = f0val;
				fivalm1 = fival;
				df0dxm1 = df0dx;
				dfidxm1 = dfidx;
	#endif
	#ifdef STRATEGY_02
				_resm2 = _resm1;
				_resm1 = _rescur;
				calcResCur(fival, df0dx, dfidx);
				_muVal = Min(Min(Min(_rescur, _resm1), _resm2), 1E12) / pow((Real)iterOL + 3.0, 1.1);
				//cout << _rescur << '\t' << _muVal << endl << endl;
				////// res_cur 계산하고 뮤밸업데이트까지 하면 완료!
				//// 바로 위에서 구한 df0dx, dfidx (xknu로 계산한걸로!!!!!)
	#endif

				//cout << "---------------" << endl;
				//cout << iterOL << endl << endl;
				//cout << xk << endl << endl;
				//cout << f0valknu << endl << endl;
				//cout << fivalknu << endl << endl << endl;

				iterOL++;
			}

			resultX = xknu;
			resultFunc = f0valknu(0);

			if (Strategy_Scale)
				restoreResultScale();
			return Failure_exceedMaxIterOL;
		}




		void GCMMAOptimization::calcLowUpp(int iter, const VectorX & xk, const VectorX & xkm1, const VectorX & xkm2)
		{
			if (iter == 0 || iter == 1) // first or second loop only
			{
				for (int j = 0; j < _xN; j++)
				{
					_ollow(j) = xk(j) - _ASYINIT * (_maxX(j) - _minX(j));
					_olupp(j) = xk(j) + _ASYINIT * (_maxX(j) - _minX(j));
				}
			}
			else
			{
				for (int j = 0; j < _xN; j++)
				{
					if ((xk(j) - xkm1(j)) * (xkm1(j) - xkm2(j)) < 0)
					{
						_ollow(j) = xk(j) - _ASYDECR * (xkm1(j) - _ollowm1(j));
						_olupp(j) = xk(j) + _ASYDECR * (_oluppm1(j) - xkm1(j));
					}
					else if ((xk(j) - xkm1(j)) * (xkm1(j) - xkm2(j)) > 0)
					{
						_ollow(j) = xk(j) - _ASYINCR * (xkm1(j) - _ollowm1(j));
						_olupp(j) = xk(j) + _ASYINCR * (_oluppm1(j) - xkm1(j));
					}
					else
					{
						_ollow(j) = xk(j) - 1.0 * (xkm1(j) - _ollowm1(j));
						_olupp(j) = xk(j) + 1.0 * (_oluppm1(j) - xkm1(j));
					}
				}
			}
		}

		void GCMMAOptimization::calcAlphaBeta(const VectorX & xk)
		{
			Vector3 tmpVec;
			for (int j = 0; j < _xN; j++)
			{
				// for alpha
				tmpVec(0) = _minX(j);
				tmpVec(1) = _ollow(j) + _ALBEFA*(xk(j) - _ollow(j));
				tmpVec(2) = xk(j) - _MOVE0*(_maxX(j) - _minX(j));
				_olalpha(j) = tmpVec.maxCoeff();

				// for beta
				tmpVec(0) = _maxX(j);
				tmpVec(1) = _olupp(j) - _ALBEFA*(_olupp(j) - xk(j));
				tmpVec(2) = xk(j) + _MOVE0*(_maxX(j) - _minX(j));
				_olbeta(j) = tmpVec.minCoeff();
			}
		}

		void GCMMAOptimization::calcPlusMinusMatrix(const MatrixX & mat, MatrixX & matp, MatrixX & matm)
		{
			for (int i = 0; i < mat.rows(); i++)
			{
				for (int j = 0; j < mat.cols(); j++)
				{
					if (mat(i, j) > 0)
					{
						matp(i, j) = mat(i, j);
						matm(i, j) = 0;
					}
					else if (mat(i, j) < 0)
					{
						matp(i, j) = 0;
						matm(i, j) = -mat(i, j);
					}
					else
					{
						matp(i, j) = 0;
						matm(i, j) = 0;
					}
				}
			}
		}


	#ifdef STRATEGY_01
		void GCMMAOptimization::calcSigma(int iter, const VectorX & xk, const VectorX & xkm1, const VectorX & xkm2)
		{
			if (iter == 0 || iter == 1) // first or second loop only
			{
				for (int j = 0; j < _xN; j++)
					_olsigma(j) = _ASYINIT * (_maxX(j) - _minX(j));
			}
			else
			{
				for (int j = 0; j < _xN; j++)
				{
					if ((xk(j) - xkm1(j)) * (xkm1(j) - xkm2(j)) < 0)
						_olsigma(j) *= _ASYDECR;
					else if ((xk(j) - xkm1(j)) * (xkm1(j) - xkm2(j)) > 0)
						_olsigma(j) *= _ASYINCR;
					//else
					//   _olsigma(j) *= 1.0;
					_olsigma(j) = Max(0.01*(_maxX(j) - _minX(j)), Min(_olsigma(j), 10 * (_maxX(j) - _minX(j))));
				}
			}
		}
		void GCMMAOptimization::calcInitialRho_st01(int iter, const VectorX & xk, const VectorX & xkm1, const MatrixX & df0dx, const MatrixX & df0dxm1, const MatrixX & dfidx, const MatrixX & dfidxm1)
		{
			Real tmpnum, tmpden, tmpreal;
			if (iter == 0)
			{
				_ilrho0 = 1.0;
				for (int i = 0; i < _ineqN; i++)
					_ilrhoi(i) = 1.0;
			}
			else
			{
				// calculate _ols
				for (int j = 0; j < _xN; j++)
					_ols(j) = xk(j) - xkm1(j);



				// calculate _ilrho0
				for (int j = 0; j < _xN; j++)
				{
					_olt(j) = df0dx(0, j) - df0dxm1(0, j);
					_olb(j) = 2 * _olsigma(j) * abs(df0dx(0, j));
				}

				tmpnum = 0; tmpden = 0;
				for (int j = 0; j < _xN; j++)
				{
					tmpnum += _ols(j) * _olt(j);
					tmpden += _ols(j) * _ols(j);
				}

				_oleta = Min(1E3, Max(1E-3, tmpnum / tmpden));

				tmpreal = 0;
				for (int j = 0; j < _xN; j++)
					tmpreal += _oleta * _olsigma(j) * _olsigma(j) - _olb(j);
				tmpreal /= _xN;

				if (tmpreal > 0)
					_ilrho0 = tmpreal;
				else
					_ilrho0 = Max(0.1*_ilrho0, 1E-5);

				// calculate _ilrhoi
				for (int i = 0; i < _ineqN; i++)
				{

					for (int j = 0; j < _xN; j++)
					{
						_olt(j) = dfidx(i, j) - dfidxm1(i, j);
						_olb(j) = 2 * _olsigma(j) * abs(dfidx(i, j));
					}

					tmpnum = 0; tmpden = 0;
					for (int j = 0; j < _xN; j++)
					{
						tmpnum += _ols(j) * _olt(j);
						tmpden += _ols(j) * _ols(j);
					}

					_oleta = Min(1E3, Max(1E-3, tmpnum / tmpden));

					tmpreal = 0;
					for (int j = 0; j < _xN; j++)
						tmpreal += _oleta * _olsigma(j) * _olsigma(j) - _olb(j);
					tmpreal /= _xN;

					if (tmpreal > 0)
						_ilrhoi(i) = tmpreal;
					else
						_ilrhoi(i) = Max(0.1*_ilrhoi(i), 1E-5);

				}

			}
		}
	#else

		void GCMMAOptimization::calcInitialRho(const MatrixX & df0dx, const MatrixX & dfidx)
		{
			// input variable size:   df0dx(1, _xN), dfidx(_ineqN, _xN)
			// output variable size: Real rho0, VectorX rhoi(_ineqN);

			Real tmpSum = 0;
			for (int j = 0; j < _xN; j++)
				tmpSum += abs(df0dx(0, j)) * (_maxX(j) - _minX(j));
			_ilrho0 = 0.1 * tmpSum / (Real)_xN;

			for (int i = 0; i < _ineqN; i++)
			{
				tmpSum = 0;
				for (int j = 0; j < _xN; j++)
					tmpSum += abs(dfidx(i, j)) * (_maxX(j) - _minX(j));
				_ilrhoi(i) = Max(0.1 * tmpSum / (Real)_xN, 1E-6);
			}
		}
	#endif

		void GCMMAOptimization::calcPQR(const MatrixX & df0dxp, const MatrixX & df0dxm, const MatrixX & dfidxp, const MatrixX & dfidxm, const VectorX & xk, const VectorX & f0val, const VectorX & fival)
		{
			// input variable size:
			// Real rho0
			// VectorX rhoi(_ineqN)
			// MatrixX df0dxp(1, _xN), df0dxm
			// MatrixX dfidxp(_ineqN, _xN), dfidxm
			// VectorX low(_xN), upp, xk
			// VectorX f0val(1), fival(_ineqN)

			// output variable size:
			// VectorX p0(_xN), q0(_xN)
			// MatrixX pi(_ineqN, _xN), qi(_ineqN, _xN)
			// Real r0,  VectorX ri(_ineqN)
			Real tmpSum = 0;

			for (int j = 0; j < _xN; j++)
			{
				_ilp0(j) = (_olupp(j) - xk(j)) * (_olupp(j) - xk(j)) * (1.001 * df0dxp(0, j) + 0.001 * df0dxm(0, j) + _ilrho0 / (_maxX(j) - _minX(j)));
				_ilq0(j) = (xk(j) - _ollow(j)) * (xk(j) - _ollow(j)) * (0.001 * df0dxp(0, j) + 1.001 * df0dxm(0, j) + _ilrho0 / (_maxX(j) - _minX(j)));

				tmpSum += _ilp0(j) / (_olupp(j) - xk(j)) + _ilq0(j) / (xk(j) - _ollow(j));
			}
			_ilr0 = f0val(0) - tmpSum;

			for (int i = 0; i < _ineqN; i++)
			{
				tmpSum = 0;
				for (int j = 0; j < _xN; j++)
				{
					_ilpi(i, j) = (_olupp(j) - xk(j)) * (_olupp(j) - xk(j)) * (1.001 * dfidxp(i, j) + 0.001 * dfidxm(i, j) + _ilrhoi(i) / (_maxX(j) - _minX(j)));
					_ilqi(i, j) = (xk(j) - _ollow(j)) * (xk(j) - _ollow(j)) * (0.001 * dfidxp(i, j) + 1.001 * dfidxm(i, j) + _ilrhoi(i) / (_maxX(j) - _minX(j)));

					tmpSum += _ilpi(i, j) / (_olupp(j) - xk(j)) + _ilqi(i, j) / (xk(j) - _ollow(j));
				}
				_ilri(i) = fival(i) - tmpSum;
			}

		}

		bool GCMMAOptimization::testILSuccess(const VectorX & testx, VectorX & f0valknu, VectorX & fivalknu, VectorX & f0tvalknu, VectorX & fitvalknu)
		{
			//VectorX f0val(1), f0tval(1), fival(_ineqN), fitval(_ineqN);
			_objectf(_objectFunc, testx, f0valknu, _scaleObjFunc, _scaleX);
			_ineqf(_ineqConstraint, testx, fivalknu, _scaleIneqFunc, _scaleX);

			calcf0tilde(testx, f0tvalknu);
			calcfitilde(testx, fitvalknu);

			//cout << "testILSuccess " << f0valknu(0) << '\t' << f0tvalknu(0) << endl;

			bool ret = false;

	#ifdef STRATEGY_02
			if (f0tvalknu(0) + _muVal*Max(1, abs(f0tvalknu(0))) >= f0valknu(0))
			{
				ret = true;
				for (int i = 0; i < _ineqN; i++)
				{
					if (fitvalknu(i) + _muVal*Max(1, abs(fitvalknu(i)))< fivalknu(i))
					{
						ret = false;
						break;
					}
				}
			}
	#else
			if (f0tvalknu(0) >= f0valknu(0))
			{
				ret = true;
				for (int i = 0; i < _ineqN; i++)
				{
					if (fitvalknu(i) < fivalknu(i))
					{
						ret = false;
						break;
					}
				}
			}
	#endif

			return ret;
		}

		GCMMAReturnFlag GCMMAOptimization::checkSolution(const VectorX & fival, GCMMAReturnFlag inFlag)
		{
			// called just before return success flag
			for (int i = 0; i < _ineqN; i++)
			{
				if (RealBigger(fival(i), 0.0, 1E-7))
					return Failure_exceedInequality;
			}
			return inFlag;
		}


		void GCMMAOptimization::calcf0tilde(const VectorX & x, VectorX & f0tval)
		{
			f0tval(0) = _ilr0;
			for (int j = 0; j < _xN; j++)
				f0tval(0) += _ilp0(j) / (_olupp(j) - x(j)) + _ilq0(j) / (x(j) - _ollow(j));
		}

		void GCMMAOptimization::calcfitilde(const VectorX & x, VectorX & fitval)
		{
			for (int i = 0; i < _ineqN; i++)
			{
				fitval(i) = _ilri(i);
				for (int j = 0; j < _xN; j++)
					fitval(i) += _ilpi(i, j) / (_olupp(j) - x(j)) + _ilqi(i, j) / (x(j) - _ollow(j));
			}
		}

		void GCMMAOptimization::allocOLvar(void)
		{
			_ollow.resize(_xN);
			_ollowm1.resize(_xN);
			_olupp.resize(_xN);
			_oluppm1.resize(_xN);
			_olalpha.resize(_xN);
			_olbeta.resize(_xN);

	#ifdef STRATEGY_01
			_olsigma.resize(_xN);
			_ols.resize(_xN);
			_olt.resize(_xN);
			_olb.resize(_xN);
	#endif
		}

		void GCMMAOptimization::allocILvar(void)
		{
			_ilp0.resize(_xN);
			_ilq0.resize(_xN);
			_ilpi.resize(_ineqN, _xN);
			_ilqi.resize(_ineqN, _xN);
			_ilri.resize(_ineqN);
			_ilrhoi.resize(_ineqN);
		}


		void GCMMAOptimization::updateRho0i(const VectorX & xknu, const VectorX & xk, const VectorX & f0valknu, const VectorX & fivalknu, const VectorX & f0tvalknu, const VectorX & fitvalknu)
		{
			Real dknu = 0;
			for (int j = 0; j < _xN; j++)
				dknu += (_olupp(j) - _ollow(j)) * (xknu(j) - xk(j)) * (xknu(j) - xk(j)) / ((_olupp(j) - xknu(j))*(xknu(j) - _ollow(j))*(_maxX(j) - _minX(j)));

			Real deltaknu0 = (f0valknu(0) - f0tvalknu(0)) / dknu;
			if (deltaknu0 > 0)
				_ilrho0 = Min(1.1*(_ilrho0 + deltaknu0), 10 * _ilrho0);

			//if (1.1*(_ilrho0 + deltaknu0) < 10 * _ilrho0)
			//	_ilrho0 = 1.1*(_ilrho0 + deltaknu0);
			//else
			//	_ilrho0 = 10 * _ilrho0;

			VectorX deltaknui = (fivalknu - fitvalknu) / dknu;
			for (int i = 0; i < _ineqN; i++)
			{
				//cout << "================" << endl;
				//cout << i << '\t' << _ilrhoi(i) << '\t' << deltaknui(i) << endl;
				//cout << 1.1*(_ilrhoi(i) + deltaknui(i)) << '\t' << 10 * _ilrhoi(i) << endl;
				//Real tmpval;
				//if (deltaknui(i) > 0)
				//	if (1.1*(_ilrhoi(i) + deltaknui(i)) < 10 * _ilrhoi(i))
				//		_ilrhoi(i) = 1.1*(_ilrhoi(i) + deltaknui(i));
				//	else
				//		_ilrhoi(i) = 10 * _ilrhoi(i);

				if (deltaknui(i) > 0)
					_ilrhoi(i) = Min(1.1*(_ilrhoi(i) + deltaknui(i)), 10 * _ilrhoi(i));
			}
		}

		GCMMAReturnFlag GCMMA_PDIPM::solveSubProblem(VectorX & xout)
		{
			// input variable size:
			// VectorX p0(_xN), q0(_xN)
			// MatrixX pi(_ineqN, _xN), qi(_ineqN, _xN)
			// VectorX ri(_ineqN)
			// VectorX alpha(_xN), beta, low, upp

			// output variable size:
			// VectorX xout(_xN)


			// initialize
			//VectorX bi = -_ilri;
			initializeSubProb();
			VectorX delw(_subDimW);
			Real tau;

			int iterSub = 0, maxIterSub = 1000;
			bool solFound = false;
			while (iterSub < maxIterSub)
			{
				// step 1: calculate gradient of w
				//calcGradientW(p0, pi, q0, qi, bi, delw);
				calcGradientW(delw);

				//cout << "[delw]" << endl << delw << endl << endl;

				// step 2: calculate step length 'tau'
				if (calcStepLength(delw, tau) == subProblemFailure_calcTau)
					return subProblemFailure;

				//cout << "tau : " << tau << endl << endl;

				// step 3: update w
				_subw += tau * delw;
				separateFromW();

				// step 4: update epsilon
				//if (calcNormResidual(p0, pi, q0, qi, bi, delw, 0.0, -1) < 0.9 * _subeps)
				if (calcNormResidual(delw, 0.0, -1) < 0.9 * _subeps)
					_subeps *= 0.1;

				// step 5: check terminate condition
				//if (_subeps <= 1E-6)
				if (_subeps <= 1E-4)
				{
					solFound = true;
					break;
				}

				iterSub++;
			}

			//VectorX x(_xN);
			//calcx(_sublam, x);
			//VectorX result = _ineqConstraint->func(x);
			//for (int i = 0; i < _ineqN; i++)
			//{
			//	if (result(i) > 0)
			//		cout << "inequality error" << endl;
			//}

			//Real W;
			//VectorX dW(_ineqN);
			//cout << "sublam" << endl << _sublam << endl << endl;
			//calcW(_sublam, W);
			//calcdW(_sublam, dW);
			//cout << "W : " << W << endl;
			//cout << "lam * dW : " << VectorInner(_sublam, dW, _ineqN) << endl;


			//cout << iterSub << endl;
			//cout << _subeps << endl;
			xout = _subx;


			if (solFound)
				return subProblemSuccess;
			else
			{
				//cout << "maxiter" << endl;
				//return subProblemSuccess;
				return subProblemFailure;
			}
				

		}

		void GCMMA_PDIPM::allocSUBvar(void)
		{
			// x, y, z, lam, xsi, eta, mu, zet, s (in order)
			_subDimW = _xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN + 1 + _ineqN;
			_subw.resize(_subDimW);

			_subx.resize(_xN);
			_suby.resize(_ineqN);
			_sublam.resize(_ineqN);
			_subxsi.resize(_xN);
			_subeta.resize(_xN);
			_submu.resize(_ineqN);
			_subs.resize(_ineqN);

			// addition part
			tmpsubx.resize(_xN);
			tmpsublam.resize(_ineqN);
			tmpplam.resize(_xN);
			tmpqlam.resize(_xN);
			tmpdpsidx.resize(_xN);
			tmpgival.resize(_ineqN);

			G.resize(_ineqN, _xN);
			xadi.resize(_xN);
			bxdi.resize(_xN);
			ydi.resize(_ineqN);
			ldi.resize(_ineqN);

			Dx.resize(_xN, _xN); Dx.setZero();
			iDx.resize(_xN, _xN); iDx.setZero();
			Dly.resize(_ineqN, _ineqN); Dly.setZero();
			iDly.resize(_ineqN, _ineqN); iDly.setZero();
			iDy.resize(_ineqN);

			dxt.resize(_xN);
			dyt.resize(_ineqN);
			dlt.resize(_ineqN);
			dlyt.resize(_ineqN);

			DlyGDxGt.resize(_ineqN, _ineqN);
			iDlyGDxGt.resize(_ineqN, _ineqN);

			DxGtiDlyG.resize(_xN, _xN);
			iDxGtiDlyG.resize(_xN, _xN);

			MatSizeineqNbyxN.resize(_ineqN, _xN);
			MatSizexNbyineqN.resize(_xN, _ineqN);

			delx.resize(_xN);
			dellam.resize(_ineqN);

			resvec.resize(_subDimW);
		}

		void GCMMA_PDIPM::initializeSubProb(void)
		{
			// x, y, z, lam, xsi, eta, mu, zet, s (in order)
			//_subDimW = _xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN + 1 + _ineqN;
			//_subw.resize(_subDimW);

			//_subx.resize(_xN);
			//_suby.resize(_ineqN);
			//_sublam.resize(_ineqN);
			//_subxsi.resize(_xN);
			//_subeta.resize(_xN);
			//_submu.resize(_ineqN);
			//_subs.resize(_ineqN);
			//resvec.resize(_subDimW);

			_subx = 0.5 * (_olalpha + _olbeta);
			_suby.setOnes();
			_subz = 1;
			_subzet = 1;
			_sublam.setOnes();
			_subs.setOnes();

			for (int j = 0; j < _xN; j++)
			{
				if (1 / (_subx(j) - _olalpha(j))>1)
					_subxsi(j) = 1 / (_subx(j) - _olalpha(j));
				else
				{
					_subxsi(j) = 1;
				}

				if (1 / (_olbeta(j) - _subx(j)) > 1)
					_subeta(j) = 1 / (_olbeta(j) - _subx(j));
				else
				{
					_subeta(j) = 1;
				}
			}

			for (int i = 0; i < _ineqN; i++)
			{
				if (0.5 * _ci(i) > 1)
					_submu(i) = 0.5 * _ci(i);
				else
					_submu(i) = 1;
			}

			combineToW(_subx, _suby, _subz, _sublam, _subxsi, _subeta, _submu, _subzet, _subs, _subw);

			_subeps = 1.0;
		}


		void GCMMA_PDIPM::separateFromW(void)
		{
			// save member variables x,y,z,lam,xsi... from member variable w
			int idx = 0;
			_subx = _subw.block(idx, 0, _xN, 1);         idx += _xN;
			_suby = _subw.block(idx, 0, _ineqN, 1);         idx += _ineqN;
			_subz = _subw(idx);                        idx++;
			_sublam = _subw.block(idx, 0, _ineqN, 1);      idx += _ineqN;
			_subxsi = _subw.block(idx, 0, _xN, 1);         idx += _xN;
			_subeta = _subw.block(idx, 0, _xN, 1);         idx += _xN;
			_submu = _subw.block(idx, 0, _ineqN, 1);      idx += _ineqN;
			_subzet = _subw(idx);                     idx++;
			_subs = _subw.block(idx, 0, _ineqN, 1);
		}

		void GCMMA_PDIPM::separateFromW(const VectorX & w, VectorX & x, VectorX & y, Real & z, VectorX & lam, VectorX & xsi, VectorX & eta, VectorX & mu, Real & zet, VectorX & s)
		{
			// save x,y,z,lam,xsi... from w
			int idx = 0;
			x = w.block(idx, 0, _xN, 1);         idx += _xN;
			y = w.block(idx, 0, _ineqN, 1);         idx += _ineqN;
			z = w(idx);                        idx++;
			lam = w.block(idx, 0, _ineqN, 1);      idx += _ineqN;
			xsi = w.block(idx, 0, _xN, 1);         idx += _xN;
			eta = w.block(idx, 0, _xN, 1);         idx += _xN;
			mu = w.block(idx, 0, _ineqN, 1);      idx += _ineqN;
			zet = w(idx);                     idx++;
			s = w.block(idx, 0, _ineqN, 1);
		}

		void GCMMA_PDIPM::combineToW(const VectorX & x, const VectorX & y, const Real & z, const VectorX & lam, const VectorX & xsi, const VectorX & eta, const VectorX & mu, const Real & zet, const VectorX & s, VectorX & w)
		{
			// save w from x,y,z,lam,xsi,...
			int idx = 0;
			w.block(idx, 0, _xN, 1) = x;         idx += _xN;
			w.block(idx, 0, _ineqN, 1) = y;         idx += _ineqN;
			w(idx) = z;                        idx++;
			w.block(idx, 0, _ineqN, 1) = lam;      idx += _ineqN;
			w.block(idx, 0, _xN, 1) = xsi;         idx += _xN;
			w.block(idx, 0, _xN, 1) = eta;         idx += _xN;
			w.block(idx, 0, _ineqN, 1) = mu;      idx += _ineqN;
			w(idx) = zet;                     idx++;
			w.block(idx, 0, _ineqN, 1) = s;
		}


		void GCMMA_PDIPM::calcGradientW(VectorX & delw)
		{
			// output delw consists of delx, dely, delz, dellam, delxsi,...
			delw.setZero();
			delx.setZero();
			dellam.setZero();

			calcpqlam(_ilp0, _ilpi, _sublam, tmpplam);
			calcpqlam(_ilq0, _ilqi, _sublam, tmpqlam);
			calcdpsidx(tmpplam, tmpqlam, _subx, tmpdpsidx);
			calcgi(_subx, tmpgival);

			//cout << "tmpdpsidx : " << tmpdpsidx << endl << endl;
			//cout << "tmpgival : " << tmpgival << endl << endl;
			//cout << "bi : " << bi << endl << endl;

			for (int i = 0; i < _ineqN; i++)
				for (int j = 0; j < _xN; j++)
					G(i, j) = _ilpi(i, j) / pow(_olupp(j) - _subx(j), 2) - _ilqi(i, j) / pow(_subx(j) - _ollow(j), 2);

			Gt = G.transpose();

			xadi = _subx - _olalpha;
			for (int i = 0; i < _xN; i++)
				xadi(i) = 1.0 / xadi(i);
			bxdi = _olbeta - _subx;
			for (int i = 0; i < _xN; i++)
				bxdi(i) = 1.0 / bxdi(i);
			for (int i = 0; i < _ineqN; i++)
				ydi(i) = 1.0 / _suby(i);
			for (int i = 0; i < _ineqN; i++)
				ldi(i) = 1.0 / _sublam(i);

			for (int i = 0; i < _xN; i++)
				Dx(i, i) = 2 * tmpplam(i) / pow(_olupp(i) - _subx(i), 3) + 2 * tmpqlam(i) / pow(_subx(i) - _ollow(i), 3) + xadi(i) * _subxsi(i) + bxdi(i) * _subeta(i);
			for (int i = 0; i < _xN; i++)
				iDx(i, i) = 1 / Dx(i, i);
			for (int i = 0; i < _ineqN; i++)
				iDy(i) = 1 / (_di(i) + ydi(i) * _submu(i));
			for (int i = 0; i < _ineqN; i++)
				Dly(i, i) = ldi(i) * _subs(i) + iDy(i);
			for (int i = 0; i < _ineqN; i++)
				iDly(i, i) = 1 / Dly(i, i);

			dxt = tmpdpsidx - _subeps * xadi + _subeps * bxdi;
			for (int i = 0; i < _ineqN; i++)
				dyt(i) = _ci(i) + _di(i) * _suby(i) - _sublam(i) - _subeps * ydi(i);
			dzt = _a0 - _sublam.transpose() * _ai - _subeps / _subz;
			dlt = tmpgival - _subz * _ai - _suby + _ilri + _subeps * ldi;
			for (int i = 0; i < _ineqN; i++)
				dlyt(i) = dlt(i) + iDy(i) * dyt(i);

			if (_xN > _ineqN)
			{
				// equation (5.20)
				iDlyGDxGt.setZero();

				// Calculate (Dly + G * iDx * G.transpose())
				Real sum = 0;
				for (int i = 0; i < _xN; i++) // Calculate G * iDx
				{
					for (int j = 0; j < _ineqN; j++)
						MatSizeineqNbyxN(j, i) = G(j, i) * iDx(i, i);
				}

				for (int i = 0; i < _ineqN; i++) // Calculate Dly + M * G.transpose
				{
					for (int j = 0; j < _ineqN; j++)
					{
						for (int k = 0; k < _xN; k++)
							sum += MatSizeineqNbyxN(i, k) * Gt(k, j);
						DlyGDxGt(i, j) = sum;
						sum = 0;
					}
					DlyGDxGt(i, i) += Dly(i, i);
				}

				//iDlyGDxGt = DlyGDxGt.inverse();
				PosDefMatrixInverse(DlyGDxGt, _ineqN, iDlyGDxGt);

				// Calculate dellam, delz, delx
				for (int i = 0; i < _xN; i++) // dellam
				{
					for (int j = 0; j < _ineqN; j++)
						dellam(j) += G(j, i) * iDx(i, i) * dxt(i);
				}
				for (int i = 0; i < _ineqN; i++)
					dellam(i) = dlyt(i) - dellam(i);
				for (int i = 0; i < _ineqN; i++)
				{
					for (int j = 0; j < _ineqN; j++)
						delw(_xN + _ineqN + 1 + i) += iDlyGDxGt(i, j) * dellam(j);
				}
				delw(_xN + _ineqN) = -_subz * dzt / _subzet; // delz
				for (int i = 0; i < _xN; i++) // delx
				{
					for (int j = 0; j < _ineqN; j++)
						delx(i) += Gt(i, j) * delw(_xN + _ineqN + 1 + j);
				}
				for (int i = 0; i < _xN; i++)
					delx(i) += dxt(i);
				for (int i = 0; i < _xN; i++)
					delw(i) = -iDx(i, i) * delx(i);
			}
			else
			{
				// equation (5.22)
				iDxGtiDlyG.setZero();

				// Calculate (Dx + G.transpose()*iDly*G) 
				Real sum = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					for (int j = 0; j < _xN; j++)
						MatSizexNbyineqN(j, i) = Gt(j, i) * iDly(i, i);
				}
				for (int i = 0; i < _xN; i++)
				{
					for (int j = 0; j < _xN; j++)
					{
						for (int k = 0; k < _ineqN; k++)
							sum += MatSizexNbyineqN(i, k) * G(k, j);
						DxGtiDlyG(i, j) = sum;
						sum = 0;
					}
					DxGtiDlyG(i, i) += Dx(i, i);
				}

				//iDxGtiDlyG = DxGtiDlyG.inverse();
				PosDefMatrixInverse(DxGtiDlyG, _xN, iDxGtiDlyG);

				// Calculate delx, delz, dellam
				for (int i = 0; i < _ineqN; i++) // delx
				{
					for (int j = 0; j < _xN; j++)
						delx(j) += Gt(j, i) * iDly(i, i) * dlyt(i);
				}
				for (int i = 0; i < _xN; i++)
					delx(i) = -dxt(i) - delx(i);
				for (int i = 0; i < _xN; i++)
				{
					for (int j = 0; j < _xN; j++)
						delw(i) += iDxGtiDlyG(i, j) * delx(j);
				}
				delw(_xN + _ineqN) = -_subz * dzt / _subzet; // delz
				for (int i = 0; i < _ineqN; i++) // dellam
				{
					for (int j = 0; j < _xN; j++)
						dellam(i) += G(i, j) * delw(j);
				}
				for (int i = 0; i < _ineqN; i++)
					dellam(i) += dlyt(i);
				for (int i = 0; i < _ineqN; i++)
					delw(_xN + _ineqN + 1 + i) = iDly(i, i) * dellam(i);
			}
			for (int i = 0; i < _ineqN; i++) // dely
				delw(i + _xN) = iDy(i) * (delw(i + _xN + _ineqN + 1) - dyt(i));
			for (int i = 0; i < _xN; i++) // delxsi
				delw(i + _xN + _ineqN + 1 + _ineqN) = -xadi(i) * (_subxsi(i) * delw(i) - _subeps) - _subxsi(i);
			for (int i = 0; i < _xN; i++) // deleta
				delw(i + _xN + _ineqN + 1 + _ineqN + _xN) = bxdi(i) * (_subeta(i) * delw(i) + _subeps) - _subeta(i);
			for (int i = 0; i < _ineqN; i++) // delmu
				delw(i + _xN + _ineqN + 1 + _ineqN + _xN + _xN) = -ydi(i) * (_submu(i) * delw(i + _xN) - _subeps) - _submu(i);
			delw(_xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN) = -_subzet * delw(_xN + _ineqN) / _subz - _subzet + _subeps / _subz; // delzet
			for (int i = 0; i < _ineqN; i++) // dels
				delw(i + _xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN + 1) = -ldi(i) * (_subs(i) * delw(i + _xN + _ineqN + 1) - _subeps) - _subs(i);
		}

		void GCMMA_PDIPM::calcpqlam(const VectorX & pq0, const MatrixX & pqi, const VectorX & lam, VectorX & pqlam)
		{
			// input variable size:
			// VectorX pq0: p0(_xN) or q0(_xN)
			// MatrixX pqi: pi(_ineqN, _xN) or qi(_ineqN, _xN)
			// VectorX lam(_ineqN)

			// output variable size:
			// VectorX pqlam: plam(_xN) or qlam(_xN)

			//cout << "lam" << endl << lam << endl << endl;
			//cout << "pq0" << endl << pq0 << endl << endl;
			//cout << "pqi" << endl << pqi << endl << endl;

			for (int j = 0; j < _xN; j++)
			{
				pqlam(j) = pq0(j);
				for (int i = 0; i < _ineqN; i++)
				{
					pqlam(j) += lam(i) * pqi(i, j);
				}
			}
		}

		void GCMMA_PDIPM::calcdpsidx(const VectorX & plam, const VectorX & qlam, const VectorX & x, VectorX & dpsidx)
		{
			//cout << "plam" << endl << plam << endl << endl;
			//cout << "qlam" << endl << qlam << endl << endl;
			//cout << "olupp" << endl << _olupp << endl << endl;
			//cout << "_ollow" << endl << _ollow << endl << endl;

			// equation 5.8
			for (int j = 0; j < _xN; j++)
				dpsidx(j) = plam(j) / pow(_olupp(j) - x(j), 2) - qlam(j) / pow(x(j) - _ollow(j), 2);
		}

		void GCMMA_PDIPM::calcgi(const VectorX & x, VectorX & gival)
		{
			// equation 5.2
			for (int i = 0; i < _ineqN; i++)
			{
				gival(i) = 0;
				for (int j = 0; j < _xN; j++)
					gival(i) += _ilpi(i, j) / (_olupp(j) - x(j)) + _ilqi(i, j) / (x(j) - _ollow(j));
			}
		}

		GCMMAReturnFlag GCMMA_PDIPM::calcStepLength(const VectorX & delw, Real & minValLeq)
		{
			// minValLeq has the minimum value of sths (t <= sth)
			// maxValGeq has the maximum value of sths (t >= sth)
			minValLeq = 1;
			Real maxValGeq = -std::numeric_limits<Real>::max();
			Real coef = 0.99;

			// loop for 'xj + t*delxj - alphaj >= 0.01 * (xj - alphaj)
			for (int j = 0; j < _xN; j++)
			{
				if (delw(j) < 0)
				{
					// update minValLeq
					//if (coef*(_olalpha(j) - _subx(j)) / delw(j) < minValLeq)
					if ((std::abs(coef*(_olalpha(j) - _subx(j)) / delw(j)) > 0.00001) && coef*(_olalpha(j) - _subx(j)) / delw(j) < minValLeq)
						minValLeq = coef*(_olalpha(j) - _subx(j)) / delw(j);
				}
				else if (delw(j) > 0)
				{
					// update maxValGeq
					//if (coef*(_olalpha(j) - _subx(j)) / delw(j) > maxValGeq)
					if ((std::abs(coef*(_olalpha(j) - _subx(j)) / delw(j)) > 0.00001) && coef*(_olalpha(j) - _subx(j)) / delw(j) > maxValGeq)
						maxValGeq = coef*(_olalpha(j) - _subx(j)) / delw(j);
				}
			}

			// loop for 'betaj - (xj + t*delxj) >= 0.01 * (betaj - xj)'
			for (int j = 0; j < _xN; j++)
			{
				if (delw(j) > 0)
				{
					// update minValLeq
					//if (coef*(_olbeta(j) - _subx(j)) / delw(j) < minValLeq)
					if ((std::abs(coef*(_olbeta(j) - _subx(j)) / delw(j)) > 0.00001) && coef*(_olbeta(j) - _subx(j)) / delw(j) < minValLeq)
						minValLeq = coef*(_olbeta(j) - _subx(j)) / delw(j);
				}
				else if (delw(j) < 0)
				{
					// update maxValGeq
					//if (coef*(_olbeta(j) - _subx(j)) / delw(j) > maxValGeq)
					if ((std::abs(coef*(_olbeta(j) - _subx(j)) / delw(j)) > 0.00001) && coef*(_olbeta(j) - _subx(j)) / delw(j) > maxValGeq)
						maxValGeq = coef*(_olbeta(j) - _subx(j)) / delw(j);
				}
			}

			// loop for (y,z,lam,...) + t * (dely, delz, dellam, ...) >= 0.01 * (y, z, lam, ...)
			for (int idx = _xN; idx < _subDimW; idx++)
			{
				if (delw(idx) < 0)
				{
					// update minValLeq
					//if (-coef * _subw(idx) / delw(idx) < minValLeq)
					if ((std::abs(-coef * _subw(idx) / delw(idx)) > 0.00001) && -coef * _subw(idx) / delw(idx) < minValLeq)
						minValLeq = -coef * _subw(idx) / delw(idx);
				}
				else if (delw(idx) > 0)
				{
					// update maxValGeq
					//if (-coef * _subw(idx) / delw(idx) > maxValGeq)
					if ((std::abs(-coef * _subw(idx) / delw(idx)) > 0.00001) && -coef * _subw(idx) / delw(idx) > maxValGeq)
						maxValGeq = -coef * _subw(idx) / delw(idx);
				}
			}

			//LOGIF(maxValGeq <= minValLeq, "'minValLeq' must be larger than 'maxValGeq'");
			if (RealBigger(maxValGeq, minValLeq, 1E-5))
			{
				//cout << maxValGeq << '\t' << minValLeq << endl;
				return subProblemFailure_calcTau;
			}
			//cout << "maxValGeq : " << maxValGeq << endl;
			//cout << "minVelLeg : " << minValLeq << endl;

			//Real resNorm = calcNormResidual(p0, pi, q0, qi, bi, delw, 0.0, 2);
			Real resNorm = calcNormResidual(delw, 0.0, 2);
			bool tauFound = false;
			int iterTau = 0, maxIterTau = 1000;

			//cout << "minValLeq : " << minValLeq << endl;
			while (iterTau < maxIterTau)
			{
				//if (calcNormResidual(p0, pi, q0, qi, bi, delw, minValLeq, 2) < resNorm)
				if (calcNormResidual(delw, minValLeq, 2) < resNorm)
				{
					tauFound = true;
					break;
				}
				minValLeq *= 0.5;
				iterTau++;

				if (iterTau == maxIterTau)
				{
					//cout << "init : " << init << endl;
					//cout << "maxValGeq : " << maxValGeq << endl;
					//cout << "minValLeg : " << minValLeq << endl;
					//cout << endl;
				}

			}
			//LOGIF(tauFound, "failed to find step length 'tau'");

			if (tauFound)
				return subProblemSuccess;
			else
				return subProblemFailure_calcTau;
		}

		Real GCMMA_PDIPM::calcNormResidual(const VectorX& delw, Real stepLength, int normCh)
		{
			//VectorX resvec(_subDimW);

			//cout << "subx" << endl << _subx << endl << endl;

			//VectorX tmpsubx(_xN), tmpsublam(_ineqN);
			for (int i = 0; i < _xN; i++)
				tmpsubx(i) = _subx(i) + stepLength * delw(i);
			for (int i = 0; i < _ineqN; i++)
				tmpsublam(i) = _sublam(i) + stepLength * delw(i + _xN + _ineqN + 1);

			//cout << "tmpsubx" << endl << tmpsubx << endl << endl;

			//VectorX tmpplam(_xN), tmpqlam(_xN), tmpdpsidx(_xN), tmpgival(_ineqN);
			calcpqlam(_ilp0, _ilpi, tmpsublam, tmpplam);
			calcpqlam(_ilq0, _ilqi, tmpsublam, tmpqlam);
			calcdpsidx(tmpplam, tmpqlam, tmpsubx, tmpdpsidx);
			calcgi(tmpsubx, tmpgival);

			//cout << "tmpdpsidx" << endl << tmpdpsidx << endl << endl;
			//cout << "_subxsi" << endl << _subxsi << endl;
			//cout << "_subeta" << endl << _subeta << endl;

			int idx = 0;
			for (int i = 0; i < _xN; i++)
				resvec(i + idx) = tmpdpsidx(i) - _subxsi(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN) + _subeta(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN + _xN);
			idx += _xN;
			for (int i = 0; i < _ineqN; i++)
				resvec(i + idx) = _ci(i) + _di(i) * (_suby(i) + stepLength * delw(i + _xN)) - tmpsublam(i) - (_submu(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN + _xN + _xN));
			idx += _ineqN;
			resvec(idx) = _a0 - (_subzet + stepLength * delw(_xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN)) - tmpsublam.transpose() * _ai;
			idx += 1;
			for (int i = 0; i < _ineqN; i++)
				resvec(i + idx) = tmpgival(i) - (_subz + stepLength * delw(_xN + _ineqN)) * _ai(i) - (_suby(i) + stepLength * delw(i + _xN)) + (_subs(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN + 1)) + _ilri(i);
			idx += _ineqN;
			for (int i = 0; i < _xN; i++)
				resvec(i + idx) = (_subxsi(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN)) * (tmpsubx(i) - _olalpha(i)) - _subeps;
			idx += _xN;
			for (int i = 0; i < _xN; i++)
				resvec(i + idx) = (_subeta(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN + _xN)) * (_olbeta(i) - tmpsubx(i)) - _subeps;
			idx += _xN;
			for (int i = 0; i < _ineqN; i++)
				resvec(i + idx) = (_submu(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN + _xN + _xN)) * (_suby(i) + stepLength * delw(i + _xN)) - _subeps;
			idx += _ineqN;
			resvec(idx) = (_subzet + stepLength * delw(_xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN)) * (_subz + stepLength * delw(_xN + _ineqN)) - _subeps;
			idx += 1;
			for (int i = 0; i < _ineqN; i++)
				resvec(i + idx) = tmpsublam(i) * (_subs(i) + stepLength * delw(i + _xN + _ineqN + 1 + _ineqN + _xN + _xN + _ineqN + 1)) - _subeps;

			//cout << "resvec" << endl << resvec << endl << endl;

			Real normVal;
			switch (normCh)
			{
			case 2: // Euclidean norm
				normVal = resvec.norm();
				break;
			case -1: // infinite norm
				normVal = resvec.cwiseAbs().maxCoeff();
				break;
			default:
				LOG("wrong choice - function 'calcNormResidual'");
				break;
			}
			return normVal;

			return 0;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		void GCMMA_PDIPM::calcx(const VectorX & lam, VectorX & subx)
		{
			Real ltpj, ltqj, tmpval;
			for (int j = 0; j < _xN; j++)
			{
				ltpj = 0; ltqj = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					ltpj += lam(i) * _ilpi(i, j);
					ltqj += lam(i) * _ilqi(i, j);
				}
				//cout << p0(j) + ltpj << endl;
				//cout << q0(j) + ltqj << endl << endl;
				tmpval = (sqrt(_ilp0(j) + ltpj) * _ollow(j) + sqrt(_ilq0(j) + ltqj) * _olupp(j)) / (sqrt(_ilp0(j) + ltpj) + sqrt(_ilq0(j) + ltqj));
				subx(j) = Max(_olalpha(j), Min(_olbeta(j), tmpval));
				//cout << _olalpha(j) << '\t' << _olbeta(j) << '\t' << tmpval << endl << endl;
			}
		}

		void GCMMA_PDIPM::calcdW(const VectorX & lam, VectorX & dW)
		{
			// calc x/y 필요한가.....
			calcx(lam, _subx);
			calcy(lam, _suby);

			//cout << _TR_subx << endl;
			//cout << _TR_suby << endl;

			Real tmpval;
			for (int i = 0; i < _ineqN; i++)
			{
				tmpval = 0;
				for (int j = 0; j < _xN; j++)
					tmpval += _ilpi(i, j) / (_olupp(j) - _subx(j)) + _ilqi(i, j) / (_subx(j) - _ollow(j));

				dW(i) = -tmpval - _ilri(i) + _suby(i);
			}
		}

		void GCMMA_PDIPM::calcy(const VectorX & lam, VectorX & suby)
		{
			for (int i = 0; i < _ineqN; i++)
				suby(i) = Max(0.0, (lam(i) - _ci(i)) / _di(i));
		}

		void GCMMA_PDIPM::calcW(const VectorX & lam, Real & W)
		{
			calcx(lam, _subx);
			calcy(lam, _suby);

			W = 0;
			Real tmpval0, tmpval1;


			tmpval0 = 0;
			for (int i = 0; i < _ineqN; i++)
				tmpval0 += lam(i) * _ilri(i);

			W += _ilr0 + tmpval0;

			for (int j = 0; j < _xN; j++)
			{
				tmpval0 = 0; tmpval1 = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					tmpval0 += lam(i)*_ilpi(i, j);
					tmpval1 += lam(i)*_ilqi(i, j);
				}
				W += (_ilp0(j) + tmpval0) / (_olupp(j) - _subx(j)) + (_ilq0(j) + tmpval1) / (_subx(j) - _ollow(j));
			}

			for (int i = 0; i < _ineqN; i++)
				W += _suby(i) * (_ci(i) + 0.5*_di(i)*_suby(i) - lam(i));

			W *= -1;
		}
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		GCMMAReturnFlag GCMMA_TRM::solveSubProblem(VectorX & xout)
		{
			// input variable size:
			// VectorX p0(_xN), q0(_xN)
			// MatrixX pi(_ineqN, _xN), qi(_ineqN, _xN)
			// VectorX ri(_ineqN)
			// VectorX alpha(_xN), beta, low, upp

			// output variable size:
			// VectorX xout(_xN)

			// initialize
			initializeSubProb();
			_bUpdated = true;

			//saveVectorX2txt123(_subdW, filedd);
			//saveVectorX2txt123(_subdWm1, filedd);
			//saveRealX2txt123(_radius, filedd);

			//std::string filename = "C:/Users/crazy/Desktop/mma_new/parameter2/";
			//saveVectorX2txt(_ci, filename + "_ci.txt");
			//saveVectorX2txt(_di, filename + "_di.txt");
			//saveVectorX2txt(_ollow, filename + "_ollow.txt");
			//saveVectorX2txt(_olupp, filename + "_olupp.txt");
			//saveVectorX2txt(_olalpha, filename + "_olalpha.txt");
			//saveVectorX2txt(_olbeta, filename + "_olbeta.txt");
			//saveVectorX2txt(_ilp0, filename + "_ilp0.txt");
			//saveVectorX2txt(_ilq0, filename + "_ilq0.txt");
			//saveMatrixX2txt(_ilpi, filename + "_ilpi.txt");
			//saveMatrixX2txt(_ilqi, filename + "_ilqi.txt");
			//cout << _ilr0 << endl;
			//saveVectorX2txt(_ilri, filename + "_ilri.txt");


			//int iterSub = 0, maxIterSub = 10000, iterSub1 = 0, maxIterSub1 = 20000;
			int iterSub = 0, maxIterSub = 10, iterSub1 = 0, maxIterSub1 = 20;
			bool solFound = false;
			while (iterSub < maxIterSub && iterSub1 < maxIterSub1)
				//while(1)
			{
				//cout << "===================================================" << endl << endl;
				// step 1-1: calculate eta - equation(16)
				calceta();
				//cout << _subeta << endl << endl;
				//saveRealX2txt123(_subeta, filedd);

				// step 1-2: calculate lambda_hat - equation(18)
				calclamhat();
				//cout << _TR_sublamhat << endl << endl;

				//saveVectorX2txt123(_sublam, filedd);
				//saveVectorX2txt123(_sublamhat, filedd);

				// step 2: calculate theta
				calcW(_sublam, _subW);
				calcW(_sublamhat, _subWhat);
				calcm(_sublam, _subm);
				calcm(_sublamhat, _submhat);
				_subtheta = (_subW - _subWhat) / (_subm - _submhat);

				//saveRealX2txt123(_subW, filedd);
				//saveRealX2txt123(_subWhat, filedd);
				//saveRealX2txt123(_subm, filedd);
				//saveRealX2txt123(_submhat, filedd);
				//saveRealX2txt123(_subtheta, filedd);

				//cout << _TR_subW << endl << endl;
				//cout << _TR_subWhat << endl << endl;
				//cout << _TR_subm << endl << endl;
				//cout << _TR_submhat << endl << endl;
				//cout << _TR_subtheta << endl << endl;

				// step 3: update lambda
				_sublamm1 = _sublam;
				_subdWm1 = _subdW;
				if (_subtheta > _TR_v)
				{
					_bUpdated = true;
					_sublam = _sublamhat;
					calcdW(_sublam, _subdW);
					//cout << _TR_sublam << endl << endl;
					//cout << _TR_subdW << endl << endl;
					iterSub++;
					//cout << iterSub << '\t' << _subW << '\t' << _subW - _subWhat << '\t'
					//	<< VectorInner(_sublam, _subdW, _ineqN) << '\t' << (_sublam - _sublamm1).norm() << _radius << endl;
					//cout << iterSub << '\t' << VectorInner(_sublam - _sublamm1, _subdW, _ineqN) << endl;
					//if (abs(VectorInner(_sublam - _sublamm1, _subdW, _ineqN)) < 1E-2)
					//	break;
				}
				else
				{
					_bUpdated = false;
					//_bUpdated = true;
				}

				// step 4: update radius of trust region
				//cout << _TR_gamma2 << endl;
				//cout << _TR_gamma0 << '\t' << _TR_gamma1 << endl;
				if (_subtheta >= _TR_w)
				{
					_radius *= _TR_gamma2;
				}
				else if (_subtheta > _TR_v)
				{
					_radius *= 1;
				}
				else
				{
					//_radius *= (_TR_gamma0 + _TR_gamma1) / 2;
					_radius *= _TR_gamma1;
				}


				//if (iterSub1 > 5 && abs(VectorInner(_sublam, _subdW, _ineqN)) < 1E+2)
				// break;

				//if (_bUpdated && abs(VectorInner(_sublam - _sublamm1, _subdW, _ineqN)) < 1E-2)
				//{
				// cout << iterSub << endl;
				// break;
				//}
				//cout << VectorInner(_sublam, _subdW, _ineqN) << endl;
				//if ( iterSub > 10 &&(abs(VectorInner(_sublam, _subdW, _ineqN)) < 1E-3))
				//{
				// cout << _subW << endl;
				// solFound = true;
				// cout << iterSub << endl;
				// break;
				//}

				//if (_bUpdated)
				//{
				// cout << VectorInner(_sublam, _subdW, _ineqN) << endl;
				// //cout << (_sublam - _sublamm1).norm() << '\t' << _subW << '\t' <<_subWhat<< endl;
				//}
				//if (_radius < 1E-3) // terminate condition
				//{
				// solFound = true;
				// break;
				//}
				iterSub1++;
			}

			//VectorX x(_xN), y(_ineqN);
			//calcx(_sublam, x);
			//calcy(_sublam, y);
			//VectorX result = _ineqConstraint->func(x);
			//VectorX resulttilde(_ineqN);
			//calcfitilde(x, resulttilde);
			//for (int i = 0; i < _ineqN; i++)
			//{
			// if (result(i) > 0)
			//  cout << "inequality error" << endl;
			// if (resulttilde(i) - y(i) > 0)
			//  cout << "ineqqqqqq error" << endl;
			//}

			//cout << "iterSub : " << iterSub << endl;
			//cout << "iterSub1 : " << iterSub1 << endl;
			//cout << "_sublam" << endl << _sublam << endl << endl;
			//cout << "W : " << _subW << endl;
			//calcdW(_sublam, _subdW);
			//cout << "lam * dW : " << VectorInner(_sublam, _subdW) << endl;
			//cout << "dW : " << _subdW << endl << endl;

			//if (!solFound)
			// LOG("exceeded max iteration number - 'TRsolveSubProblem'");

			//cout << iterSub << endl;
			//cout << _subeps << endl;
			//cout << "lambda: " << _sublam << endl;
			calcx(_sublam, _subx);
			calcy(_sublam, _suby);

			xout = _subx;

			// 현재 구현 상태는 loop 돌다가 iterMax 치면 나오고.. 수렴조건 안쓰는 상태임....
			return subProblemSuccess;
		}

	#ifdef STRATEGY_02
		void GCMMA_PDIPM::calcResCur(const VectorX & fival, const MatrixX & df0dx, const MatrixX & dfidx)
		{
			Real tmpval;
			_rescur = 0;

			for (int j = 0; j < _xN; j++)
			{
				tmpval = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					tmpval += _sublam(i) * dfidx(i, j);
				}
				tmpval += df0dx(j);
				if (tmpval > 0)
				{
					tmpval *= _subx(j) - _minX(j);
					_rescur += tmpval * tmpval;
				}
				else
				{
					tmpval *= -1;
					tmpval *= _maxX(j) - _subx(j);
					_rescur += tmpval * tmpval;
				}
			}

			for (int i = 0; i < _ineqN; i++)
			{
				tmpval = _ci(i) + _di(i) *_suby(i) - _sublam(i) - _submu(i);
				_rescur += tmpval * tmpval;
			}

			tmpval = 0;
			for (int i = 0; i < _ineqN; i++)
				tmpval += _ai(i) * _sublam(i);
			tmpval *= -1;
			tmpval += _a0 - _subzet;
			_rescur += tmpval * tmpval;

			for (int i = 0; i < _ineqN; i++)
			{
				tmpval = fival(i) - _ai(i)*_subz - _suby(i);
				if (tmpval > 0)
				{
					_rescur += tmpval * tmpval;
				}
				else
				{
					tmpval *= -_sublam(i);
					_rescur += tmpval * tmpval;
				}
			}

			for (int i = 0; i < _ineqN; i++)
			{
				tmpval = -_suby(i);
				if (tmpval > 0)
				{
					_rescur += tmpval * tmpval;
				}
				else
				{
					tmpval *= -_submu(i);
					_rescur += tmpval * tmpval;
				}
			}

			tmpval = -_subz;
			if (tmpval > 0)
			{
				_rescur += tmpval * tmpval;
			}
			else
			{
				tmpval *= -_subzet;
				_rescur += tmpval * tmpval;
			}

			_rescur = sqrt(_rescur);
		}
	#endif

		void GCMMA_TRM::allocSUBvar(void)
		{
			_sublam.resize(_ineqN);
			_sublamm1.resize(_ineqN);
			_sublamhat.resize(_ineqN);

			_subdW.resize(_ineqN);
			_subdWm1.resize(_ineqN);

			_subs.resize(_ineqN);
			_subt.resize(_ineqN);

			_subx.resize(_xN);
			_suby.resize(_ineqN);
		}

		void GCMMA_TRM::setParametersTR(const Real & v, const Real & w, const Real & gam0, const Real & gam1, const Real & gam2)
		{
			// parameter setting...
			// 0 < v < w < 1
			_TR_v = v;// 0.3;
			_TR_w = w;// 0.7;
					  // 0 < gamma0 <= gamma1 < 1 <= gamma2
			_TR_gamma0 = gam0;// 0.5;
			_TR_gamma1 = gam1;// 0.7;
			_TR_gamma2 = gam2;// 1.2;
		}

		void GCMMA_TRM::initializeSubProb(void)
		{
			// insert values...
			_sublam.setZero();
			//_TR_sublam.setConstant(2E-3);
			_sublamm1.setConstant(1E-3);

			//_sublam.setOnes();
			//_sublamm1.setConstant(1 + 1E-3);

			calcdW(_sublam, _subdW);
			calcdW(_sublamm1, _subdWm1);

			_radius = 0.1 * _subdW.norm();
			//_radius = 0.1 * abs(_subdW.maxCoeff());
		}

		void GCMMA_TRM::calcx(const VectorX & lam, VectorX & subx)
		{
			Real ltpj, ltqj, tmpval;
			for (int j = 0; j < _xN; j++)
			{
				ltpj = 0; ltqj = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					ltpj += lam(i) * _ilpi(i, j);
					ltqj += lam(i) * _ilqi(i, j);
				}
				//cout << p0(j) + ltpj << endl;
				//cout << q0(j) + ltqj << endl << endl;
				tmpval = (sqrt(_ilp0(j) + ltpj) * _ollow(j) + sqrt(_ilq0(j) + ltqj) * _olupp(j)) / (sqrt(_ilp0(j) + ltpj) + sqrt(_ilq0(j) + ltqj));
				subx(j) = Max(_olalpha(j), Min(_olbeta(j), tmpval));
				//cout << _olalpha(j) << '\t' << _olbeta(j) << '\t' << tmpval << endl << endl;
			}
		}

		void GCMMA_TRM::calcy(const VectorX & lam, VectorX & suby)
		{
			for (int i = 0; i < _ineqN; i++)
				suby(i) = Max(0.0, (lam(i) - _ci(i)) / _di(i));
		}

		void GCMMA_TRM::calcW(const VectorX & lam, Real & W)
		{
			calcx(lam, _subx);
			calcy(lam, _suby);

			W = 0;
			Real tmpval0, tmpval1;

			tmpval0 = 0;
			for (int i = 0; i < _ineqN; i++)
				tmpval0 += lam(i) * _ilri(i);

			W += _ilr0 + tmpval0;

			for (int j = 0; j < _xN; j++)
			{
				tmpval0 = 0; tmpval1 = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					tmpval0 += lam(i)*_ilpi(i, j);
					tmpval1 += lam(i)*_ilqi(i, j);
				}
				W += (_ilp0(j) + tmpval0) / (_olupp(j) - _subx(j)) + (_ilq0(j) + tmpval1) / (_subx(j) - _ollow(j));
			}

			for (int i = 0; i < _ineqN; i++)
				W += _suby(i) * (_ci(i) + 0.5*_di(i)*_suby(i) - lam(i));

			W *= -1;
		}

		void GCMMA_TRM::calcdW(const VectorX & lam, VectorX & dW)
		{
			// calc x/y 필요한가.....
			calcx(lam, _subx);
			calcy(lam, _suby);

			//cout << _TR_subx << endl;
			//cout << _TR_suby << endl;

			Real tmpval;
			for (int i = 0; i < _ineqN; i++)
			{
				tmpval = 0;
				for (int j = 0; j < _xN; j++)
					tmpval += _ilpi(i, j) / (_olupp(j) - _subx(j)) + _ilqi(i, j) / (_subx(j) - _ollow(j));

				dW(i) = -tmpval - _ilri(i) + _suby(i);
			}
		}

		void GCMMA_TRM::calcm(const VectorX & lam, Real & m)
		{
			Real tmpval = 0;
			for (int i = 0; i < _ineqN; i++)
				tmpval += _subdW(i) * (lam(i) - _sublam(i));
			m = _subW + tmpval;

			tmpval = 0;
			for (int i = 0; i < _ineqN; i++)
				tmpval += (lam(i) - _sublam(i)) * (lam(i) - _sublam(i));

			m += 0.5 * _subeta * tmpval;
		}

		void GCMMA_TRM::calceta(void)
		{
			//cout << _TR_sublam << endl << endl;
			//cout << _TR_sublamm1 << endl << endl;
			//cout << _TR_subdW << endl << endl;
			//cout << _TR_subdWm1 << endl << endl;
			if (_bUpdated)
			{
				_subs = _sublam - _sublamm1;
				_subt = _subdW - _subdWm1;

				Real tmpnum = 0, tmpden = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					tmpnum += _subs(i) * _subt(i);
					tmpden += _subs(i) * _subs(i);
				}
				//_subeta = tmpnum / tmpden;
				_subeta = Max(1E-3, Min(1E3, tmpnum / tmpden));
				//cout << tmpnum / tmpden << endl;
				// _subeta = Max(0.5E-3, Min(1E3, tmpnum / tmpden));
			}

			//if (RealLess(tmpden, 1E-2))
			// _subeta = 1E-2;
			//else
			// _subeta = tmpnum / tmpden;


			//cout << tmpnum << endl;
			//cout << tmpden << endl;
			//cout << _subeta << endl;
		}

		void GCMMA_TRM::calclamhat(void)
		{
			for (int i = 0; i < _ineqN; i++)
			{
				//cout << _TR_sublam(i) + _TR_radius << '\t' << _TR_sublam(i) - _TR_radius << '\t' << _TR_sublam(i) - (_TR_subdW(i)) / (_TR_subeta) << endl << endl;
				_sublamhat(i) = Min(_sublam(i) + _radius, Max(Max(0.0, _sublam(i) - _radius), _sublam(i) - (_subdW(i)) / (_subeta)));
				//cout << _TR_sublamhat(i) << endl << endl;
			}
		}

		///////////////////////////////////////////////////////
		////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////
		void GCMMA_GDM::GD_initializeSubProb()
		{
			_sublam.resize(_ineqN);
			_sublam.setZero();
			_subx.resize(_xN);
			_suby.resize(_ineqN);
			_subdW.resize(_ineqN);
		}

		void GCMMA_GDM::calcx(const VectorX& lam, /* output */ VectorX& subx)
		{
			Real ltpj, ltqj, tmpval;
			for (int j = 0; j < _xN; j++)
			{
				ltpj = 0; ltqj = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					ltpj += lam(i) * _ilpi(i, j);
					ltqj += lam(i) * _ilqi(i, j);
				}
				//cout << p0(j) + ltpj << endl;
				//cout << q0(j) + ltqj << endl << endl;
				tmpval = (sqrt(_ilp0(j) + ltpj) * _ollow(j) + sqrt(_ilq0(j) + ltqj) * _olupp(j)) / (sqrt(_ilp0(j) + ltpj) + sqrt(_ilq0(j) + ltqj));
				subx(j) = Max(_olalpha(j), Min(_olbeta(j), tmpval));
				//cout << _olalpha(j) << '\t' << _olbeta(j) << '\t' << tmpval << endl << endl;
			}
		}

		void GCMMA_GDM::calcy(const VectorX& lam, /* output */ VectorX& suby)
		{
			for (int i = 0; i < _ineqN; i++)
				suby(i) = Max(0.0, (lam(i) - _ci(i)) / _di(i));
		}

		void GCMMA_GDM::calcW(const VectorX& lam, /* output */ Real& W)
		{
			calcx(lam, _subx);
			calcy(lam, _suby);

			W = 0;
			Real tmpval0, tmpval1;

			tmpval0 = 0;
			for (int i = 0; i < _ineqN; i++)
				tmpval0 += lam(i) * _ilri(i);

			W += _ilr0 + tmpval0;

			for (int j = 0; j < _xN; j++)
			{
				tmpval0 = 0; tmpval1 = 0;
				for (int i = 0; i < _ineqN; i++)
				{
					tmpval0 += lam(i)*_ilpi(i, j);
					tmpval1 += lam(i)*_ilqi(i, j);
				}
				W += (_ilp0(j) + tmpval0) / (_olupp(j) - _subx(j)) + (_ilq0(j) + tmpval1) / (_subx(j) - _ollow(j));
			}

			for (int i = 0; i < _ineqN; i++)
				W += _suby(i) * (_ci(i) + 0.5*_di(i)*_suby(i) - lam(i));

			W *= -1;
		}

		void GCMMA_GDM::calcdW(const VectorX& lam, /* output */ VectorX& dW)
		{
			// calc x/y 필요한가.....
			calcx(lam, _subx);
			calcy(lam, _suby);

			//cout << _TR_subx << endl;
			//cout << _TR_suby << endl;

			Real tmpval;
			for (int i = 0; i < _ineqN; i++)
			{
				tmpval = 0;
				for (int j = 0; j < _xN; j++)
					tmpval += _ilpi(i, j) / (_olupp(j) - _subx(j)) + _ilqi(i, j) / (_subx(j) - _ollow(j));

				dW(i) = -tmpval - _ilri(i) + _suby(i);
			}
		}

		GCMMAReturnFlag GCMMA_GDM::solveSubProblem(/* output */ VectorX& xout)
		{
			GD_initializeSubProb();

			int iter = 0, maxIter = 100000;
			bool break_swi = false;

			calcW(_sublam, _subW);
			calcdW(_sublam, _subdW);

			Real stepsize = 2;
			Real v = 0;

			while (iter < maxIter)
			{
				//cout << "iter : " << iter << endl;
				//cout << "_sublam" << endl << _sublam << endl << endl;
				//cout << "_subdW" << endl << _subdW << endl << endl;

				_sublam = _sublam - stepsize * _subdW;

				for (int i = 0; i < _ineqN; i++)
				{
					if (_sublam(i) < 0)
						_sublam(i) = 0;
				}

				v = _sublam.transpose() * _subdW;
				//calcW(_sublam, _subW);
				calcdW(_sublam, _subdW);

				//cout << "_sublam" << endl << _sublam << endl << endl;
				//cout << "_subdW" << endl << _subdW << endl << endl;

				iter++;

				if (std::abs(v) < 20)
					break_swi = true;

				if (break_swi)
					break;
			}

			VectorX x(_xN), y(_ineqN);
			calcx(_sublam, x);
			calcy(_sublam, y);
			VectorX result = _ineqConstraint->func(x);
			for (int i = 0; i < _ineqN; i++)
			{
				if (result(i) > 0)
					cout << "inequality error" << endl;
			}

			//calcW(_sublam, _subW);
			//cout << iter << endl;
			//cout << v << endl;
			//cout << _subW << endl;
			//cout << "_sublam" << endl;
			//cout << _sublam << endl << endl;
			//cout << "_subdW" << endl << _subdW << endl;

			calcx(_sublam, _subx);
			calcy(_sublam, _suby);

			xout = _subx;
			return GCMMAReturnFlag::Success_tolFunc;
		}
	}
}