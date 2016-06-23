#pragma once

#include "Function.h"
#include "Common.h"
#include "Interpolation.h"

#include <irUtils\Diagnostic.h>

#define STRATEGY_01
//#define STRATEGY_02

using namespace irLib::irMath;

namespace irLib
{
	namespace irTG
	{
		class GCMMAOptimization;
		class GCMMA_PDIPM;
		class GCMMA_TRM;
		class GCMMA_GDM;

		enum GCMMAReturnFlag
		{
			Success_tolFunc,
			Success_tolX,

			quasiSuccess_subProbFailure,
			

			Failure_exceedMaxIterOL,
			Failure_exceedInequality,

			subProblemFailure,
			subProblemSuccess,
			subProblemFailure_calcTau,
		};

		typedef void(*objf)(FunctionPtr f, const VectorX&, VectorX&, const Real, const VectorX&);
		typedef void(*objJ)(FunctionPtr f, const VectorX&, MatrixX&, const Real, const VectorX&);
		typedef void(*Inequalf)(FunctionPtr f, const VectorX&, VectorX&, const VectorX&, const VectorX&);
		typedef void(*InequalJ)(FunctionPtr f, const VectorX&, MatrixX&, const VectorX&, const VectorX&);

		void ObjFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const Real scaleObjFunc, const VectorX& scaleX);
		void ObjJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const Real scaleObjFunc, const VectorX& scaleX);
		void InequalFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const VectorX& scaleIneqFunc, const VectorX& scaleX);
		void InequalJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const VectorX& scaleIneqFunc, const VectorX& scaleX);

		void ScaleObjFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const Real scaleObjFunc, const VectorX& scaleX);
		void ScaleObjJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const Real scaleObjFunc, const VectorX& scaleX);
		void ScaleInequalFcnEval(FunctionPtr f, const VectorX& params, VectorX& fval, const VectorX& scaleIneqFunc, const VectorX& scaleX);
		void ScaleInequalJacobianEval(FunctionPtr f, const VectorX& params, MatrixX& dfdx, const VectorX& scaleIneqFunc, const VectorX& scaleX);

		void displayGCMMAResult(GCMMAReturnFlag retFlag);


		/*
		* GCMMAOpimization Class
		*/
		class GCMMAOptimization
		{
		public:
			GCMMAOptimization(const int xN, const int ineqN, bool st_scale = true);
			virtual ~GCMMAOptimization() {}

			GCMMAReturnFlag solve(const VectorX& initialX);

			const VectorX& getResultX() const { return resultX; }
			const Real getResultFunc() const { return resultFunc; }

			void setXN(const int xN) { _xN = xN; }
			void setIneqN(const int ineqN) { _ineqN = ineqN; }
			void setObjectiveFunction(const FunctionPtr& objectFunc) { _objectFunc = objectFunc; }
			void setInequalityConstraint(const FunctionPtr& ineqConstraint) { _ineqConstraint = ineqConstraint; }
			void setParameters(const Real& albefa, const Real& move0, const Real& asyinit, const Real& asydecr, const Real& asyincr);
			void setCoefficients(const Real& a0, const VectorX& ai, const VectorX& ci, const VectorX& di);
			void setMinMax(const VectorX& minX, const VectorX& maxX);
			void setTolX(const Real tolX) { _tolX = tolX; }
			void setTolFunc(const Real tolFunc) { _tolFunc = tolFunc; }
			void setMaxIterOL(const int maxIterOL) { _maxIterOL = maxIterOL; }
			void setMaxIterIL(const int maxIterIL) { _maxIterIL = maxIterIL; }

		private:
			virtual GCMMAReturnFlag solveSubProblem(/* output */ VectorX& xout) = 0;
			virtual void allocSUBvar(void) = 0;
			void initialize(int xN, int ineqN);
			void allocOLvar(void);
			void allocILvar(void);
			void restoreResultScale(void);
			void calScaleFactors(VectorX& x);
			void calcPQR(const MatrixX& df0dxp, const MatrixX& df0dxm, const MatrixX& dfidxp, const MatrixX& dfidxm, const VectorX& xk, const VectorX& f0val, const VectorX& fival);
			void calcf0tilde(const VectorX& x, /* output */ VectorX& f0tval);
			void calcfitilde(const VectorX& x, /* output */ VectorX& fitval);
			void calcLowUpp(int iter, const VectorX& xk, const VectorX& xkm1, const VectorX& xkm2);
			void calcAlphaBeta(const VectorX& xk);
			void calcPlusMinusMatrix(const MatrixX& mat, /* output */ MatrixX& matp, MatrixX& matm);
			void updateRho0i(const VectorX& xknu, const VectorX& xk, const VectorX& f0valknu, const VectorX& fivalknu, const VectorX& f0tvalknu, const VectorX& fitvalknu);
			bool testILSuccess(const VectorX& testx, /* output */ VectorX& f0valknu, VectorX& fivalknu, VectorX& f0tvalknu, VectorX& fitvalknu);
			GCMMAReturnFlag checkSolution(const VectorX& fival, GCMMAReturnFlag inFlag);

#ifdef STRATEGY_01
			void calcSigma(int iter, const VectorX& xk, const VectorX& xkm1, const VectorX& xkm2);
			void calcInitialRho_st01(int iter, const VectorX& xk, const VectorX& xkm1, const MatrixX& df0dx, const MatrixX& df0dxm1,
				const MatrixX& dfidx, const MatrixX& dfidxm1);
#else
			void calcInitialRho(const MatrixX& df0dx, const MatrixX& dfidx);
#endif
#ifdef STRATEGY_02
			Real _rescur, _resm1, _resm2; ///> reidue of KKT condition: current(cur), 1step before(m1), 2step before(m2)
			Real _muVal;
			virtual void calcResCur(const VectorX& fival, const MatrixX& df0dx, const MatrixX& dfidx) = 0;
#endif

		public:
			int _xN;
			int _ineqN;
			bool Strategy_Scale;
			FunctionPtr _objectFunc;
			FunctionPtr _ineqConstraint;

			Real _a0;
			VectorX _ai;
			VectorX _ci;
			VectorX _di;

			VectorX _ollow;
			VectorX _ollowm1;
			VectorX _olupp;
			VectorX _oluppm1;
			VectorX _olalpha;
			VectorX _olbeta;

			VectorX _ilp0;
			VectorX _ilq0;
			MatrixX _ilpi;
			MatrixX _ilqi;
			Real _ilr0;
			Real _ilrho0;
			VectorX _ilri;
			VectorX _ilrhoi;

		private:
			objf _objectf;
			objJ _objectJ;
			Inequalf _ineqf;
			InequalJ _ineqJ;

			Real _ALBEFA;
			Real _MOVE0;
			Real _ASYINIT;
			Real _ASYDECR;
			Real _ASYINCR;
			Real resultFunc;
			VectorX resultX;
			VectorX _minX;
			VectorX _maxX;
			VectorX _scaleIneqFunc;
			VectorX _scaleX;
			Real	_scaleObjFunc;
			Real _tolX;
			Real _tolFunc;
			int _maxIterOL;
			int _maxIterIL;

#ifdef STRATEGY_01
			Real _oleta;
			VectorX _olsigma;
			VectorX _ols;
			VectorX _olt;
			VectorX _olb;
#endif

		public:
			//void saveMatrixX2txt(MatrixX in, std::string filename);
			//void saveVectorX2txt(VectorX in, std::string filename);
		};


		/*
		* GCMMA_PDIPM Class
		* Solve GCMMA subproblem by using 'Primal-Dual Interior-Point Method'
		*/
		class GCMMA_PDIPM : public GCMMAOptimization
		{
		public:
			GCMMA_PDIPM(int xN, int ineqN) : GCMMAOptimization(xN, ineqN) {}
			GCMMAReturnFlag solveSubProblem(/* output */ VectorX& xout);

		private:
			void allocSUBvar(void);
			void initializeSubProb(void);
			void separateFromW(void);

			void calcx(const VectorX& lam, /* output */ VectorX& subx);
			void calcy(const VectorX& lam, /* output */ VectorX& suby);
			void calcgi(const VectorX& x, /* output */ VectorX& gival);
			void calcW(const VectorX& lam, /* output */ Real& W);
			void calcdW(const VectorX& lam, /* output */ VectorX& dW);
			void calcGradientW(/* output */ VectorX& delw);
			void calcpqlam(const VectorX& pq0, const MatrixX& pqi, const VectorX& lam, /* output */ VectorX& pqlam);
			void calcdpsidx(const VectorX& plam, const VectorX& qlam, const VectorX& x, /* output */ VectorX& dpsidx);
			void separateFromW(const VectorX& w, /* output */ VectorX& x, VectorX& y, Real& z, VectorX& lam, VectorX& xsi, VectorX& eta, VectorX& mu, Real& zet, VectorX& s);
			void combineToW(const VectorX& x, const VectorX& y, const Real& z, const VectorX& lam, const VectorX& xsi, const VectorX& eta, const VectorX& mu, const Real& zet, const VectorX& s, /* output */ VectorX& w);

			GCMMAReturnFlag calcStepLength(const VectorX& delw, Real& minValLeq);
			Real calcNormResidual(const VectorX& delw, Real stepLength, int normCh);

		private:
			int  _subDimW;
			VectorX _subw;
			VectorX _subx;
			VectorX _suby;
			VectorX _subs;
			VectorX _submu;
			VectorX _sublam;
			VectorX _subxsi;
			VectorX _subeta;

			Real _subz;
			Real _subzet;
			Real _subeps;

			VectorX tmpsubx;
			VectorX tmpsublam;
			VectorX tmpplam;
			VectorX tmpqlam;
			VectorX tmpdpsidx;
			VectorX tmpgival;

			MatrixX G;
			MatrixX Gt;
			VectorX xadi;
			VectorX bxdi;
			VectorX ydi;
			VectorX ldi;

			MatrixX Dx;
			MatrixX iDx;
			MatrixX Dly;
			MatrixX iDly;
			VectorX iDy;
			VectorX dxt;
			VectorX dyt;
			VectorX dlt;
			VectorX dlyt;
			Real dzt;

			MatrixX DlyGDxGt;
			MatrixX iDlyGDxGt;
			MatrixX DxGtiDlyG;
			MatrixX iDxGtiDlyG;
			MatrixX MatSizeineqNbyxN;
			MatrixX MatSizexNbyineqN;
			VectorX delx;
			VectorX dellam;
			VectorX resvec;
		};


		/*
		* GCMMA_TRM Class
		* Solve GCMMA subproblem by using 'Trust-Region Method'
		*/
		class GCMMA_TRM : public GCMMAOptimization
		{
		private:
			// parameters for trust-region
			Real _TR_v;
			Real _TR_w;
			Real _TR_gamma0;
			Real _TR_gamma1;
			Real _TR_gamma2;

			VectorX _sublam, _sublamm1, _sublamhat;
			VectorX _subdW, _subdWm1;
			Real _subW, _subWhat;
			Real _subm, _submhat;

			VectorX _subs; // s = lam - lamm1;
			VectorX _subt; // t = dw - dwm1;
			Real _subeta;
			Real _radius;
			Real _subtheta;

			VectorX _subx;
			VectorX _suby;

			bool _bUpdated;

		public:
			GCMMA_TRM(int xN, int ineqN) : GCMMAOptimization(xN, ineqN)
			{
				//setParametersTR(0.0001, 0.25, 0.25, 0.5, 2);
				setParametersTR(0.25, 0.5, 0.0, 0.25, 2);
			}
			GCMMAReturnFlag solveSubProblem(/* output */ VectorX& xout);
		private:
			void allocSUBvar(void);
			void setParametersTR(const Real& v, const Real& w, const Real& gam0, const Real& gam1, const Real& gam2);
			void initializeSubProb(void);
			void calcx(const VectorX& lam, /* output */ VectorX& subx);
			void calcy(const VectorX& lam, /* output */ VectorX& suby);
			void calcW(const VectorX& lam, /* output */ Real& W);
			void calcdW(const VectorX& lam, /* output */ VectorX& dW);
			void calcm(const VectorX& lam, /* output */ Real& m);
			void calceta(void);
			void calclamhat(void);
		};


		/*
		* GCMMA_GDM Class
		* Solve GCMMA subproblem by using 'Gradient-Descent Method'
		*/
		class GCMMA_GDM : public GCMMAOptimization
		{
		private:
			VectorX _subx;
			VectorX _suby;
			VectorX _sublam;

			Real _subW;
			VectorX _subdW;
		public:
			GCMMA_GDM(int xN, int ineqN) : GCMMAOptimization(xN, ineqN) {}
			~GCMMA_GDM() {}

		private:
			void allocSUBvar(void) {}
			void GD_initializeSubProb();
			void calcx(const VectorX& lam, /* output */ VectorX& subx);
			void calcy(const VectorX& lam, /* output */ VectorX& suby);
			void calcW(const VectorX& lam, /* output */ Real& W);
			void calcdW(const VectorX& lam, /* output */ VectorX& dW);
		public:
			GCMMAReturnFlag solveSubProblem(/* output */ VectorX& xout);
		};
	}
}