#pragma once
#include "./include/ReflexxesAPI.h"
#include "./include/RMLPositionFlags.h"
#include "./include/RMLPositionInputParameters.h"
#include "./include/RMLPositionOutputParameters.h"

#include <vector>

#include <Eigen/Dense>
#include <irMath\Constant.h>
//#include <rovin\Math\Constant.h>

using namespace irLib::irMath;
//using namespace rovin::Math;

namespace Reflexxes
{
	class ReflexxesWrapper
	{
	public:
		ReflexxesWrapper(unsigned int DOF, double timeStep)
			:_DOF(DOF), _RML(DOF, timeStep), _IP(DOF), _OP(DOF)
		{
			_Flag.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;

			//_q_waypoint.setZero(DOF, numWaypoints);
			//_qdot_waypoint.setZero(DOF, numWaypoints);

			_dqLim = VectorX::Ones(DOF) * RealMax;
			_ddqLim = VectorX::Ones(DOF) * RealMax;
			_dddqLim = VectorX::Ones(DOF) * RealMax;

			_qResult.resize(_DOF, 1);
			_dqResult.resize(_DOF, 1);
			_ddqResult.resize(_DOF, 1);
		}

		const MatrixX&	solve()
		{
			//	Initialize
			for (int i = 0; i < _DOF; i++)
			{
				_IP.CurrentPositionVector->VecData[i] = _q_waypoint(i,0);
				_IP.CurrentVelocityVector->VecData[i] = _qdot_waypoint(i, 0);
				_IP.CurrentAccelerationVector->VecData[i] = 0.0;

				_IP.MaxVelocityVector->VecData[i] = _dqLim[i];
				_IP.MaxAccelerationVector->VecData[i] = _ddqLim[i];
				_IP.MaxJerkVector->VecData[i] = _dddqLim[i];

				_IP.SelectionVector->VecData[i] = true;
			}

			int	ResultValue = 0;
			int count = 0;
			//	put initial state.
			for (int i = 0; i < _DOF; i++)
			{
				_qResult(i, count) = _IP.CurrentPositionVector->VecData[i];
				_dqResult(i, count) = _IP.CurrentVelocityVector->VecData[i];
				_ddqResult(i, count) = _IP.CurrentAccelerationVector->VecData[i];
			}
			count++;
			for (int destPointIdx = 1; destPointIdx < _q_waypoint.cols(); destPointIdx++)
			{
				for (int i = 0; i < _DOF; i++)
				{
					_IP.TargetPositionVector->VecData[i] = _q_waypoint(i,destPointIdx);
					_IP.TargetVelocityVector->VecData[i] = _qdot_waypoint(i, destPointIdx);
				}
				ResultValue = ReflexxesAPI::RML_WORKING;
				while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
				{
					ResultValue = _RML.RMLPosition(_IP, &_OP, _Flag);
					if (ResultValue < 0)
					{
						printf("An error occurred (%d).\n", ResultValue);
						break;
					}

					for (unsigned int ij = 0; ij < _IP.CurrentPositionVector->GetVecDim(); ij++)
						_IP.CurrentPositionVector->VecData[ij] = _OP.NewPositionVector->VecData[ij];
					for (unsigned int ij = 0; ij < _IP.CurrentVelocityVector->GetVecDim(); ij++)
						_IP.CurrentVelocityVector->VecData[ij] = _OP.NewVelocityVector->VecData[ij];
					for (unsigned int ij = 0; ij < _IP.CurrentAccelerationVector->GetVecDim(); ij++)
						_IP.CurrentAccelerationVector->VecData[ij] = _OP.NewAccelerationVector->VecData[ij];

					//_IP.CurrentPositionVector = _OP.NewPositionVector;
					//_IP.CurrentVelocityVector = _OP.NewVelocityVector;
					//_IP.CurrentAccelerationVector = _OP.NewAccelerationVector;

					_qResult.conservativeResize(Eigen::NoChange, count + 1);
					_dqResult.conservativeResize(Eigen::NoChange, count + 1);
					_ddqResult.conservativeResize(Eigen::NoChange, count + 1);

					for (int i = 0; i < _DOF; i++)
					{
						_qResult(i, count) = _IP.CurrentPositionVector->VecData[i];
						_dqResult(i, count) = _IP.CurrentVelocityVector->VecData[i];
						_ddqResult(i, count) = _IP.CurrentAccelerationVector->VecData[i];
					}

					count++;
				}

			}
			return _qResult;
		}

		const MatrixX&	getResultPos() const { return _qResult; }
		const MatrixX&	getResultVel() const { return _dqResult; }
		const MatrixX&	getResultAcc() const { return _ddqResult; }
		

		void			setWayPointsPos(const MatrixX&	pos)
		{
			_q_waypoint = pos;
			if (_qdot_waypoint.size() != _q_waypoint.size())
			{
				_qdot_waypoint.setZero(pos.rows(), pos.cols());
			}
		}

		void			setWayPointsVel(const MatrixX&	vel)
		{
			_qdot_waypoint = vel;
		}

		//	Point which generated motion will pass (include initial and final position)
		MatrixX							_q_waypoint, _qdot_waypoint;

		//VectorX							_q0, _dq0, _ddq0;
		//VectorX							_qf, _dqf;
		VectorX							_dqLim, _ddqLim, _dddqLim;
		
	protected:
		int								_DOF;
		ReflexxesAPI					_RML;
		RMLPositionInputParameters		_IP;
		RMLPositionOutputParameters		_OP;
		RMLPositionFlags				_Flag;

		MatrixX							_qResult;
		MatrixX							_dqResult;
		MatrixX							_ddqResult;
	};

}