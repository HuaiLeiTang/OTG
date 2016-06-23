#include "State.h"

#include <irUtils\Diagnostic.h>

using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		State::State(const unsigned int dof) : _dof(dof)
		{
			for (unsigned int i = 0; i < _dof + 1; i++)
			{
				_linkState.push_back(LinkState());
			}
			for (unsigned int i = 0; i < _dof; i++)
			{
				_jointState.push_back(JointState());
			}
		}

		void State::setJointPos(const VectorX& q)
		{
			LOGIF(q.size() == _dof, "[ERROR] setJointPos: size of input must be equal to dof.");
			for (unsigned int i = 0; i < _dof; i++)
			{
				_jointState[i].setJointPos(q(i));
			}
			setKinematicsUpToDate(KINEMATICS::KIN_JOINTPOS_CHANGED, false);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTPOS_CHANGED, false);
		}

		void State::setJointVel(const VectorX& qdot)
		{
			LOGIF(qdot.size() == _dof, "[ERROR] setJointVel: size of input must be equal to dof.");
			for (unsigned int i = 0; i < _dof; i++)
			{
				_jointState[i].setJointVel(qdot(i));
			}
			setKinematicsUpToDate(KINEMATICS::KIN_JOINTVEL_CHANGED, false);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTVEL_CHANGED, false);
		}

		void State::setJointAcc(const VectorX& qddot)
		{
			LOGIF(qddot.size() == _dof, "[ERROR] setJointAcc: size of input must be equal to dof.");
			for (unsigned int i = 0; i < _dof; i++)
			{
				_jointState[i].setJointAcc(qddot(i));
			}
			setKinematicsUpToDate(KINEMATICS::KIN_JOINTACC_CHANGED, false);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTACC_CHANGED, false);
		}

		void State::setJointTorque(const VectorX& tau)
		{
			LOGIF(tau.size() == _dof, "[ERROR] setJointTorque: size of input must be equal to dof.");
			for (unsigned int i = 0; i < _dof; i++)
			{
				_jointState[i].setJointTorque(tau(i));
			}
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTTORQUE_CAHGNED, false);
		}

		void State::setJointPos(const unsigned int idx, const irMath::Real q)
		{
			_jointState[idx].setJointPos(q);
			setKinematicsUpToDate(KINEMATICS::KIN_JOINTPOS_CHANGED, false);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTPOS_CHANGED, false);
		}

		void State::setJointVel(const unsigned int idx, const irMath::Real qdot)
		{
			_jointState[idx].setJointVel(qdot);
			setKinematicsUpToDate(KINEMATICS::KIN_JOINTVEL_CHANGED, false);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTVEL_CHANGED, false);
		}

		void State::setJointAcc(const unsigned int idx, const irMath::Real qddot)
		{
			_jointState[idx].setJointAcc(qddot);
			setKinematicsUpToDate(KINEMATICS::KIN_JOINTACC_CHANGED, false);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTACC_CHANGED, false);
		}

		void State::setJointTorque(const unsigned int idx, const irMath::Real tau)
		{
			_jointState[idx].setJointTorque(tau);
			setDynamicsUpToDate(DYNAMICS::DYN_JOINTTORQUE_CAHGNED, false);
		}

		const VectorX State::getJointPos() const
		{
			VectorX q(_dof);
			for (unsigned int i = 0; i < _dof; i++)
			{
				q(i) = _jointState[i].getJointPos();
			}
			return q;
		}

		const VectorX State::getJointVel() const
		{
			VectorX qdot(_dof);
			for (unsigned int i = 0; i < _dof; i++)
			{
				qdot(i) = _jointState[i].getJointVel();
			}
			return qdot;
		}

		const VectorX State::getJointAcc() const
		{
			VectorX qddot(_dof);
			for (unsigned int i = 0; i < _dof; i++)
			{
				qddot(i) = _jointState[i].getJointAcc();
			}
			return qddot;
		}

		const VectorX State::getJointTorque() const
		{
			VectorX tau(_dof);
			for (unsigned int i = 0; i < _dof; i++)
			{
				tau(i) = _jointState[i].getJointTorque();
			}
			return tau;
		}

		const irMath::Real State::getJointPos(const unsigned int idx) const
		{
			return _jointState[idx].getJointPos();
		}

		const irMath::Real State::getJointVel(const unsigned int idx) const
		{
			return _jointState[idx].getJointVel();
		}

		const irMath::Real State::getJointAcc(const unsigned int idx) const
		{
			return _jointState[idx].getJointAcc();
		}

		const irMath::Real State::getJointTorque(const unsigned int idx) const
		{
			return _jointState[idx].getJointTorque();
		}

		const irMath::se3& State::getJointScrew(const unsigned int idx) const
		{
			return _jointState[idx].getJointScrew();
		}

		const irMath::se3& State::getJointScrewDot(const unsigned int idx) const
		{
			return _jointState[idx].getJointScrewDot();
		}

		const irMath::SE3& State::getLinkSE3(const unsigned int idx) const
		{
			return _linkState[idx].getLinkSE3();
		}

		const irMath::se3& State::getLinkVel(const unsigned int idx) const
		{
			return _linkState[idx].getLinkVel();
		}

		const irMath::se3& State::getLinkAcc(const unsigned int idx) const
		{
			return _linkState[idx].getLinkAcc();
		}

		void State::setKinematicsUpToDate(const unsigned int kinematics, bool sw)
		{
			if (sw)
			{
				_kinematicsUpToDate |= kinematics;
			}
			else _kinematicsUpToDate &= ~kinematics;
		}

		void State::setDynamicsUpToDate(const unsigned int dynamics, bool sw)
		{
			if (sw)
			{
				_dynamicsUpToDate |= dynamics;
			}
			else _dynamicsUpToDate &= ~dynamics;
		}
	}
}