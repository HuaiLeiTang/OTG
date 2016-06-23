#pragma once

#include <vector>
#include <memory>

#include <irMath\Constant.h>
#include <irMath\LieGroup.h>

namespace irLib
{
	namespace irDyn
	{
		class SerialOpenChain;

		class State;
		class LinkState;
		class JointState;

		typedef std::shared_ptr<State> StatePtr;

		const enum KINEMATICS
		{
			POSITION = 1 << 0,
			VELOCITY = 1 << 1,
			ACCELERATION = 1 << 2,
			JACOBIAN = 1 << 3,
			JACOBIANDOT = 1 << 2,

			KIN_JOINTPOS_CHANGED = POSITION | VELOCITY | ACCELERATION | JACOBIAN | JACOBIANDOT,
			KIN_JOINTVEL_CHANGED = VELOCITY | ACCELERATION | JACOBIANDOT,
			KIN_JOINTACC_CHANGED = ACCELERATION
		};

		const enum DYNAMICS
		{
			FORWARD = 1 << 0,
			INVERSE = 1 << 1,

			DYN_JOINTPOS_CHANGED = FORWARD | INVERSE,
			DYN_JOINTVEL_CHANGED = FORWARD | INVERSE,
			DYN_JOINTACC_CHANGED = FORWARD | INVERSE,
			DYN_JOINTTORQUE_CAHGNED = FORWARD | INVERSE
		};

		class State
		{
			friend class State;
			friend class SerialOpenChain;

		public:
			State(const unsigned int);
			~State() {}

			LinkState& getLinkState(const unsigned int idx) { return _linkState[idx]; }
			JointState& getJointState(const unsigned int idx) { return _jointState[idx]; }

			const LinkState& getLinkState(const unsigned int idx) const { return _linkState[idx]; }
			const JointState& getJointState(const unsigned int idx) const { return _jointState[idx]; }

			void setJointPos(const irMath::VectorX&);
			void setJointVel(const irMath::VectorX&);
			void setJointAcc(const irMath::VectorX&);
			void setJointTorque(const irMath::VectorX&);

			void setJointPos(const unsigned int, const irMath::Real);
			void setJointVel(const unsigned int, const irMath::Real);
			void setJointAcc(const unsigned int, const irMath::Real);
			void setJointTorque(const unsigned int, const irMath::Real);

			const irMath::VectorX getJointPos() const;
			const irMath::VectorX getJointVel() const;
			const irMath::VectorX getJointAcc() const;
			const irMath::VectorX getJointTorque() const;

			const irMath::Real getJointPos(const unsigned int idx) const;
			const irMath::Real getJointVel(const unsigned int idx) const;
			const irMath::Real getJointAcc(const unsigned int idx) const;
			const irMath::Real getJointTorque(const unsigned int idx) const;


			const irMath::se3& getJointScrew(const unsigned int idx) const;
			const irMath::se3& getJointScrewDot(const unsigned int idx) const;

			const irMath::SE3& getLinkSE3(const unsigned int idx) const;
			const irMath::se3& getLinkVel(const unsigned int idx) const;
			const irMath::se3& getLinkAcc(const unsigned int idx) const;

			bool getKinematicsUpToDate(const unsigned int kinematics) { return (_kinematicsUpToDate & kinematics) == kinematics; }
			bool getDynamicsUpToDate(const unsigned int dynamics) { return (_dynamicsUpToDate & dynamics) == dynamics; }

		private:
			unsigned int _dof;

			std::vector< LinkState, Eigen::aligned_allocator< LinkState > > _linkState;
			std::vector< JointState, Eigen::aligned_allocator< JointState > > _jointState;

			unsigned int _kinematicsUpToDate;
			unsigned int _dynamicsUpToDate;

		public:
			void setKinematicsUpToDate(const unsigned int, bool = (true));
			void setDynamicsUpToDate(const unsigned int, bool = (true));
		};

		class LinkState
		{
			friend class State;
			friend class SerialOpenChain;

		public:
			LinkState() : _T(irMath::SE3()), _V(irMath::se3::Zero()), _Vdot(irMath::se3::Zero()) {}
			~LinkState() {}

			const irMath::SE3& getLinkSE3() const { return _T; }
			const irMath::se3& getLinkVel() const { return _V; }
			const irMath::se3& getLinkAcc() const { return _Vdot; }

		private:
			irMath::SE3 _T;
			irMath::se3 _V;
			irMath::se3 _Vdot;

		private:
			void setLinkSE3(const irMath::SE3& T) { _T = T; }
			void setLinkVel(const irMath::se3& V) { _V = V; }
			void setLinkAcc(const irMath::se3& Vdot) { _Vdot = Vdot; }
		};

		class JointState
		{
			friend class SerialOpenChain;

		public:
			JointState() : _expUpToDate(false), _q(0.0), _qdot(0.0), _qddot(0.0) {}
			~JointState() {}

			void setJointPos(const irMath::Real q) { _q = q; setExpUpToDate(false); }
			void setJointVel(const irMath::Real qdot) { _qdot = qdot; }
			void setJointAcc(const irMath::Real qddot) { _qddot = qddot; }
			void setJointTorque(const irMath::Real tau) { _tau = tau; }
			void setJointScrew(const irMath::se3& screw) { _screw = screw; }
			void setJointScrewDot(const irMath::se3& screwDot) { _screwDot = screwDot; }

			const irMath::Real getJointPos() const { return _q; }
			const irMath::Real getJointVel() const { return _qdot; }
			const irMath::Real getJointAcc() const { return _qddot; }
			const irMath::Real getJointTorque() const { return _tau; }
			const irMath::dse3& getJointF() const { return _F; }
			const irMath::SE3& getJointExp() const { return _exp; }
			const irMath::se3& getJointScrew() const { return _screw; }
			const irMath::se3& getJointScrewDot() const { return _screwDot; }

			bool getExpUpToDate() { return _expUpToDate; }

		private:
			irMath::Real _q;
			irMath::Real _qdot;
			irMath::Real _qddot;

			irMath::Real _tau;
			irMath::dse3 _F;

			irMath::se3 _screw;
			irMath::se3 _screwDot;

			irMath::SE3 _exp;
			bool _expUpToDate;

		private:
			void setJointF(const irMath::dse3& F) { _F = F; }

			void setJointExp(const irMath::SE3& exp) { _exp = exp; }
			void setExpUpToDate(bool sw = (true)) { _expUpToDate = sw; }
		};
	}
}