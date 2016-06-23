#pragma once

#include <vector>

#include "Link.h"
#include "Joint.h"
#include "State.h"

namespace irLib
{
	namespace irDyn
	{
		class SerialOpenChain;

		class SerialOpenChainLink;
		class SerialOpenChainJoint;

		typedef std::shared_ptr<SerialOpenChain> SerialOpenChainPtr;

		class SerialOpenChain
		{
		public:
			SerialOpenChain() {}
			~SerialOpenChain() {}

			void addLink(const LinkPtr&);
			void addJoint(const JointPtr&, const irMath::SE3&, const irMath::SE3&);

			const LinkPtr& getLinkPtr(const unsigned int) const;
			const JointPtr& getJointPtr(const unsigned int) const;
			const irMath::SE3& getTransform_FromParentToJoint(const unsigned int) const;
			const irMath::SE3& getTransform_FromJointToChild(const unsigned int) const;

			const irMath::SE3& getBaseT() const { return _baseT; }
			const irMath::se3& getBaseV() const { return _baseV; }
			const irMath::se3& getBaseVdot() const { return _baseVdot; }
			const irMath::SE3& getToolTipSE3() const { return _ToolTip; }

			const unsigned int getNumOfLink() const { return _links.size(); }
			const unsigned int getNumOfJoint() const { return _joints.size(); }
			const irMath::Inertia& getLinkInertia(unsigned int) const;
			const irMath::se3& getJointScrew(unsigned int) const;

			void setLinkInertia(unsigned int, const irMath::Inertia&);

			void completeAssembling(const irMath::SE3& = (irMath::SE3()), const irMath::se3& = (irMath::se3::Zero()), const irMath::se3& = (irMath::se3::Zero()));
			void setBaseFrameSE3(const irMath::SE3& baseT) { _baseT = baseT; }
			void setBaseFrameVel(const irMath::se3& baseV) { _baseV = baseV; }
			void setBaseFrameAcc(const irMath::se3& baseVdot) { _baseVdot = baseVdot; }
			void setToolTipSE3(const irMath::SE3& ToolTip) { _ToolTip = ToolTip; }

			StatePtr makeState() const;

			bool isComplete() const { return _complete; }

		public:
			std::vector< SerialOpenChainLink, Eigen::aligned_allocator< SerialOpenChainLink > > _links;
			std::vector< SerialOpenChainJoint, Eigen::aligned_allocator< SerialOpenChainJoint > > _joints;

			irMath::SE3 _baseT;
			irMath::se3 _baseV;
			irMath::se3 _baseVdot;

			irMath::SE3 _ToolTip;

			bool _complete;

		public:
			/*!
			* \brief Serial Open Chain kinematics functions
			*/
			void solveJointExponentialMapping(StatePtr&, const unsigned int);

			void solveForwardKinematics(StatePtr&);
			void solveDiffForwardKinematics(StatePtr&);
			void solve2ndDiffForwardKinematics(StatePtr&);

			void solveInverseKinematics(StatePtr&, const irMath::SE3&);

			irMath::Matrix6X computeJacobian(StatePtr&);
			irMath::Matrix6X computeJacobianDot(StatePtr&);

			irMath::Matrix6X computeDJacobian(StatePtr&, irMath::VectorX dq); // dJ
			irMath::Matrix6X computeDJacobianDot(StatePtr&, irMath::VectorX dq, irMath::VectorX dqdot); // d\dot{J}
			std::vector<irMath::VectorX> solveInverKinematicsOnlyForEfort(const irMath::SE3& goalT);

		public:
			/*!
			* \brief Serial Open Chain dynamics functions
			*/
			void solveInverseDynamics(StatePtr&, const irMath::dse3& = (irMath::dse3::Zero()));

			irMath::MatrixX computeDiffInverseDynamics(StatePtr&, const irMath::MatrixX&, const irMath::MatrixX&, const irMath::MatrixX&, const irMath::dse3& = (irMath::dse3::Zero()));

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		class SerialOpenChainLink
		{
		public:
			SerialOpenChainLink(const LinkPtr& link) : _link(link) {}
			~SerialOpenChainLink() {}

			void setM(const irMath::SE3& M) { _M = M; }
			void setG(const irMath::Inertia& G) { _G = G; }

			const LinkPtr& getLinkPtr() const { return _link; }

			const irMath::SE3& getM() const { return _M; }
			const irMath::Inertia& getG() const { return _G; }

		public:
			LinkPtr _link;

			irMath::SE3 _M;
			irMath::Inertia _G;
		};

		class SerialOpenChainJoint
		{
		public:
			SerialOpenChainJoint(const JointPtr& joint, const irMath::SE3& FromParentToJoint, const irMath::SE3& FromJointToChild) : _joint(joint),
				_Tpj(FromParentToJoint), _Tjc(FromJointToChild) {}
			~SerialOpenChainJoint() {}

			void setScrew(const irMath::se3& screw) { _screw = screw; }

			const JointPtr& getJointPtr() const { return _joint; }

			const irMath::SE3& getTransform_FromParentToJoint() const { return _Tpj; }
			const irMath::SE3& getTransform_FromJointToChild() const { return _Tjc; }

			const irMath::se3& getScrew() const { return _screw; }

		private:
			JointPtr _joint;
			irMath::SE3 _Tpj, _Tjc;

			irMath::se3 _screw;
		};
	}
}