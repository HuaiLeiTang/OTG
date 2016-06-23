#include "SerialOpenChain.h"

using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		void SerialOpenChain::addLink(const LinkPtr& link)
		{
			_links.push_back(SerialOpenChainLink(link));
		}

		void SerialOpenChain::addJoint(const JointPtr& joint, const SE3& FromParentToJoint, const SE3& FromJointToChild)
		{
			_joints.push_back(SerialOpenChainJoint(joint, FromParentToJoint, FromJointToChild));
		}

		const LinkPtr& SerialOpenChain::getLinkPtr(const unsigned int idx) const
		{
			LOGIF(idx < _links.size(), "[ERROR] Link index is larger than number of Links.");
			return _links[idx].getLinkPtr();
		}

		const JointPtr& SerialOpenChain::getJointPtr(const unsigned int idx) const
		{
			LOGIF(idx < _joints.size(), "[ERROR] Joint index is larger than number of Joints.");
			return _joints[idx].getJointPtr();
		}

		const irMath::SE3& SerialOpenChain::getTransform_FromParentToJoint(const unsigned int idx) const
		{
			LOGIF(idx < _joints.size(), "[ERROR] Joint index is larger than number of Joints.");
			return _joints[idx].getTransform_FromParentToJoint();
		}

		const irMath::SE3& SerialOpenChain::getTransform_FromJointToChild(const unsigned int idx) const
		{
			LOGIF(idx < _joints.size(), "[ERROR] Joint index is larger than number of Joints.");
			return _joints[idx].getTransform_FromJointToChild();
		}

		const irMath::Inertia& SerialOpenChain::getLinkInertia(unsigned int idx) const
		{
			LOGIF(idx < _links.size(), "[ERROR] Link index is larger than number of Links.");
			return _links[idx].getG();
		}

		void SerialOpenChain::setLinkInertia(unsigned int idx, const irMath::Inertia& inertia)
		{
			LOGIF(idx < _links.size(), "[ERROR] Link index is larger than number of Links.");
			_links[idx].setG(inertia);
		}

		const irMath::se3& SerialOpenChain::getJointScrew(unsigned int idx) const
		{
			LOGIF(idx < _joints.size(), "[ERROR] Joint index is larger than number of Joints.");
			return _joints[idx].getScrew();
		}

		void SerialOpenChain::completeAssembling(const irMath::SE3& baseT, const irMath::se3& baseV, const irMath::se3& baseVdot)
		{
			LOGIF(_links.size() == _joints.size() + 1, "[ERROR] The number of links must be equal to the number of joints plus one.");

			_baseT = baseT;
			_baseV = baseV;
			_baseVdot = baseVdot;

			unsigned int dof = _joints.size();

			SE3 T;
			_links[0].setM(T);
			_links[0].setG(Inertia::changeFrame(T, _links[0].getLinkPtr()->getInertia()));
			for (unsigned int i = 0; i < dof; i++)
			{
				T *= _joints[i].getTransform_FromParentToJoint();
				_joints[i].setScrew(SE3::Ad(T, _joints[i].getJointPtr()->getAxis()));
				T *= _joints[i].getTransform_FromJointToChild();
				_links[i + 1].setM(T);
				_links[i + 1].setG(Inertia::changeFrame(T, _links[i + 1].getLinkPtr()->getInertia()));
			}

			_complete = true;
		}

		StatePtr SerialOpenChain::makeState() const
		{
			return StatePtr(new State(getNumOfJoint()));
		}
	}
}