#include "SerialOpenChain.h"

using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		void SerialOpenChain::solveInverseDynamics(StatePtr& state, const irMath::dse3& endeffectorF)
		{
			if (state->getDynamicsUpToDate(DYNAMICS::INVERSE)) return;

			solveDiffForwardKinematics(state);
			solve2ndDiffForwardKinematics(state);

			unsigned int dof = _joints.size();

			irMath::dse3 Fee = SE3::Ad(_links[dof].getM().inverse()).transpose() * endeffectorF;
			irMath::dse3 F(Fee);
			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& link = state->getLinkState(dof - i);
				JointState& joint = state->getJointState(dof - i - 1);

				const Matrix6& G = static_cast<const Matrix6&>(_links[dof - i].getG());
				F += G * link.getLinkAcc() - SE3::adTranspose(link.getLinkVel(), G * link.getLinkVel());

				Real tau = _joints[dof - i - 1].getScrew().dot(F);
				tau += getJointPtr(dof - i - 1)->getSpringConstant() * joint.getJointPos() + getJointPtr(dof - i - 1)->getDamperConstant() * joint.getJointVel();
				if (RealBigger(joint.getJointVel(), 0.0))
					tau += getJointPtr(dof - i - 1)->getFrictionConstant();
				else if (RealLess(joint.getJointVel(), 0.0))
					tau -= getJointPtr(dof - i - 1)->getFrictionConstant();

				joint.setJointF(F);
				joint.setJointTorque(tau);

				F = SE3::InvAd(joint.getJointExp()).transpose() * F;
			}

			state->setDynamicsUpToDate(DYNAMICS::INVERSE);
		}

		irMath::MatrixX SerialOpenChain::computeDiffInverseDynamics(StatePtr& state, const irMath::MatrixX& dqdp, const irMath::MatrixX& dqdotdp, const irMath::MatrixX& dqddotdp, const irMath::dse3& endeffectorF)
		{
			solveInverseDynamics(state, endeffectorF);

			unsigned int dof = _joints.size(); ///< Robot degree of freedom
			unsigned int pN = dqdp.cols(); ///< number of parameters
			unsigned int linkN = getNumOfLink();

			// variables for forward iteration
			std::vector<MatrixX> dVdp(linkN), dVdotdp(linkN);

			// variables for backward iteration
			std::vector<MatrixX> dFdp(dof);
			MatrixX dtaudp(dof, pN);

			// Initialization
			dVdp[0] = MatrixX::Zero(6, pN);
			dVdotdp[0] = MatrixX::Zero(6, pN);
			dFdp[0] = MatrixX::Zero(6, pN);

			// Forward recursion
			MatrixX curdVdp = MatrixX::Zero(6, pN);
			MatrixX currdVdotdp = MatrixX::Zero(6, pN);

			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& plink = state->getLinkState(i + 1);
				LinkState& clink = state->getLinkState(i);
				JointState& joint = state->getJointState(i);

				const se3& screw = _joints[i].getScrew();

				// calculate current dVdp, dimension 6 * pN
				curdVdp = SE3::InvAd(joint.getJointExp()) * curdVdp +
					screw * dqdotdp.row(i) -
					SE3::ad(screw, plink.getLinkVel()) * dqdp.row(i);

				// calculate current dVdotdq, dimension 6 * pN
				currdVdotdp = SE3::InvAd(joint.getJointExp()) * currdVdotdp +
					screw * dqddotdp.row(i) -
					SE3::ad(screw, plink.getLinkVel()) * dqdotdp.row(i) -
					SE3::ad(screw) * curdVdp * joint.getJointVel() -
					(SE3::ad(screw) * SE3::InvAd(joint.getJointExp(), clink.getLinkAcc())) * dqdp.row(i);

				// save data
				dVdp[i + 1] = curdVdp;
				dVdotdp[i + 1] = currdVdotdp;
			}

			// Backward recursion
			MatrixX curdFdp = MatrixX::Zero(6, pN);

			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& link = state->getLinkState(dof - i);
				JointState& joint = state->getJointState(dof - i - 1);

				const se3& screw = _joints[dof - i - 1].getScrew();

				// calculate current dFdp, dimension 6 * pN
				const Matrix6& G = static_cast<const Matrix6&>(_links[dof - i].getG());
				curdFdp += G * dVdotdp[dof - i] -
					SE3::adTranspose(link.getLinkVel()) * G * dVdp[dof - i];
				se3 tmp = G * link.getLinkVel();
				for (unsigned int k = 0; k < pN; k++)
				{
					curdFdp.col(k) -= SE3::adTranspose(dVdp[dof - i].col(k), tmp);
				}

				// calculate current dtaudp, dimension 1 * pN
				dtaudp.row(dof - i - 1) = screw.transpose() * curdFdp + getJointPtr(dof - i - 1)->getSpringConstant()*dqdp.row(dof - i - 1) + getJointPtr(dof - i - 1)->getDamperConstant()*dqdotdp.row(dof - i - 1);

				curdFdp = SE3::InvAd(joint.getJointExp()).transpose() * (SE3::adTranspose(-screw, joint.getJointF()) * dqdp.row(dof - i - 1) + curdFdp);
			}

			return dtaudp;
		}
	}
}