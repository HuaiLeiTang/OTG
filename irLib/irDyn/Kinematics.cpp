#include "SerialOpenChain.h"

using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		void SerialOpenChain::solveJointExponentialMapping(StatePtr& state, const unsigned int idx)
		{
			JointState& joint = state->getJointState(idx);

			if (joint.getExpUpToDate()) return;

			joint.setJointExp(SE3::Exp(_joints[idx].getScrew(), joint.getJointPos()));
			joint.setExpUpToDate(true);
		}

		void SerialOpenChain::solveForwardKinematics(StatePtr& state)
		{
			LOGIF(_complete, "[ERROR] solveForwardKinematics: Assembly is not complete");
			if (state->getKinematicsUpToDate(KINEMATICS::POSITION)) return;

			unsigned int dof = _joints.size();

			for (unsigned int i = 0; i < dof; i++)
			{
				solveJointExponentialMapping(state, i);
			}

			SE3 T(_baseT);
			state->getLinkState(0).setLinkSE3(T * _links[0].getM());
			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& link = state->getLinkState(i + 1);
				T *= state->getJointState(i).getJointExp();
				link.setLinkSE3(T * _links[i + 1].getM());
			}

			state->setKinematicsUpToDate(KINEMATICS::POSITION);
		}

		void SerialOpenChain::solveDiffForwardKinematics(StatePtr& state)
		{
			LOGIF(_complete, "[ERROR] solveDiffForwardKinematics: Assembly is not complete");
			if (state->getKinematicsUpToDate(KINEMATICS::VELOCITY)) return;

			unsigned int dof = _joints.size();

			for (unsigned int i = 0; i < dof; i++)
			{
				solveJointExponentialMapping(state, i);
			}

			se3 V(_baseV);
			state->getLinkState(0).setLinkVel(V);
			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& link = state->getLinkState(i + 1);
				JointState& joint = state->getJointState(i);
				V = SE3::InvAd(joint.getJointExp(), V) + _joints[i].getScrew()*joint.getJointVel();
				link.setLinkVel(V);
			}

			state->setKinematicsUpToDate(KINEMATICS::VELOCITY);
		}

		void SerialOpenChain::solve2ndDiffForwardKinematics(StatePtr& state)
		{
			LOGIF(_complete, "[ERROR] solve2ndDiffForwardKinematics: Assembly is not complete");
			if (state->getKinematicsUpToDate(KINEMATICS::ACCELERATION)) return;

			if (!state->getKinematicsUpToDate(KINEMATICS::VELOCITY)) solveDiffForwardKinematics(state);

			unsigned int dof = _joints.size();

			for (unsigned int i = 0; i < dof; i++)
			{
				solveJointExponentialMapping(state, i);
			}

			se3 Vdot(_baseVdot);
			state->getLinkState(0).setLinkAcc(Vdot);
			for (unsigned int i = 0; i < dof; i++)
			{
				LinkState& link = state->getLinkState(i + 1);
				JointState& joint = state->getJointState(i);
				Vdot = SE3::InvAd(joint.getJointExp(), Vdot) + SE3::ad(link.getLinkVel(), _joints[i].getScrew()*joint.getJointVel()) + _joints[i].getScrew()*joint.getJointAcc();
				link.setLinkAcc(Vdot);
			}

			state->setKinematicsUpToDate(KINEMATICS::ACCELERATION);
		}

		void SerialOpenChain::solveInverseKinematics(StatePtr& state, const SE3& goalT)
		{
			while (true)
			{
				solveForwardKinematics(state);
				se3 S = SE3::Ad(_links[getNumOfLink() - 1].getM(), SE3::Log(_ToolTip*goalT.inverse() * state->getLinkSE3(getNumOfLink() - 1)));
				if (S.norm() < InverseKinematicsExitCondition) break;
				state->setJointPos(state->getJointPos() - pInv(computeJacobian(state)) * S);
			}
		}

		Matrix6X SerialOpenChain::computeJacobian(StatePtr& state)
		{
			LOGIF(_complete, "[ERROR] computeJacobian: Assembly is not complete");

			unsigned int dof = _joints.size();

			if (!state->getKinematicsUpToDate(KINEMATICS::JACOBIAN))
			{
				SE3 T;
				for (unsigned int i = 0; i < dof; i++)
				{
					state->getJointState(dof - i - 1).setJointScrew(SE3::Ad(T.inverse(), _joints[dof - i - 1].getScrew()));
					solveJointExponentialMapping(state, dof - i - 1);
					T = state->getJointState(dof - i - 1).getJointExp() * T;
				}
				state->setKinematicsUpToDate(KINEMATICS::JACOBIAN);
			}

			Matrix6X jacobian(6, dof);
			for (unsigned int i = 0; i < dof; i++)
			{
				jacobian.col(i) = state->getJointScrew(i);
			}

			return jacobian;
		}

		Matrix6X SerialOpenChain::computeJacobianDot(StatePtr& state)
		{
			LOGIF(_complete, "[ERROR] computeJacobianDot: Assembly is not complete");
			if (!state->getKinematicsUpToDate(KINEMATICS::JACOBIAN)) computeJacobian(state);

			unsigned int dof = _joints.size();

			if (!state->getKinematicsUpToDate(KINEMATICS::JACOBIANDOT))
			{
				Matrix6 ad_sum = Matrix6::Zero();
				for (unsigned int i = 0; i < dof; i++)
				{
					ad_sum += SE3::ad(state->getJointScrew(dof - i - 1) * state->getJointVel(dof - i - 1));
					state->getJointState(dof - i - 1).setJointScrewDot(-ad_sum * state->getJointScrew(dof - i - 1));
				}
				state->setKinematicsUpToDate(KINEMATICS::JACOBIANDOT);
			}

			Matrix6X jacobianDot(6, dof);
			for (unsigned int i = 0; i < dof; i++)
			{
				jacobianDot.col(i) = state->getJointScrewDot(i);
			}

			return jacobianDot;
		}

		Matrix6X SerialOpenChain::computeDJacobian(StatePtr& state, VectorX dq)
		{
			LOGIF(_complete, "[ERROR] computeDJacobian: Assembly is not complete");
			if (!state->getKinematicsUpToDate(KINEMATICS::JACOBIAN)) computeJacobian(state);

			unsigned int dof = _joints.size();

			Matrix6X Djacobian(6, dof);
			Matrix6 ad_sum = Matrix6::Zero();
			for (unsigned int i = 0; i < dof; i++)
			{
				ad_sum += SE3::ad(state->getJointScrew(dof - i - 1) * dq(dof - i - 1));
				Djacobian.col(dof - i - 1) = -ad_sum * state->getJointScrew(dof - i - 1);
			}

			return Djacobian;
		}

		Matrix6X SerialOpenChain::computeDJacobianDot(StatePtr& state, VectorX dq, VectorX dqdot)
		{
			LOGIF(_complete, "[ERROR] computeDJacobianDot: Assembly is not complete");
			if (!state->getKinematicsUpToDate(KINEMATICS::JACOBIAN)) computeJacobian(state);
			if (!state->getKinematicsUpToDate(KINEMATICS::JACOBIANDOT)) computeJacobianDot(state);

			unsigned int dof = _joints.size();

			Matrix6X Djacobian = computeDJacobian(state, dq);
			Matrix6X DjacobianDot(6, dof);
			Matrix6 ad_sum = Matrix6::Zero();
			Matrix6 dad_sum = Matrix6::Zero();
			Matrix6 adad_sum = Matrix6::Zero();
			for (unsigned int i = 0; i < dof; i++)
			{
				ad_sum += SE3::ad(state->getJointScrew(dof - i - 1) * state->getJointVel(dof - i - 1));
				adad_sum += SE3::ad(state->getJointScrew(dof - i - 1) * dq(dof - i - 1));
				dad_sum += SE3::ad(state->getJointScrew(dof - i - 1) * dqdot(dof - i - 1) - adad_sum * state->getJointScrew(dof - i - 1) * state->getJointVel(dof - i - 1));

				DjacobianDot.col(dof - i - 1) = -dad_sum * state->getJointScrew(dof - i - 1) - ad_sum * Djacobian.col(dof - i - 1);
			}

			return DjacobianDot;
		}

		std::vector<irMath::VectorX> SerialOpenChain::solveInverKinematicsOnlyForEfort(const irMath::SE3& goalT)
		{
			MatrixX screw(6, 6);
			Vector3 pw;
			bool valid[2];
			valid[0] = false;
			valid[1] = false;

			for (int i = 0; i < 6; i++)
				screw.col(i) = _joints[i].getScrew();
			pw = _links[4]._M.getPosition();

			SE3 M = _links[6]._M;
			SE3 gd = goalT*M.inverse();

			Vector3 pw_now = gd.getRotation().matrix()*pw + gd.getPosition();
			Vector3 z1 = screw.col(0).head<3>();
			Vector3 p1 = z1.cross(screw.col(1).tail<3>());
			Vector3 pw_proj1 = pw - (z1.transpose()*(pw - p1))(0)*z1;
			Vector3 pw_now_proj1 = pw_now - (z1.transpose()*(pw_now - p1))(0)*z1;

			Vector4 q1;
			q1(0) = std::acos((pw_proj1.transpose()*pw_now_proj1)(0) / pw_proj1.norm() / pw_now_proj1.norm());
			if (((pw_proj1.cross(pw_now_proj1)).transpose()*z1)(0) < 0.0)
			{
				q1(0) = -q1(0);
			}
			q1(1) = q1(0);
			q1(2) = q1(0) + irMath::PI;
			q1(3) = q1(2);

			Vector3 z2 = screw.col(1).head<3>();
			Vector3 p2 = screw.col(1).head<3>().cross(screw.col(1).tail<3>());
			Vector3 p3 = screw.col(2).head<3>().cross(screw.col(2).tail<3>());

			Vector3 pw_proj2 = pw - (z2.transpose()*(pw - p2))(0)*z2;
			Vector3 p3_proj2 = p3 - (z2.transpose()*(p3 - p2))(0)*z2;
			Real L2 = (p3_proj2 - p2).norm();
			Vector3 vec3w = pw_proj2 - p3_proj2;
			Real L3 = vec3w.norm();
			Real alpha = atan2(vec3w(2), vec3w(0));

			Vector4 q2, q3;

			Vector2 theta3;
			Vector2 theta2;

			for (int i = 0; i < 2; i++)
			{
				SE3 T1 = SE3::Exp(screw.col(0).head<3>() * q1(2 * i), screw.col(0).tail<3>() * q1(2 * i));
				Vector3 z2_now = T1.getRotation().matrix() * screw.col(1).head<3>();
				Vector3 p2_now = T1.getRotation().matrix() * p2 + T1.getPosition();
				Vector3 pw_now_proj2_now = pw_now - (z2_now.transpose()*(pw_now - p2_now))(0)*z2_now;
				Vector3 vec2w = pw_now_proj2_now - p2_now;
				Real L = vec2w.norm();
				if (L > L2 + L3)
					continue;
				valid[i] = true;
				theta3(0) = std::acos(0.5*(L2*L2 + L3*L3 - L*L) / (L2*L3));
				theta3(1) = -theta3(0);
				Real phi = std::acos(0.5*(L2*L2 + L*L - L3*L3) / (L2*L));
				Vector3 vec_y = (vec2w.transpose()*screw.col(0).head<3>())(0) * screw.col(0).head<3>();
				Vector3 vec_x = vec2w - vec_y;
				Real theta = std::atan2(screw.col(0).head<3>().transpose()*(vec_y), vec_x.norm());
				theta2(0) = theta + phi;
				theta2(1) = theta - phi;

				if ((z1.transpose()*vec_x.cross(z2_now))(0) > 0.0)
				{
					theta3 = theta3 - alpha*Vector2::Ones();
					q2.segment(i * 2, 2) = 0.5*irMath::PI*Vector2::Ones() - theta2;
					q3.segment(i * 2, 2) = 0.5*irMath::PI*Vector2::Ones() - theta3;
				}
				else
				{
					theta3 = theta3 + alpha*Vector2::Ones();
					q2.segment(i * 2, 2) = theta2 - 0.5*irMath::PI*Vector2::Ones();
					q3.segment(i * 2, 2) = 0.5*irMath::PI*Vector2::Ones() + theta3;
				}
			}

			Eigen::Matrix<irMath::Real, 8, 1> q4, q5, q6;
			for (int i = 0; i < 4; i++)
			{
				SE3 g = (SE3::Exp(screw.col(0).head<3>()*q1(i), screw.col(0).tail<3>()*q1(i)) *
					SE3::Exp(screw.col(1).head<3>()*q2(i), screw.col(1).tail<3>()*q2(i)) *
					SE3::Exp(screw.col(2).head<3>()*q3(i), screw.col(2).tail<3>()*q3(i))).inverse() * gd;

				Vector3 z = g.getRotation().matrix()*screw.col(5).head<3>();
				Real theta5 = std::acos((screw.col(5).head<3>().transpose()*z)(0));
				Vector3 z_proj = z - (z.transpose()*screw.col(5).head<3>())(0) * screw.col(5).head<3>();
				Vector3 vec = screw.col(4).head<3>().cross(screw.col(3).head<3>());
				Real n_z_proj = z_proj.norm();
				Real theta4;
				if (n_z_proj < RealEps)
				{
					theta4 = 0;
				}
				else
				{
					theta4 = std::acos((vec.transpose() * z_proj)(0) / z_proj.norm());
					if ((screw.col(3).head<3>().transpose() * vec.cross(z_proj))(0) > 0.0)
					{
						theta4 = abs(theta4);
					}
					else
					{
						theta4 = -abs(theta4);
					}
				}

				SE3 g_ = (SE3::Exp(screw.col(3).head<3>()*theta4, screw.col(3).tail<3>()*theta4) *
					SE3::Exp(screw.col(4).head<3>()*theta5, screw.col(4).tail<3>()*theta5)).inverse() * g;
				Vector3 w6 = SE3::Log(g_).head<3>();
				Real theta6 = w6.norm();
				if ((screw.col(5).head<3>().transpose() * w6)(0) < 0.0)
				{
					theta6 = -theta6;
				}
				if (theta4 < 0.0)
				{
					q4.segment(2 * i, 2) << theta4, theta4 + irMath::PI;
				}
				else
				{
					q4.segment(2 * i, 2) << theta4, theta4 - irMath::PI;
				}
				q5.segment(2 * i, 2) << theta5, -theta5;
				if (theta6 < 0.0)
				{
					q6.segment(2 * i, 2) << theta6, theta6 + irMath::PI;
				}
				else
				{
					q6.segment(2 * i, 2) << theta6, theta6 - irMath::PI;
				}
			}
			Eigen::Matrix<irMath::Real, 8, 1> newq1, newq2, newq3;
			newq1(0) = newq1(1) = q1(0);
			newq1(2) = newq1(3) = q1(1);
			newq1(4) = newq1(5) = q1(2);
			newq1(6) = newq1(7) = q1(3);

			newq2(0) = newq2(1) = q2(0);
			newq2(2) = newq2(3) = q2(1);
			newq2(4) = newq2(5) = q2(2);
			newq2(6) = newq2(7) = q2(3);

			newq3(0) = newq3(1) = q3(0);
			newq3(2) = newq3(3) = q3(1);
			newq3(4) = newq3(5) = q3(2);
			newq3(6) = newq3(7) = q3(3);

			std::vector<VectorX> q;
			VectorX qelem(6);
			for (int i = 0; i < 8; i++)
			{
				if (!valid[0] && i >= 0 && i < 4) continue;
				if (!valid[1] && i >= 4 && i < 8) continue;

				qelem(0) = newq1(i);
				qelem(1) = newq2(i);
				qelem(2) = newq3(i);
				qelem(3) = q4(i);
				qelem(4) = q5(i);
				qelem(5) = q6(i);
				q.push_back(qelem);
			}

			return q;
		}
	}
}