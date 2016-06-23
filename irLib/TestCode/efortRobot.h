#pragma once

#include <vector>
#include <irDyn/SerialOpenChain.h>

using namespace irLib::irDyn;
using namespace irLib::irMath;

class efortRobot : public irLib::irDyn::SerialOpenChain
{
public:
	efortRobot() : SerialOpenChain()
	{
		unsigned int DOF = 6;

		irLib::irMath::VectorX linkLength(6);
		linkLength << 0.504, 0.170, 0.780, 0.140, 0.760, 0.125;

		for (unsigned int i = 0; i < DOF + 1; i++)
		{
			addLink(irLib::irDyn::LinkPtr(new irLib::irDyn::Link()));
		}

		addJoint(irLib::irDyn::JointPtr(new irLib::irDyn::Joint()), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, linkLength(0))), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
		addJoint(irLib::irDyn::JointPtr(new irLib::irDyn::Joint()), irLib::irMath::SE3(irLib::irMath::SO3::RotX(-irLib::irMath::PI_HALF)*irLib::irMath::SO3::RotZ(-irLib::irMath::PI_HALF), irLib::irMath::Vector3(linkLength(1), 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
		addJoint(irLib::irDyn::JointPtr(new irLib::irDyn::Joint()), irLib::irMath::SE3(irLib::irMath::Vector3(linkLength(2), 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
		addJoint(irLib::irDyn::JointPtr(new irLib::irDyn::Joint()), irLib::irMath::SE3(irLib::irMath::SO3::RotX(-irLib::irMath::PI_HALF), irLib::irMath::Vector3(linkLength(3), linkLength(4), 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
		addJoint(irLib::irDyn::JointPtr(new irLib::irDyn::Joint()), irLib::irMath::SE3(irLib::irMath::SO3::RotX(irLib::irMath::PI_HALF), irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));
		addJoint(irLib::irDyn::JointPtr(new irLib::irDyn::Joint()), irLib::irMath::SE3(irLib::irMath::SO3::RotX(-irLib::irMath::PI_HALF), irLib::irMath::Vector3(0, 0, 0)), irLib::irMath::SE3(irLib::irMath::Vector3(0, 0, 0)));


		irLib::irMath::MatrixX parameter(10, 6);
		parameter << 1.0000, -0.2726, 0.4164, 0.0379, 0.0379, 0.0379
			, 1.0000, 72.0862, 6.3893, 0.3574, -0.1061, -0.0759
			, 1.0000, -0.0000, 23.4120, 0.0124, 0.1181, 0.0256
			, 1.0000, 1.0000, -0.4166, -1.0914, -0.0124, 0.1181
			, 1.0000, 14.9404, 0.1093, 1.6195, 0.8195, 0.8034
			, 1.0000, -14.8427, -0.0116, 0.1966, 0.4121, 0.8312
			, 0.0478, 9.4016, 1.7068, -0.3880, 0.8152, 0.5500
			, 1.0000, -1.2929, 5.4276, -0.1850, 0.1343, -0.0108
			, 1.0000, 0.0320, 0.6453, 0.1541, -0.1275, 0.0534
			, 1.0000, -4.4974, -0.1622, -0.1995, -0.0834, 0.0317;

		irLib::irMath::VectorX m = parameter.row(0);
		irLib::irMath::MatrixX mx = parameter.block(1, 0, 3, 6);
		irLib::irMath::MatrixX inr_info = parameter.block(4, 0, 6, 6).transpose();
		irLib::irMath::Matrix3 I;
		std::vector<irLib::irMath::Inertia, Eigen::aligned_allocator<irLib::irMath::Inertia>> G(6);
		irLib::irMath::Vector3 p;
		for (int i = 0; i < 6; i++)
		{
			I(0, 0) = inr_info(i, 0);
			I(1, 1) = inr_info(i, 1);
			I(2, 2) = inr_info(i, 2);
			I(0, 1) = I(1, 0) = inr_info(i, 3);
			I(0, 2) = I(2, 0) = inr_info(i, 4);
			I(1, 2) = I(2, 1) = inr_info(i, 5);
			p = -mx.col(i);
			G[i] = irLib::irMath::Inertia(I, p, m(i));
		}

		for (unsigned int i = 0; i < DOF; i++)
		{
			getLinkPtr(i + 1)->setInertia(G[i]);
		}

		irLib::irMath::VectorX L(6);
		irLib::irMath::VectorX R(6);
		irLib::irMath::VectorX kt(6);
		irLib::irMath::VectorX kb(6);
		irLib::irMath::VectorX J(6);
		irLib::irMath::VectorX gearRatio(6);
		L << 3.5, 3.5, 5.2, 8, 8, 8;
		L *= 1e-3;

		R << 0.58, 0.58, 0.8, 2.9, 2.9, 7.5;
		kt << 0.73, 0.73, 0.5, 0.4, 0.4, 0.39;
		kb = kt;
		J << 1.06, 1.06, 0.13, 0.044, 0.044, 0.027;
		J *= 1e-3;
		gearRatio << 147, 153, 153, 76.95, 80, 51;
		for (int i = 0; i < 6; i++)
		{
			getJointPtr(i)->setInductance(L(i));
			getJointPtr(i)->setResistance(R(i));
			getJointPtr(i)->setMotorConstant(kt(i));
			getJointPtr(i)->setBackEMFConstant(kb(i));
			getJointPtr(i)->setRotorInertia(J(i));
			getJointPtr(i)->setGearRatio(gearRatio(i));
		}

		// set constraints

		irLib::irMath::VectorX taumax(6);
		irLib::irMath::VectorX taumin(6);
		irLib::irMath::VectorX qmax(6);
		irLib::irMath::VectorX qmin(6);
		irLib::irMath::VectorX qdotmax(6);
		irLib::irMath::VectorX qdotmin(6);
		irLib::irMath::VectorX qddotmax(6);
		irLib::irMath::VectorX qddotmin(6);
		irLib::irMath::VectorX qdddotmax(6);
		irLib::irMath::VectorX qdddotmin(6);
		irLib::irMath::VectorX kv(6);
		irLib::irMath::VectorX kc(6);
		irLib::irMath::VectorX smax(6);
		irLib::irMath::VectorX imax(6);

		qmax << 175, 90, 70, 180, 135, 360;
		qmax *= irLib::irMath::PI / 180;

		qmin << 175, 100, 145, 180, 135, 360;
		qmin *= -irLib::irMath::PI / 180;

		qdotmax << 100, 80, 140, 290, 290, 440;
		qdotmax *= irLib::irMath::PI / 180;

		qddotmax = 5 * qdotmax;             // user defined
		qddotmin = -qddotmax;
		qdddotmax = 300 * irLib::irMath::VectorX::Ones(6);          // user defined
		qdddotmin = -qdddotmax;
		taumax = 3000 * irLib::irMath::VectorX::Ones(6);          // user defined

		kv << 105.4116, 107.5105, 8.1031, 6.5430, 11.3551, 4.2050;
		kc << 92.3012, 117.3703, 57.8389, 10.0524, 24.6819, 20.32;
		smax << 3000, 3000, 4500, 4500, 4500, 4500;
		smax *= 2 * irLib::irMath::PI / 60;    // smax = [5000, 5000, 5000, 5000, 5000, 50000] * 2 * pi / 60,    % motor maximum
		imax << 39, 39, 20, 12, 12, 7.2;

		irLib::irMath::VectorX temptau = (imax.cwiseProduct(gearRatio)).cwiseProduct(kt);
		irLib::irMath::VectorX tempvel = smax.cwiseQuotient(gearRatio);
		taumax = taumax.cwiseMin(temptau);
		qdotmax = qdotmax.cwiseMin(tempvel);
		qdotmin = -qdotmax;
		taumin = -taumax;
		for (unsigned int i = 0; i < 6; i++)
		{
			getJointPtr(i)->setDamperConstant(kv(i));
			getJointPtr(i)->setFrictionConstant(kc(i));
			getJointPtr(i)->setLimitPos(qmin(i), qmax(i));
			getJointPtr(i)->setLimitVel(qdotmin(i), qdotmax(i));
			getJointPtr(i)->setLimitAcc(qddotmin(i), qddotmax(i));
			getJointPtr(i)->setLimitJerk(qdddotmin(i), qdddotmax(i));
			getJointPtr(i)->setLimitTorque(taumin(i), taumax(i));

			//std::cout << i << ": " << qmin(i) << " " << qmax(i) << " " << qdotmin(i) << " " << qdotmax(i) << " " << qddotmin(i) << " " << qddotmax(i) << " " << taumin(i) << " " << taumax(i) << std::endl;
		}

		irLib::irMath::se3 Vdot = irLib::irMath::se3::Zero();
		Vdot(5) = 9.8;
		completeAssembling(irLib::irMath::SE3(), irLib::irMath::se3::Zero(), Vdot);
		setToolTipSE3(irLib::irMath::SE3(irLib::irMath::SO3(), irLib::irMath::Vector3(0, 0, 0.125)));

		irLib::irDyn::StatePtr efortState = makeState();
		irLib::irMath::VectorX q(6);
		q.setZero();
		efortState->setJointPos(q);
		solveForwardKinematics(efortState);
		irLib::irMath::Vector4	orange(254 / 255.0, 193 / 255.0, 27 / 255.0, 1.0), black(55 / 255.0, 55 / 255.0, 55 / 255.0, 1.0);
		for (unsigned int i = 0; i < 1; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK0_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(0).getLinkSE3().inverse());
			STL_file->setDimension(1);
			STL_file->setColor(orange);
			getLinkPtr(0)->addDrawingGeomtryInfo(STL_file);
		}
		for (unsigned int i = 0; i < 6; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK1_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(1).getLinkSE3().inverse());
			STL_file->setDimension(1);
			if (i == 1 || i == 2 || i == 3 || i == 4)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			getLinkPtr(1)->addDrawingGeomtryInfo(STL_file);
		}
		for (unsigned int i = 0; i < 1; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK2_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(2).getLinkSE3().inverse());
			STL_file->setDimension(1);
			STL_file->setColor(orange);
			getLinkPtr(2)->addDrawingGeomtryInfo(STL_file);
		}
		for (unsigned int i = 0; i < 7; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK3_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(3).getLinkSE3().inverse());
			STL_file->setDimension(1);
			if (i == 1 || i == 2 || i == 3 || i == 4 || i == 5)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			getLinkPtr(3)->addDrawingGeomtryInfo(STL_file);
		}
		for (unsigned int i = 0; i < 8; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK4_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(4).getLinkSE3().inverse());
			STL_file->setDimension(1);
			if (i == 0)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			getLinkPtr(4)->addDrawingGeomtryInfo(STL_file);
		}
		for (unsigned int i = 0; i < 3; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK5_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(5).getLinkSE3().inverse());
			STL_file->setDimension(1);
			STL_file->setColor(orange);
			getLinkPtr(5)->addDrawingGeomtryInfo(STL_file);
		}
		for (unsigned int i = 0; i < 1; i++)
		{
			std::shared_ptr<irLib::irDyn::Mesh> STL_file(new irLib::irDyn::Mesh(std::string("../Data/CAD/efort_robot/LINK6_0") + std::to_string(i + 1) + std::string(".STL")));
			STL_file->setFrame(efortState->getLinkState(6).getLinkSE3().inverse());
			STL_file->setDimension(1);
			if (i == 0)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			getLinkPtr(6)->addDrawingGeomtryInfo(STL_file);
		}
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};