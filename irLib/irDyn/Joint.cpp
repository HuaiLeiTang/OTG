#include "Joint.h"

using namespace irLib::irMath;

namespace irLib
{
	namespace irDyn
	{
		Joint::Joint() : _rotorinertia(0), _resistance(0), _inductance(0), _gearRatio(0),
			_springConstant(0), _damperConstant(0), _frictionConstant(0)
		{
			_axis.setZero();
			// z-axis (w,v) = (0, 0, 1, 0, 0, 0)
			_axis[2] = 1;

			_LimitPosLower = std::numeric_limits<Real>::min();
			_LimitPosUpper = std::numeric_limits<Real>::max();

			_LimitVelLower = std::numeric_limits<Real>::min();
			_LimitVelUpper = std::numeric_limits<Real>::max();

			_LimitAccLower = std::numeric_limits<Real>::min();
			_LimitAccUpper = std::numeric_limits<Real>::max();

			_LimitJerkLower = std::numeric_limits<Real>::min();
			_LimitJerkUpper = std::numeric_limits<Real>::max();

			_LimitTorqueLower = std::numeric_limits<Real>::min();
			_LimitTorqueUpper = std::numeric_limits<Real>::max();
		}

		Joint::Joint(const Vector6 & axis) : _axis(axis), _rotorinertia(0), _resistance(0), _inductance(0), _gearRatio(0),
			_springConstant(0), _damperConstant(0), _frictionConstant(0)
		{
			_LimitPosLower = std::numeric_limits<Real>::min();
			_LimitPosUpper = std::numeric_limits<Real>::max();

			_LimitVelLower = std::numeric_limits<Real>::min();
			_LimitVelUpper = std::numeric_limits<Real>::max();

			_LimitAccLower = std::numeric_limits<Real>::min();
			_LimitAccUpper = std::numeric_limits<Real>::max();

			_LimitJerkLower = std::numeric_limits<Real>::min();
			_LimitJerkUpper = std::numeric_limits<Real>::max();

			_LimitTorqueLower = std::numeric_limits<Real>::min();
			_LimitTorqueUpper = std::numeric_limits<Real>::max();
		}

		Joint::~Joint() {}

		void Joint::setRotorInertia(const Real RI)
		{
			_rotorinertia = RI;
		}

		void Joint::setResistance(const Real R)
		{
			_resistance = R;
		}

		void Joint::setInductance(const Real In)
		{
			_inductance = In;
		}

		void Joint::setGearRatio(const Real Gear)
		{
			_gearRatio = Gear;
		}

		void Joint::setMotorConstant(const Real MC)
		{
			_motorConstant = MC;
		}

		void Joint::setBackEMFConstant(const Real BEMFC)
		{
			_backEMFConstant = BEMFC;
		}

		void Joint::setSpringConstant(const Real Spring)
		{
			_springConstant = Spring;
		}

		void Joint::setDamperConstant(const Real Damper)
		{
			_damperConstant = Damper;
		}

		void Joint::setFrictionConstant(const Real Friction)
		{
			_frictionConstant = Friction;
		}

		bool Joint::setLimitPos(const Real lower, const Real upper)
		{
			_LimitPosLower = lower;
			_LimitPosUpper = upper;
			return true;
		}

		bool Joint::setLimitVel(const Real lower, const Real upper)
		{
			_LimitVelLower = lower;
			_LimitVelUpper = upper;
			return true;
		}

		bool Joint::setLimitAcc(const Real lower, const Real upper)
		{
			_LimitAccLower = lower;
			_LimitAccUpper = upper;
			return true;
		}

		bool Joint::setLimitJerk(const Real lower, const Real upper)
		{
			_LimitJerkLower = lower;
			_LimitJerkUpper = upper;
			return true;
		}

		bool Joint::setLimitTorque(const Real lower, const Real upper)
		{
			_LimitTorqueLower = lower;
			_LimitTorqueUpper = upper;
			return true;
		}

		void Joint::setAxis(const Vector3 & axis)
		{
			_axis[0] = axis[0];
			_axis[1] = axis[1];
			_axis[2] = axis[2];
		}

		void Joint::setAxis(const Vector6 & axis)
		{
			_axis = axis;
		}

		const Real Joint::getRotorInertia() const
		{
			return _rotorinertia;
		}

		const Real Joint::getResistance() const
		{
			return _resistance;
		}

		const Real Joint::getInductance() const
		{
			return _inductance;
		}

		const Real Joint::getGearRatio() const
		{
			return _gearRatio;
		}

		const Real Joint::getMotorConstant() const
		{
			return _motorConstant;
		}

		const Real Joint::getBackEMFConstant() const
		{
			return _backEMFConstant;
		}

		const Real Joint::getSpringConstant() const
		{
			return _springConstant;
		}

		const Real Joint::getDamperConstant() const
		{
			return _damperConstant;
		}

		const Real Joint::getFrictionConstant() const
		{
			return _frictionConstant;
		}

		const Real Joint::getLimitPosLower() const
		{
			return _LimitPosLower;
		}

		const Real Joint::getLimitPosUpper() const
		{
			return _LimitPosUpper;
		}

		const Real Joint::getLimitVelLower() const
		{
			return _LimitVelLower;
		}

		const Real Joint::getLimitVelUpper() const
		{
			return _LimitVelUpper;
		}

		const Real Joint::getLimitAccLower() const
		{
			return _LimitAccLower;
		}

		const Real Joint::getLimitAccUpper() const
		{
			return _LimitAccUpper;
		}

		const Real Joint::getLimitJerkLower() const
		{
			return _LimitJerkLower;
		}

		const Real Joint::getLimitJerkUpper() const
		{
			return _LimitJerkUpper;
		}

		const Real Joint::getLimitTorqueLower() const
		{
			return _LimitTorqueLower;
		}

		const Real Joint::getLimitTorqueUpper() const
		{
			return _LimitTorqueUpper;
		}

		const Vector6 & Joint::getAxis() const
		{
			return _axis;
		}

		JointPtr Joint::copy() const
		{
			JointPtr clone(new Joint(_axis));
			clone->setRotorInertia(_rotorinertia);
			clone->setResistance(_resistance);
			clone->setInductance(_inductance);
			clone->setGearRatio(_gearRatio);
			clone->setSpringConstant(_springConstant);
			clone->setDamperConstant(_damperConstant);
			clone->setFrictionConstant(_frictionConstant);
			clone->setLimitPos(_LimitPosLower, _LimitPosUpper);
			clone->setLimitVel(_LimitVelLower, _LimitVelUpper);
			clone->setLimitAcc(_LimitAccLower, _LimitAccUpper);
			clone->setLimitJerk(_LimitJerkLower, _LimitJerkUpper);
			clone->setLimitTorque(_LimitTorqueLower, _LimitTorqueUpper);
			return clone;
		}
	}
}