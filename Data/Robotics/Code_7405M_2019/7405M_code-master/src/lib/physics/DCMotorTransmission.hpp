//
// Created by alexweiss on 7/4/19.
//

#ifndef INC_7405M_CODE_DCMOTOR_HPP
#define INC_7405M_CODE_DCMOTOR_HPP

#include "../utility/Units.hpp"

namespace physics {
    class DCMotorTransmission {
    protected:
        const units::QAngularSpeed speed_per_volt_; // RADIANS / SECOND
        const units::QTorque torque_per_volt_; // Nm / VOLT
        const units::Number friction_voltage_; // voltage to overcome friction (V)
    public:
        DCMotorTransmission(units::QAngularSpeed speed_per_volt, units::QTorque torque_per_volt, units::Number friction_voltage);
        units::QAngularSpeed speed_per_volt();
        units::QTorque torque_per_volt();
        units::Number friction_voltage();
        units::QAngularSpeed free_speed_at_voltage(units::Number voltage);
        units::QTorque get_torque_at_voltage(units::QAngularSpeed speed, units::Number voltage);
        units::Number get_voltage_for_torque(units::QAngularSpeed speed, units::QTorque torque);

    };
}


#endif //INC_7405M_CODE_DCMOTOR_HPP
