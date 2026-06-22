#pragma once
#include <Arduino.h>

class SpeedPID {
    private:
        double kp, ki, kd;
        double out_min, out_max;
        double prev_error;
        double output;
        double error;
        double filt_deriv   = 0.0; // メンバにする
        double integral     = 0;
        double integral_max = 0;
        double integral_min = 0;

    public:
        SpeedPID(double kp, double ki, double kd, double out_min, double out_max, double integral_min, double integral_max)
            : kp(kp), ki(ki), kd(kd), out_min(out_min), out_max(out_max), integral(0.0), integral_min(integral_min),
              integral_max(integral_max) {}

        double update(double target, double measurement, double dt) {
            if (dt <= 0.0) return output;

            error = target - measurement;

            double delta_error = error - prev_error;

            integral += error * dt;
            integral = constrain(integral, integral_min, integral_max);

            constexpr double deriv_alpha = 0.1;

            double derivative = (error - prev_error) / dt;

            filt_deriv += deriv_alpha * (derivative - filt_deriv);

            double du = kp * delta_error + ki * integral + kd * filt_deriv;

            double tentative = output + du;

            if ((tentative > out_max && du > 0) || (tentative < out_min && du < 0)) {
                integral -= error * dt;

                du = kp * delta_error + ki * integral + kd * filt_deriv;
            }

            output += du;
            output = constrain(output, out_min, out_max);

            prev_error = error;
            return output;
        }

        double getError() { return error; }

        void reset() {
            prev_error = 0.;
            output     = 0.;
            filt_deriv = 0.;
            integral   = 0.;
        }
};
