package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Don't use this
 */
@Deprecated
public class PID {
    public double Kp = 0;
    public double Ki = 0;
    public double Kd = 0;

    double lastReference = 0;
    double integralSum = 0;
    double lastError = 0;

    double maxIntegralSum = 0.25 / Ki;

    double a = 0.8;
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;

    ElapsedTime timer = new ElapsedTime();

    public PID(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }

    public double calculate(double reference, double current) {
        double error = reference - current;
        double errorChange = error - lastError;

        currentFilterEstimate = (a * previousFilterEstimate) + (1-a) * errorChange;
        previousFilterEstimate = currentFilterEstimate;

        double derivative = currentFilterEstimate / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }

        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        // reset integral sum upon setpoint changes
        if (reference != lastReference) {
            integralSum = 0;
        }

        lastReference = reference;
        lastError = error;
        timer.reset();

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}
