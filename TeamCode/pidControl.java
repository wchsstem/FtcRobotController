package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double integralSum = 0.0;
    private double Kp;
    private double Ki;
    private double Kd;
    private double Kf;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;
    private double integralLimit = 1.0; // Example limit for anti-windup

    public PIDController(double Kp, double Ki, double Kd, double Kf) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.Kf = Kf;
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        double deltaTime = timer.seconds();

        // Proportional term
        double proportional = Kp * error;

        // Integral term with anti-windup values
        integralSum += error * deltaTime;
        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));
        double integral = Ki * integralSum;

        // Derivative term
        double derivative = Kd * (error - lastError) / deltaTime;
        lastError = error;

        // Feedforward term
        double feedforward = Kf * reference;

        // PID output
        double output = proportional + integral + derivative + feedforward;
        
        timer.reset();

        return output;
    }

    // methods used to dynamically set PID coefficients
    public void setKp(double Kp) {
        this.Kp = Kp;
    }

    public void setKi(double Ki) {
        this.Ki = Ki;
    }

    public void setKd(double Kd) {
        this.Kd = Kd;
    }

    public void setKf(double Kf) {
        this.Kf = Kf;
    }

    public void setIntegralLimit(double limit) {
        this.integralLimit = limit;
    }
}