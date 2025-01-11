package org.firstinspires.ftc.teamcode.simulation;

public class SimulatedMotor {
    private double currentPosition = 0.0;
    private double velocity = 0.0;
    private double power = 0.0;

    // Update the motor's position based on the applied power and elapsed time
    public void update(double deltaTime) {
        // Simple simulation: position changes linearly based on power
        velocity = power * 100; // Example: 100 ticks per second per power unit
        currentPosition += velocity * deltaTime;
    }

    // Set the motor power
    public void setPower(double power) {
        this.power = power;
    }

    // Get the current position of the motor
    public double getCurrentPosition() {
        return currentPosition;
    }
}