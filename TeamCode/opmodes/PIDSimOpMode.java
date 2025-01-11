package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.simulation.SimulatedMotor;

@TeleOp(name = "PID Simulation", group = "Examples")
public class PIDSimulationOpMode extends LinearOpMode {
    private SimulatedMotor motor;
    private PIDController pidController;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the simulated motor
        motor = new SimulatedMotor();

        // Initialize PID Controller with coefficients
        pidController = new PIDController(0.1, 0.01, 0.1, 0.0);

        // Wait for the start signal
        waitForStart();

        // Reset the timer at the start
        long lastTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            // Calculate the elapsed time since the last update
            long currentTime = System.currentTimeMillis();
            double deltaTime = (currentTime - lastTime) / 1000.0;
            lastTime = currentTime;

            // Set target position (example: 1000 ticks)
            double targetPosition = 1000;

            // Calculate power using PID controller
            double power = pidController.PIDControl(targetPosition, motor.getCurrentPosition());

            // Set motor power with bounds checking
            motor.setPower(Math.max(-1.0, Math.min(1.0, power)));

            // Update the simulated motor's position
            motor.update(deltaTime);

            // Telemetry for debugging
            telemetry.addData("Target", targetPosition);
            telemetry.addData("Current Position", motor.getCurrentPosition());
            telemetry.addData("Power", power);
            telemetry.addData("Error", targetPosition - motor.getCurrentPosition());
            telemetry.update();

            // Small delay to prevent overloading the loop
            sleep(50); // Adjust loop timing as needed
        }
    }
}