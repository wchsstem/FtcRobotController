package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PIDController;

@TeleOp(name = "PID Controller", group = "Examples")
public class PIDOpMode extends LinearOpMode {
    private DcMotorEx motor;
    private PIDController pidController = new PIDController();

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the start signal
        waitForStart();
        
        // Reset the timer at the start
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive()) {
            // Set target position (example: 1000 ticks)
            double targetPosition = 1000;

            // Calculate power using PID controller
            double power = pidController.PIDControl(targetPosition, motor.getCurrentPosition());

            // Set motor power with bounds checking
            motor.setPower(Math.max(-1.0, Math.min(1.0, power)));

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