package org.firstinspires.ftc.teamcode.opmodes;

// Package importation
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID Controller", group = "Examples")
public class PIDController extends LinearOpMode {
    private DcMotorEx motor;

    private double integralSum = 0.0;
    private double Kp = 0.1; // Proportional coefficient
    private double Ki = 0.01; // Integral coefficient
    private double Kd = 0.1; // Derivative coefficient
    private double Kf = 0.0; // Feedforward coefficient
    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the motor
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Wait for the start signal
        waitForStart();
        
        // Reset the timer at the start
        timer.reset();

        while (opModeIsActive()) {
            // Set target position (example: 1000 ticks)
            double targetPosition = 1000;

            // Calculate power using PID controller
            double power = PIDControl(targetPosition, motor.getCurrentPosition());

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
    }5

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);

        timer.reset();
        
        return output;
    }
}


// PID SIMULATION IN CASE ROBOT IS INACCESSIBLE!
    


*/