package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SwerveDriveWheel {
    private String Name; // Name, used for logging and telemetry
    private DcMotor DriveMotor;
    private CRServo AngleServo;
    private AnalogInput AngleSensor;
    org.firstinspires.ftc.robotcore.external.Telemetry Telemetry;

    public SwerveDriveWheel(org.firstinspires.ftc.robotcore.external.Telemetry telemetry, String name, DcMotor driveMotor, CRServo angleServo, AnalogInput angleSensor) {
        Telemetry = telemetry;
        Name = name;
        DriveMotor = driveMotor;
        AngleServo = angleServo;
        AngleSensor = angleSensor;
    }

    //private double ERROR_TO_SERVO_POWER = 0.005;
    private double MAXIMUM_SERVO_POWER = 1.0;
    private double ANGLE_ERROR_TOLERANCE = 0.5;
    private double MINIMUM_SERVO_POWER = 0.025;

    ElapsedTime timer = new ElapsedTime();

    // PID Variables
    double previousError = 0.0;
    double integral = 0.0;

    public void drive(double targetAngle, double motorPower, double Kp, double Ki, double Kd) {
    // TODO Tune PID controller. If I did this right each module should be able to recieve its own PID constants from CompetitonTeleop
    // TODO Figure out how to get wheels to maintain last heading after stick is no longer being moved
        // I referenced this website to figure out PID stuff :https://www.ctrlaltftc.com/the-pid-controller
        // not really sure if the timer thingy is needed

        double currentAngle = (AngleSensor.getVoltage() / 3.3) * 360 * -1; // Flip direction so clockwise is positive (with zero being forward)

        // PID calculations
        // Clamp the integral term to prevent windup
        integral = Math.max(-10.0, Math.min(10.0, integral));

        //Calculate proportional value
        double angleError = targetAngle - currentAngle;

        // Derivative
        double deltaError = (angleError - previousError) / timer.seconds(); // Calculate the change in error with respect to time

        // Integral
        integral = integral + (angleError * timer.seconds()); // Accumulate error for the integral term

        // Save the current error for the next loop iteration
        previousError = angleError;

        timer.reset();

        integral = 0;

        // Compute the shortest path rather than the naive difference.
        while (angleError < 0) {
            angleError += 360;
        }
        angleError = ((angleError + 180) % 360) - 180;
        // If driving the wheel backwards would be faster than spinning it around, do that.
        if (angleError > 90) {
            angleError -= 180;
            motorPower *= -1;
        }
        if (angleError < -90) {
            angleError += 180;
            motorPower *= -1;
        }

        //PID output
        double servoPower = (Kp * angleError) + (Ki * integral) + (Kd * deltaError);

        servoPower += Math.signum(angleError) * MINIMUM_SERVO_POWER;
        servoPower = Math.min(servoPower, MAXIMUM_SERVO_POWER);
        servoPower = Math.max(servoPower, -MAXIMUM_SERVO_POWER);
        if (Math.abs(angleError) < ANGLE_ERROR_TOLERANCE) {
            servoPower = 0;
        }
        AngleServo.setPower(servoPower);

        //Telemetry.addData(Name + " Angle", currentAngle);
        //Telemetry.addData(Name + " Power", motorPower);
        Telemetry.addData("proportional", Kp);
        Telemetry.addData("Integral", Ki);
        Telemetry.addData("Derivative", Kd);

        DriveMotor.setPower(motorPower);

        Telemetry.update();
        }


    }

