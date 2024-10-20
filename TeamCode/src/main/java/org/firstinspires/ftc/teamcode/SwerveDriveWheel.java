package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    private double ERROR_TO_SERVO_POWER = 0.01;
    private double MAXIMUM_SERVO_POWER = 1.0;
    private double ANGLE_ERROR_TOLERANCE = 0.5;

    public void drive(double targetAngle, double motorPower) {
        double currentAngle = (AngleSensor.getVoltage() / 3.3) * 360 * -1; // Flip direction so clockwise is positive (with zero being forward)
        double angleError = targetAngle - currentAngle;
        // Compute the shortest path rather than the naive difference.
        while (angleError < 0) { angleError += 360; }
        angleError = ((angleError + 180) % 360) - 180;
        // If driving the wheel backwards would be faster than spinning it around, do that.
        if (angleError > 90) { angleError -= 180; motorPower *= -1; }
        if (angleError < -90) { angleError += 180; motorPower *= -1; }

        // Drive the servo. Do nothing if the angle error is small enough.
        double servoPower = angleError * ERROR_TO_SERVO_POWER;
        servoPower = Math.min(servoPower, MAXIMUM_SERVO_POWER);
        servoPower = Math.max(servoPower, -MAXIMUM_SERVO_POWER);
        if (Math.abs(angleError) < ANGLE_ERROR_TOLERANCE) {
            servoPower = 0;
        }
        AngleServo.setPower(servoPower);

        Telemetry.addData(Name + " Angle", currentAngle);
        Telemetry.addData(Name + " Power", motorPower);

        DriveMotor.setPower(motorPower);
    }
}
