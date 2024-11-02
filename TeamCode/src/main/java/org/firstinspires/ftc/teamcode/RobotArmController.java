package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotArmController {
    // TODO: Get these calibrated better
    private double EXTENSION_LENGTH_TO_COUNTS = 112.0; // Motor 28 counts/revolution, geared 3:1 -> 4:1 -> 1" diameter spool, means roughly 112 counts/inch
    private double PICKUP_SPEED_TO_POWER = 1.0;
    private double ANGLE_FROM_ENCODER_COUNTS = -(360.0 / 8192.0); // 8192 counts per revolution
    private double ANGLE_ERROR_TOLERANCE = 1; // Allow 1 degree of angle error
    private double ANGLE_ERROR_TO_POWER = 0.03;

    private Telemetry telemetry;

    private CRServo pickupServoL, pickupServoR;
    private Servo wristServoL, wristServoR;

    private DcMotor angleMotorL, angleMotorR, extensionMotor;

    // TODO: Sensor inputs for arm angle and extension

    private double extensionTarget = 0.0; // Target extension length
    private double angleTarget = 0.0;     // Target arm angle
    private double wristTarget = 0.0;     // Target wrist angle
    private double pickupSpeed = 0.0;     // Target pickup *speed*

    private double MAXIMUM_ARM_EXTENSION = 3650.0;

    //ChatGPT wrote all of the PID stuff idk
    // PID Constants
    private double Kp = 1.0; // Proportional gain
    private double Ki = 0.12; // Integral gain
    private double Kd = 0.01; // Derivative gain

    // PID Variables
    public double previousError = 0.0;
    public double integral = 0.0;

    public RobotArmController(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

         pickupServoL = hardwareMap.crservo.get("PickupServoL");
         pickupServoR = hardwareMap.crservo.get("PickupServoR");
         pickupServoL.setDirection(DcMotorSimple.Direction.REVERSE);

         wristServoL = hardwareMap.servo.get("WristServoL");
         wristServoR = hardwareMap.servo.get("WristServoR");
         wristServoR.setDirection(Servo.Direction.REVERSE);



         angleMotorL = hardwareMap.dcMotor.get("AngleMotorL");
         angleMotorR = hardwareMap.dcMotor.get("AngleMotorR");
         angleMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
         angleMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Hold position when on target
         angleMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Hold position when on target
        //angleTarget= 50;

         extensionMotor = hardwareMap.dcMotor.get("ExtensionMotor");
         extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         extensionMotor.setPower(0.0); // Start up in brake mode
    }


    public void driveDirectly(double armExtensionIn, boolean backwards, double armAngleInput , double wristInput, double pickup) {
        extensionTarget += armExtensionIn *50;

        if ((backwards) && (extensionTarget <= 0)) {
            extensionTarget = -100;
        }

        if ((extensionTarget < 0) && (!backwards)) {
            extensionTarget = 0 ;
        }

        if (extensionTarget > MAXIMUM_ARM_EXTENSION) {
            extensionTarget = MAXIMUM_ARM_EXTENSION;
        }

        telemetry.addData("backwards", backwards);
        telemetry.addData("extensionPosition", extensionMotor.getCurrentPosition());
        telemetry.addData("extensionTarget", extensionTarget);

        extensionMotor.setTargetPosition((int)(extensionTarget));
        extensionMotor.setPower(1.0); // Run at 50% power in run-to-position mode
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //double wristPosition = (wristInput + 1.0)/2.0;
        wristTarget += wristInput * 0.05;
        wristTarget = Math.min(Math.max(wristTarget, 0.0), 1.0);
        wristServoL.setPosition(wristTarget);
        //wristServoR.setPosition(wristTarget);


        if (pickup > 0) {
            pickupServoL.setPower(pickup * 0.20);
            pickupServoR.setPower(pickup * 0.20);
        }
        else {
            pickupServoL.setPower(pickup);
            pickupServoR.setPower(pickup);
        }

        // The arm angle uses two motors and a single through-bore encoder on the shaft,
        // so we have to handle the closed-loop position control ourselves by reading the
        // through-bore encoder count, figuring out the error, and applying an appropriate
        // feedback.
        //
        // The through-bore encoder is connected in place of one of the arm angle motors'
        // built-in motor encoder.
        /* angleTarget+=armAngleInput;
        if (angleTarget < 0) {
            angleTarget = 0;
        }
        //if (angleTarget > 135) {
           // angleTarget = 135;
        //}
        double angleCurrent = (double) angleMotorL.getCurrentPosition() * ANGLE_FROM_ENCODER_COUNTS;
        angleCurrent = (angleCurrent + 720) % 360;
        double angleError = (angleTarget - angleCurrent);
        // double angleCorrection = 0; // angleError * ANGLE_ERROR_TO_POWER;

        //lines 111 - 120 is more ChatGPT code
        // PID calculations
        integral += angleError; // Accumulate error for the integral term

        double deltaError = angleError - previousError; // Calculate the change in error


        // Calculate the PID output
        double angleCorrection = (Kp * angleError) + (Ki * integral) + (Kd * deltaError);

        // Save the current error for the next loop iteration
        previousError = angleError;

        // Clamp the integral term to prevent windup
        integral = Math.max(-10.0, Math.min(10.0, integral));

        // Reset integral if error is small
        if (Math.abs(angleError) < 2.5) { // Adjust threshold as needed
            angleCorrection = 0;
        }

        angleCorrection += (angleCurrent < 120) ? 0.1 : -0.1;
        //angleCorrection += Math.signum(angleError) * 0.1; // Static bias

        // Clamp the angleCorrection values to ensure it doesn't exceed maximum motor values
        angleCorrection = Math.max(-0.25, Math.min(angleCorrection, 0.25));

        angleMotorL.setPower(angleCorrection);
        angleMotorR.setPower(angleCorrection);

        telemetry.addData("angleTarget", angleTarget);
        telemetry.addData("angleCurrent", angleCurrent);
        telemetry.addData("wristAngleL", wristServoL.getPosition());
        telemetry.addData("wristAngleR", wristServoR.getPosition());

         */


    }

    public void setpoint(double wristAngle) {
        //extensionTarget = armExtension;
        //angleTarget = armAngle;
        wristTarget = wristAngle;
        //pickupSpeed = pickup;
    }

    public void setpointFloorPickup() {
        this.setpoint(0.7);
    }

    public void setpointSubPickUp() {
        this.setpoint(1);
    }

    public void setpointScoreHighBasket() {
        this.setpoint(0.6);
    }

}

