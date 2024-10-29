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
    private double ANGLE_FROM_ENCODER_COUNTS = (360.0 / 8192.0); // 8192 counts per revolution
    private double ANGLE_ERROR_TOLERANCE = 1; // Allow 1 degree of angle error
    private double ANGLE_ERROR_TO_POWER = 0.01;

    private Telemetry telemetry;

    private CRServo pickupServoL, pickupServoR;
    private Servo wristServoL, wristServoR;

    private DcMotor angleMotorL, angleMotorR, extensionMotor;

    // TODO: Sensor inputs for arm angle and extension

    private double extensionTarget = 0.0; // Target extension length
    private double angleTarget = 0.0;     // Target arm angle
    private double wristTarget = 0.0;     // Target wrist angle
    private double pickupSpeed = 0.0;     // Target pickup *speed*

    public RobotArmController(Telemetry telemetry, HardwareMap hardwareMap) {
         pickupServoL = hardwareMap.crservo.get("PickupServoL");
         pickupServoR = hardwareMap.crservo.get("PickupServoR");
         pickupServoR.setDirection(DcMotorSimple.Direction.REVERSE);

         wristServoL = hardwareMap.servo.get("WristServoL");
         wristServoR = hardwareMap.servo.get("WristServoR");
         wristServoR.setDirection(Servo.Direction.REVERSE);

         angleMotorL = hardwareMap.dcMotor.get("AngleMotorL");
         angleMotorR = hardwareMap.dcMotor.get("AngleMotorR");
         angleMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
         angleMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Hold position when on target
         angleMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Hold position when on target

         extensionMotor = hardwareMap.dcMotor.get("ExtensionMotor");
         extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         extensionMotor.setPower(0.0); // Start up in brake mode
    }

    public void setpoint(double armExtension, double armAngle, double wristAngle, double pickup) {
        extensionTarget = armExtension;
        angleTarget = armAngle;
        wristTarget = wristAngle;
        pickupSpeed = pickup;
    }

    public void adjust(double armExtension, double armAngle, double wristAngle, double pickup) {
        extensionTarget += armExtension;
        angleTarget += armAngle;
        wristTarget += wristAngle;
        pickupSpeed = pickup;
    }

    public void setpointFloorPickup() {
        this.setpoint(0, 0, 0, 0);
    }

    public void setpointScoreOnBarPosition() {
        this.setpoint(0, 0, 0, 0);
    }

    public void setpointScoreOnBarRelease() {
        this.setpoint(0, 0, 0, 0);
    }

    public void update() {
        extensionMotor.setTargetPosition((int) (extensionTarget * EXTENSION_LENGTH_TO_COUNTS));
        extensionMotor.setPower(0.5); // Run at 50% power in run-to-position mode

        wristServoL.setPosition(wristTarget);
        wristServoR.setPosition(wristTarget);
        pickupServoL.setPower(pickupSpeed * PICKUP_SPEED_TO_POWER);
        pickupServoR.setPower(pickupSpeed * PICKUP_SPEED_TO_POWER);

        // The arm angle uses two motors and a single through-bore encoder on the shaft,
        // so we have to handle the closed-loop position control ourselves by reading the
        // through-bore encoder count, figuring out the error, and applying an appropriate
        // feedback.
        //
        // The through-bore encoder is connected in place of one of the arm angle motors'
        // built-in motor encoder.
        //
        // TODO: Use the appropriate motor (L or R) corresponding to which one got the through-bore encoder instead of the built-in motor encoder
        double angleCurrent = (double) angleMotorL.getCurrentPosition() * ANGLE_FROM_ENCODER_COUNTS;
        double angleError = (angleTarget - angleCurrent);
        if (Math.abs(angleError) < ANGLE_ERROR_TOLERANCE) {
            angleError = 0;
        }
        angleMotorL.setPower(angleError * ANGLE_ERROR_TO_POWER);
        angleMotorR.setPower(angleError * ANGLE_ERROR_TO_POWER);
    }
}
