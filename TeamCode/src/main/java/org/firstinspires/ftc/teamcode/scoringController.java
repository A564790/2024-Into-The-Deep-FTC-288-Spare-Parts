package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class scoringController {

    private Telemetry telemetry;

    private double shoulderTarget = 0;

    private Servo lowerPickupServo, wristServo, armServo, shoulderServo, upperPickupServo, upperArmServo;

    private DcMotor elevatorMotorR, elevatorMotorL, extensionMotor;

    private double maximumVerticalExtension = 0;
    private double maximumVerticalRetraction = 0;

    private double maximumHorizontalExtention = 0;
    private double maximumHorizontalRetraction = 0;


    public scoringController(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        lowerPickupServo = hardwareMap.servo.get("placeholder");
        wristServo = hardwareMap.servo.get("placeholder");
        armServo = hardwareMap.servo.get("placeholder");
        shoulderServo = hardwareMap.servo.get("placeholder");
        upperPickupServo = hardwareMap.servo.get("placeholder");
        upperArmServo = hardwareMap.servo.get("placeholder");

        elevatorMotorL = hardwareMap.dcMotor.get("placeholder");
        elevatorMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotorR = hardwareMap.dcMotor.get("placeholder");
        elevatorMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionMotor = hardwareMap.dcMotor.get("placeholder");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void shoulderDrive(double shoulderIn) {

        if (shoulderIn != 0) {
            shoulderTarget = (shoulderIn + 1.0) / 2.0;
            shoulderTarget = Math.min(Math.max(shoulderIn, 0.0), 1.0);
            shoulderServo.setPosition(shoulderTarget);

            telemetry.addData("lower pickup servo", lowerPickupServo.getPosition());
            telemetry.addData("wrist servo", lowerPickupServo.getPosition());
            telemetry.addData("arm servo", lowerPickupServo.getPosition());
            telemetry.addData("shoulder servo", lowerPickupServo.getPosition());
            telemetry.addData("upper pickup servo", lowerPickupServo.getPosition());

            telemetry.addData("elevatorMotorR", elevatorMotorR.getCurrentPosition());
            telemetry.addData("elevatorMotorL", elevatorMotorL.getCurrentPosition());

            telemetry.addData("extensionMotor", extensionMotor.getCurrentPosition());
        }
        else {
            shoulderTarget = 0;
        }

    }
    //TODO figure out slide motor stuff. Probably do same run to position stuff as on the previous arm. Copy will's code from RobotArmController

    public void presetIntakePoint(double shoulderInput, double armInput, double wristInput, double lowerPickupInput) {
        //TODO implement finite state machine logic so all servos move to their assigned positions at the same time

        shoulderServo.setPosition(shoulderInput);

        armServo.setPosition(armInput);

        wristServo.setPosition(wristInput);

        lowerPickupServo.setPosition(lowerPickupInput);
    }

    public void presetUpperPickupPoint(double upperPickupInput, double upperArmInput) {

        upperPickupServo.setPosition(upperPickupInput);
        upperArmServo.setPosition(upperArmInput);
    }

    public void intakePresetPoint1() {
        this.presetIntakePoint(0,0,0,0);
    }

    public void intakePresetPoint2() {
        this.presetIntakePoint(0,0,0,0);
    }

    public void intakePresetPoint3() {
        this.presetIntakePoint(0,0,0,0);
    }

    public void intakePresetPoint4() {
        this.presetIntakePoint(0,0,0,0);
    }


    public void upperPickupPresetPoint1() {
        this.presetUpperPickupPoint(0,0);
    }

    public void upperPickupPresetPoint2() {
        this.presetUpperPickupPoint(0,0);
    }

    public void upperPickupPresetPoint3() {
        this.presetUpperPickupPoint(0,0);
    }

    public void upperPickupPresetPoint4() {
        this.presetUpperPickupPoint(0,0);
    }

}
