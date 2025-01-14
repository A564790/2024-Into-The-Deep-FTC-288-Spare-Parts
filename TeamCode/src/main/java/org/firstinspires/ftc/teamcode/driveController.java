package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class driveController {
    private Telemetry telemetry;

    private DcMotor leftFront, rightFront, leftRear, rightRear;


    public driveController(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        leftFront = hardwareMap.get(DcMotor.class, "frontLeftDriveMotor");
        rightFront = hardwareMap.get(DcMotor.class, "frontRightDriveMotor");
        leftRear = hardwareMap.get(DcMotor.class, "backLeftDriveMotor");
        rightRear = hardwareMap.get(DcMotor.class, "backRightDriveMotor");

    }
    public void drive(double leftFrontPower, double rightFrontPower, double leftRearPower, double rightRearPower){
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftRear.setPower(leftRearPower);
        rightRear.setPower(rightRearPower);
    }

}
