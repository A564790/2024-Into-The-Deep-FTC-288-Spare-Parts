package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

// Recommended reading: https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html

@TeleOp(name = "Swerve Test", group = "")
public class SwerveTest extends LinearOpMode {
    private SwerveDriveWheel LFWheel, LRWheel, RFWheel, RRWheel;
    private SwerveDriveCoordinator SwerveDrive;

    private double TRANSLATE_DEFAULT_SPEED = 0.4;
    private double TRANSLATE_HIGH_SPEED = 1.0;

    private double CALIBRATE_TRANSLATE_X = 1.0;
    private double CALIBRATE_TRANSLATE_Y = -1.0; // Gamepad Y axes are inverted from what you'd expect -- down is positive by default. So we negate it here.
    private double CALIBRATE_ROTATE = 1.0;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        LFWheel = new SwerveDriveWheel(
                telemetry,
                "LF",
                hardwareMap.dcMotor.get("LFDrive"),
                hardwareMap.crservo.get("LFSteer"),
                hardwareMap.analogInput.get("LFsteer")
        );
        LRWheel = new SwerveDriveWheel(
                telemetry,
                "LR",
                hardwareMap.dcMotor.get("LRDrive"),
                hardwareMap.crservo.get("LRSteer"),
                hardwareMap.analogInput.get("LRsteer")
        );
        RFWheel = new SwerveDriveWheel(
                telemetry,
                "RF",
                hardwareMap.dcMotor.get("RFDrive"),
                hardwareMap.crservo.get("RFSteer"),
                hardwareMap.analogInput.get("RFsteer")
        );
        RRWheel = new SwerveDriveWheel(
                telemetry,
                "RR",
                hardwareMap.dcMotor.get("RRDrive"),
                hardwareMap.crservo.get("RRSteer"),
                hardwareMap.analogInput.get("RRsteer")
        );
        SwerveDrive = new SwerveDriveCoordinator(telemetry, LFWheel, LRWheel, RFWheel, RRWheel);

        // Put initialization blocks here.
        waitForStart();

        // Put run blocks here.
        while (opModeIsActive()) {
            double translateSpeed = TRANSLATE_DEFAULT_SPEED;
            if (gamepad1.right_bumper) {
                translateSpeed = TRANSLATE_HIGH_SPEED;
            }

            SwerveDrive.drive(
                    gamepad1.left_stick_x * CALIBRATE_TRANSLATE_X * translateSpeed,
                    gamepad1.left_stick_y * CALIBRATE_TRANSLATE_Y * translateSpeed,
                    gamepad1.right_stick_x * CALIBRATE_ROTATE
            );

            telemetry.update();
        }
    }
}