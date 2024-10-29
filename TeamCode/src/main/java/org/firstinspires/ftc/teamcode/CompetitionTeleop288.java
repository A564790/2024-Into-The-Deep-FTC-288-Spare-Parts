package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Recommended reading: https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html

@TeleOp(name = "Competition Teleop", group = "")
public class CompetitionTeleop288 extends LinearOpMode {
    private SwerveDriveWheel LFWheel, LRWheel, RFWheel, RRWheel;
    private SwerveDriveCoordinator SwerveDrive;
    private RobotArmController RobotArm;


    private double TRANSLATE_DEFAULT_SPEED = 0.5;
    private double TRANSLATE_HIGH_SPEED = 1.0;

    private double CALIBRATE_TRANSLATE_X = 1.0;
    private double CALIBRATE_TRANSLATE_Y = -1.0; // Gamepad Y axes are inverted from what you'd expect -- down is positive by default. So we negate it here.
    private double CALIBRATE_ROTATE = 1.0;

    private final double JOYSTICK_DEAD_ZONE = 0.20;

    IMU imu;

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
        RobotArm = new RobotArmController(telemetry, hardwareMap);

        // Put initialization blocks here.
        telemetry.addData("Status", "Waiting for Start");    //
        telemetry.update();
        waitForStart();

        // IMU parameters and initialization
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuParams);

        // Put run blocks here.
        while (opModeIsActive()) {
            // Speed adjustment: Robot translation uses 50% speed until the high-speed trigger is pressed.
            double translateSpeed = TRANSLATE_DEFAULT_SPEED;
            if (gamepad1.right_trigger != 0) {
                translateSpeed = TRANSLATE_HIGH_SPEED;
            }

            // IMU debug logging
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z) velocity", angularVelocity.zRotationRate);
            telemetry.addData("Pitch (X) velocity", angularVelocity.xRotationRate);
            telemetry.addData("Roll (Y) velocity", angularVelocity.yRotationRate);

            // Field-centric control works by using the IMU heading to translate a field-centric
            // X-Y movement input into the robot-centric coordinate system. This is just a simple
            // vector rotation by the robot's current heading.
            if (gamepad1.left_trigger != 0) {
                telemetry.addData("Info", "Resetting IMU orientation");
                imu.initialize(imuParams); // Reset field-centric positioning when trigger is pressed
            }
            double inputDriveX = inputScaling(gamepad1.left_stick_x) * CALIBRATE_TRANSLATE_X * translateSpeed;
            double inputDriveY = inputScaling(gamepad1.left_stick_y) * CALIBRATE_TRANSLATE_Y * translateSpeed;
            double inputDriveRotation = inputScaling(gamepad1.right_stick_x) * CALIBRATE_ROTATE;
            double yawDegrees = imu.getRobotYawPitchRollAngles().getYaw(); // TODO: Ensure yaw is mapped correctly
            // Rotate joystick X/Y from field-centric to robot-centric coordinates
            double robotDriveX = inputDriveX * cos(toRadians(yawDegrees)) - inputDriveY * sin(toRadians(yawDegrees));
            double robotDriveY = inputDriveX * sin(toRadians(yawDegrees)) + inputDriveY * cos(toRadians(yawDegrees));
            SwerveDrive.drive(robotDriveX, robotDriveY, inputDriveRotation);

            // Robot arm control. When various buttons are pushed they directly set the target
            // angles and speed to some value, and when no button is pressed the targets are
            // updated by integrating the joystick inputs (so pushing the stick up makes the
            // target angle increase over time).
            if (gamepad2.a) {
                // TODO: Decide if named setpoint methods are useful or if this should just be setpoint(1.1, 0, 2, 100) or something
                RobotArm.setpointFloorPickup();
            } else if (gamepad2.b) {
                RobotArm.setpointScoreOnBarPosition();
            } else if (gamepad2.x) {
                RobotArm.setpointScoreOnBarRelease();
            } else {
                RobotArm.adjust(
                        // TODO: Assign sticks properly
                        inputScaling(gamepad2.left_stick_y),
                        inputScaling(gamepad2.left_stick_x),
                        inputScaling(gamepad2.right_stick_x),
                        inputScaling(gamepad2.right_stick_y)
                );
            }
            RobotArm.update();

            telemetry.update();
        }
    }

    double inputScaling(double x) {
        double sign = Math.signum(x);
        double magnitude = Math.abs(x);
        if (magnitude < JOYSTICK_DEAD_ZONE) {
            magnitude = 0.0;
        } else {
            magnitude = (magnitude - JOYSTICK_DEAD_ZONE) / (1.0 - JOYSTICK_DEAD_ZONE);
        }
        magnitude = Math.pow(magnitude, 2.0);
        return sign * magnitude;
    }
}