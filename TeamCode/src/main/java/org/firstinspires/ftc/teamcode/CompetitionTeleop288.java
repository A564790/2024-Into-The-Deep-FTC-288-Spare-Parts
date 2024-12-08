package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Recommended reading: https://compendium.readthedocs.io/en/latest/tasks/drivetrains/swerve.html
// I'm illiterate

@TeleOp(name = "Competition Teleop", group = "")
public class CompetitionTeleop288 extends LinearOpMode {
    private SwerveDriveWheel LFWheel, LRWheel, RFWheel, RRWheel;
    private SwerveDriveCoordinator SwerveDrive;


    private double TRANSLATE_DEFAULT_SPEED = 0.7;
    private double TRANSLATE_HIGH_SPEED = 0.8;

    private double CALIBRATE_TRANSLATE_X = 1.0;
    private double CALIBRATE_TRANSLATE_Y = -1.0; // Gamepad Y axes are inverted from what you'd expect -- down is positive by default. So we negate it here.
    private double CALIBRATE_ROTATE = 1.0;
    private double proportional = 0;
    private double integral = 0;
    private double derivative = 0;


    private final double JOYSTICK_DEAD_ZONE = 0.20;

    private DcMotor angleMotorL = null;
    private DcMotor angleMotorR = null;

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


        imu = hardwareMap.get(IMU.class, "imu");

        SwerveDrive = new SwerveDriveCoordinator(telemetry, LFWheel, LRWheel, RFWheel, RRWheel);
        //RobotArm = new RobotArmController(telemetry, hardwareMap);
        scoringController robotScoring = new scoringController(telemetry, hardwareMap);

        // Put initialization blocks here.
        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();
        waitForStart();

        // IMU parameters and initialization
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(imuParams);

        // Previous iteration gamepad states for edge detection
        Gamepad prevGamepad1 = new Gamepad();
        Gamepad prevGamepad2 = new Gamepad();
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        // Put run blocks here.
        while (opModeIsActive()) {
            // Update sampled gamepad states. Sampling the current gamepad state as 'currentGamepadN'
            // is necessary for reliable edge triggering on button presses.
            prevGamepad1.copy(currentGamepad1);
            prevGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Speed adjustment: Robot translation uses 50% speed until the high-speed trigger is pressed.
            double translateSpeed = TRANSLATE_DEFAULT_SPEED;
            if (currentGamepad1.right_trigger != 0) {
                translateSpeed = TRANSLATE_HIGH_SPEED;
            }

            // IMU debug logging
            //YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            //AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            //telemetry.addData("Yaw (Z)", orientation.getYaw(AngleUnit.DEGREES));
            //telemetry.addData("Pitch (X)", orientation.getPitch(AngleUnit.DEGREES));
            //telemetry.addData("Roll (Y)", orientation.getRoll(AngleUnit.DEGREES));
            //telemetry.addData("Yaw (Z) velocity", angularVelocity.zRotationRate);
            //telemetry.addData("Pitch (X) velocity", angularVelocity.xRotationRate);
            //telemetry.addData("Roll (Y) velocity", angularVelocity.yRotationRate);

            // Field-centric control works by using the IMU heading to translate a field-centric
            // X-Y movement input into the robot-centric coordinate system. This is just a simple
            // vector rotation by the robot's current heading.
            if (currentGamepad1.left_trigger != 0) {
                telemetry.addData("Info", "Resetting IMU orientation");
                imu.initialize(imuParams); // Reset field-centric positioning when trigger is pressed
            }

            double inputDriveX = inputScaling(-currentGamepad1.left_stick_x) * CALIBRATE_TRANSLATE_X * translateSpeed;
            double inputDriveY = inputScaling(-currentGamepad1.left_stick_y) * CALIBRATE_TRANSLATE_Y * translateSpeed;
            double inputDriveRotation = inputScaling(-currentGamepad1.right_stick_x) * CALIBRATE_ROTATE;
            double yawDegrees = -imu.getRobotYawPitchRollAngles().getYaw();
            //yawDegrees = 0;
            // Rotate joystick X/Y from field-centric to robot-centric coordinates
            double robotDriveX = inputDriveX * cos(toRadians(yawDegrees)) - inputDriveY * sin(toRadians(yawDegrees));
            //robotDriveX = 0;
            double robotDriveY = inputDriveX * sin(toRadians(yawDegrees)) + inputDriveY * cos(toRadians(yawDegrees));

            //This is my weird way of tuning PID values using the controller
            if (currentGamepad1.dpad_down) {
                proportional -= 0.0001;
            }
            if (currentGamepad1.dpad_up) {
                proportional += 0.0001;
            }
            if (currentGamepad1.dpad_left) {
                integral -= 0.0001;
            }
            if (currentGamepad1.dpad_right) {
                integral += 0.0001;
            }
            if (currentGamepad1.y) {
                derivative -= 0.0001;
            }
            if (currentGamepad1.a) {
                derivative += 0.0001;
            }
            //I'm not sure if this is the best way to get PID constants from here into SwerveDriveWheel but it works
            SwerveDrive.drive(robotDriveX, robotDriveY, inputDriveRotation, proportional, integral, derivative);


            //Robot Scoring control
            double liftControl = -inputScaling(currentGamepad2.left_stick_y);
            double intakeSlideControl = inputScaling(currentGamepad2.right_stick_y);
            double intakeShoulderControl = inputScaling(currentGamepad2.right_stick_x);
            double intakeWristControl = inputScaling(currentGamepad2.right_trigger - currentGamepad2.left_trigger);
            if (currentGamepad2.a && !prevGamepad2.a) {
                robotScoring.intakeClawToggle();
            }
            if (currentGamepad2.x && !prevGamepad2.x) {
                robotScoring.upperShoulderPresetWall();
            }
            if (currentGamepad2.b && !prevGamepad2.b) {
                robotScoring.upperShoulderPresetBar();
            }
            if (currentGamepad2.y && !prevGamepad2.y) {
                robotScoring.upperClawToggle();
            }
            if (currentGamepad2.left_bumper && !prevGamepad2.left_bumper) {
                robotScoring.pickupPrepare();
            } else if (currentGamepad2.right_bumper && !prevGamepad2.right_bumper) {
                robotScoring.pickupBegin();
            }
            robotScoring.drive(liftControl, intakeSlideControl, intakeShoulderControl, intakeWristControl);

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