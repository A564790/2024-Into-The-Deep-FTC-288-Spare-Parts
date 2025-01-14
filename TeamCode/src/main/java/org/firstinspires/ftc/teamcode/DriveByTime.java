package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


@Autonomous(name = "DriveByTime")
public class DriveByTime extends LinearOpMode {

    private DigitalChannel limitSwitch;

    final double JOYSTICK_DEAD_ZONE = 0.20;
    final double JOYSTICK_MOVEMENT_SENSITIVITY = 0.75;
    final double JOYSTICK_ROTATION_SENSITIVITY = 1.00;
    private double x_direct; //Positive is right on joystick, negative is left
    private double y_direct;//positive is forward on joystick, negative is backward
    //adjust x_direct and y_direct accordingly for speed and direction
//input scaling and other joystick code is here to automatically calculate motor direction appropriately and simplify each direction command
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

    IMU imu;

    @Override
    public void runOpMode(){
        imu = hardwareMap.get(IMU.class, "imu");


        driveController mechDrive = new driveController(telemetry, hardwareMap);
        scoringController robotScoring = new scoringController(telemetry, hardwareMap);
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );


        imu.initialize(imuParams);
        waitForStart();
        ElapsedTime runtime = new ElapsedTime();// START TIMER AFTER START RATHER THAN INIT FOR CONSISTENCY
        while (opModeIsActive()){

            double joystickMovementY = inputScaling(y_direct) * JOYSTICK_MOVEMENT_SENSITIVITY;  // Note: pushing stick forward gives negative value
            double joystickMovementX = inputScaling(x_direct) * JOYSTICK_MOVEMENT_SENSITIVITY;
            double yaw = (inputScaling(0) * JOYSTICK_ROTATION_SENSITIVITY) * 0.75;

            //get robot orientation from imu
            double robotHeading = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            //input movement values into vector translation in 2d theorem
            double theta = -robotHeading;
            double movementX = joystickMovementX * cos(toRadians(theta)) - joystickMovementY * sin(toRadians(theta));
            double movementY = joystickMovementX * sin(toRadians(theta)) + joystickMovementY * cos(toRadians(theta));

            if (gamepad1.left_trigger > 0.000) {
                movementX = movementX * 0.45;
                movementY = movementY * 0.45;
                yaw = yaw * 0.45;
            }
            if (gamepad1.left_trigger > 0.000 && gamepad1.left_trigger < 0.001) {
                movementX = movementX / 0.45;
                movementY = movementY / 0.45;
                yaw = yaw / 0.45;
            }

            double leftFrontPower = (movementY + movementX + yaw);
            double rightFrontPower = (movementY - movementX - yaw);
            double leftBackPower = (movementY - movementX + yaw);
            double rightBackPower = (movementY + movementX - yaw);

            //normalize power variables to prevent motor power from exceeding 1.0
            double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            maxPower = Math.max(maxPower, Math.abs(leftBackPower));
            maxPower = Math.max(maxPower, Math.abs(rightBackPower));
            if (maxPower > 1.0) {
                leftFrontPower /= maxPower;
                rightFrontPower /= maxPower;
                leftBackPower /= maxPower;
                rightBackPower /= maxPower;

            }



            if (runtime.seconds()<2) {//section that gets arm in pre score position
                //robotScoring.upperShoulderPresetBar();
                //robotScoring.drive(0.27, 0.0, 0.0, 0.0, false);
                // goes up to 1404
            }

            //TODO: Adjust x and y direct values and time to account for faster and lighter drivetrain
            if (runtime.seconds()>3 && runtime.seconds()<4){//drive for 2 seconds to bar
                y_direct = 0.8;
                mechDrive.drive(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }

            if (runtime.seconds()>5 && runtime.seconds()<8){//score on bar
                y_direct = 0.0;
                mechDrive.drive(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                //robotScoring.elevatorScore();
                //robotScoring.drive(0.75, 0.0, 0.0, 0.0, false);
                //goes up to 1460.25
            }

            if (runtime.seconds()>8 && runtime.seconds() < 11){//release from bar and drive away
                //robotScoring.upperClawToggle();
                //robotScoring.drive(0.0, 0.0, 0.0, 0.0, false);
                //doesn't move
                y_direct = -0.8;
                mechDrive.drive(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }
            if (runtime.seconds()>11 && runtime.seconds() < 13){//Strafe into park zone
                x_direct = -0.5;
                y_direct = 0.0;
                mechDrive.drive(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
            }
            if (runtime.seconds()>13 && runtime.seconds()>14) { //Stop and lower elevator and put arm in wall position
                x_direct = 0.0;
                mechDrive.drive(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
                //robotScoring.upperShoulderPresetWall();
                //robotScoring.drive(-0.7, 0.0, 0.0, 0.0, false);
                //goes to 565.5
            }
            if (runtime.seconds()>14){//stop lowering elevator
                //robotScoring.drive(0.0, 0.0, 0.0, 0.0, false);
            }




            telemetry.addData("Elapsed time: ", runtime.seconds());
            telemetry.update();
        }




    }
}
