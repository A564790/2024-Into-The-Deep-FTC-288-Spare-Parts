package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DriveByTime")
public class DriveByTime extends LinearOpMode {

    private SweeDriveWheel LFWheel, LRWheel, RFWheel, RRWheel;
    private SwerveDriveCoordinator SwerveDrive;
    private DigitalChannel limitSwitch;
    private double TRANSLATE_DEFAULT_SPEED = 0.7;
    private double CALIBRATE_TRANSLATE_X = 1.0;
    private double CALIBRATE_TRANSLATE_Y = -1.0; // Gamepad Y axes are inverted from what you'd expect -- down is positive by default. So we negate it here.
    private double CALIBRATE_ROTATE = 1.0;
    double x_direct = 0.0; //These values are passed into the usual swerve code instead of controller inputs.
    double y_direct = 0.0;
    double rotate = 0.0;
    @Override
    public void runOpMode(){
        LFWheel = new SweeDriveWheel(
                telemetry,
                "LF",
                hardwareMap.dcMotor.get("LFDrive"),
                hardwareMap.crservo.get("LFSteer"),
                hardwareMap.analogInput.get("LFsteer")
        );
        LRWheel = new SweeDriveWheel(
                telemetry,
                "LR",
                hardwareMap.dcMotor.get("LRDrive"),
                hardwareMap.crservo.get("LRSteer"),
                hardwareMap.analogInput.get("LRsteer")
        );
        RFWheel = new SweeDriveWheel(
                telemetry,
                "RF",
                hardwareMap.dcMotor.get("RFDrive"),
                hardwareMap.crservo.get("RFSteer"),
                hardwareMap.analogInput.get("RFsteer")
        );
        RRWheel = new SweeDriveWheel(
                telemetry,
                "RR",
                hardwareMap.dcMotor.get("RRDrive"),
                hardwareMap.crservo.get("RRSteer"),
                hardwareMap.analogInput.get("RRsteer")
        );


        SwerveDrive = new SwerveDriveCoordinator(telemetry, LFWheel, LRWheel, RFWheel, RRWheel);
        scoringController robotScoring = new scoringController(telemetry, hardwareMap);

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();//THE SOURCE OF ALL JANK!! START TIMER AFTER START RATHER THAN INIT FOR CONSISTENCY
        while (opModeIsActive()){
            double translateSpeed = TRANSLATE_DEFAULT_SPEED;
            double inputDriveX = x_direct * CALIBRATE_TRANSLATE_X * translateSpeed;
            double inputDriveY = y_direct * CALIBRATE_TRANSLATE_Y * translateSpeed;
            double inputDriveRotation = rotate * CALIBRATE_ROTATE;



            if (runtime.seconds()<2) {//section that gets arm in pre score position
                robotScoring.upperShoulderPresetBar();
                robotScoring.drive(0.1, 0.0, 0.0, 0.0, false);
            }
            if (runtime.seconds()>3 && runtime.seconds()<4){//drive for 2 seconds to bar
                y_direct = 0.8;
                SwerveDrive.drive(inputDriveX, inputDriveY, inputDriveRotation);
            }

            if (runtime.seconds()>5 && runtime.seconds()<8){//score on bar
                y_direct = 0.0;
                SwerveDrive.drive(inputDriveX, inputDriveY, inputDriveRotation);
                robotScoring.elevatorScore();
                robotScoring.drive(0.75, 0.0, 0.0, 0.0, false);
            }

            if (runtime.seconds()>8 && runtime.seconds() < 11){//release from bar and drive away
                robotScoring.upperClawToggle();
                robotScoring.drive(0.0, 0.0, 0.0, 0.0, false);
                y_direct = -0.8;
                SwerveDrive.drive(inputDriveX, inputDriveY, inputDriveRotation);
            }
            if (runtime.seconds()>11 && runtime.seconds() < 13){//Strafe into park zone
                x_direct = -0.5;
                y_direct = 0.0;
                SwerveDrive.drive(inputDriveX, inputDriveY, inputDriveRotation);
            }
            if (runtime.seconds()>13 && runtime.seconds()>14) { //Stop and lower elevator and put arm in wall position
                x_direct = 0.0;
                SwerveDrive.drive(inputDriveX, inputDriveY, inputDriveRotation);
                robotScoring.upperShoulderPresetWall();
                robotScoring.drive(-0.5, 0.0, 0.0, 0.0, false);
            }
            if (runtime.seconds()>14){//stop lowering elevator
                robotScoring.drive(0.0, 0.0, 0.0, 0.0, false);
            }


            telemetry.addData("Elapsed time: ", runtime.seconds());
            telemetry.update();
        }




    }
}
