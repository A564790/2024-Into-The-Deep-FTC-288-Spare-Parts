package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        lowerPickupServo = hardwareMap.servo.get("Intake Claw");
        wristServo = hardwareMap.servo.get("Intake Wrist");
        armServo = hardwareMap.servo.get("Intake Elbow");
        shoulderServo = hardwareMap.servo.get("Intake Shoulder");
        upperPickupServo = hardwareMap.servo.get("Top Claw");
        upperArmServo = hardwareMap.servo.get("Top Shoulder");

        elevatorMotorL = hardwareMap.dcMotor.get("Left Lift");
        elevatorMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotorR = hardwareMap.dcMotor.get("Right Lift");
        elevatorMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorR.setDirection(DcMotorSimple.Direction.REVERSE);

        extensionMotor = hardwareMap.dcMotor.get("Extension Slide");
        extensionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private double elevatorTarget = 0.0;
    private double extensionTarget = 0.0;
    private double intakeShoulderTarget = 0.5; // 0.5 = Aligned to Middle
    private double intakeWristTarget = 0; // 0 = Claw Aligned with Extension
    private double intakeClawTarget = 0.0; // 0 = Open, 1 = Closed
    private double intakeElbowTarget = 1.0; // 1 = Inside Robot, 0 = At Floor
    private double upperClawTarget = 1.0; // 1 = Closed, 0 = Open
    private double upperShoulderTarget = 1.0; // 1 = Towards Wall, 0 = Inside Robot

    // True while the intake pickup/handoff state machine is running
    private boolean pickupHandoffStateMachine = false;
    private ElapsedTime pickupHandoffTimer;

    // Blocks can be picked up along their short axis by closing the claw, or along
    // their long axis by _opening_ the claw inside the block. These two modes need
    // different servo angles for the handoff. We assume that if the claw was open
    // when the pickup was initiated it's on the short edge, and the converse.
    private boolean pickupHandoffLongEdge = false;

    public void intakeClawToggle() {
        if (pickupHandoffStateMachine) {
            return; // Ignored while doing pickup/handoff
        }
        intakeClawTarget = 1.0 - intakeClawTarget;
    }
    public void upperShoulderPresetWall() {
        if (pickupHandoffStateMachine) {
            return; // Ignored while doing pickup/handoff
        }
        upperShoulderTarget = 1.0; // TODO: Establish preset angle
    }
    public void upperShoulderPresetBar() {
        if (pickupHandoffStateMachine) {
            return; // Ignored while doing pickup/handoff
        }
        upperShoulderTarget = 0.2; // TODO: Establish preset angle
    }
    public void upperClawToggle() {
        if (pickupHandoffStateMachine) {
            return; // Ignored while doing pickup/handoff
        }
        upperClawTarget = 1.0 - upperClawTarget;
    }
    public void pickupPrepare() {
        pickupHandoffStateMachine = false;
        pickupHandoffTimer = null;
        intakeElbowTarget = 0.0;
    }
    public void pickupBegin() {
        pickupHandoffStateMachine = true;
        pickupHandoffTimer = new ElapsedTime();
        pickupHandoffLongEdge = (intakeClawTarget > 0.5);
    }
    public void drive(double liftControl, double intakeSlideControl, double intakeShoulderControl, double intakeWristControl) {
        if (pickupHandoffStateMachine) {
            telemetry.addData("Pickup State Machine", pickupHandoffTimer.toString());
            this.runPickupStateMachine();
        } else {
            elevatorTarget += liftControl * 75;
            extensionTarget += intakeSlideControl * 50;
            intakeShoulderTarget += 0.015 * intakeShoulderControl;
            intakeWristTarget += 0.05 * intakeWristControl;
        }

        // Clamp encoder count targets to just barely inside the mechanical range
        elevatorTarget = Math.max(0, Math.min(elevatorTarget, 3100));
        extensionTarget = Math.max(0, Math.min(extensionTarget, 1900));

        // Drive DC motors to target values
        elevatorMotorL.setTargetPosition((int) elevatorTarget);
        elevatorMotorR.setTargetPosition((int) elevatorTarget);
        extensionMotor.setTargetPosition((int) extensionTarget);
        elevatorMotorL.setPower(1.0);
        elevatorMotorR.setPower(1.0);
        extensionMotor.setPower(1.0);
        elevatorMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Elevator Target", elevatorTarget);
        telemetry.addData("Elevator Position (L)", elevatorMotorL.getCurrentPosition());
        telemetry.addData("Elevator Position (R)", elevatorMotorR.getCurrentPosition());
        telemetry.addData("Extension Position", extensionMotor.getCurrentPosition());

        // Drive servo motors to target values
        shoulderServo.setPosition(intakeShoulderTarget);
        wristServo.setPosition(intakeWristTarget);
        lowerPickupServo.setPosition(intakeClawTarget);
        armServo.setPosition(intakeElbowTarget);
        upperPickupServo.setPosition(upperClawTarget);
        upperArmServo.setPosition(upperShoulderTarget);
    }

    public void runPickupStateMachine() {
        double t = pickupHandoffTimer.seconds();
        // t *= 0.1; // uncomment to run slower for target value calibration
        if (t < 0.100) {
            intakeElbowTarget = 0.0;
            elevatorTarget = (pickupHandoffLongEdge ? 370 : 400); // TODO: Calibrate handoff height
            upperShoulderTarget = 0.5;
            upperClawTarget = 0.0;
        } else if (t < 0.250) {
            intakeClawTarget = (pickupHandoffLongEdge ? 0.0 : 1.0);
        } else if (t < 0.400) {
            intakeShoulderTarget = 0.5;
            intakeWristTarget = (pickupHandoffLongEdge ? 0.0 : 0.5);
            intakeElbowTarget = 1.0; // TODO: Calibrate handoff angle
        } else if (t < 1.000) {
            extensionTarget = 0; // TODO: Calibrate handoff extension
        } else if (t < 1.300) {
            upperShoulderTarget = 0.0; // TODO: Calibrate handoff angle
        } else if (t < 1.600) {
            upperClawTarget = 1.0;
        } else if (t < 1.800) {
            intakeClawTarget = (pickupHandoffLongEdge ? 1.0 : 0.0);
        } else if (t < 2.800) {
            upperShoulderTarget = 0.8 * (t - 1.8); // smoother linear motion to avoid inertial movement of the game element
        } else {
            pickupHandoffStateMachine = false;
        }
    }

//    public void shoulderDrive(double shoulderIn) {
//
//        if (shoulderIn != 0) {
//            shoulderTarget = (shoulderIn + 1.0) / 2.0;
//            shoulderTarget = Math.min(Math.max(shoulderIn, 0.0), 1.0);
//            shoulderServo.setPosition(shoulderTarget);
//
//            telemetry.addData("lower pickup servo", lowerPickupServo.getPosition());
//            telemetry.addData("wrist servo", lowerPickupServo.getPosition());
//            telemetry.addData("arm servo", lowerPickupServo.getPosition());
//            telemetry.addData("shoulder servo", lowerPickupServo.getPosition());
//            telemetry.addData("upper pickup servo", lowerPickupServo.getPosition());
//
//            telemetry.addData("elevatorMotorR", elevatorMotorR.getCurrentPosition());
//            telemetry.addData("elevatorMotorL", elevatorMotorL.getCurrentPosition());
//
//            telemetry.addData("extensionMotor", extensionMotor.getCurrentPosition());
//        }
//        else {
//            shoulderTarget = 0;
//        }
//
//    }
//    //TODO figure out slide motor stuff. Probably do same run to position stuff as on the previous arm. Copy will's code from RobotArmController
//
//    public void presetIntakePoint(double shoulderInput, double armInput, double wristInput, double lowerPickupInput) {
//        //TODO implement finite state machine logic so all servos move to their assigned positions at the same time
//
//        shoulderServo.setPosition(shoulderInput);
//
//        armServo.setPosition(armInput);
//
//        wristServo.setPosition(wristInput);
//
//        lowerPickupServo.setPosition(lowerPickupInput);
//    }
//
//    public void presetUpperPickupPoint(double upperPickupInput, double upperArmInput) {
//
//        upperPickupServo.setPosition(upperPickupInput);
//        upperArmServo.setPosition(upperArmInput);
//    }
//
//    public void intakePresetPoint1() {
//        this.presetIntakePoint(0,0,0,0);
//    }
//
//    public void intakePresetPoint2() {
//        this.presetIntakePoint(0,0,0,0);
//    }
//
//    public void intakePresetPoint3() {
//        this.presetIntakePoint(0,0,0,0);
//    }
//
//    public void intakePresetPoint4() {
//        this.presetIntakePoint(0,0,0,0);
//    }
//
//
//    public void upperPickupPresetPoint1() {
//        this.presetUpperPickupPoint(0,0);
//    }
//
//    public void upperPickupPresetPoint2() {
//        this.presetUpperPickupPoint(0,0);
//    }
//
//    public void upperPickupPresetPoint3() {
//        this.presetUpperPickupPoint(0,0);
//    }
//
//    public void upperPickupPresetPoint4() {
//        this.presetUpperPickupPoint(0,0);
//    }
//
}
