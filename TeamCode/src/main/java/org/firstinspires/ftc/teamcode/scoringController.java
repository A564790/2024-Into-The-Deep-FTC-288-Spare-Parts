package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class scoringController {

    private Telemetry telemetry;

    private double shoulderTarget = 0;

    private TouchSensor limitSwitch;

    private Servo lowerPickupServo, wristServo, armServo, shoulderServo, upperPickupServo, upperArmServo;

    private DcMotor elevatorMotorR, elevatorMotorL, extensionMotor;

    private double maximumVerticalExtension = 0;
    private double maximumVerticalRetraction = 0;

    private double maximumHorizontalExtention = 0;
    private double maximumHorizontalRetraction = 0;

    double iterations = 0;


    public scoringController(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;

        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");


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
    private double intakeWristTarget = 0.5; // 0 = Claw Aligned with Extension
    private double intakeClawTarget = 0.0; // 0 = Open, 1 = Closed
    private double intakeElbowTarget = 1.0; // 1 = Inside Robot, 0 = At Floor
    private double upperClawTarget = 1.0; // 1 = Closed, 0 = Open
    private double upperShoulderTarget = 1.0; // 1 = Towards Wall, 0 = Inside Robot

    // True while the intake pickup/handoff state machine is running
    private boolean pickupHandoffStateMachineZero = false;
    private boolean pickupHandoffStateMachine = false;
    private boolean pickupHandoffStateMachineTwo = false;
    private boolean stateMachineActive = false;
    private ElapsedTime slowDownTimer;
    private ElapsedTime pickupHandoffTimerZero;
    private ElapsedTime pickupHandoffTimer;
    private ElapsedTime pickupHandoffTimerTwo;

    // Blocks can be picked up along their short axis by closing the claw, or along
    // their long axis by _opening_ the claw inside the block. These two modes need
    // different servo angles for the handoff. We assume that if the claw was open
    // when the pickup was initiated it's on the short edge, and the converse.
    private boolean pickupHandoffLongEdge = false;

    public void intakeClawToggle() {
        if (stateMachineActive) {
            return; // Ignored while doing pickup/handoff
        }
        intakeClawTarget = 1.0 - intakeClawTarget;
    }
    public void uppShoulderPresetBasket() {
        if (stateMachineActive) {
            return;
        }
        upperShoulderTarget = 0.35;
    }
    public void upperShoulderPresetWall() {
        if (stateMachineActive) {
            return; // Ignored while doing pickup/handoff
        }
        upperShoulderTarget = 1.0;
        elevatorTarget = 600;
    }
    /* public void upperShoulderPresetBar() {
        if (pickupHandoffStateMachine || pickupHandoffStateMachineTwo) {
            return; // Ignored while doing pickup/handoff
        }
        upperShoulderTarget = 0.25;
    }

     */
    public void elbowRetract() {
        if (stateMachineActive) {
            return;
        }
        intakeElbowTarget = 1.0;
        intakeWristTarget = 0.5;
    }
    public void elevatorScore() {
        if (stateMachineActive) {
            return;
        }
        upperShoulderTarget = 0.25;
        elevatorTarget = 1450;
    }
    public void upperClawToggle() {
        if (stateMachineActive) {
            return; // Ignored while doing pickup/handoff
        }
        upperClawTarget = 1.0 - upperClawTarget;
    }
    public void upperClawPosition(double position) {
        if (stateMachineActive) {
            return;
        }
        upperClawTarget = position;
    }
    public void upperArmPosition(double Position) {
        if (stateMachineActive) {
            return;
        }
        upperShoulderTarget = Position;
    }
    public void pickupPrepare() {
        pickupHandoffStateMachineZero = false;
        pickupHandoffStateMachine = false;
        pickupHandoffStateMachineTwo = false;
        stateMachineActive = false;
        pickupHandoffTimerZero = null;
        pickupHandoffTimer = null;
        pickupHandoffTimerTwo = null;
        intakeElbowTarget = 0.05;
    }
    public void chickenPecker() {
        pickupHandoffStateMachineZero = true;
        pickupHandoffTimerZero = new ElapsedTime();
        pickupHandoffLongEdge = (intakeClawTarget > 0.5);
    }
    public void pickupStageOne() {
        stateMachineActive = true;
        pickupHandoffStateMachine = true;
        pickupHandoffTimer = new ElapsedTime();
    }
    public void pickupStageTwo() {
        stateMachineActive = true;
        pickupHandoffStateMachineTwo = true;
        pickupHandoffTimerTwo = new ElapsedTime();
    }

    public void drive(double liftControl, double intakeSlideControl, double intakeShoulderControl, double intakeWristControl, boolean override) {
        if (pickupHandoffStateMachineZero) {
            telemetry.addData("Pickup State Machine Stage 0", pickupHandoffTimerZero.toString());
            this.runPickupStateMachineStageZero();
        }
        if (pickupHandoffStateMachine) {
            telemetry.addData("Pickup State Machine Stage 1", pickupHandoffTimer.toString());
            this.runPickupStateMachineStageOne();
        }
        if (pickupHandoffStateMachineTwo) {
            telemetry.addData("Pickup State Machine Stage 2", pickupHandoffTimerTwo.toString());
            this.runPickupStateMachineStageTwo();
        }
        else {
            elevatorTarget += liftControl * 75;
            extensionTarget += intakeSlideControl * 50;
            intakeShoulderTarget += 0.015 * intakeShoulderControl;
            intakeShoulderTarget = Math.max(0, Math.min(1.0, intakeShoulderTarget));
            intakeWristTarget += 0.05 * intakeWristControl;
            intakeWristTarget = Math.max(0, Math.min(1.0, intakeWristTarget));
        }

        // Clamp encoder count targets to just barely inside the mechanical range
        extensionTarget = Math.max(0, Math.min(extensionTarget, 2170));

       /* // Drive DC motors to target values
        elevatorMotorL.setTargetPosition((int) elevatorTarget);
        elevatorMotorR.setTargetPosition((int) elevatorTarget);
        extensionMotor.setTargetPosition((int) extensionTarget);
        elevatorMotorL.setPower(1.0);
        elevatorMotorR.setPower(1.0);
        extensionMotor.setPower(1.0);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        */

        if (limitSwitch.isPressed()) {
            elevatorMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elevatorMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if  (!limitSwitch.isPressed() && override) {
            elevatorTarget = Math.max(-3000, Math.min(elevatorTarget, 3100));
            elevatorTarget -= 75;
            elevatorMotorR.setTargetPosition((int)elevatorTarget);
            elevatorMotorL.setTargetPosition((int)elevatorTarget);
            iterations += 1;
            telemetry.addData("limitSwitch", iterations);
        }
        else {
            // Drive DC motors to target values
            elevatorTarget = Math.max(0, Math.min(elevatorTarget, 3100)); // 5 ticks = roughly 1/16 of an inch
            elevatorMotorL.setTargetPosition((int) elevatorTarget);
            elevatorMotorR.setTargetPosition((int) elevatorTarget);
            extensionMotor.setTargetPosition((int) extensionTarget);
            elevatorMotorL.setPower(1.0);
            elevatorMotorR.setPower(1.0);
            extensionMotor.setPower(1.0);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        telemetry.addData("LimitSwitch", limitSwitch.isPressed());
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

    public void runPickupStateMachineStageZero() {
        double x = pickupHandoffTimerZero.seconds();
        if (x < 0.100) {
            intakeElbowTarget = 0.0;
        } else if (x < 0.250) {
            intakeClawTarget = (pickupHandoffLongEdge ? 0.0 : 1.0);}
        else if (x < 0.8) {
            intakeElbowTarget = 0.23;
        }
    }
    public void runPickupStateMachineStageOne() {
        double t = pickupHandoffTimer.seconds();
         t *= 0.5; // uncomment to run slower for target value calibration
        if (t < 0.100) {
            elevatorTarget = (pickupHandoffLongEdge ? 780 : 795);
            upperClawTarget = 0.0;
        }  else if (t < 0.400) {
            intakeShoulderTarget = 0.5;
            intakeWristTarget = (pickupHandoffLongEdge ? 0.0 : 0.5);
            intakeElbowTarget = 0.8;
        } else if (t < 1.000) {
            extensionTarget = 0;
        } else {
            stateMachineActive = false;
        }
    }
    public void runPickupStateMachineStageTwo() {
        double T = pickupHandoffTimerTwo.seconds();
        if (T < 1.0) {
            upperShoulderTarget = 1.0 - T;  // Smoothly decrease from 1.0 to 0.0 over 0.9 seconds
        }  else if (T < 1.1) {
        upperShoulderTarget = 0.0;
        } else if (T < 1.4) {
            upperClawTarget = 1.0;
        } else if (T < 1.7) {
            intakeClawTarget = (pickupHandoffLongEdge ? 1.0 : 0.0);
        } else if (T < 2.7) {
            upperShoulderTarget = 0.35;
        } else {
            stateMachineActive = false;
            pickupHandoffStateMachine = false;
            pickupHandoffStateMachineTwo = false;
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
