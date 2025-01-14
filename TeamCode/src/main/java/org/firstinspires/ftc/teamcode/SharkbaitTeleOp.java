/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.constants.TeleOpServoConstants;

import java.lang.Math;
import java.util.TimerTask;
import java.util.Timer;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Sharkbait TeleOp")

public class SharkbaitTeleOp extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor BRMotor = null;
    
    private DcMotor InSlide = null;
    private DcMotor OtSlideF = null;
    private DcMotor OtSlideM = null;
    private DcMotor OtSlideB = null;
    
    private Servo InLDiffyServo = null;
    private Servo InRDiffyServo = null;
    private Servo InLeftElbowServo = null;
    private Servo InRightElbowServo = null;
    private Servo InClawServo = null;
    
    private Servo OtClawServo = null;
    private Servo OtLeftElbowServo = null;
    private Servo OtRightElbowServo = null;
    private Servo OtCoaxialServo = null;
    private Servo OtLinkageLServo = null;
    private Servo OtLinkageRServo = null;
    private Limelight3A litty = null;

    private int intakeIndex = 0;
    private int outtakeIndex = 0;

    private int diffyRotateIndex = 0;

    private boolean inClawDelay = false;
    private boolean otClawDelay = false;
    private boolean diffyRotateDelay = false;
    private boolean inClawOpened = true;
    private boolean otClawOpened = true;

    private boolean oldCrossPressed = true;
    private boolean oldTrianglePressed = true;
    private boolean oldCirclePressed = true;
    private boolean oldSquarePressed = true;
    private boolean oldUpDpadPressed = true;
    private boolean oldDownDpadPressed = true;
    private boolean oldLeftDpadPressed = true;
    private boolean oldRightDpadPressed = true;
    private boolean oldLeftBumper = true;
    private boolean oldRightBumper = true;
    private double oldLeftTrigger = 0.0;
    private double oldRightTrigger = 0.0;

    
    private boolean isInArmMoving = false;
    private boolean isOtArmMoving = false;
    
    private boolean isInDiffyMoving = false;
    private boolean isOtCoaxialMoving = false;

    private double[] ILEServoPositions = TeleOpServoConstants.ILEServoPositions;
    private double[] IREServoPositions = TeleOpServoConstants.IREServoPositions;
    private double[] OLEServoPositions = TeleOpServoConstants.OLEServoPositions;
    private double[] OREServoPositions = TeleOpServoConstants.OREServoPositions;
    private double[] LDServoPositions = TeleOpServoConstants.LDServoPositions;
    private double[] RDServoPositions = TeleOpServoConstants.RDServoPositions;
    private double[] OAServoPositions = TeleOpServoConstants.OAServoPositions;
    private double[] ICServoPositions = TeleOpServoConstants.ICServoPositions;
    private double[] OCServoPositions = TeleOpServoConstants.OCServoPositions;
    private double[] OKLServoPositions = TeleOpServoConstants.OKLServoPositions;
    private double[] OKRServoPositions = TeleOpServoConstants.OKRServoPositions;
    
    private double[] SlowModeSpeed = TeleOpServoConstants.SlowModeSpeed;

    private int inPosition = 1;


    private final int DELAY_BETWEEN_MOVES = 100;


    @Override
    public void runOpMode() {

        class setIsInArmMoving extends TimerTask {
            boolean val;
            public setIsInArmMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isInArmMoving = val;
            }
        }
        class setIsOtArmMoving extends TimerTask {
            boolean val;
            public setIsOtArmMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isOtArmMoving = val;
            }
        }
        class setIsOtCoaxialMoving extends TimerTask {
            boolean val;

            public setIsOtCoaxialMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isOtCoaxialMoving = val;
            }
        }
        class setInClawDelay extends TimerTask {
            boolean val;
            public setInClawDelay(boolean v) { this.val = v; }
            public void run() {
                inClawDelay = val;
            }
        }


        class setIsInDiffyMoving extends TimerTask {
            boolean val;
            public setIsInDiffyMoving(boolean v) {
                this.val = v;
            }
            public void run() {
                isInDiffyMoving = val;
            }
        }

        class setOtClawDelay extends TimerTask {
            boolean val;
            public setOtClawDelay(boolean v) {
                this.val = v;
            }
            public void run() {
                otClawDelay = val;
            }
        }

        class setDiffyRotateDelay extends TimerTask {
            boolean val;
            public setDiffyRotateDelay(boolean v) {
                this.val = v;
            }
            public void run() {
                diffyRotateDelay = val;
            }
        }
         class OuttakeExtension extends TimerTask {
            double i;
            public OuttakeExtension(double i) { this.i = i; }
            public void run() {
                OtSlideF.setPower(i);
                OtSlideM.setPower(i);
                OtSlideB.setPower(i);
            }
        }
        class MoveInElbowServosPosition extends TimerTask {
            int i;
            public MoveInElbowServosPosition(int i) {
                this.i = i;
            }
            public void run() {
                InLeftElbowServo.setPosition(ILEServoPositions[i]);
                InRightElbowServo.setPosition(IREServoPositions[i]);

                telemetry.addData("intakeIndex", i);
                telemetry.update();
                intakeIndex = i;
            }
        }
        class MoveOtElbowServosPosition extends TimerTask {
            int i;
            public MoveOtElbowServosPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtLeftElbowServo.setPosition(OLEServoPositions[i]);
                OtRightElbowServo.setPosition(OREServoPositions[i]);

                telemetry.addData("intakeIndex", i);
                telemetry.update();
                outtakeIndex = i;
            }
        }

        class MoveInDiffyServoPosition extends TimerTask {
            int i;
            double k;
            public MoveInDiffyServoPosition(int i) { this.i = i; }
            public MoveInDiffyServoPosition(double k) { this.k = k; }
            public void run() {
                InLDiffyServo.setPosition(LDServoPositions[i]);
                InRDiffyServo.setPosition(RDServoPositions[i]);


                //InLDiffyServo.setPosition(InLDiffyServo.getPosition() + i);
                //InRDiffyServo.setPosition(InRDiffyServo.getPosition() + i);
            }
        }

        class MoveOtCoaxialServoPosition extends TimerTask {
            int i;
            public MoveOtCoaxialServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtCoaxialServo.setPosition(OAServoPositions[i]);
            }
        }
        class MoveOtLinkageServoPosition extends TimerTask {
            int i;
            public MoveOtLinkageServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtLinkageLServo.setPosition(OKLServoPositions[i]);
                OtLinkageRServo.setPosition(OKRServoPositions[i]);
            }
        }

        class MoveOtClawServoPosition extends TimerTask {
            int i;
            public MoveOtClawServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                OtClawServo.setPosition(OCServoPositions[i]);
            }
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        InSlide = hardwareMap.get(DcMotor.class, "IS");
        OtSlideF = hardwareMap.get(DcMotor.class, "OF");
        OtSlideM = hardwareMap.get(DcMotor.class, "OM");
        OtSlideB = hardwareMap.get(DcMotor.class, "OB");
        
        InLeftElbowServo = hardwareMap.get(Servo.class, "ILE");
        InRightElbowServo = hardwareMap.get(Servo.class, "IRE");
        InClawServo = hardwareMap.get(Servo.class, "IC");
        InLDiffyServo = hardwareMap.get(Servo.class, "LD");
        InRDiffyServo = hardwareMap.get(Servo.class, "RD");

        OtLeftElbowServo = hardwareMap.get(Servo.class, "OLE");
        OtRightElbowServo = hardwareMap.get(Servo.class, "ORE");
        OtClawServo = hardwareMap.get(Servo.class, "OC");
        OtCoaxialServo = hardwareMap.get(Servo.class, "OA");
        OtLinkageLServo = hardwareMap.get(Servo.class, "OKL");
        OtLinkageRServo = hardwareMap.get(Servo.class, "OKR");
        Timer timer = new Timer();

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        InSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //In the future will need code for Encoders on the Motors

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);

        InSlide.setDirection(DcMotor.Direction.REVERSE);
        OtSlideF.setDirection(DcMotor.Direction.REVERSE);
        OtSlideM.setDirection(DcMotor.Direction.FORWARD);
        OtSlideB.setDirection(DcMotor.Direction.REVERSE);

        OtSlideB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        OtSlideB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        
        InLeftElbowServo.setDirection(Servo.Direction.REVERSE);
        InRightElbowServo.setDirection(Servo.Direction.FORWARD);
        OtLeftElbowServo.setDirection(Servo.Direction.FORWARD);
        OtRightElbowServo.setDirection(Servo.Direction.FORWARD);
        
        InClawServo.setDirection(Servo.Direction.REVERSE);
        OtClawServo.setDirection(Servo.Direction.REVERSE);

        OtCoaxialServo.setDirection(Servo.Direction.FORWARD);
        OtLinkageLServo.setDirection(Servo.Direction.REVERSE);
        OtLinkageRServo.setDirection(Servo.Direction.FORWARD);
        
        InLDiffyServo.setDirection(Servo.Direction.FORWARD);
        InRDiffyServo.setDirection(Servo.Direction.REVERSE);

        

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*litty = hardwareMap.get(Limelight3A.class, "pipeline");
        litty.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        telemetry.setMsTransmissionInterval(11);
        litty.pipelineSwitch(2);
        litty.start();*/

        InLDiffyServo.setPosition(LDServoPositions[2]);//3 no auto
        InRDiffyServo.setPosition(RDServoPositions[2]);//3 no auto
        InClawServo.setPosition(ICServoPositions[0]);
        OtClawServo.setPosition(OCServoPositions[0]);
        OtLinkageLServo.setPosition(OKLServoPositions[0]);
        OtLinkageRServo.setPosition(OKRServoPositions[0]);
        InLeftElbowServo.setPosition(ILEServoPositions[2]);//3 no auto
        InRightElbowServo.setPosition(IREServoPositions[2]);//3 no auto
        OtLeftElbowServo.setPosition(OLEServoPositions[0]);
        OtRightElbowServo.setPosition(OREServoPositions[0]);
        OtCoaxialServo.setPosition(OAServoPositions[0]);
        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
//        InLeftElbowServo.setPosition(ILEServoPositions[1]);
//        InRightElbowServo.setPosition(IREServoPositions[1]);
//        InLDiffyServo.setPosition(LDServoPositions[1]);
//        InRDiffyServo.setPosition(RDServoPositions[1]);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = gamepad1.right_stick_x;
            final double FLPower = r * Math.cos(robotAngle) + rightX;
            final double FRPower = r * Math.sin(robotAngle) - rightX;
            final double BLPower = r * Math.sin(robotAngle) + rightX;
            final double BRPower = r * Math.cos(robotAngle) - rightX;

            // Send calculated power to wheels
            double rt = gamepad1.right_trigger;
            double lt = gamepad1.left_trigger;
            if(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right) {
                if(gamepad1.dpad_up) { //all positive
                    FLMotor.setPower(SlowModeSpeed[1]);
                    FRMotor.setPower(SlowModeSpeed[1]);
                    BLMotor.setPower(SlowModeSpeed[1]);
                    BRMotor.setPower(SlowModeSpeed[1]);
                } else if (gamepad1.dpad_down){ //all negative
                    FLMotor.setPower(SlowModeSpeed[0]);
                    FRMotor.setPower(SlowModeSpeed[0]);
                    BLMotor.setPower(SlowModeSpeed[0]);
                    BRMotor.setPower(SlowModeSpeed[0]);
                } else if (gamepad1.dpad_left){
                    FLMotor.setPower(SlowModeSpeed[0] * 1.15);
                    FRMotor.setPower(SlowModeSpeed[1] * 1.15);
                    BLMotor.setPower(SlowModeSpeed[1]);
                    BRMotor.setPower(SlowModeSpeed[0]);
                } else if (gamepad1.dpad_right){
                    FLMotor.setPower(SlowModeSpeed[1] * 1.15);
                    FRMotor.setPower(SlowModeSpeed[0] * 1.15);
                    BLMotor.setPower(SlowModeSpeed[0]);
                    BRMotor.setPower(SlowModeSpeed[1]);
                }
            } else if (rt > 0.2 || lt > 0.2) {
                if (-gamepad1.right_stick_y > 0) {
                    if (rt > 0.2) {
                        FLMotor.setPower(rt);
                        FRMotor.setPower(-rt + Math.abs(gamepad1.right_stick_y));
                        BLMotor.setPower(-rt + Math.abs(gamepad1.right_stick_y));
                        BRMotor.setPower(rt);
                    } else {
                        FLMotor.setPower(-lt + Math.abs(gamepad1.right_stick_y));
                        FRMotor.setPower(lt);
                        BLMotor.setPower(lt);
                        BRMotor.setPower(-lt + Math.abs(gamepad1.right_stick_y));
                    }
                } else {
                    if (rt > 0.2) {
                        FLMotor.setPower(rt - Math.abs(gamepad1.right_stick_y));
                        FRMotor.setPower(-rt);
                        BLMotor.setPower(-rt);
                        BRMotor.setPower(rt - Math.abs(gamepad1.right_stick_y));
                    } else {
                        FLMotor.setPower(-lt);
                        FRMotor.setPower(lt - Math.abs(gamepad1.right_stick_y));
                        BLMotor.setPower(lt - Math.abs(gamepad1.right_stick_y));
                        BRMotor.setPower(-lt);
                    }
                }
            } else {
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }

            InSlide.setPower(gamepad2.left_stick_y * 0.75);
            OtSlideF.setPower(gamepad2.right_stick_y);
            OtSlideM.setPower(gamepad2.right_stick_y);
            OtSlideB.setPower(gamepad2.right_stick_y);
            telemetry.addData("praying for a new life", OtSlideB.getCurrentPosition());

            boolean circlePressed = gamepad2.circle;
            boolean trianglePressed = gamepad2.triangle;
            boolean squarePressed = gamepad2.square;
            boolean crossPressed = gamepad2.cross;
            boolean dpadUpPressed = gamepad2.dpad_up;
            boolean dpadDownPressed = gamepad2.dpad_down;
            boolean dpadLeftPressed = gamepad2.dpad_left;
            boolean dpadRightPressed = gamepad2.dpad_right;
            boolean leftBumper = gamepad2.left_bumper;
            boolean rightBumper = gamepad2.right_bumper;
            double leftTrigger = gamepad2.left_trigger;
            double rightTrigger = gamepad2.right_trigger;
            

            if (leftBumper && !inClawDelay && !oldLeftBumper) { //close Intake claw
                new setInClawDelay(true).run();
                if (inClawOpened) { InClawServo.setPosition(ICServoPositions[1]); } //open Intake Claw
                else { InClawServo.setPosition(ICServoPositions[0]); }//close Intake Claw
                timer.schedule(new setInClawDelay(false), 3 * DELAY_BETWEEN_MOVES);
                inClawOpened = !inClawOpened;
            }

            if (rightBumper && !otClawDelay && !oldRightBumper) { //close Outtake claw
                new setOtClawDelay(true).run();
                if (outtakeIndex == 3 && otClawOpened) {
                    timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtCoaxialServoPosition(0), 4 *DELAY_BETWEEN_MOVES);
                } else if (outtakeIndex == 1 & !otClawOpened) {
                    timer.schedule(new MoveOtElbowServosPosition(6), 2 *DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtCoaxialServoPosition(5), 0 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtClawServoPosition(0), 11 * DELAY_BETWEEN_MOVES);
                    timer.schedule(new MoveOtLinkageServoPosition(2), 5 * DELAY_BETWEEN_MOVES);
                } else {
                    if (otClawOpened){ OtClawServo.setPosition(OCServoPositions[1]); } // close Outtake claw
                    else{ OtClawServo.setPosition(OCServoPositions[0]); } //open Outtake claw
                }
                timer.schedule(new setOtClawDelay(false), 3 * DELAY_BETWEEN_MOVES);
                otClawOpened = !otClawOpened;
            }
            
            /*if (crossPressed && !oldCrossPressed && !isOtArmMoving) { //pickup sample from intake outtakeIndex = 0
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtCoaxialServoPosition(4), 2 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(4), 4 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(2), 8 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(5),  14* DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(5), 16 * DELAY_BETWEEN_MOVES);
//                if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 6 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtClawServoPosition(1), 16 * DELAY_BETWEEN_MOVES);


                timer.schedule(new setIsOtArmMoving(false), 18 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 18 *DELAY_BETWEEN_MOVES);
                
            } else */if (circlePressed && !oldCirclePressed && !isOtArmMoving) { //pickup specimen from wall outtakeIndex = 3
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(0), 0 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(3), 6 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(3), 8 *DELAY_BETWEEN_MOVES);
               if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 11 * DELAY_BETWEEN_MOVES);


                timer.schedule(new setIsOtArmMoving(false), 12 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 12 *DELAY_BETWEEN_MOVES);

            } else if (trianglePressed && !oldTrianglePressed && !isOtArmMoving) { //outtake from back of robot outtakeIndex = 2
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(0), 0 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(1), 6 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(1), 8 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(2), 8 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(2), 10 *DELAY_BETWEEN_MOVES);
                if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 15 * DELAY_BETWEEN_MOVES);


                timer.schedule(new setIsOtArmMoving(false), 14 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 14 *DELAY_BETWEEN_MOVES);

            } else if (squarePressed && !oldSquarePressed && !isOtArmMoving) { //outtake from front of robot outtakeIndex = 1
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtCoaxialServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(1), 2 *DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtLinkageServoPosition(1), 10 *DELAY_BETWEEN_MOVES);
                if (otClawOpened) timer.schedule(new MoveOtClawServoPosition(0), 12 * DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsOtArmMoving(false), 12 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 12 *DELAY_BETWEEN_MOVES);

            }
            
            if (dpadDownPressed && !oldDownDpadPressed && !isInArmMoving) { //intake position intakeIndex = 0
                new setIsInArmMoving(true).run();
                //new setIsInDiffyMoving(true).run();
                //timer.schedule(new MoveInDiffyServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                //timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                inPosition = 0;
            } else if (dpadRightPressed && !oldRightDpadPressed && !isInArmMoving) { //position for getting over submersible walls intakeIndex = 2
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                    timer.schedule(new MoveInDiffyServoPosition(2), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(2), 2 *DELAY_BETWEEN_MOVES);


                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                inPosition = 2;
            } else if (dpadLeftPressed && !oldLeftDpadPressed && !isInArmMoving) { //position for scanning for samples intakeIndex = 1
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                LDServoPositions[1] = 0.5;
                RDServoPositions[1] = 0.5;
                timer.schedule(new MoveInDiffyServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(1), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                inPosition = 1;
            } /*else if (dpadUpPressed && !oldUpDpadPressed && !isInArmMoving) { //position for transferring sample to outtake intakeIndex = 3
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                timer.schedule(new MoveInDiffyServoPosition(3), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(3), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                inPosition = 3;
            }
            */
            if (inPosition == 1 || inPosition == 0){
                if(leftTrigger > 0.1 /*&& oldLeftTrigger < 0.2 && !diffyRotateDelay*/) {
                    new setIsInDiffyMoving(true).run();
                    new setDiffyRotateDelay(true).run();
                    if(LDServoPositions[1] >= 0.02 && RDServoPositions[1] <= 0.98) {
                        LDServoPositions[1] = LDServoPositions[1] - 0.005;
                        RDServoPositions[1] = RDServoPositions[1] + 0.005;
                    }
//                    LDServoPositions[1] = Math.max(0, LDServoPositions[1] - 0.15);
//                    RDServoPositions[1] = Math.min(1, RDServoPositions[1] + 0.15);
                    new MoveInDiffyServoPosition(1).run();
                    timer.schedule(new setIsInDiffyMoving(false), 10);
                    timer.schedule(new setDiffyRotateDelay(false), 10);
                }else if(rightTrigger > 0.1 /*&& oldRightTrigger < 0.2 && !diffyRotateDelay*/){
                    new setIsInDiffyMoving(true).run();
                    new setDiffyRotateDelay(true).run();
                    if(LDServoPositions[1] <= 0.98 && RDServoPositions[1] >= 0.02) {
                        LDServoPositions[1] = LDServoPositions[1] + 0.005;
                        RDServoPositions[1] = RDServoPositions[1] - 0.005;
                    }
//                    LDServoPositions[1] = Math.min(1, LDServoPositions[1] + 0.15);
//                    RDServoPositions[1] = Math.max(0, RDServoPositions[1] - 0.15);
                    new MoveInDiffyServoPosition(1).run();
                    timer.schedule(new setIsInDiffyMoving(false), 10);
                    timer.schedule(new setDiffyRotateDelay(false), 10);
                }
                /*
                litty.pipelineSwitch(0);//red
//                    litty.pipelineSwitch(1);//blue
                LLResult seeingStuff = litty.getLatestResult();
                if (seeingStuff != null && seeingStuff.isValid()) {

                    LLResultTypes.ColorResult colorResult;
                    List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();

                    if (!colorResults.isEmpty()) {
                        colorResult = colorResults.get(0);
                        List<List<Double>> targetCorners = colorResult.getTargetCorners();
                        if (!targetCorners.isEmpty()) {
                            List<Double> c1 = targetCorners.get(0);
                            List<Double> c2 = targetCorners.get(1);
                            List<Double> c3 = targetCorners.get(2);
                            List<Double> c4 = targetCorners.get(3);

                            double c1c2YOffset = c1.get(1) - c2.get(1);
                            double c1c2XOffset = c1.get(0) - c2.get(1);
                            double c1c3XOffset = c1.get(0) - c3.get(0);
                            double c1c3YOffset = c1.get(1) - c3.get(1);
                            double Angleish;
                            boolean isHorizontal = false;

                            if (c1c2XOffset > c1c3YOffset) {
                                isHorizontal = true;
                            } else {
                                isHorizontal = false;
                            }
                            if(isHorizontal){ Angleish = (Math.atan2(c1c2YOffset, c1c2XOffset)); }
                            else { Angleish = 90 + (Math.atan2(c1c3YOffset, c1c3XOffset)); }
                            telemetry.addData("Angle:", Angleish);
                            timer.schedule(new MoveInDiffyServoPosition(Angleish), 0 * DELAY_BETWEEN_MOVES);
                        }
                    }
                }else {
                    litty.pipelineSwitch(2);//yellow
                    seeingStuff = litty.getLatestResult();
                    if (seeingStuff != null && seeingStuff.isValid()) {

                        LLResultTypes.ColorResult colorResult;
                        List<LLResultTypes.ColorResult> colorResults = seeingStuff.getColorResults();

                        if (!colorResults.isEmpty()) {
                            colorResult = colorResults.get(0);
                            List<List<Double>> targetCorners = colorResult.getTargetCorners();
                            if (!targetCorners.isEmpty()) {
                                List<Double> c1 = targetCorners.get(0);
                                List<Double> c2 = targetCorners.get(1);
                                List<Double> c3 = targetCorners.get(2);
                                List<Double> c4 = targetCorners.get(3);

                                double c1c2YOffset = c1.get(1) - c2.get(1);
                                double c1c2XOffset = c1.get(0) - c2.get(1);
                                double c1c3XOffset = c1.get(0) - c3.get(0);
                                double c1c3YOffset = c1.get(1) - c3.get(1);
                                double Angleish;
                                boolean isHorizontal = false;

                                if (c1c2XOffset > c1c3YOffset) {
                                    isHorizontal = true;
                                } else {
                                    isHorizontal = false;
                                }
                                if (isHorizontal) {
                                    Angleish = (Math.atan2(c1c2YOffset, c1c2XOffset));
                                } else {
                                    Angleish = 90 + (Math.atan2(c1c3YOffset, c1c3XOffset));
                                }
                                telemetry.addData("Angle:", Angleish);
                                timer.schedule(new MoveInDiffyServoPosition(Angleish), 0 * DELAY_BETWEEN_MOVES);
                            }
                        }
                    }
                }*/
            }



            // Show the elapsed game time and wheel power.
            //   telemetry.addData("Status", "Run Time: " + runtime.toString());
            //   telemetry.addData("INDEX", index % LEServoPositions.length);
//            telemetry.addData("", LEServoPositions[index]);
//            telemetry.addData("", REServoPositions[index]);
//            telemetry.addData("", RWServoPositions[index]);
//            telemetry.addData("Left", LeftSlide.getCurrentPosition());
//            telemetry.addData("Right", LeftSlide.getCurrentPosition());
            telemetry.update();
            oldCrossPressed = crossPressed;
            oldCirclePressed = circlePressed;
            oldSquarePressed = squarePressed;
            oldTrianglePressed = trianglePressed;
            oldUpDpadPressed = dpadUpPressed;
            oldDownDpadPressed = dpadDownPressed;
            oldLeftDpadPressed = dpadLeftPressed;
            oldRightDpadPressed = dpadRightPressed;
            oldLeftBumper = leftBumper;
            oldRightBumper = rightBumper;
            oldLeftTrigger = leftTrigger;
            oldRightTrigger = rightTrigger;


        }
    }
}
