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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotor OtSlideL = null;
    private DcMotor OtSlideM = null;
    private DcMotor OtSlideR = null;
    
    private CRServo InLDiffyServo = null;
    private CRServo InRDiffyServo = null;
    private Servo InLeftElbowServo = null;
    private Servo InRightElbowServo = null;
    private Servo InClawServo = null;
    
    private Servo OtClawServo = null;
    private Servo OtLeftElbowServo = null;
    private Servo OtRightElbowServo = null;
    private Servo OtCoaxialServo = null;
    private Servo OtLinkageServo = null;

    private int intakeIndex = 0;
    private int outtakeIndex = 0;

    private boolean inClawDelay = false;
    private boolean otClawDelay = false;

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
    
    private boolean isInArmMoving = false;
    private boolean isOtArmMoving = false;
    
    private boolean isInDiffyMoving = false;
    private boolean isOtCoaxialMoving = false

    private double[] ILEServoPositions = TeleOpServoConstants.ILEServoPositions;
    private double[] IREServoPositions = TeleOpServoConstants.IREServoPositions;
    private double[] OLEServoPositions = TeleOpServoConstants.OLEServoPositions;
    private double[] OREServoPositions = TeleOpServoConstants.OREServoPositions;
    private double[] LDServoPositions = TeleOpServoConstants.LDServoPositions;
    private double[] RDServoPositions = TeleOpServoConstants.RDServoPositions;
    private double[] OAServoPositions = TeleOpServoConstants.OAServoPositions;
    private double[] ICServoPositions = TeleOpServoConstants.ICServoPositions;
    private double[] OCServoPositions = TeleOpServoConstants.OCServoPositions;
    private double[] OKServoPositions = TeleOpServoConstants.OKServoPositions;
    
    private double[] SlowModeSpeed = TeleOpServoConstants.SlowModeSpeed;


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
        class setInClawDelay extends TimerTask {
            boolean val;
            public setInClawDelay(boolean v) {
                this.val = v;
            }
            public void run() {
                inClawDelay = val;
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
         class OuttakeExtension extends TimerTask {
            double i;
            public OuttakeExtension(double i) {
                this.i = i;
            }
            public void run() {
                OtSlideL.setPower(i);
                OtSlideM.setPower(i);
                OtSlideR.setPower(i);
            }
        }
        class MoveOtElbowServosPosition extends TimerTask {
            int i;
            public MoveOtElbowServosPosition(int i) {
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
        class MoveInElbowServosPosition extends TimerTask {
            int i;
            public MoveInElbowServosPosition(int i) {
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
            public MoveInDiffyServoPosition(int i) {
                this.i = i;
            }
            public void run() {
                InLDiffyServo.setPosition(LDServoPositions[i]);
                InRDiffyServo.setPosition(RDServoPositions[i]);

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
        OtSlideL = hardwareMap.get(DcMotor.class, "OL");
        OtSlideM = hardwareMap.get(DcMotor.class, "OM");
        OtSlideR = hardwareMap.get(DcMotor.class, "OR");
        
        InLeftElbowServo = hardwareMap.get(Servo.class, "ILE");
        InRightElbowServo = hardwareMap.get(Servo.class, "IRE");
        InClawServo = hardwareMap.get(Servo.class, "IC");
        InLDiffyServo = hardwareMap.get(CRServo.class, "LD");
        InRDiffyServo = hardwareMap.get(CRServo.class, "RD");

        OtLeftElbowServo = hardwareMap.get(Servo.class, "OLE");
        OtRightElbowServo = hardwareMap.get(Servo.class, "ORE");
        OtClawServo = hardwareMap.get(Servo.class, "OC");
        OtCoaxialServo = hardwareMap.get(Servo.class, "OA");
        OtLinkageServo = hardwareMap.get(Servo.class, "OK");
        Timer timer = new Timer();

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        InSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //In the future will need code for Encoders on the Motors

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        InSlide.setDirection(DcMotor.Direction.REVERSE);
        OtSlideL.setDirection(DcMotor.Direction.FORWARD);
        OtSlideM.setDirection(DcMotor.Direction.REVERSE);
        OtSlideR.setDirection(DcMotor.Direction.FORWARD);
        
        
        InLeftElbowServo.setDirection(Servo.Direction.FORWARD);
        InRightElbowServo.setDirection(Servo.Direction.REVERSE);
        OtLeftElbowServo.setDirection(Servo.Direction.FORWARD);
        OtRightElbowServo.setDirection(Servo.Direction.REVERSE);
        
        InClawServo.setDirection(Servo.Direction.FORWARD);
        OtClawServo.setDirection(Servo.Direction.FORWARD);

        OtCoaxialServo.setDirection(Servo.Direction.FORWARD);
        OtLinkageServo.setDirection(Servo.Direction.FORWARD);
        
        InLDiffyServo.setDirection(CRServo.Direction.FORWARD);
        InRDiffyServo.setDirection(CRServo.Direction.FORWARD);

        

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        

        
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        
    
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

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
                    FLMotor.setPower(SlowModeSpeed[0]);
                    FRMotor.setPower(SlowModeSpeed[1]);
                    BLMotor.setPower(SlowModeSpeed[0]);
                    BRMotor.setPower(SlowModeSpeed[1]);
                } else if (gamepad1.dpad_right){
                    FLMotor.setPower(SlowModeSpeed[1]);
                    FRMotor.setPower(SlowModeSpeed[0]);
                    BLMotor.setPower(SlowModeSpeed[1]);
                    BRMotor.setPower(SlowModeSpeed[0]);
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
                //   }
            } else {
                FLMotor.setPower(FLPower);
                FRMotor.setPower(FRPower);
                BLMotor.setPower(BLPower);
                BRMotor.setPower(BRPower);
            }

            InSlide.setPower(gamepad2.right_stick_y);
            timer.schedule(new OuttakeExtension(gamepad2.left_stick_y), 0);
            
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
            

            if (leftBumper && inClawOpened && !inClawDelay) { //close Intake claw
                new setInClawDelay(true).run();
                InClawServo.setPosition(ICServoPositions[1]);
                timer.schedule(new setInClawDelay(false), 10 * DELAY_BETWEEN_MOVES);
            } else if (leftBumper && !inClawOpened && !inClawDelay) { //open Intake claw
                new setInClawDelay(true).run();
                InClawServo.setPosition(ICServoPositions[0]
                timer.schedule(new setInClawDelay(false), 10 * DELAY_BETWEEN_MOVES);
            }
            
            if (rightBumper && otClawOpened && !OtClawDelay) { //close Outtake claw
                new setOtClawDelay(true).run();
                OtClawServo.setPosition(OCServoPositions[1]);
                timer.schedule(new setOtClawDelay(false), 10 * DELAY_BETWEEN_MOVES);
            } else if (rightBumper && !otClawOpened && !otClawDelay) { //open Outtake claw
                new setOtClawDelay(true).run();
                OtClawServo.setPosition(OCServoPositions[0]);
                timer.schedule(new setOtClawDelay(false), 10 * DELAY_BETWEEN_MOVES);
            }
            
            if (crossPressed && !oldCrossPressed && !isOtArmMoving) { //pickup sample from intake outtakeIndex = 0
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtCoaxialServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsOtArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 4 *DELAY_BETWEEN_MOVES);
                
            } else if (circlePressed && !oldCirclePressed && !isOtArmMoving) { //pickup specimen from wall outtakeIndex = 3
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtCoaxialServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsOtArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 4 *DELAY_BETWEEN_MOVES);

            } else if (trianglePressed && !oldTrianglePressed && !isOtArmMoving) { //outtake from back of robot outtakeIndex = 2
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtCoaxialServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsOtArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 4 *DELAY_BETWEEN_MOVES);

            } else if (squarePressed && !oldSquarePressed && !isOtArmMoving) { //outtake from front of robot outtakeIndex = 3
                //needs code for linkage, wont do until we have fixed linkage slides
                new setIsOtArmMoving(true).run();
                new setIsOtCoaxialMoving(true).run();
                timer.schedule(new MoveOtCoaxialServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveOtElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsOtArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsOtCoaxialMoving(false), 4 *DELAY_BETWEEN_MOVES);
            }
            
            if (dpadDownPressed && !oldDownDpadPressed && !isInArmMoving) { //intake position intakeIndex = 0
                //Needs Limelight Logic and to move down to the servo positions for the Diffy and the Elbow
                
            } else if (dpadRightPressed && !oldRightDpadPressed && !isInArmMoving) { //position for getting over submersible walls intakeIndex = 2
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                timer.schedule(new MoveInDiffyServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                
            } else if (dpadLeftPressed && !oldLeftDpadPressed && !isInArmMoving) { //position for scanning for samples intakeIndex = 1
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                timer.schedule(new MoveInDiffyServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
                
            } else if (dpadUpPressed && !oldUpDpadPressed && !isInArmMoving) { //position for transferring sample to outtake intakeIndex = 3
                new setIsInArmMoving(true).run();
                new setIsInDiffyMoving(true).run();
                timer.schedule(new MoveInDiffyServoPosition(0), 0 * DELAY_BETWEEN_MOVES);
                timer.schedule(new MoveInElbowServosPosition(0), 2 *DELAY_BETWEEN_MOVES);

                timer.schedule(new setIsInArmMoving(false), 4 *DELAY_BETWEEN_MOVES);
                timer.schedule(new setIsInDiffyMoving(false), 4 *DELAY_BETWEEN_MOVES);
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


        }
    }
}
