/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AutoServoConstants;
import org.firstinspires.ftc.teamcode.constants.TeleOpServoConstants;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

import java.util.Arrays;
import java.util.Timer;
import java.util.TimerTask;


/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the TeamElement when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous(name = "Left")
//@Disabled
public class SharkbaitOohaha extends LinearOpMode
{
    enum DriveDirection {
        FORWARD,
        LEFT,
        RIGHT,
        BACKWARD
    }

    enum StartingPositionEnum {
        LEFT,
        RIGHT
    }

    enum SlidePackDirection {
        UP,
        DOWN
    }

    private ElapsedTime runtime = new ElapsedTime();
    private Timer timer = new Timer();
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

    private ElapsedTime eTime = new ElapsedTime();

    private int index = 0;
    private int wristIndex;
    private double[] ILEServoPositions = AutoServoConstants.ILEServoPositions;
    private double[] IREServoPositions = AutoServoConstants.IREServoPositions;
    private double[] OLEServoPositions = AutoServoConstants.OLEServoPositions;
    private double[] OREServoPositions = AutoServoConstants.OREServoPositions;
    private double[] LDServoPositions = AutoServoConstants.LDServoPositions;
    private double[] RDServoPositions = AutoServoConstants.RDServoPositions;
    private double[] OAServoPositions = AutoServoConstants.OAServoPositions;
    private double[] ICServoPositions = AutoServoConstants.ICServoPositions;
    private double[] OCServoPositions = AutoServoConstants.OCServoPositions;
    private double[] OKLServoPositions = AutoServoConstants.OKLServoPositions;
    private double[] OKRServoPositions = AutoServoConstants.OKRServoPositions;
    private final int DELAY_BETWEEN_MOVES = 100;

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



    StartingPositionEnum sideOfFieldToStartOn = StartingPositionEnum.LEFT;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */


        telemetry.addData("Status", "sInitialized");
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

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        InSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        OtSlideB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        InLeftElbowServo.setDirection(Servo.Direction.REVERSE);
        InRightElbowServo.setDirection(Servo.Direction.FORWARD);
        OtLeftElbowServo.setDirection(Servo.Direction.FORWARD);
        OtRightElbowServo.setDirection(Servo.Direction.FORWARD);

        InClawServo.setDirection(Servo.Direction.FORWARD);
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



        InLDiffyServo.setPosition(LDServoPositions[3]);
        InRDiffyServo.setPosition(RDServoPositions[3]);
        InClawServo.setPosition(ICServoPositions[0]);
        OtClawServo.setPosition(OCServoPositions[1]);
        OtLinkageLServo.setPosition(OKLServoPositions[0]);
        OtLinkageRServo.setPosition(OKRServoPositions[0]);
        InLeftElbowServo.setPosition(ILEServoPositions[3]);
        InRightElbowServo.setPosition(IREServoPositions[3]);
        OtLeftElbowServo.setPosition(OLEServoPositions[7]);
        OtRightElbowServo.setPosition(OREServoPositions[7]);
        OtCoaxialServo.setPosition(OAServoPositions[0]);
        // Wait for the game to start (driver presses PLAY)



        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(12, -65, Math.PI / 2));
//        timer.schedule(new PutGrabberToCertainPosition(0), 3000);


        waitForStart();
        InLeftElbowServo.setPosition(ILEServoPositions[2]);
        InRightElbowServo.setPosition(IREServoPositions[2]);
        InLDiffyServo.setPosition(LDServoPositions[2]);
        InRDiffyServo.setPosition(RDServoPositions[2]);

        while (opModeIsActive())
        {

            doActions(drive, sideOfFieldToStartOn);

            // Don't burn CPU cycles busy-looping in this sample
            sleep(15000);
            break;
        }

    }
    public class BeforePlaceSpecimen1 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new OuttakeExtension(-1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(.25), 3 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(0), 7 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtClawServoPosition(1), 3 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtCoaxialServoPosition(1), 3 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtElbowServosPosition(1), 5 *DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtLinkageServoPosition(1), 13 *DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class BeforePlaceSpecimen2 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new OuttakeExtension(-1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(.25), 4 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(0), 9 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtClawServoPosition(1), 4 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtCoaxialServoPosition(1), 4 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtElbowServosPosition(1), 6 *DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtLinkageServoPosition(1), 13 *DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class BeforePlaceSpecimen3 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new OuttakeExtension(-1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(.25), 4 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(0), 9 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtClawServoPosition(1), 5 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtCoaxialServoPosition(1), 5 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtElbowServosPosition(1), 7 *DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtLinkageServoPosition(1), 13 *DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class PlaceSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new MoveOtElbowServosPosition(6), 0 *DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtCoaxialServoPosition(5), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtLinkageServoPosition(2), 3 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtClawServoPosition(0), 6 * DELAY_BETWEEN_MOVES);

            return false;
        }
    }

    public class GrabSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(-1), 1 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(.25), 2 * DELAY_BETWEEN_MOVES);
            timer.schedule(new OuttakeExtension(0), 4 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtCoaxialServoPosition(0), 4 *DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class PickupSpecimenOffWall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new MoveOtClawServoPosition(1), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtLinkageServoPosition(0), 1 *DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtCoaxialServoPosition(3), 5 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtElbowServosPosition(3), 8 *DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveOtClawServoPosition(0), 11 * DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class IntakeOverWall implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new MoveInDiffyServoPosition(2), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveInElbowServosPosition(2), 2 *DELAY_BETWEEN_MOVES);
            return false;
        }
    }

    public class IntakeInRobot implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            timer.schedule(new MoveInDiffyServoPosition(3), 0 * DELAY_BETWEEN_MOVES);
            timer.schedule(new MoveInElbowServosPosition(3), 2 *DELAY_BETWEEN_MOVES);
            return false;
        }
    }






    private void doActions(MecanumDrive drive, StartingPositionEnum position) {
        boolean needInvert = (position != StartingPositionEnum.RIGHT);
        double multiplier = 1;
        if (needInvert) {
            multiplier = -1;
        }



        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(drive.pose); //actually a genius
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-20.0, 45.0);

        //preload 1
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(-6, -32), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(0, new BeforePlaceSpecimen1()).build()));
        Actions.runBlocking(new SequentialAction( actionBuilder.afterTime(.5, new PlaceSpecimen()).build(), actionBuilder.afterTime(.5, new IntakeInRobot()).build()));
        actionBuilder.strafeToConstantHeading(new Vector2d(-6, -45), new TranslationalVelConstraint(100.0)).build();
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(40, -45), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1.5, new IntakeOverWall()).build(), actionBuilder.afterTime(1.5, new PickupSpecimenOffWall()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(28, -11), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1, new IntakeInRobot()).build()));
        //over to first sample
        Actions.runBlocking(new ParallelAction((actionBuilder.strafeToLinearHeading(new Vector2d(48, -12), 9*(Math.PI/16), new TranslationalVelConstraint(100.0)).build()), actionBuilder.afterTime(0, new PickupSpecimenOffWall()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        //bring sample to human
        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(48, -55), new TranslationalVelConstraint(100.0)).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        //back to samples
        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(44, -11), new TranslationalVelConstraint(70.0)).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        //move to second sample
        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(56, -12), new TranslationalVelConstraint(70.0)).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        //bring sample to human
        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(56, -50), new TranslationalVelConstraint(70.0)).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(56, -42), new TranslationalVelConstraint(70.0)).build());
        actionBuilder = drive.actionBuilder(drive.pose);
        //back to samples
        //Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(50, -8), new TranslationalVelConstraint(70.0)).build());
        //actionBuilder = drive.actionBuilder(drive.pose);
        //move to third sample

        //Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(62, -12), new TranslationalVelConstraint(70.0)).build());
        //actionBuilder = drive.actionBuilder(drive.pose);
        //bring sample to human
        //Actions.runBlocking(actionBuilder.strafeToConstantHeading(new Vector2d(64, -50), new TranslationalVelConstraint(70.0)).build());

        //move to wall and pick up specimen 2
        Actions.runBlocking(new SequentialAction(actionBuilder.strafeToConstantHeading(new Vector2d(52, -60)).build(), actionBuilder.afterTime(0.25, new GrabSpecimen()).build()));
        actionBuilder.strafeToConstantHeading(new Vector2d(52, -45)).build();
        actionBuilder = drive.actionBuilder(drive.pose);
        actionBuilder.waitSeconds(1);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(-5, -34.5/*HERE*/), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1, new BeforePlaceSpecimen2()).build(), actionBuilder.afterTime(0, new IntakeOverWall()).build()));
        Actions.runBlocking(new SequentialAction( actionBuilder.afterTime(0.5, new PlaceSpecimen()).build(), actionBuilder.afterTime(.5, new IntakeInRobot()).build()));
        actionBuilder.waitSeconds(0.5);
        actionBuilder.strafeToConstantHeading(new Vector2d(-5, -50), new TranslationalVelConstraint(100.0)).build();
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(48, -50), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1.5, new IntakeOverWall()).build(), actionBuilder.afterTime(1.5, new PickupSpecimenOffWall()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        //specimen 3
        /*Actions.runBlocking(new SequentialAction(actionBuilder.strafeToConstantHeading(new Vector2d(52, -60)).build(), actionBuilder.afterTime(0.25, new GrabSpecimen()).build(), actionBuilder.afterTime(0, new IntakeOverWall()).build()));
        actionBuilder.strafeToConstantHeading(new Vector2d(52, -45)).build();
        actionBuilder = drive.actionBuilder(drive.pose);
        actionBuilder.waitSeconds(0.5);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(-5, -35/*HERE*//*), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1, new BeforePlaceSpecimen3()).build(), actionBuilder.afterTime(0, new IntakeOverWall()).build()));
        Actions.runBlocking(new SequentialAction( actionBuilder.afterTime(0.5, new PlaceSpecimen()).build(), actionBuilder.afterTime(.5, new IntakeInRobot()).build()));
        actionBuilder.waitSeconds(1);
        actionBuilder.strafeToConstantHeading(new Vector2d(-5, -50), new TranslationalVelConstraint(100.0)).build();
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(48, -50), new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1.5, new IntakeOverWall()).build(), actionBuilder.afterTime(1.5, new PickupSpecimenOffWall()).build()));
        actionBuilder = drive.actionBuilder(drive.pose);*/
        //specimen 4
        //Actions.runBlocking(actionBuilder.afterTime(1, new IntakeOverWall()).build());
        //Actions.runBlocking(actionBuilder.afterTime(0, new PickupSpecimenOffWall()).build());
        //Actions.runBlocking(new SequentialAction(actionBuilder.strafeToConstantHeading(new Vector2d(48, -58)).build(), actionBuilder.afterTime(0.5, new GrabSpecimen()).build()));
        //actionBuilder = drive.actionBuilder(drive.pose);
        //actionBuilder.waitSeconds(1);
        //Actions.runBlocking(new ParallelAction(actionBuilder.strafeToConstantHeading(new Vector2d(0, -30),  new TranslationalVelConstraint(100.0)).build(), actionBuilder.afterTime(1, new BeforePlaceSpecimen()).build()));
        //actionBuilder = drive.actionBuilder(drive.pose);
        //Actions.runBlocking(new SequentialAction( actionBuilder.afterTime(0.5, new PlaceSpecimen()).build(), actionBuilder.afterTime(.5, new IntakeInRobot()).build(), actionBuilder.strafeToConstantHeading(new Vector2d(0, -45), new TranslationalVelConstraint(100.0)).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        //park
        Actions.runBlocking(new SequentialAction(actionBuilder.strafeToConstantHeading(new Vector2d(52, -60)).build()));
        actionBuilder = drive.actionBuilder(drive.pose);
        Actions.runBlocking(actionBuilder.afterTime(1, new IntakeOverWall()).build());
        Actions.runBlocking(actionBuilder.afterTime(0, new PickupSpecimenOffWall()).build());

    }

    private DriveDirection getCorrectDirection(DriveDirection direction, boolean needInvert) {
        if (!needInvert)
            return direction;

        DriveDirection invertedDirection = direction;
        switch (direction) {
            case LEFT:
                invertedDirection = DriveDirection.RIGHT;
                break;
            case RIGHT:
                invertedDirection = DriveDirection.LEFT;
                break;
            case FORWARD:
                invertedDirection = DriveDirection.BACKWARD;
                break;
            case BACKWARD:
                invertedDirection = DriveDirection.FORWARD;
                break;
            default:
                break;
        }

        return invertedDirection;
    }
}