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

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Encoder", group="Robot")
//@Disabled
public class autoTest extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotorEx slideR = null;
    private DcMotorEx slideL = null;
    private DcMotorEx armHinge = null;
    private CRServo tongue;
    private Servo claw;
    private Servo wrist;
    private Servo backWrist;
    private Servo backClaw;
    int slideTarget;
    boolean slideMoving;
    int slideLevel;
    boolean slideInput;
    int armTarget;
    boolean armMoving;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");
        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        armHinge = hardwareMap.get(DcMotorEx.class, "armHinge");
        tongue = hardwareMap.get(CRServo.class, "tongue");
        claw = hardwareMap.get(Servo.class,"claw");
        wrist = hardwareMap.get(Servo.class,"wrist");
        backWrist = hardwareMap.get(Servo.class, "backWrist");
        backClaw = hardwareMap.get(Servo.class, "backClaw");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //reset encoder
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armHinge.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //brake motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armHinge.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //slide stuff
        slideR.setTargetPosition(0);
        slideL.setTargetPosition(0);
        armHinge.setTargetPosition(0);
        slideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armHinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        slideTarget = 0;
        slideMoving = false;
        slideLevel = 0;
        slideInput = false;
        armTarget = 0;
        armMoving = false;
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        armHinge.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        backWrist.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(0);
        claw.setPosition(0.2);
        backWrist.setPosition(0);
        backClaw.setPosition(0);

        /*leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                leftFront.getCurrentPosition(),
                leftBack.getCurrentPosition(),
                rightFront.getCurrentPosition(),
                rightBack.getCurrentPosition());*/
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)

        Pose2d initialPose = new Pose2d(0, 63, Math.toRadians(270));
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);

        int visionOutputPosition = 1;
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(-PI / 2)
                .lineToY(34) //fwd to bar
                //hang specimen #1
                .lineToY(50) //pull out from bar a bit
                .splineToConstantHeading(new Vector2d(-40, 25), -PI / 2) //to blue sample
                .splineToConstantHeading(new Vector2d(-45, 10), -PI / 2) //to blue sample
                .splineToConstantHeading(new Vector2d(-45, 48), -PI / 2) //to human player zone
                .splineToConstantHeading(new Vector2d(-45, 10), -PI / 2) //to blue sample #2
                .splineToConstantHeading(new Vector2d(-55, 12), -PI / 2) //to blue sample #2
                .splineToConstantHeading(new Vector2d(-55, 48), -PI / 2) //to human player zone
                .lineToY(45) //pull out from wall a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI / 2) //to wall - to pick up specimen
                //grab specimen #2
                .splineToConstantHeading(new Vector2d(0, 34), -PI / 2) //to submersible
                //place specimen #2
                .lineToY(45) //pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-54, 10), -PI / 2) //to blue sample #3
                .splineToConstantHeading(new Vector2d(-61, 12), -PI / 2)//to blue sample #3
                .splineToConstantHeading(new Vector2d(-61, 48), -PI / 2) //to human player zone
                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI / 2) // to wall - to pick up specimen
                //grab specimen #3
                .lineToY(45) // pull out from wall a bit
                .splineToConstantHeading(new Vector2d(0, 34), -PI / 2); //to submersible
        //place specimen #3
                /*
                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI/2) // to wall - to pick up specimen
                //grab specimen #4
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2) //to submersible
                //place specimen #4
                .lineToY(45) // pull out from submersible a bit
                .splineToConstantHeading(new Vector2d(-35, 56), -PI/2) // to wall - to pick up specimen
                //grab specimen #5
                .splineToConstantHeading(new Vector2d(0, 34), -PI/2);//to submersible
                //place specimen #5*/


        waitForStart();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        if (isStopRequested()) return;
        Action chosenOne = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        chosenOne
                        //split up trajectory ltr
                )
        );

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }


    public void slide(int lvl) {
        if (lvl == 1) {
            while (slideR.getCurrentPosition() < 2750 && opModeIsActive()) {
                slideR.setVelocity(3000);
                slideL.setVelocity(3000);
                slideR.setTargetPosition(2750);
                slideL.setTargetPosition(2750);
            }
        } else {
            while (slideR.getCurrentPosition() > 0 && opModeIsActive()) {
                slideR.setVelocity(3000);
                slideL.setVelocity(3000);
                slideR.setTargetPosition(0);
                slideL.setTargetPosition(0);
                if (slideR.getCurrentPosition() < 30) {
                    slideR.setVelocity(0);
                    slideL.setVelocity(0);
                }
            }
        }
    }

    public void specimenHang(){
        while (slideR.getCurrentPosition() < 1240 && opModeIsActive()) {
            slideR.setVelocity(3000);
            slideL.setVelocity(3000);
            slideR.setTargetPosition(1240);
            slideL.setTargetPosition(1240);
        }
    }

    public void arm(int ticks) {
        armHinge.setVelocity(1000);
        armHinge.setTargetPosition(ticks);
        if (ticks > armHinge.getCurrentPosition()) {
            while (armHinge.getCurrentPosition() < ticks && opModeIsActive()) {
                armHinge.setVelocity(1000);
                armHinge.setTargetPosition(ticks);
            }
        } else {
            while (armHinge.getCurrentPosition() > ticks && opModeIsActive()) {
                armHinge.setVelocity(1000);
                armHinge.setTargetPosition(ticks);
            }
        }
        armMoving = true;
        telemetry.addData("arm:", armHinge.getCurrentPosition());
        telemetry.addData("tgt pos:", armHinge.getTargetPosition());
        telemetry.addData("gp:", gamepad2.left_stick_y);
        telemetry.update();
    }

    public void transfer() { //fix
        //motor first
        telemetry.addData("Sdlie r", slideR.getCurrentPosition());
        telemetry.addData("tgt", slideR.getTargetPosition());
        telemetry.update();
        while ((slideR.getCurrentPosition() > 210 || slideR.getCurrentPosition() < 190) && opModeIsActive()) {
            slideR.setVelocity(1000);
            slideL.setVelocity(1000);
            slideR.setTargetPosition(200);
            slideL.setTargetPosition(200);
            slideLevel = 0;
        }
        while (armHinge.getCurrentPosition() < -90 && opModeIsActive()) {
            armHinge.setTargetPosition(-90);
            armMoving = true;
        }
        while (wrist.getPosition() != 0 && opModeIsActive()) {
            wrist.setPosition(0);
        }
        sleep(250);
        claw.setPosition(0.2);
    }

    public void grab() {
        claw.setPosition(0.2);
        sleep(300);
        wrist.setPosition(0.62);
        armHinge.setVelocity(1000);
        armHinge.setTargetPosition(-550);
        armHinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (armHinge.getCurrentPosition() > -545 && opModeIsActive()) {
            armHinge.setVelocity(1000);
        }
        claw.setPosition(0.55);
        sleep(500);
    }

    public void tongue(int pwr) { //-1 is out, 1 is in
        tongue.setPower(pwr);
    }

    public void claw(double pos) { //0.2 is open, 0.55 is closed

        while (claw.getPosition() != pos && opModeIsActive()) {
            claw.setPosition(pos);
        }
    }

    public void wrist(double pos) { //0 is back, 0.62 is down for grab
        while (wrist.getPosition() != pos && opModeIsActive()) {
            wrist.setPosition(pos);
        }
    }

    public void backClaw(double pos){ //0 is closed, 0.35 is open
        while (backClaw.getPosition() != pos && opModeIsActive()) {
            backClaw.setPosition(pos);
        }
    }

    public void backWrist(double pos){ //0 is out, 0.65 is towards the other claw
        while (backWrist.getPosition() != pos && opModeIsActive()) {
            backWrist.setPosition(pos);
        }
    }
}


/*
 *  Method to perform a relative move, based on encoder counts.
 *  Encoders are not reset as the move is based on the current position.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the OpMode running.
 */