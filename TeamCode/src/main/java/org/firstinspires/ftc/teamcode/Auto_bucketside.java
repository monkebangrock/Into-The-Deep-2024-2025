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

@Autonomous
public class Auto_bucketside extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx slideR = null;
    private DcMotorEx slideL = null;
    private DcMotorEx armHinge = null;
    private CRServo tongue;
    private Servo claw;
    private Servo wrist;
    private Servo bucket;
    int velocity;
    int slideTarget;
    boolean slideMoving;
    int slideLevel;
    boolean slideInput;
    int armTarget;
    boolean armMoving;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightBack");
        slideR = hardwareMap.get(DcMotorEx.class, "slideR");
        slideL = hardwareMap.get(DcMotorEx.class, "slideL");
        armHinge = hardwareMap.get(DcMotorEx.class, "armHinge");
        tongue = hardwareMap.get(CRServo.class, "tongue");
        claw = hardwareMap.get(Servo.class,"claw");
        wrist = hardwareMap.get(Servo.class,"wrist");
        bucket = hardwareMap.get(Servo.class,"bucket");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //reset encoder
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armHinge.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //brake motors
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        armHinge.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        velocity = 1000;
        bucket(0);
        autoforward(18);
        autocwspin(40);
        autoback(9);
        slide(1);
        velocity = 600;
        autoback(2);
        bucket(0.5);
        autoforward(5);
        velocity = 1000;
        slide(0);
        autoccwspin(40);
        autoforward(5);
        grab();
        deposit();
        /*autoccwspin(80);
        bucket(0);
        slide(0);
        autoforward(16);
        grab();
        deposit();
        autoback(13);
        slide(1);
        autoback(3);
        bucket(0);
        autoforward(24);
        slide(0);
        autocwspin(90);
        autoforward(24);
        arm(-100);*/

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void autoccwspin (int deg){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(deg*10);
        leftBackDrive.setTargetPosition(deg*10);
        rightFrontDrive.setTargetPosition(deg*10);
        rightBackDrive.setTargetPosition(deg*10);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(leftFrontDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition())>15){
            leftFrontDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
        }
        stopdrive();
        sleep(100);
    }

    public void autocwspin (int deg){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(-deg*10);
        leftBackDrive.setTargetPosition(-deg*10);
        rightFrontDrive.setTargetPosition(-deg*10);
        rightBackDrive.setTargetPosition(-deg*10);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(leftFrontDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition())>15){
            leftFrontDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
        }
        stopdrive();
        sleep(100);
    }

    public void autoforward (int in){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(-in*45);
        leftBackDrive.setTargetPosition(-in*45);
        rightFrontDrive.setTargetPosition(in*45);
        rightBackDrive.setTargetPosition(in*45);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(leftFrontDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition())>15){
            leftFrontDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
        }
        stopdrive();
        sleep(100);
    }

    public void autoback (int in){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(in*45);
        leftBackDrive.setTargetPosition(in*45);
        rightFrontDrive.setTargetPosition(-in*45);
        rightBackDrive.setTargetPosition(-in*45);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(leftFrontDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition())>15){
            leftFrontDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
        }
        stopdrive();
        sleep(100);
    }
    public void autoright (int in){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(-in*45);
        leftBackDrive.setTargetPosition(in*45);
        rightFrontDrive.setTargetPosition(-in*45);
        rightBackDrive.setTargetPosition(in*45);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(leftFrontDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition())>15){
            leftFrontDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
        }
        stopdrive();
        sleep(100);
    }

    public void autoleft (int in){
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setTargetPosition(in*45);
        leftBackDrive.setTargetPosition(-in*45);
        rightFrontDrive.setTargetPosition(in*45);
        rightBackDrive.setTargetPosition(-in*45);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Math.abs(leftFrontDrive.getTargetPosition()-leftFrontDrive.getCurrentPosition())>15){
            leftFrontDrive.setVelocity(velocity);
            leftBackDrive.setVelocity(velocity);
            rightFrontDrive.setVelocity(velocity);
            rightBackDrive.setVelocity(velocity);
        }
        stopdrive();
        sleep(100);
    }

    public void stopdrive(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void slide(int lvl){
        if(lvl == 1){
            while(slideR.getCurrentPosition()<2750){
                slideR.setVelocity(3000);
                slideL.setVelocity(3000);
                slideR.setTargetPosition(2750);
                slideL.setTargetPosition(2750);
            }
        }
        else{
            while(slideR.getCurrentPosition()>0){
                slideR.setVelocity(3000);
                slideL.setVelocity(3000);
                slideR.setTargetPosition(0);
                slideL.setTargetPosition(0);
            }
        }
    }

    public void arm(int ticks){
        armHinge.setVelocity(1000);
        armHinge.setTargetPosition(ticks);
        if(ticks>armHinge.getCurrentPosition()){
            while(armHinge.getCurrentPosition()<ticks){
                armHinge.setVelocity(1000);
                armHinge.setTargetPosition(ticks);
            }
        }
        else{
            while(armHinge.getCurrentPosition()>ticks){
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

    public void deposit(){
        //motor first
        telemetry.addData("Sdlie r", slideR.getCurrentPosition());
        telemetry.addData("tgt", slideR.getTargetPosition());
        telemetry.update();
        while(slideR.getCurrentPosition()>210 || slideR.getCurrentPosition()<190){
            slideR.setVelocity(1000);
            slideL.setVelocity(1000);
            slideR.setTargetPosition(200);
            slideL.setTargetPosition(200);
            slideLevel=0;
        }
        while (armHinge.getCurrentPosition() < -90) {
            armHinge.setTargetPosition(-90);
            armMoving = true;
        }
        bucket.setPosition(0);
        while(wrist.getPosition()!= 0){
            wrist.setPosition(0);
        }
        sleep(250);
        claw.setPosition(0.2);
    }

    public void grab(){
        armHinge.setVelocity(1000);
        claw.setPosition(0.2);
        while(armHinge.getCurrentPosition()>-800) {
            armHinge.setTargetPosition(-800);
        }
        armMoving = true;
        wrist.setPosition(0.4);
        claw.setPosition(0.55);
    }

    public void tongue(int pwr){ //-1 is out, 1 is in
        tongue.setPower(pwr);
    }

    public void claw(double pos){ //0.2 is open, 0.55 is closed

        while(claw.getPosition() != pos){
            claw.setPosition(pos);
        }
    }

    public void wrist(double pos){ //0 is back, 0.15 is in line w/ arm
        while(wrist.getPosition() != pos){
            wrist.setPosition(pos);
        }
    }

    public void bucket(double pos){
        while(bucket.getPosition() != pos){
            bucket.setPosition(pos);
        }
    }
}
