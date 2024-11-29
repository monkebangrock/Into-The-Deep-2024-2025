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
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //reset encoder
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armHinge.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        armHinge.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        waitForStart();
        velocity = 1600;
        slide(1);
        autoforward(24);
        slide(0);
        arm(-100);
        autoback(24);
        slide(-1);
        arm(-400);
        autoright(74);
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
        leftFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        while(leftFrontDrive.isBusy()||leftBackDrive.isBusy()||rightFrontDrive.isBusy()||rightBackDrive.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        telemetry.addData("status", "autoccwspin");
        telemetry.update();

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
        leftFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        while(leftFrontDrive.isBusy()||leftBackDrive.isBusy()||rightFrontDrive.isBusy()||rightBackDrive.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        stopdrive();
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
        leftFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        while(leftFrontDrive.isBusy()||leftBackDrive.isBusy()||rightFrontDrive.isBusy()||rightBackDrive.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        stopdrive();
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
        leftFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        while(leftFrontDrive.isBusy()||leftBackDrive.isBusy()||rightFrontDrive.isBusy()||rightBackDrive.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        stopdrive();
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
        leftFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        while(leftFrontDrive.isBusy()||leftBackDrive.isBusy()||rightFrontDrive.isBusy()||rightBackDrive.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        stopdrive();
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
        leftFrontDrive.setVelocity(velocity);
        leftBackDrive.setVelocity(velocity);
        rightFrontDrive.setVelocity(velocity);
        rightBackDrive.setVelocity(velocity);
        while(leftFrontDrive.isBusy()||leftBackDrive.isBusy()||rightFrontDrive.isBusy()||rightBackDrive.isBusy()) {
            // Let the drive team see that we're waiting on the motor
            telemetry.addData("Status", "Waiting for the motor to reach its target");
            telemetry.update();
        }
        stopdrive();
    }

    public void stopdrive(){
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void slide(){
        if (!slideInput){ //slide input true is dpad clicked
            if (gamepad2.dpad_up) { //when up on dpad is pressed
                slideInput = true;
                if(slideLevel == 0){ //low basket
                    slideR.setVelocity(5000);
                    slideL.setVelocity(5000);
                    slideTarget = 1360;
                    slideR.setTargetPosition(slideTarget);
                    slideL.setTargetPosition(slideTarget);
                    slideLevel = 1;
                    telemetry.addData("level","1");
                    telemetry.update();
                }
                else if(slideLevel == 1){ //high basket
                    slideTarget = 2750;
                    slideR.setTargetPosition(slideTarget);
                    slideL.setTargetPosition(slideTarget);
                    slideLevel = 2;
                    telemetry.addData("level","2");
                    telemetry.update();
                }
            }
            else if(gamepad2.dpad_down){
                slideInput = true;
                if(slideLevel == 2){ //low basket
                    slideTarget = 1360;
                    slideR.setTargetPosition(slideTarget);
                    slideL.setTargetPosition(slideTarget);
                    slideLevel = 1;
                    telemetry.addData("level","1");
                    telemetry.update();
                }
                else if(slideLevel == 1){ //down
                    slideTarget = 0;
                    slideR.setTargetPosition(slideTarget);
                    slideL.setTargetPosition(slideTarget);
                    slideLevel = 0;
                    telemetry.addData("level","0");
                    telemetry.update();
                }
            }
        }
        else {
            if (!gamepad2.dpad_down && !gamepad2.dpad_up){ //when arrow button (dpad up/down) is released
                slideInput = false;
                if(slideR.getCurrentPosition() < 10 && slideTarget == 0){
                    slideR.setVelocity(0);
                    slideL.setVelocity(0);
                }
                /*telemetry.addData("Right position:", slideR.getCurrentPosition());
                telemetry.addData("Left position:", slideL.getCurrentPosition());
                telemetry.addData("velocity", slideR.getVelocity());
                telemetry.update();*/
            }
        }
    }

    public void slide(int ticks){
        slideR.setVelocity(5000);
        slideL.setVelocity(5000);
        slideR.setTargetPosition(ticks);
        slideL.setTargetPosition(ticks);
    }

    public void arm(int ticks){
        armHinge.setVelocity(1000);
        armHinge.setTargetPosition(ticks);
        armMoving = true;
        telemetry.addData("arm:", armHinge.getCurrentPosition());
        telemetry.addData("tgt pos:", armHinge.getTargetPosition());
        telemetry.addData("gp:", gamepad2.left_stick_y);
        telemetry.update();
    }

    public void tongue(int pwr){
        tongue.setPower(pwr);
    }
}
