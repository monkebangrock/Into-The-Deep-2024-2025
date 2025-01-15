/* Copyright (c) 2021 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp
//@Disabled
public class MainFieldCentric80Speed extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;
    private DcMotorEx slideR = null;
    private DcMotorEx slideL = null;
    private DcMotorEx armHinge = null;
    private Servo tongue;
    private Servo claw;
    private Servo wrist;
    private Servo backWrist;
    private Servo backClaw;
    private Servo rotWrist;
    int slideTarget;
    boolean slideMoving;
    int slideLevel;
    boolean slideInput;
    int armTarget;
    boolean armMoving;
    boolean xPressed;
    boolean yPressed;
    boolean aPressed;
    boolean depositMode;
    boolean bPressed;
    boolean bucketMode;
    boolean guidePressed;
    double rotWristPos;
    double tonguePos;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;

    // Some constant values
    final double FRONT_CLAW_OPENED = 0.1;
    final double FRONT_CLAW_CLOSED = 0.32;
    final double BACK_CLAW_OPENED = 0.1;
    final double BACK_CLAW_CLOSED = 0.32;
    final int ARM_POS_UP = -225;
    final int ARM_POS_DOWN = -750;
    final int ARM_POS_TILT = -1300;
    final int SLIDES_SPECIMEN_DOWN = 0;
    final int SLIDES_SPECIMEN_TRANSFER = 190;
    final int SLIDES_SPECIMEN_PREP_HANG = 1000;
    final int SLIDES_SPECIMEN_HANG = 1600;
    final int SLIDES_ROBOT_HANG = 1500;
    final double FRONT_WRIST_HORIZONTAL = 0.61;




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
        tongue = hardwareMap.get(Servo.class, "tongue");
        claw = hardwareMap.get(Servo.class,"claw");
        wrist = hardwareMap.get(Servo.class,"wrist");
        backWrist = hardwareMap.get(Servo.class, "backWrist");
        backClaw = hardwareMap.get(Servo.class, "backClaw");
        rotWrist = hardwareMap.get(Servo.class, "rotWrist");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

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
        slideTarget = 0;
        slideMoving = false;
        slideLevel = 0;
        slideInput = false;
        armTarget = 0;
        armMoving = false;
        xPressed = false;
        yPressed = false;
        aPressed = false;
        depositMode = false;
        bPressed = false;
        bucketMode = false;
        guidePressed = false;
        rotWristPos = FRONT_WRIST_HORIZONTAL;
        tonguePos = 0;

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        slideL.setDirection(DcMotorSimple.Direction.REVERSE);
        armHinge.setDirection(DcMotorSimple.Direction.REVERSE);
        wrist.setDirection(Servo.Direction.REVERSE);
        backWrist.setDirection(Servo.Direction.REVERSE);
        tongue.setDirection(Servo.Direction.REVERSE);
        wrist.setPosition(0.48);
        claw.setPosition(FRONT_CLAW_OPENED);
        backWrist.setPosition(0);
        backClaw.setPosition(BACK_CLAW_CLOSED);
        rotWrist.setPosition(rotWristPos);
        tongue.setPosition(0);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);



        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        blinkinLedDriver.setPattern(pattern);

        leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        imu.resetYaw();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*scaling equation:
             * P = sign(x) * k * |x|^n
             * P: scaled value
             * x: gamepad input
             * sign(x): pos/neg of x
             * k: max power constant
             * n: reduces sensitivity for smaller values of x - greater value of n makes smaller values less powerful
             * */
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double yscaled = y != 0 ? Math.signum(y) * Math.pow(Math.abs(y), 2) : 0;
            double x = gamepad1.left_stick_x;
            double xscaled = x != 0 ? Math.signum(x) * Math.pow(Math.abs(x), 2) : 0;
            double rx = gamepad1.right_stick_x;
            double rxscaled = rx != 0 ? Math.signum(rx) * Math.pow(Math.abs(rx), 2) : 0;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.x) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = xscaled * Math.cos(-botHeading) - yscaled * Math.sin(-botHeading);
            double rotY = xscaled * Math.sin(-botHeading) + yscaled * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rxscaled) / denominator;
            double backLeftPower = (rotY - rotX + rxscaled) / denominator;
            double frontRightPower = (rotY - rotX - rxscaled) / denominator;
            double backRightPower = (rotY + rotX - rxscaled) / denominator;

            leftFrontDrive.setPower(0.8*frontLeftPower);
            leftBackDrive.setPower(0.8*backLeftPower);
            rightFrontDrive.setPower(0.8*frontRightPower);
            rightBackDrive.setPower(0.8*backRightPower);

            slide();
            arm();
            tongue();
            grabDeposit();
            claw();
            lights();
            tiltHang();
            backClaw();
            rotWrist();
            telemetry.addData("slide height", slideR.getCurrentPosition());
            if(bucketMode){
                telemetry.addData("Mode: ", "Bucket/Hang");
            }
            else{
                telemetry.addData("Mode: ", "Specimen");
            }
            telemetry.addData("arm pos", armHinge.getCurrentPosition());
            telemetry.addData("tgt pos", armHinge.getTargetPosition());
            telemetry.update();
        }
    }

    public void slide(){
        if(!guidePressed){
            if(gamepad2.guide && bucketMode){
                guidePressed = true;
                bucketMode = false;
            }
            else if(gamepad2.guide){
                guidePressed = true;
                bucketMode = true;
            }
        }
        else{
            if (!gamepad2.guide){
                guidePressed = false;
            }
        }
        if(bucketMode){
            if(slideR.getCurrentPosition() < 20 && slideLevel == 0){
                //turns off motors when at 0 position
                slideR.setMotorDisable();
                slideL.setMotorDisable();
                /*  comment out the hang slides -- now using B-Button (i.e. bPressed)
                if(gamepad2.dpad_down && slideLevel == 0){
                    slideInput = true;
                    slideTarget = 0;
                    slideR.setMotorEnable();
                    slideL.setMotorEnable();
                    slideR.setVelocity(5000);
                    slideL.setVelocity(5000);
                    slideR.setTargetPosition(slideTarget);
                    slideL.setTargetPosition(slideTarget);
                    slideLevel = -1;
                    telemetry.addData("level:","0");
                    telemetry.addData("power:", "OFF");
                    telemetry.update();
                }
                */
            }
            if (!slideInput){ //slide input true is dpad clicked
                if (gamepad2.dpad_up) { //when up on dpad is pressed
                    slideMoving = true;
                    slideInput = true;
                    slideR.setMotorEnable();
                    slideL.setMotorEnable();
                    if(slideLevel <= 0){ //low basket
                        slideR.setVelocity(5000);
                        slideL.setVelocity(5000);
                        slideTarget = 1450; //1360;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 1;
                        telemetry.addData("level:","1");
                        telemetry.update();
                    }
                    else if(slideLevel == 1){
                        slideTarget = 2750;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 2;
                        telemetry.addData("level:","2");
                        telemetry.update();
                    }
                }
                else if(gamepad2.dpad_down){
                    slideInput = true;
                    if(slideLevel == 2){ //low basket
                        slideTarget = 1450; //1360;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 1;
                        telemetry.addData("level:","1");
                        telemetry.update();
                    }
                    else if(slideLevel == 1){ //down
                        slideTarget = 0;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 0;
                        telemetry.addData("level:","0");
                        telemetry.update();
                    }
                    else if(slideLevel <= 0){
                        slideTarget = 0;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = -1;
                        telemetry.addData("level:","0");
                        telemetry.update();
                    }
                }
            }
            else {
                if (!gamepad2.dpad_down && !gamepad2.dpad_up && !yPressed){ //when arrow button (dpad up/down) is released
                    slideInput = false;
                    slideMoving = false;
                /*telemetry.addData("Right position:", slideR.getCurrentPosition());
                telemetry.addData("Left position:", slideL.getCurrentPosition());
                telemetry.addData("velocity", slideR.getVelocity());
                telemetry.update();*/
                }
            }
        }
        else{
            // Specimen mode
            if(slideR.getCurrentPosition() < 20 && slideLevel == 0){
                //turns off motors when at 0 position
                slideR.setMotorDisable();
                slideL.setMotorDisable();
                if(gamepad2.dpad_down && slideLevel == 0){
                    slideInput = true;
                    slideTarget = SLIDES_SPECIMEN_DOWN;
                    slideR.setMotorEnable();
                    slideL.setMotorEnable();
                    slideR.setVelocity(5000);
                    slideL.setVelocity(5000);
                    slideR.setTargetPosition(slideTarget);
                    slideL.setTargetPosition(slideTarget);
                    slideLevel = -1;
                    telemetry.addData("level:","0");
                    telemetry.addData("power:", "OFF");
                    telemetry.update();
                }
            }
            if (!slideInput){ //slide input true is dpad clicked
                if (gamepad2.dpad_up) { //when up on dpad is pressed
                    slideMoving = true;
                    slideInput = true;
                    slideR.setMotorEnable();
                    slideL.setMotorEnable();
                    if(slideLevel <= 0){ //low basket
                        slideR.setVelocity(5000);
                        slideL.setVelocity(5000);
                        slideTarget = SLIDES_SPECIMEN_PREP_HANG;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 1;
                        telemetry.addData("level:","1");
                        telemetry.update();
                    }
                    else if(slideLevel == 1){
                        slideTarget = SLIDES_SPECIMEN_HANG;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 2;
                        telemetry.addData("level:","2");
                        telemetry.update();
                        sleep(500);
                        backClaw.setPosition(BACK_CLAW_OPENED);
                    }
                }
                else if(gamepad2.dpad_down){
                    slideInput = true;
                    if(slideLevel == 2){ //low basket
                        slideTarget = SLIDES_SPECIMEN_PREP_HANG;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 1;
                        telemetry.addData("level:","1");
                        telemetry.update();
                    }
                    else if(slideLevel == 1){ //down
                        slideTarget = SLIDES_SPECIMEN_DOWN;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 0;
                        telemetry.addData("level:","0");
                        telemetry.update();
                    }
                    else if(slideLevel <= 0){
                        slideTarget = SLIDES_SPECIMEN_DOWN;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = -1;
                        telemetry.addData("level:","0");
                        telemetry.update();
                    }
                }
            }
            else {
                if (!gamepad2.dpad_down && !gamepad2.dpad_up && !yPressed){ //when arrow button (dpad up/down) is released
                    slideInput = false;
                    slideMoving = false;
                /*telemetry.addData("Right position:", slideR.getCurrentPosition());
                telemetry.addData("Left position:", slideL.getCurrentPosition());
                telemetry.addData("velocity", slideR.getVelocity());
                telemetry.update();*/
                }
            }
        }
    }

    public void arm(){
        armHinge.setTargetPosition(armTarget);
        int armSpeed = 20;
        if (armTarget < -750)
            armSpeed = 5;
        if (armTarget>= -890 && gamepad2.right_stick_y > 0){
            armHinge.setMotorEnable();
            armHinge.setVelocity(900);
            // Go down
            armTarget -= (int)(gamepad2.right_stick_y * armSpeed);
            if (armTarget<-890){
                armTarget = -890;
            }
            armMoving = true;
            telemetry.addData("arm pos", armHinge.getCurrentPosition());
            telemetry.addData("tgt pos", armHinge.getTargetPosition());
            telemetry.update();
            //upDown.setVelocity(500*upness);
        }
        else if (gamepad2.right_stick_y < 0 && armTarget <=0){
            armHinge.setMotorEnable();
            armHinge.setVelocity(900);
            // Go up
            armTarget -= (int)(gamepad2.right_stick_y * armSpeed);
            if (armTarget>0){
                armTarget = 0;
            }
            armMoving = true;
            telemetry.addData("arm pos", armHinge.getCurrentPosition());
            telemetry.addData("tgt pos", armHinge.getTargetPosition());
            telemetry.update();
        } else if (gamepad2.right_stick_y==0 && armHinge.getCurrentPosition() > -500) {
            armHinge.setMotorDisable();
        }
    }

    public void tongue(){
        if (gamepad2.left_bumper && tonguePos > 0) {
            tonguePos -= 0.01;
        } else if (gamepad2.right_bumper && tonguePos <0.37) {
            tonguePos += 0.01;
        }
        tongue.setPosition(tonguePos);

    }

    public void grabDeposit(){
        if(!yPressed) {
            if (gamepad2.y && depositMode) { //transfer from front claw to back
                yPressed = true;
                //motor first
                int target = SLIDES_SPECIMEN_TRANSFER;
                backClaw.setPosition(BACK_CLAW_OPENED);
                backWrist.setPosition(0.62);
                rotWrist.setPosition(FRONT_WRIST_HORIZONTAL);
                rotWristPos = FRONT_WRIST_HORIZONTAL;
                wrist.setPosition(0.04);
                tongue.setPosition(0);
                tonguePos = 0;
                while((slideR.getCurrentPosition()>(target+10) || slideR.getCurrentPosition()<(target-10)) && opModeIsActive()){
                    slideR.setVelocity(1000);
                    slideL.setVelocity(1000);
                    slideR.setTargetPosition(target);
                    slideL.setTargetPosition(target);
                    slideLevel=0;
                    telemetry.addData("SlideR Pos", slideR.getCurrentPosition());
                    telemetry.addData("SlideR Tgt", slideR.getTargetPosition());
                    telemetry.update();
                }
                armTarget = ARM_POS_UP;
                int curPos = armHinge.getCurrentPosition();
                while ((curPos<(ARM_POS_UP-5) || curPos>(ARM_POS_UP+5))&& opModeIsActive()) {
                    armHinge.setMotorEnable();
                    armHinge.setVelocity(1500);
                    armHinge.setTargetPosition(ARM_POS_UP);
                    armMoving = true;
                    curPos = armHinge.getCurrentPosition();
                }
                backClaw.setPosition(BACK_CLAW_CLOSED);
                sleep(300);
                claw.setPosition(FRONT_CLAW_OPENED);
                sleep(200);
                backWrist.setPosition(0);
                depositMode = false;

                // get slide in prep position
                slideR.setVelocity(5000);
                slideL.setVelocity(5000);
                slideTarget = SLIDES_SPECIMEN_PREP_HANG;
                slideR.setTargetPosition(slideTarget);
                slideL.setTargetPosition(slideTarget);
                slideLevel = 1;

            }
            else if(gamepad2.y){ //grab mode
                yPressed = true;
                tongue.setPosition(0.2);
                tonguePos = 0.2;
                wrist.setPosition(0.7);
                claw.setPosition(FRONT_CLAW_OPENED);
                armTarget = ARM_POS_DOWN;
                int curPos = armHinge.getCurrentPosition();
                while (curPos!=ARM_POS_DOWN && opModeIsActive()) {
                    armHinge.setMotorEnable();
                    armHinge.setVelocity(800);
                    armHinge.setTargetPosition(ARM_POS_DOWN);
                    armMoving = true;
                    curPos = armHinge.getCurrentPosition();
                }
                depositMode = true;
            }
        }
        else if (!gamepad2.y){
            yPressed = false;
            armMoving = false;
        }
    }

    public void claw(){//open-close
        if(armHinge.getCurrentPosition()<-500){
            wrist.setPosition(0.7);
        }
        else if(armHinge.getCurrentPosition()>-500 && !yPressed){
            wrist.setPosition(0.48);
        }
        if(!xPressed){
            if(gamepad2.x && claw.getPosition()>=(FRONT_CLAW_CLOSED-0.1)){
                xPressed = true;
                claw.setPosition(FRONT_CLAW_OPENED);
            }
            else if(gamepad2.x){
                xPressed = true;
                claw.setPosition(FRONT_CLAW_CLOSED);
            }
        }
        else{
            if (!gamepad2.x){
                xPressed = false;
            }
        }
    }

    public void tiltHang(){
        if(!bPressed){
            if(gamepad2.b){
                bPressed = true;

                // arm down first
                armHinge.setMotorEnable();
                armHinge.setVelocity(900);
                armHinge.setTargetPosition(ARM_POS_DOWN);

                // Start hang: robot needs to be backed up against submersible.
                // Roll forward few inches to avoid crashing claw into bar
                leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                leftFrontDrive.setTargetPosition(50);
                leftBackDrive.setTargetPosition(50);
                rightFrontDrive.setTargetPosition(50);
                rightBackDrive.setTargetPosition(50);
                leftFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                leftBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightFrontDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                rightBackDrive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                leftFrontDrive.setVelocity(200);
                leftBackDrive.setVelocity(200);
                rightFrontDrive.setVelocity(200);
                rightBackDrive.setVelocity(200);

                // Set the arm and wrist in position to push down
                tongue.setPosition(0.2);
                wrist.setPosition(0);

                sleep(1000);

                // Put slides up
                slideTarget = SLIDES_ROBOT_HANG;
                slideR.setTargetPosition(slideTarget);
                slideL.setTargetPosition(slideTarget);
                slideR.setVelocity(5000);
                slideL.setVelocity(5000);
                slideLevel = 2;

                sleep(1500);

                // Tilt the robot -- push arm down
                while (armHinge.getCurrentPosition() > (ARM_POS_TILT+50) && opModeIsActive()) {
                    armHinge.setTargetPosition(ARM_POS_TILT);
                    armMoving = true;
                }

                // Retract the slides -- hang the robot
                slideTarget = SLIDES_SPECIMEN_DOWN;
                slideR.setTargetPosition(slideTarget);
                slideL.setTargetPosition(slideTarget);
                slideLevel = 0;

                // Pull the arm up
                armHinge.setTargetPosition(-50);

                // Stop drive motors
                leftFrontDrive.setMotorDisable();
                leftBackDrive.setMotorDisable();
                rightFrontDrive.setMotorDisable();
                rightBackDrive.setMotorDisable();

                sleep(5000);
                armHinge.setMotorDisable();

                // Wait for end of match
                sleep(30000);
            }
        }
        else{
            if (!gamepad2.b){
                bPressed = false;
            }
        }
    }

    public void backClaw(){
        if(!aPressed){
            if(gamepad2.a && backClaw.getPosition()>=(BACK_CLAW_CLOSED-0.1)){
                aPressed = true;
                backClaw.setPosition(BACK_CLAW_OPENED);
            }
            else if(gamepad2.a){
                aPressed = true;
                backClaw.setPosition(BACK_CLAW_CLOSED);
            }
        }
        else{
            if (!gamepad2.a){
                aPressed = false;
            }
        }
    }


    public void rotWrist(){
        if (rotWrist.getPosition() < 1 && gamepad2.left_stick_x < 0){
            // Go right
            rotWristPos += 0.01;
            rotWrist.setPosition(rotWristPos);
            //upDown.setVelocity(500*upness);
        }
        else if (gamepad2.left_stick_x > 0 && rotWrist.getPosition() > 0){
            // Go left
            rotWristPos -= 0.01;
            rotWrist.setPosition(rotWristPos);
            //upDown.setVelocity(500*upness);
        }
    }

    public void lights(){
        if(runtime.seconds()<=60 && runtime.seconds()>=59){
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_STROBE;
            blinkinLedDriver.setPattern(pattern);
        }
        if(runtime.seconds()<=105 && runtime.seconds()>=104){
            pattern = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
            blinkinLedDriver.setPattern(pattern);
        }

    }
}
