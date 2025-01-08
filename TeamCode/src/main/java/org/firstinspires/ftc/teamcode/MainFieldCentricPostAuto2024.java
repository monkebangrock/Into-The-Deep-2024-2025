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
public class MainFieldCentricPostAuto2024 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
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
    private Servo backWrist;
    private Servo backClaw;
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
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;


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
        backWrist = hardwareMap.get(Servo.class, "backWrist");
        backClaw = hardwareMap.get(Servo.class, "backClaw");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        //reset encoder
        leftFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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
        wrist.setPosition(0);
        claw.setPosition(0.2);
        backWrist.setPosition(0);
        backClaw.setPosition(0);

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

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            slide();
            arm();
            tongue();
            grabDeposit();
            claw();
            lights();
            tiltHang();
            backClaw();
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
                        slideTarget = 1360;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 1;
                        telemetry.addData("level:","1");
                        telemetry.update();
                    }
                    else if(slideLevel == 1 && armHinge.getCurrentPosition()> -300){ //high basket
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
                        slideTarget = 1360;
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
            if(slideR.getCurrentPosition() < 20 && slideLevel == 0){
                //turns off motors when at 0 position
                slideR.setMotorDisable();
                slideL.setMotorDisable();
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
                        slideTarget = 800;
                        slideR.setTargetPosition(slideTarget);
                        slideL.setTargetPosition(slideTarget);
                        slideLevel = 1;
                        telemetry.addData("level:","1");
                        telemetry.update();
                    }
                    else if(slideLevel == 1 && armHinge.getCurrentPosition()> -300){ //high basket
                        slideTarget = 1225;
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
                        slideTarget = 800;
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
    }

    public void arm(){
        armHinge.setTargetPosition(armTarget);
        //armHinge.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (armTarget>= -890 && gamepad2.right_stick_y > 0 && slideLevel<2){
            armHinge.setVelocity(900);
            armHinge.setMotorEnable();
            // Go down
            armTarget -= (int)(gamepad2.right_stick_y*20);
            if (armTarget<-890){
                armTarget = -890;
            }
            armMoving = true;
            telemetry.addData("arm pos", armHinge.getCurrentPosition());
            telemetry.addData("tgt pos", armHinge.getTargetPosition());
            telemetry.update();
            //upDown.setVelocity(500*upness);
        }
        else if (gamepad2.right_stick_y < 0 && armTarget <=0 && slideLevel<2){
            armHinge.setVelocity(900);
            armHinge.setMotorEnable();
            // Go up
            armTarget -= (int)(gamepad2.right_stick_y*20);
            if (armTarget>0){
                armTarget = 0;
            }
            armMoving = true;
            telemetry.addData("arm pos", armHinge.getCurrentPosition());
            telemetry.addData("tgt pos", armHinge.getTargetPosition());
            telemetry.update();
            //upDown.setVelocity(500*upness);
        }
        else {
            if (armMoving){
                armTarget = armHinge.getCurrentPosition();
                armHinge.setMotorDisable();
                armMoving = false;
                // STOP
                //upDown.setVelocity(0);
                //Do nothing for now - holding position
            }
        }

    }

    public void tongue(){
        if (gamepad2.left_bumper) {
            tongue.setPower(1);
        } else if (gamepad2.right_bumper) {
            tongue.setPower(-1);
        } else {
            tongue.setPower(0);
        }
    }

    public void grabDeposit(){
        if(!yPressed) {
            if (gamepad2.y && depositMode) { //transfer from front claw to back
                yPressed = true;
                //motor first
                telemetry.addData("Sdlie r", slideR.getCurrentPosition());
                telemetry.addData("tgt", slideR.getTargetPosition());
                telemetry.update();
                while((slideR.getCurrentPosition()>190 || slideR.getCurrentPosition()<170) && opModeIsActive()){
                    slideR.setVelocity(1000);
                    slideL.setVelocity(1000);
                    slideR.setTargetPosition(180);
                    slideL.setTargetPosition(180);
                    slideLevel=0;
                }
                backClaw.setPosition(0.35);
                backWrist.setPosition(0.65);
                while (armHinge.getCurrentPosition() < 0 && opModeIsActive()) {
                    armHinge.setVelocity(900);
                    armHinge.setMotorEnable();
                    armHinge.setTargetPosition(0);
                    tongue.setPower(1);
                    armMoving = true;
                }
                armHinge.setMotorDisable();
                tongue.setPower(0);
                while(wrist.getPosition()!= 0 && opModeIsActive()){
                    wrist.setPosition(0);
                }
                sleep(600);
                backClaw.setPosition(0);
                sleep(300);
                claw.setPosition(0.2);
                sleep(300);
                backWrist.setPosition(0);
                depositMode = false;
            }
            else if(gamepad2.y){ //grab mode
                yPressed = true;
                /*while(slideR.getCurrentPosition()>1360){
                    slideR.setVelocity(1000);
                    slideL.setVelocity(1000);
                    slideR.setTargetPosition(210);
                    slideL.setTargetPosition(210);
                    slideLevel=0;
                }*/
                while (armHinge.getCurrentPosition() > -550 && opModeIsActive()) {
                    armHinge.setVelocity(900);
                    armHinge.setMotorEnable();
                    armHinge.setTargetPosition(-550);
                    if(armHinge.getCurrentPosition() < -450){
                        tongue.setPower(-1);
                    }
                    else{
                        tongue.setPower(0);
                    }
                    armMoving = true;
                }
                armHinge.setMotorDisable();
                wrist.setPosition(0.62);
                claw.setPosition(0.2);
                depositMode = true;
            }
        }
        else if (!gamepad2.y){
            yPressed = false;
            armMoving = false;
        }
    }

    public void claw(){//open-close
        if(armHinge.getCurrentPosition()<-300 && !bPressed){
            wrist.setPosition(0.62);
        }
        if(!xPressed){
            if(gamepad2.x && claw.getPosition()==0.55){
                xPressed = true;
                claw.setPosition(0.2);
            }
            else if(gamepad2.x){
                xPressed = true;
                claw.setPosition(0.55);
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
                wrist.setPosition(0);
                while (armHinge.getCurrentPosition() > -1400 && opModeIsActive()) {
                    armHinge.setTargetPosition(-1400);
                    armMoving = true;
                }
                int temp = (int)(getRuntime());
                while(getRuntime() < temp+1 && opModeIsActive()){
                    tongue.setPower(-1);
                }
                temp = (int)(getRuntime());
                while(getRuntime()< temp+1 && opModeIsActive()){
                    leftFrontDrive.setPower(-1);
                    leftBackDrive.setPower(-1);
                    rightFrontDrive.setPower(-1);
                    rightBackDrive.setPower(-1);
                }
                leftFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
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
            if(gamepad2.a && backClaw.getPosition()==0){
                aPressed = true;
                backClaw.setPosition(0.35);
            }
            else if(gamepad2.a){
                aPressed = true;
                backClaw.setPosition(0);
            }
        }
        else{
            if (!gamepad2.a){
                aPressed = false;
            }
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
