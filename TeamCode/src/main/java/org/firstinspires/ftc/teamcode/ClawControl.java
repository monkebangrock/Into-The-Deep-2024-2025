package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Claw Control", group = "Linear Opmode")
public class ClawControl extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        Servo servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double tgtPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(tgtPower);

            if (gamepad1.y) {
                servoTest.setPosition(0.4);
            } else if (gamepad1.x) {
                servoTest.setPosition(0.65);
            }

            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.update();
        }
    }
}

