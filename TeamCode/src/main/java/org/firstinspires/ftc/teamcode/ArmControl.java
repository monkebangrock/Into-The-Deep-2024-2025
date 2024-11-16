package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmControl extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo armServo = hardwareMap.get(Servo.class, "armServo");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                armServo.setPosition(1.0);
            }

            else if (gamepad1.dpad_down) {
                armServo.setPosition(0.0);
            } else {
                armServo.setPosition(0.5);
            }


            telemetry.addData("Servo Position", armServo.getPosition());
            telemetry.update();
        }
    }
}
