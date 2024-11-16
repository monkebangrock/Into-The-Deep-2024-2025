package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Local variables for hardware
        DcMotor motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        Servo servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double tgtPower = -this.gamepad1.left_stick_y;

            // Motor control
            motorTest.setPower(tgtPower);

            // Servo control
            if (gamepad1.y) {
                servoTest.setPosition(0.4);
            } else if (gamepad1.x) {
                servoTest.setPosition(0.65);
            }

            // Telemetry updates
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.update();
        }
    }
}
