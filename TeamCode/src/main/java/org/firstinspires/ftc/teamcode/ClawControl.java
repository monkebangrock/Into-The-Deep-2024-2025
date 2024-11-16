package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ClawControl extends LinearOpMode {
    private DcMotor clawMotor;
    private Servo clawServo;

    @Override
    public void runOpMode() {
        clawMotor = hardwareMap.get(DcMotor.class, "motorTest");
        clawServo = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Motor control with deadzone
            double tgtPower = -this.gamepad1.left_stick_y;
            if (Math.abs(tgtPower) < 0.05) {
                tgtPower = 0;
            }
            clawMotor.setPower(tgtPower);

            // Servo control
            if (gamepad1.y) {
                clawServo.setPosition(0.4);
            } else if (gamepad1.x) {
                clawServo.setPosition(0.65);
            }

            // Telemetry updates
            telemetry.addData("Status", "Running");
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", clawMotor.getPower());
            telemetry.addData("Servo Position", clawServo.getPosition());
            telemetry.update();
        }
    }
}