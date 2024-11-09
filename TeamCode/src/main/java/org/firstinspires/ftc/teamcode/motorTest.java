package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class motorTest extends LinearOpMode{
    private DcMotor motorTest;

    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        motorTest.setPower(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            motorTest.setPower(0.7);
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}

