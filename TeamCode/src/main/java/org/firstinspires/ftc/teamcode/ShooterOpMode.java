package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Shooter Control", group="TeleOp")
public class ShooterOpMode extends OpMode {

    private DcMotor rightWheel;
    private DcMotor leftWheel;

    private int speedLevel = 0;
    private boolean rightBumperPressed = false;

    @Override
    public void init() {
        // Initialize the motors
        rightWheel = hardwareMap.get(DcMotor.class, "rightWheel");
        leftWheel = hardwareMap.get(DcMotor.class, "leftWheel");

        // Reverse motor directions (adjust as needed for your robot)
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Detect right bumper press (only increment once per press)
        if (gamepad1.right_bumper && !rightBumperPressed) {
            speedLevel += 1000;
            rightBumperPressed = true;
        } else if (!gamepad1.right_bumper) {
            rightBumperPressed = false;
        }

        // Convert speed level to motor power (0.0 to 1.0)
        double motorPower = Math.min(speedLevel / 10000.0, 1.0);

        // Set motor power
        rightWheel.setPower(motorPower);
        leftWheel.setPower(motorPower);

        // Display current speed on driver station
        telemetry.addData("Speed Level", speedLevel);
        telemetry.addData("Motor Power", "%.2f", motorPower);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Stop the motors when OpMode is stopped
        rightWheel.setPower(0);
        leftWheel.setPower(0);
    }
}