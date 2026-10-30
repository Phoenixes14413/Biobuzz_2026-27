package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "JoystickServoControl", group = "Simple")
public class JoystickServoControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        Servo servo;
        try {
            servo = hardwareMap.get(Servo.class, "servo 0");
        } catch (Exception e) {
            telemetry.addData("Error", "Servo not found. Check config name!");
            telemetry.update();
            while (!isStopRequested() && !opModeIsActive()) idle();
            return;
        }
        telemetry.addData("Status", "Ready");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            double stick = -gamepad1.left_stick_y;
            // For continuous rotation servos: 0.5 = stop, 1 = full forward, 0 = full reverse
            double power = 0.5 + 0.5 * stick;
            servo.setPosition(power);
            telemetry.addData("Servo Power (pos)", power);
            telemetry.update();
        }
    }
}
