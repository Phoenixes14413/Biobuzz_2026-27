package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TestServoAndMotorSimple", group = "Simple")
public class testServoAndMotor extends OpMode {
    private DcMotor testMotor;
    private Servo testServo;
    private boolean servoOn = false;
    private boolean motorOn = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private double lastPosition = 0;
    private long lastTime = 0;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        testServo = hardwareMap.get(Servo.class, "testServo");
        testMotor.setPower(0);
        testServo.setPosition(0);
        lastPosition = testMotor.getCurrentPosition();
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;

        // Toggle servo full power (0 or 1) with A
        if (aPressed && !lastA) {
            servoOn = !servoOn;
            testServo.setPosition(servoOn ? 1.0 : 0.0);
        }
        lastA = aPressed;

        // Toggle motor full power (1 or 0) with B
        if (bPressed && !lastB) {
            motorOn = !motorOn;
            testMotor.setPower(motorOn ? 1.0 : 0.0);
        }
        lastB = bPressed;

        // Calculate and log RPM
        double currentPosition = testMotor.getCurrentPosition();
        long currentTime = System.nanoTime();
        double deltaPos = currentPosition - lastPosition;
        double deltaTime = (currentTime - lastTime) / 1e9; // seconds
        double rpm = 0;
        if (deltaTime > 0) {
            rpm = (deltaPos / 288.0) / deltaTime * 60.0; // 288 ticks/rev (adjust if needed)
        }
        lastPosition = currentPosition;
        lastTime = currentTime;

        telemetry.addData("Servo On", servoOn);
        telemetry.addData("Motor On", motorOn);
        telemetry.addData("Motor RPM", rpm);
        telemetry.update();
    }
}
