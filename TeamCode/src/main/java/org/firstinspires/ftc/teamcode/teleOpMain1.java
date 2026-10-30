package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

/*
 * teleOpMain1 quick reference
 * Drive: left stick controls strafe, right stick X controls rotate (gamepad1 only).
 * Intake: A toggles the intake motor at 1000 ticks/s (reversed direction).
 * Outtake: B toggles the outtake motor at 4800 ticks/s.
 * Finger Servo: Right trigger toggles servo between up and down (default down) - moves fast.
 * Telemetry displays drive power, toggle states, and servo position.
 */

@TeleOp(name = "teleOpMain1", group = "TeleOp")
public class teleOpMain1 extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotorEx intakeMotor;
    private DcMotorEx outtakeMotor;

    private ServoImplEx fingerServo;

    private static final double INTAKE_VELOCITY = -1000.0; // Half speed (1000) and reversed (negative)
    private static final double OUTTAKE_VELOCITY = 4800.0;

    private static final double FINGER_UP_POSITION = -0.2;  // Goes higher (negative value with extended PWM range)
    private static final double FINGER_DOWN_POSITION = 1.0; // Full down
    private static final double SERVO_SPEED = 1.0; // Speed: 0.0 (slow) to 1.0 (fast) - lower is faster!

    private boolean intakeEnabled = false;
    private boolean outtakeEnabled = false;
    private boolean fingerUp = false; // Default is down

    private boolean lastAPressed = false;
    private boolean lastBPressed = false;
    private boolean lastRightTrigger = false;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        fingerServo = hardwareMap.get(ServoImplEx.class, "fingerServo");

        // Extend servo PWM range to allow values beyond 0.0-1.0
        fingerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Set servo speed - lower values = faster movement
        fingerServo.setPosition(FINGER_DOWN_POSITION);
        // Note: Many servos don't support speed control, but we can try scaleRange
        fingerServo.scaleRange(0.0, 1.0); // Full range for maximum speed

        // Set motors to RUN_USING_ENCODER mode for velocity control
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setVelocity(0);
        outtakeMotor.setVelocity(0);

        telemetry.addLine("teleOpMain1 initialized");
        telemetry.addLine("Press A to toggle intake");
        telemetry.addLine("Press B to toggle outtake");
        telemetry.addLine("Right Trigger to toggle servo");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive();
        handleToggles();
        handleFingerServo();
        pushTelemetry();
    }

    private void drive() {
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        double fl = strafe + rotate;
        double bl = -strafe + rotate;
        double fr = -strafe - rotate;
        double br = strafe - rotate;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    private void handleToggles() {
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;

        // Toggle intake with A button
        if (aPressed && !lastAPressed) {
            intakeEnabled = !intakeEnabled;
            if (intakeEnabled) {
                intakeMotor.setVelocity(INTAKE_VELOCITY);
            } else {
                intakeMotor.setVelocity(0.0);
            }
        }

        // Toggle outtake with B button
        if (bPressed && !lastBPressed) {
            outtakeEnabled = !outtakeEnabled;
            if (outtakeEnabled) {
                outtakeMotor.setVelocity(OUTTAKE_VELOCITY);
            } else {
                outtakeMotor.setVelocity(0.0);
            }
        }

        lastAPressed = aPressed;
        lastBPressed = bPressed;
    }

    private void handleFingerServo() {
        boolean rightTrigger = gamepad1.right_trigger > 0.5; // Trigger pressed if > 0.5

        // Toggle servo with right trigger - instant position change for max speed
        if (rightTrigger && !lastRightTrigger) {
            fingerUp = !fingerUp;
            if (fingerUp) {
                fingerServo.setPosition(FINGER_UP_POSITION);
            } else {
                fingerServo.setPosition(FINGER_DOWN_POSITION);
            }
        }

        lastRightTrigger = rightTrigger;
    }

    private void pushTelemetry() {
        telemetry.addLine("=== MOTOR STATUS ===");
        telemetry.addData("Drive", "FL %.2f, BL %.2f, FR %.2f, BR %.2f",
                frontLeft.getPower(), backLeft.getPower(), frontRight.getPower(), backRight.getPower());

        // Debug info for intake
        telemetry.addData("Intake State", intakeEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Intake Target", "%.0f ticks/s", intakeEnabled ? INTAKE_VELOCITY : 0.0);
        telemetry.addData("Intake Actual", "%.0f ticks/s", intakeMotor.getVelocity());

        // Debug info for outtake
        telemetry.addData("Outtake State", outtakeEnabled ? "ENABLED" : "DISABLED");
        telemetry.addData("Outtake Target", "%.0f ticks/s", outtakeEnabled ? OUTTAKE_VELOCITY : 0.0);
        telemetry.addData("Outtake Actual", "%.0f ticks/s", outtakeMotor.getVelocity());

        telemetry.addData("Finger Servo", "%s (Position: %.2f)", fingerUp ? "UP" : "DOWN", fingerServo.getPosition());

        // Button state debug
        telemetry.addLine("\n=== BUTTON STATUS ===");
        telemetry.addData("A Button", gamepad1.a ? "PRESSED" : "released");
        telemetry.addData("B Button", gamepad1.b ? "PRESSED" : "released");
        telemetry.addData("Right Trigger", gamepad1.right_trigger > 0.5 ? "PRESSED" : "released");

        telemetry.addLine("\n=== CONTROLS ===");
        telemetry.addLine("DRIVE: Left Stick X (Strafe)");
        telemetry.addLine("       Right Stick X (Rotate)");
        telemetry.addLine("INTAKE: A Button (Toggle)");
        telemetry.addLine("OUTTAKE: B Button (Toggle)");
        telemetry.addLine("FINGER: Right Trigger (Toggle Up/Down - FAST)");

        telemetry.update();
    }
}