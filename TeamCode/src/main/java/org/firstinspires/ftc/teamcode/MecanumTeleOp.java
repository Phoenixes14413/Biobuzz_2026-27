package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum TeleOp FTCLib", group = "Linear Opmode")
public class MecanumTeleOp extends LinearOpMode {

    // FTCLib Motor objects for drive
    private Motor frontLeft, frontRight, backLeft, backRight;

    // FTCLib MecanumDrive
    private MecanumDrive drive;

    // FTCLib GamepadEx for enhanced gamepad features
    private GamepadEx driverOp, toolOp;

    // Motors for intake and outtake
    private MotorEx intakeMotor, outtakeMotor;

    // Servo for finger mechanism
    private SimpleServo fingerServo;

    // Odometry encoders (dead wheel pods)
    private MotorEx leftEncoder, rightEncoder, perpEncoder;

    // FTCLib Odometry
    private HolonomicOdometry odometry;

    // Odometry constants - ADJUST THESE FOR YOUR ROBOT
    private static final double TRACKWIDTH = 13.7; // Distance between parallel encoders (inches)
    private static final double CENTER_WHEEL_OFFSET = 2.4; // Distance from center of rotation to perp encoder (inches)
    private static final double TICKS_PER_INCH = 307.699557; // Encoder ticks per inch - tune this value

    // Servo positions
    private static final double FINGER_OPEN = 0.8;
    private static final double FINGER_CLOSED = 0.4;

    @Override
    public void runOpMode() {
        // Initialize drive motors using FTCLib Motor class
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        // Create MecanumDrive object
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Initialize gamepads with FTCLib GamepadEx
        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        // Initialize intake/outtake motors
        intakeMotor = new MotorEx(hardwareMap, "intake");
        outtakeMotor = new MotorEx(hardwareMap, "outtake");

        // Initialize finger servo (0-270 degree range example)
        fingerServo = new SimpleServo(hardwareMap, "fingerServo", 0, 270);

        // Initialize odometry encoders (dead wheel pods)
        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        perpEncoder = new MotorEx(hardwareMap, "perpEncoder");

        // Set distance per pulse for encoders
        leftEncoder.setDistancePerPulse(1.0 / TICKS_PER_INCH);
        rightEncoder.setDistancePerPulse(1.0 / TICKS_PER_INCH);
        perpEncoder.setDistancePerPulse(1.0 / TICKS_PER_INCH);

        // Reset encoders
        leftEncoder.resetEncoder();
        rightEncoder.resetEncoder();
        perpEncoder.resetEncoder();

        // Initialize HolonomicOdometry with suppliers
        odometry = new HolonomicOdometry(
                leftEncoder::getDistance,
                rightEncoder::getDistance,
                perpEncoder::getDistance,
                TRACKWIDTH,
                CENTER_WHEEL_OFFSET
        );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Read all button states at once for efficiency
            driverOp.readButtons();
            toolOp.readButtons();

            // Gamepad 1: Driving using FTCLib MecanumDrive
            // driveRobotCentric takes (strafeSpeed, forwardSpeed, turnSpeed)
            drive.driveRobotCentric(
                    driverOp.getLeftX(),      // Strafe (x)
                    -driverOp.getLeftY(),     // Forward (y) - negative because Y is inverted
                    driverOp.getRightX()      // Turn (rotation)
            );

            // Update odometry position
            odometry.updatePose();

            // Gamepad 2: Intake/Outtake/Finger
            // Intake motor controlled by left joystick y
            double intakePower = -toolOp.getLeftY();
            intakeMotor.set(intakePower);

            // Outtake motor controlled by right joystick y
            double outtakePower = -toolOp.getRightY();
            outtakeMotor.set(outtakePower);

            // Finger servo controlled by triggers
            if (toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) {
                fingerServo.setPosition(FINGER_OPEN); // Open
            } else if (toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1) {
                fingerServo.setPosition(FINGER_CLOSED); // Close
            }

            // Right bumper: servo up and down quickly
            if (toolOp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                fingerServo.setPosition(FINGER_OPEN);
                sleep(1000); // Wait 1 second
                fingerServo.setPosition(FINGER_CLOSED);
            }

            // Get current position from odometry
            double x = odometry.getPose().getX();
            double y = odometry.getPose().getY();
            double heading = odometry.getPose().getHeading();

            // Calculate displacement vector from origin
            double displacementMagnitude = Math.sqrt(x * x + y * y);
            double displacementAngle = Math.toDegrees(Math.atan2(y, x));

            // Telemetry
            telemetry.addData("Status", "Running");
            telemetry.addLine();

            // Drive controls
            telemetry.addData("Drive", "X:%.2f Y:%.2f R:%.2f",
                    driverOp.getLeftX(), -driverOp.getLeftY(), driverOp.getRightX());

            // Motor powers and velocities
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Outtake Power", "%.2f", outtakePower);
            telemetry.addData("Intake RPM", "%.0f", intakeMotor.getVelocity());
            telemetry.addData("Outtake RPM", "%.0f", outtakeMotor.getVelocity());

            // Drive motor velocities
            telemetry.addLine();
            telemetry.addData("FL RPM", "%.0f", frontLeft.getCorrectedVelocity());
            telemetry.addData("FR RPM", "%.0f", frontRight.getCorrectedVelocity());
            telemetry.addData("BL RPM", "%.0f", backLeft.getCorrectedVelocity());
            telemetry.addData("BR RPM", "%.0f", backRight.getCorrectedVelocity());

            // Servo position
            telemetry.addLine();
            telemetry.addData("Finger Position", "%.2f", fingerServo.getPosition());

            // Odometry telemetry
            telemetry.addLine();
            telemetry.addData("Position", "X: %.2f\" Y: %.2f\"", x, y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(heading));
            telemetry.addData("Displacement Vector", "Mag: %.2f\" Angle: %.1f°",
                    displacementMagnitude, displacementAngle);

            telemetry.update();
        }
    }
}