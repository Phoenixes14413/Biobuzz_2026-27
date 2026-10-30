package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * RobotControls class containing all basic robot control functions
 * Used by both TeleOp and Autonomous modes
 */
public class RobotControls {

    // Hardware components
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private Servo fingerServo;

    // Odometry configuration (dead wheel pods using shared encoder ports)
    private String horizontalOdometryPort = "backLeft"; // Horizontal odometry pod shares this motor's encoder port
    private String verticalOdometryPort = "backRight";   // Vertical odometry pod shares this motor's encoder port

    // Odometry tracking variables
    private int lastHorizontalEncoder = 0;
    private int lastVerticalEncoder = 0;
    private double robotX = 0; // Horizontal displacement (inches)
    private double robotY = 0; // Vertical displacement (inches)

    // Odometry constants (adjust based on your robot)
    private final double TICKS_PER_INCH = 1120; // Adjust for your odometry wheel encoders
    private final double HORIZONTAL_ODOMETRY_WHEEL_DIAMETER = 2.0; // inches
    private final double VERTICAL_ODOMETRY_WHEEL_DIAMETER = 2.0;   // inches

    /**
     * Initialize all robot hardware
     */
    public void initHardware(HardwareMap hardwareMap) {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize mechanism motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");

        // Initialize servos
        fingerServo = hardwareMap.get(Servo.class, "fingerServo");

        // Set motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set motor modes for odometry (keep encoders active)
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize odometry
        resetOdometry();
    }

    /**
     * Set drive motor powers for mecanum movement
     */
    public void setWheelPower(double frontLeft, double frontRight, double backLeft, double backRight) {
        this.frontLeft.setPower(frontLeft);
        this.frontRight.setPower(frontRight);
        this.backLeft.setPower(backLeft);
        this.backRight.setPower(backRight);
    }

    /**
     * Set drive motor powers with direction control
     * @param forward forward/backward power (-1 to 1)
     * @param strafe strafe power (-1 to 1)
     * @param rotate rotation power (-1 to 1)
     */
    public void setDriveDirection(double forward, double strafe, double rotate) {
        // Denominator ensures all powers maintain the same ratio
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1.0);

        double frontLeftPower = (forward + strafe + rotate) / denominator;
        double backLeftPower = (forward - strafe + rotate) / denominator;
        double frontRightPower = (forward - strafe - rotate) / denominator;
        double backRightPower = (forward + strafe - rotate) / denominator;

        setWheelPower(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    /**
     * Stop all drive motors
     */
    public void stopDrive() {
        setWheelPower(0, 0, 0, 0);
    }

    /**
     * Set intake motor power
     * @param power motor power (-1 to 1)
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Set outtake motor power
     * @param power motor power (-1 to 1)
     */
    public void setOuttakePower(double power) {
        outtakeMotor.setPower(power);
    }

    /**
     * Stop both intake and outtake motors
     */
    public void stopIntakeOuttake() {
        setIntakePower(0);
        setOuttakePower(0);
    }

    /**
     * Move finger servo to specific position
     * @param position servo position (0.0 to 1.0)
     */
    public void setServoPosition(double position) {
        fingerServo.setPosition(position);
    }

    /**
     * Open finger servo
     */
    public void openFingerServo() {
        setServoPosition(0.62);
    }

    /**
     * Close finger servo
     */
    public void closeFingerServo() {
        setServoPosition(0.39);
    }

    /**
     * Get current finger servo position
     * @return current servo position (0.0 to 1.0)
     */
    public double getFingerPosition() {
        return fingerServo.getPosition();
    }

    // Odometry methods for dead wheel pods

    /**
     * Reset odometry to origin
     */
    public void resetOdometry() {
        lastHorizontalEncoder = getHorizontalEncoderTicks();
        lastVerticalEncoder = getVerticalEncoderTicks();
        robotX = 0;
        robotY = 0;
    }

    /**
     * Get horizontal encoder ticks from shared port
     */
    private int getHorizontalEncoderTicks() {
        switch (horizontalOdometryPort) {
            case "frontLeft": return frontLeft.getCurrentPosition();
            case "frontRight": return frontRight.getCurrentPosition();
            case "backLeft": return backLeft.getCurrentPosition();
            case "backRight": return backRight.getCurrentPosition();
            default: return backLeft.getCurrentPosition();
        }
    }

    /**
     * Get vertical encoder ticks from shared port
     */
    private int getVerticalEncoderTicks() {
        switch (verticalOdometryPort) {
            case "frontLeft": return frontLeft.getCurrentPosition();
            case "frontRight": return frontRight.getCurrentPosition();
            case "backLeft": return backLeft.getCurrentPosition();
            case "backRight": return backRight.getCurrentPosition();
            default: return backRight.getCurrentPosition();
        }
    }

    /**
     * Update odometry calculations using dead wheel encoders
     */
    public void updateOdometry() {
        int currentHorizontalTicks = getHorizontalEncoderTicks();
        int currentVerticalTicks = getVerticalEncoderTicks();

        // Calculate delta since last update
        int horizontalDelta = currentHorizontalTicks - lastHorizontalEncoder;
        int verticalDelta = currentVerticalTicks - lastVerticalEncoder;

        // Convert ticks to inches
        double horizontalInches = horizontalDelta / TICKS_PER_INCH;
        double verticalInches = verticalDelta / TICKS_PER_INCH;

        // Update position
        robotX += horizontalInches;
        robotY += verticalInches;

        // Store current values for next iteration
        lastHorizontalEncoder = currentHorizontalTicks;
        lastVerticalEncoder = currentVerticalTicks;
    }

    /**
     * Get current robot position
     * @return double array [x, y] in inches from origin
     */
    public double[] getPosition() {
        return new double[]{robotX, robotY};
    }

    /**
     * Configure odometry ports (which motor encoders the odometry wheels share)
     * @param horizontalPort motor name for horizontal odometry wheel
     * @param verticalPort motor name for vertical odometry wheel
     */
    public void setOdometryPorts(String horizontalPort, String verticalPort) {
        this.horizontalOdometryPort = horizontalPort;
        this.verticalOdometryPort = verticalPort;
    }

    // Autonomous driving methods using odometry

    /**
     * Drive X distance (strafe) using odometry feedback
     * @param targetDistance target distance in inches (positive = right)
     * @param power motor power (0.0 to 1.0)
     * @param opMode reference for opModeIsActive()
     */
    public void driveX(double targetDistance, double power, LinearOpMode opMode) {
        resetOdometry();
        double startX = robotX;

        while (opMode.opModeIsActive() && Math.abs(robotX - startX) < Math.abs(targetDistance)) {
            updateOdometry();
            double direction = targetDistance > 0 ? 1 : -1;
            setDriveDirection(0, direction * power, 0);
            opMode.telemetry.addData("Driving X", "Current: %.2f, Target: %.2f", robotX - startX, targetDistance);
            opMode.telemetry.update();
        }
        stopDrive();
    }

    /**
     * Drive Y distance (forward/backward) using odometry feedback
     * @param targetDistance target distance in inches (positive = forward)
     * @param power motor power (0.0 to 1.0)
     * @param opMode reference for opModeIsActive()
     */
    public void driveY(double targetDistance, double power, LinearOpMode opMode) {
        resetOdometry();
        double startY = robotY;

        while (opMode.opModeIsActive() && Math.abs(robotY - startY) < Math.abs(targetDistance)) {
            updateOdometry();
            double direction = targetDistance > 0 ? 1 : -1;
            setDriveDirection(direction * power, 0, 0);
            opMode.telemetry.addData("Driving Y", "Current: %.2f, Target: %.2f", robotY - startY, targetDistance);
            opMode.telemetry.update();
        }
        stopDrive();
    }

    /**
     * Rotate by angle using time-based control (simplified)
     * @param targetAngle target angle in degrees (positive = clockwise)
     * @param power motor power (0.0 to 1.0)
     * @param opMode reference for opModeIsActive()
     */
    public void rotate(double targetAngle, double power, LinearOpMode opMode) {
        // Simple time-based rotation (can be improved with IMU)
        double rotationTime = Math.abs(targetAngle) * 20; // Adjust based on your robot
        double direction = targetAngle > 0 ? 1 : -1;

        setDriveDirection(0, 0, direction * power);
        opMode.sleep((long) rotationTime);
        stopDrive();
    }
}