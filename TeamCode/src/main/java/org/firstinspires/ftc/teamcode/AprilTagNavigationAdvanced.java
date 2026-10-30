package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Advanced AprilTag Navigation with Multiple Tag Support
 * This OpMode demonstrates advanced AprilTag features:
 * - Navigate to specific tags by ID
 * - Multi-tag detection and selection
 * - Advanced positioning and alignment
 */
@Autonomous(name = "AprilTag Navigation Advanced", group = "Autonomous")
public class AprilTagNavigationAdvanced extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Navigation parameters
    private static final double DESIRED_DISTANCE = 12.0; // inches
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    // Target tag ID (change this to navigate to different tags)
    private int TARGET_TAG_ID = 2; // Change to your target tag ID

    @Override
    public void runOpMode() {
        initHardware();
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target Tag ID", TARGET_TAG_ID);
        telemetry.addLine("\nThis OpMode will:");
        telemetry.addLine("1. Search for target AprilTag");
        telemetry.addLine("2. Navigate to the tag");
        telemetry.addLine("3. Align with the tag");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Phase 1: Search for target tag
            AprilTagDetection targetTag = searchForTag(TARGET_TAG_ID, 5.0);

            if (targetTag != null) {
                telemetry.addData("Status", "Target tag found!");
                telemetry.update();

                // Phase 2: Navigate to tag
                navigateToTagPrecise(targetTag);

                telemetry.addData("Status", "Navigation complete!");
                telemetry.update();
                sleep(1000);
            } else {
                telemetry.addData("Status", "Target tag not found");
                telemetry.update();
            }
        }

        visionPortal.close();
    }

    /**
     * Search for a specific AprilTag by ID
     */
    private AprilTagDetection searchForTag(int targetId, double timeoutSeconds) {
        double startTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime) < timeoutSeconds) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

            telemetry.addData("Searching for Tag", targetId);
            telemetry.addData("Tags Detected", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.id == targetId) {
                    telemetry.addData("Status", "Target tag FOUND!");
                    telemetry.update();
                    return detection;
                }
            }

            telemetry.update();
            sleep(100);
        }

        return null;
    }

    /**
     * Navigate to tag with precise positioning
     */
    private void navigateToTagPrecise(AprilTagDetection initialTag) {
        boolean targetReached = false;
        double startTime = getRuntime();
        double timeout = 10.0; // 10 second timeout

        while (opModeIsActive() && !targetReached && (getRuntime() - startTime) < timeout) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            AprilTagDetection currentTag = null;

            // Find the target tag in current detections
            for (AprilTagDetection detection : detections) {
                if (detection.id == TARGET_TAG_ID) {
                    currentTag = detection;
                    break;
                }
            }

            if (currentTag != null && currentTag.ftcPose != null) {
                double range = currentTag.ftcPose.range;
                double bearing = currentTag.ftcPose.bearing;
                double yaw = currentTag.ftcPose.yaw;

                telemetry.addData("Tag ID", currentTag.id);
                telemetry.addData("Range", "%.2f inches", range);
                telemetry.addData("Bearing", "%.2f degrees", bearing);
                telemetry.addData("Yaw", "%.2f degrees", yaw);

                // Check if target is reached
                if (Math.abs(range - DESIRED_DISTANCE) < 2.0 &&
                        Math.abs(bearing) < 5.0 &&
                        Math.abs(yaw) < 5.0) {
                    targetReached = true;
                    stopMotors();
                    telemetry.addData("Status", "TARGET REACHED!");
                } else {
                    // Calculate and apply motor powers
                    double drive = (range - DESIRED_DISTANCE) * SPEED_GAIN;
                    double strafe = -bearing * STRAFE_GAIN;
                    double turn = -yaw * TURN_GAIN;

                    // Clip speeds
                    drive = clipValue(drive, MAX_AUTO_SPEED);
                    strafe = clipValue(strafe, MAX_AUTO_STRAFE);
                    turn = clipValue(turn, MAX_AUTO_TURN);

                    // Apply mecanum drive
                    mecanumDrive(drive, strafe, turn);

                    telemetry.addData("Drive", "%.3f", drive);
                    telemetry.addData("Strafe", "%.3f", strafe);
                    telemetry.addData("Turn", "%.3f", turn);
                }
            } else {
                telemetry.addData("Status", "Tag lost - stopping");
                stopMotors();
            }

            telemetry.update();
            sleep(10);
        }

        stopMotors();
    }

    /**
     * Mecanum drive control
     */
    private void mecanumDrive(double drive, double strafe, double turn) {
        double frontLeftPower = drive - strafe - turn;
        double frontRightPower = drive + strafe + turn;
        double backLeftPower = drive + strafe - turn;
        double backRightPower = drive - strafe + turn;

        // Normalize powers
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        if (frontLeft != null) frontLeft.setPower(frontLeftPower);
        if (frontRight != null) frontRight.setPower(frontRightPower);
        if (backLeft != null) backLeft.setPower(backLeftPower);
        if (backRight != null) backRight.setPower(backRightPower);
    }

    /**
     * Clip value to maximum
     */
    private double clipValue(double value, double max) {
        return Math.max(-max, Math.min(max, value));
    }

    /**
     * Stop all motors
     */
    private void stopMotors() {
        if (frontLeft != null) frontLeft.setPower(0);
        if (frontRight != null) frontRight.setPower(0);
        if (backLeft != null) backLeft.setPower(0);
        if (backRight != null) backRight.setPower(0);
    }

    /**
     * Initialize hardware
     */
    private void initHardware() {
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Hardware Error", e.getMessage());
        }
    }

    /**
     * Initialize AprilTag detection
     */
    private void initAprilTag() {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }
}
