package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag Detection for FTC Autonomous
 * This OpMode demonstrates how to detect and navigate to AprilTags
 */
@Autonomous(name = "AprilTag Detection Auto", group = "Autonomous")
public class AprilTagDetectionAuto extends LinearOpMode {

    // Vision variables
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Motor variables (adjust based on your robot configuration)
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // Constants for AprilTag navigation
    private static final double DESIRED_DISTANCE = 12.0; // inches from tag
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;

    // Configure the three tag IDs you care about
    // Change these to the actual IDs printed on your tags
    private static final int TAG_ID_1 = 1;
    private static final int TAG_ID_2 = 2;
    private static final int TAG_ID_3 = 3;
    // Set this to the actual printed black-square size (edge length) of your tags, in inches
    private static final double TAG_SIZE_INCHES = 4.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        // Initialize AprilTag detection
        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Ready");
        telemetry.update();

        // Wait for the game to start
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Get list of detected AprilTags
                List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

                telemetry.addData("# AprilTags Detected", detections.size());

                // Process detections: prefer one of our three tags if present
                if (detections.size() > 0) {
                    AprilTagDetection detection = selectBestOfThree(detections);
                    if (detection == null) {
                        // Fall back to the first detection if none of our 1/2/3 are visible
                        detection = detections.get(0);
                    }

                    telemetry.addData("Tag ID", detection.id);
                    Integer label = labelForId(detection.id);
                    telemetry.addData("Selected (1/2/3)", label == null ? "(not 1/2/3)" : String.format("#%d", label));
                    telemetry.addData("Center X", "%.2f", detection.center.x);
                    telemetry.addData("Center Y", "%.2f", detection.center.y);
                    
                    // Pose information (position and orientation)
                    if (detection.ftcPose != null) {
                        telemetry.addData("Range", "%.2f inches", detection.ftcPose.range);
                        telemetry.addData("Bearing", "%.2f degrees", detection.ftcPose.bearing);
                        telemetry.addData("Yaw", "%.2f degrees", detection.ftcPose.yaw);
                        telemetry.addData("X", "%.2f inches", detection.ftcPose.x);
                        telemetry.addData("Y", "%.2f inches", detection.ftcPose.y);
                        telemetry.addData("Z", "%.2f inches", detection.ftcPose.z);

                        // Navigate to the AprilTag
                        navigateToTag(detection);
                    }
                } else {
                    telemetry.addData("Status", "No AprilTags detected");
                    // Stop all motors if no tag is detected
                    stopMotors();
                }

                telemetry.update();
                sleep(10);
            }
        }

        // Clean up
        visionPortal.close();
    }

    /**
     * If the id matches one of our three, return its label 1/2/3, else null.
     */
    private Integer labelForId(int id) {
        if (id == TAG_ID_1) return 1;
        if (id == TAG_ID_2) return 2;
        if (id == TAG_ID_3) return 3;
        return null;
    }

    /**
     * Choose the best detection among TAG_ID_1/2/3 by highest decision margin, then closest range.
     * Returns null if none of those tags are visible.
     */
    private AprilTagDetection selectBestOfThree(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;
        double bestDM = -1;
        double bestRange = Double.POSITIVE_INFINITY;

        for (AprilTagDetection d : detections) {
            if (labelForId(d.id) == null) continue; // skip other tags

            double dm = d.decisionMargin;
            double range = d.ftcPose != null ? d.ftcPose.range : Double.POSITIVE_INFINITY;

            if (dm > bestDM + 1e-6 || (Math.abs(dm - bestDM) <= 1e-6 && range < bestRange)) {
                best = d;
                bestDM = dm;
                bestRange = range;
            }
        }

        return best;
    }

    /**
     * Initialize hardware (motors)
     */
    private void initHardware() {
        // Initialize motors - adjust names to match your robot configuration
        try {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            // Set motor directions
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);

            // Set zero power behavior
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addData("Hardware Error", "Motor initialization failed");
            telemetry.addData("Error", e.getMessage());
        }
    }

    /**
     * Initialize AprilTag processor and vision portal
     */
    private void initAprilTag() {
    // Define a small library with the tags you are using so the SDK knows their size/name
    AprilTagLibrary tagLib = new AprilTagLibrary.Builder()
        .addTag(TAG_ID_1, "Tag-1", TAG_SIZE_INCHES, DistanceUnit.INCH)
        .addTag(TAG_ID_2, "Tag-2", TAG_SIZE_INCHES, DistanceUnit.INCH)
        .addTag(TAG_ID_3, "Tag-3", TAG_SIZE_INCHES, DistanceUnit.INCH)
        .build();

    // Create the AprilTag processor
    aprilTagProcessor = new AprilTagProcessor.Builder()
        .setTagLibrary(tagLib)
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setDrawTagOutline(true)
        .build();

        // Create the vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * Navigate the robot toward the detected AprilTag
     */
    private void navigateToTag(AprilTagDetection detection) {
        double range = detection.ftcPose.range;
        double bearing = detection.ftcPose.bearing;
        double yaw = detection.ftcPose.yaw;

        // Calculate drive, strafe, and turn speeds
        double drive = (range - DESIRED_DISTANCE) * SPEED_GAIN;
        double strafe = -bearing * STRAFE_GAIN;
        double turn = -yaw * TURN_GAIN;

        // Clip speeds to maximum values
        drive = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, drive));
        strafe = Math.max(-MAX_AUTO_STRAFE, Math.min(MAX_AUTO_STRAFE, strafe));
        turn = Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, turn));

        // Calculate motor powers for mecanum drive
        double frontLeftPower = drive - strafe - turn;
        double frontRightPower = drive + strafe + turn;
        double backLeftPower = drive + strafe - turn;
        double backRightPower = drive - strafe + turn;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set motor powers
        if (frontLeft != null) frontLeft.setPower(frontLeftPower);
        if (frontRight != null) frontRight.setPower(frontRightPower);
        if (backLeft != null) backLeft.setPower(backLeftPower);
        if (backRight != null) backRight.setPower(backRightPower);

        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
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
}
