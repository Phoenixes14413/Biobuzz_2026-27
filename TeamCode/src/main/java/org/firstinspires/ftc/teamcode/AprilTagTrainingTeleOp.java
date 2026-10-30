package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTag Training and Testing TeleOp
 * This OpMode allows you to see AprilTag detections in real-time
 * Perfect for training and testing your AprilTag setup
 */
@TeleOp(name = "AprilTag Training TeleOp", group = "Vision")
public class AprilTagTrainingTeleOp extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Statistics
    private int totalDetections = 0;
    private long startTime = 0;

    // Configure the three tag IDs you care about
    // Change these to the actual IDs printed on your tags
    private static final int TAG_ID_1 = 21;
    private static final int TAG_ID_2 = 22;
    private static final int TAG_ID_3 = 23;
    // Set this to the actual printed black-square size (edge length) of your tags, in inches
    private static final double TAG_SIZE_INCHES = 4.0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize AprilTag detection
        initAprilTag();

        startTime = System.currentTimeMillis();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Ready");
        telemetry.addLine("\nAprilTag Training Mode");
        telemetry.addLine("Use this to test your AprilTag detection");
        telemetry.addLine("\nControls:");
        telemetry.addLine("- Press A to toggle decimation (performance)");
        telemetry.addLine("- Press B to cycle through tag families");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Get list of detected AprilTags
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        // Update statistics
        if (detections.size() > 0) {
            totalDetections++;
        }

        long elapsedTime = (System.currentTimeMillis() - startTime) / 1000;

        // Display header
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("   APRILTAG TRAINING MODE");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addData("Runtime", "%d seconds", elapsedTime);
        telemetry.addData("Total Detections", totalDetections);
        telemetry.addLine();

        // Display detection information
        telemetry.addData("# Tags Detected", detections.size());

        // Determine which of the three tags is best seen right now
        Integer selected = selectBestOfThree(detections);
        if (selected == null) {
            telemetry.addData("Selected (1/2/3)", "None in view");
        } else {
            telemetry.addData("Selected (1/2/3)", "#%d", selected);
        }

        if (detections.size() > 0) {
            telemetry.addLine("\n--- DETECTED TAGS ---");

            // Display information for each detected tag
            for (int i = 0; i < detections.size(); i++) {
                AprilTagDetection detection = detections.get(i);

                telemetry.addLine();
                telemetry.addData("Tag #", "%d", i + 1);
                telemetry.addData("  ID", detection.id);
                telemetry.addData("  Center", "X=%.1f, Y=%.1f", 
                        detection.center.x, detection.center.y);
                telemetry.addData("  Hamming", detection.hamming);
                telemetry.addData("  Decision Margin", "%.2f", detection.decisionMargin);

                // Display pose information if available
                if (detection.ftcPose != null) {
                    telemetry.addLine("\n  Position (FTC Pose):");
                    telemetry.addData("    Range", "%.2f inches", detection.ftcPose.range);
                    telemetry.addData("    Bearing", "%.2f°", detection.ftcPose.bearing);
                    telemetry.addData("    Yaw", "%.2f°", detection.ftcPose.yaw);
                    telemetry.addData("    X", "%.2f in", detection.ftcPose.x);
                    telemetry.addData("    Y", "%.2f in", detection.ftcPose.y);
                    telemetry.addData("    Z", "%.2f in", detection.ftcPose.z);
                    telemetry.addData("    Pitch", "%.2f°", detection.ftcPose.pitch);
                    telemetry.addData("    Roll", "%.2f°", detection.ftcPose.roll);
                    telemetry.addData("    Elevation", "%.2f°", detection.ftcPose.elevation);
                }

                // Display metadata if available
                if (detection.metadata != null) {
                    telemetry.addLine("\n  Metadata:");
                    telemetry.addData("    Name", detection.metadata.name);
                    telemetry.addData("    Size", "%.2f inches", detection.metadata.tagsize);
                }

                // Add a separator between tags
                if (i < detections.size() - 1) {
                    telemetry.addLine("  ─────────────────");
                }
            }

            // Add detection quality indicator
            telemetry.addLine();
            telemetry.addLine("═══════════════════════════════");
            AprilTagDetection firstTag = detections.get(0);
            if (firstTag.decisionMargin > 100) {
                telemetry.addLine("✓ EXCELLENT DETECTION QUALITY");
            } else if (firstTag.decisionMargin > 50) {
                telemetry.addLine("✓ Good Detection Quality");
            } else {
                telemetry.addLine("⚠ Fair Detection Quality");
            }
        } else {
            telemetry.addLine();
            telemetry.addLine("No AprilTags detected");
            telemetry.addLine();
            telemetry.addLine("Tips:");
            telemetry.addLine("- Ensure proper lighting");
            telemetry.addLine("- Tag should be flat and visible");
            telemetry.addLine("- Move camera closer to tag");
            telemetry.addLine("- Check camera focus");
        }

        telemetry.addLine("═══════════════════════════════");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Clean up
        if (visionPortal != null) {
            visionPortal.close();
        }
        telemetry.addData("Status", "Stopped");
        telemetry.update();
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

    // Create the AprilTag processor with visualization and output units in inches/degrees
    aprilTagProcessor = new AprilTagProcessor.Builder()
        .setTagLibrary(tagLib)
        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
        .setDrawAxes(true)              // Draw 3D axes on tag
        .setDrawCubeProjection(true)    // Draw 3D cube projection
        .setDrawTagOutline(true)        // Draw outline around tag
        .setDrawTagID(true)             // Draw tag ID on the tag
        .build();

        // Create the vision portal
        // Note: Change "Webcam 1" to match your camera configuration name
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .build();
    }

    /**
     * From the current detections, pick which of our three interesting tag IDs (TAG_ID_1/2/3)
     * is best to use. Preference order: highest decision margin; if equal, closest range.
     * Returns 1, 2, 3 for the label, or null if none are visible.
     */
    private Integer selectBestOfThree(List<AprilTagDetection> detections) {
        AprilTagDetection best = null;
        int bestLabel = -1;

        for (AprilTagDetection d : detections) {
            int label;
            if (d.id == TAG_ID_1) label = 1;
            else if (d.id == TAG_ID_2) label = 2;
            else if (d.id == TAG_ID_3) label = 3;
            else continue; // ignore other tags

            if (best == null) {
                best = d;
                bestLabel = label;
            } else {
                // Prefer higher decision margin; tie-breaker: lower range if pose available
                double dmBest = best.decisionMargin;
                double dmNew = d.decisionMargin;
                if (dmNew > dmBest + 1e-6) {
                    best = d;
                    bestLabel = label;
                } else if (Math.abs(dmNew - dmBest) <= 1e-6) {
                    double rangeBest = best.ftcPose != null ? best.ftcPose.range : Double.POSITIVE_INFINITY;
                    double rangeNew = d.ftcPose != null ? d.ftcPose.range : Double.POSITIVE_INFINITY;
                    if (rangeNew < rangeBest) {
                        best = d;
                        bestLabel = label;
                    }
                }
            }
        }

        return best == null ? null : bestLabel;
    }
}
