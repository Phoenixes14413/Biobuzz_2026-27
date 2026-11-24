
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Field-centric mecanum TeleOp with two BNO055 IMUs.
 *
 * Gamepad 1:
 *   Left stick          Field-relative translation
 *   Right stick X       Rotate
 *   BACK (tap)          Wall-alignment yaw reset
 *   Left bumper (hold)  Robot-centric override
 *   Right bumper (hold) Slow drive
 *   D-pad up            Automatic IMU selection
 *   D-pad left          Prefer Control Hub IMU
 *   D-pad right         Prefer Expansion Hub IMU
 *
 * Gamepad 2:
 *   Left stick Y        Both intake motors
 *
 * Wall reset defines yaw=0 at the wall. The robot must
 * already be square to a wall corresponding to the
 * desired field-forward heading.
 */
@TeleOp(name = "Biobuzz Teleop 2", group = "Linear Opmode")
public class BiobuzzTeleop2 extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intakeMotor, intake2Motor;
    private IMU controlImu, expansionImu;

    // Original intake motor directions.
    private static final boolean INTAKE_REVERSED = false;
    private static final boolean INTAKE2_REVERSED = true;

    // Matches the right-side inversion from the existing FTCLib drive.
    // Verify motor directions with the robot lifted.
    private static final boolean REVERSE_LEFT_DRIVE = false;
    private static final boolean REVERSE_RIGHT_DRIVE = true;

    private static final double DEADBAND = 0.04;
    private static final double PRECISION_SCALE = 0.35;
    private static final double STRAFE_COMPENSATION = 1.0;

    // Dual-IMU disagreement and glitch detection.
    private static final double MAX_IMU_DISAGREEMENT =
            Math.toRadians(12.0);
    private static final double BLEND_TAPER =
            Math.toRadians(4.0);
    private static final double SUDDEN_JUMP =
            Math.toRadians(50.0);
    private static final double OTHER_IMU_STILL =
            Math.toRadians(15.0);

    // Wall reset:
    // Require 300 ms with under 1 degree of heading change.
    private static final double WALL_STILL_TOLERANCE =
            Math.toRadians(1.0);
    private static final long WALL_STILL_NS = 300_000_000L;
    private static final long WALL_TIMEOUT_NS = 2_000_000_000L;
    private static final long TELEMETRY_NS = 100_000_000L;

    private enum HeadingPreference {
        AUTO, CONTROL, EXPANSION
    }

    private HeadingPreference headingPreference =
            HeadingPreference.AUTO;

    private double controlYaw = Double.NaN;
    private double expansionYaw = Double.NaN;
    private double heading = Double.NaN;
    private double disagreement = Double.NaN;

    private double previousControlYaw = Double.NaN;
    private double previousExpansionYaw = Double.NaN;
    private long previousReadTimeNs = 0;

    private boolean controlQuarantined = false;
    private boolean expansionQuarantined = false;
    private String headingSource = "NONE";

    // Wall-reset state.
    private boolean wallResetPending = false;
    private boolean lastBackButton = false;

    private long wallRequestedNs = 0;
    private long wallStillSinceNs = 0;

    private int wallSensorMask = 0;

    private double wallInitialControl = Double.NaN;
    private double wallInitialExpansion = Double.NaN;

    private String wallStatus = "Ready";

    @Override
    public void runOpMode() {

        // ----------------------------
        // Drive hardware
        // ----------------------------
        frontLeft = hardwareMap.get(
                DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(
                DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(
                DcMotor.class, "backLeft");
        backRight = hardwareMap.get(
                DcMotor.class, "backRight");

        setupDriveMotor(frontLeft, REVERSE_LEFT_DRIVE);
        setupDriveMotor(backLeft, REVERSE_LEFT_DRIVE);
        setupDriveMotor(frontRight, REVERSE_RIGHT_DRIVE);
        setupDriveMotor(backRight, REVERSE_RIGHT_DRIVE);

        // ----------------------------
        // Intake hardware
        // ----------------------------
        intakeMotor = hardwareMap.get(
                DcMotorEx.class, "intake");
        intake2Motor = hardwareMap.get(
                DcMotorEx.class, "outtake");

        intakeMotor.setDirection(
                INTAKE_REVERSED
                        ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD);

        intake2Motor.setDirection(
                INTAKE2_REVERSED
                        ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD);

        intakeMotor.setPower(0);
        intake2Motor.setPower(0);

        // ----------------------------
        // IMU initialization
        // ----------------------------

        // Control Hub:
        // REV logo faces robot RIGHT.
        // HDMI/USB ports face UP.
        controlImu = initializeImu(
                "imu",
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        // Expansion Hub:
        // REV logo faces robot LEFT.
        // Mini-USB port faces DOWN.
        expansionImu = initializeImu(
                "imu2",
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        );

        telemetry.addLine("Ready: field-centric dual-IMU drive");
        telemetry.addData(
                "Control Hub IMU",
                controlImu == null ? "UNAVAILABLE" : "OK");
        telemetry.addData(
                "Expansion Hub IMU",
                expansionImu == null ? "UNAVAILABLE" : "OK");

        telemetry.addLine(
                "Start facing field-forward; hold still during initialization.");
        telemetry.addLine(
                "G1 BACK: square to reference wall, TAP to re-zero yaw.");
        telemetry.addLine(
                "G1 LB: robot-centric  RB: slow  D-pad: IMU preference");
        telemetry.addLine("G2 left stick: intake");
        telemetry.update();

        try {
            waitForStart();

            if (isStopRequested()) {
                return;
            }

            // Set the initial field-forward heading to zero.
            int initializedResets = resetBothYaws();

            wallStatus = initializedResets > 0
                    ? "Zeroed at START"
                    : "No working IMU";

            long lastTelemetryNs = 0;

            while (opModeIsActive()) {
                long now = System.nanoTime();

                // ----------------------------
                // IMU source selection
                // ----------------------------
                if (gamepad1.dpad_up) {
                    headingPreference = HeadingPreference.AUTO;
                } else if (gamepad1.dpad_left) {
                    headingPreference = HeadingPreference.CONTROL;
                } else if (gamepad1.dpad_right) {
                    headingPreference = HeadingPreference.EXPANSION;
                }

                // ----------------------------
                // BACK button wall reset
                // ----------------------------

                // Rising-edge detection: one tap = one request.
                boolean back = gamepad1.back;

                if (back && !lastBackButton) {
                    requestWallReset(now);
                }

                lastBackButton = back;

                // Update both IMUs every cycle.
                updateHeading(now);

                // Remember whether this iteration began in reset mode.
                // Even on success, motors stay stopped until the next
                // iteration has read the freshly zeroed IMU headings.
                boolean inhibitDrive = wallResetPending;

                if (wallResetPending) {
                    updateWallReset(now);
                }

                if (inhibitDrive) {
                    driveMecanum(0, 0, 0, 1.0);
                } else {
                    driveFromGamepad();
                }

                // ----------------------------
                // Original intake controls
                // ----------------------------
                double intakePower =
                        applyDeadband(gamepad2.left_stick_y);

                intakeMotor.setPower(intakePower);
                intake2Motor.setPower(intakePower);

                // ----------------------------
                // Telemetry
                // ----------------------------
                if (now - lastTelemetryNs >= TELEMETRY_NS) {

                    telemetry.addData(
                            "Drive",
                            inhibitDrive
                                    ? "STOPPED FOR WALL ZERO"
                                    : gamepad1.left_bumper || !valid(heading)
                                    ? "ROBOT-CENTRIC"
                                    : "FIELD-CENTRIC"
                    );

                    telemetry.addData("Wall reset", wallStatus);
                    telemetry.addData(
                            "Heading", formatDegrees(heading));

                    telemetry.addData(
                            "IMU source",
                            "%s (%s)",
                            headingSource,
                            headingPreference
                    );

                    telemetry.addData(
                            "Control yaw",
                            formatDegrees(controlYaw));
                    telemetry.addData(
                            "Expansion yaw",
                            formatDegrees(expansionYaw));
                    telemetry.addData(
                            "Difference",
                            formatDegrees(disagreement));

                    if (controlQuarantined || expansionQuarantined) {
                        telemetry.addLine(
                                "IMU glitch detected; BACK can re-zero while straight.");
                    } else if (valid(disagreement)
                            && disagreement > MAX_IMU_DISAGREEMENT) {
                        telemetry.addLine(
                                "IMUs disagree; AUTO uses Control Hub when available.");
                    }

                    if (!valid(heading)) {
                        telemetry.addLine(
                                "WARNING: no valid heading, robot-centric fallback.");
                    }

                    telemetry.addData(
                            "Precision",
                            gamepad1.right_bumper ? "35%" : "100%");
                    telemetry.addData(
                            "Intake", "%.2f", intakePower);

                    telemetry.addLine(
                            "BACK wall-zero | LB robot | RB slow | D-pad sensor");

                    telemetry.update();
                    lastTelemetryNs = now;
                }
            }

        } finally {
            // Always command the drivetrain and intakes to stop.
            driveMecanum(0, 0, 0, 1.0);
            intakeMotor.setPower(0);
            intake2Motor.setPower(0);
        }
    }

    // ============================================================
    // HARDWARE INITIALIZATION
    // ============================================================

    private void setupDriveMotor(
            DcMotor motor,
            boolean reversed) {

        motor.setDirection(
                reversed
                        ? DcMotor.Direction.REVERSE
                        : DcMotor.Direction.FORWARD);

        motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setPower(0);
    }

    private IMU initializeImu(
            String name,
            RevHubOrientationOnRobot.LogoFacingDirection logo,
            RevHubOrientationOnRobot.UsbFacingDirection usb) {

        try {
            IMU sensor = hardwareMap.get(IMU.class, name);

            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(logo, usb)
            );

            if (!sensor.initialize(parameters)) {
                return null;
            }

            return sensor;

        } catch (RuntimeException e) {
            telemetry.addData(
                    "IMU " + name,
                    "Init failed: %s",
                    e.getMessage()
            );
            return null;
        }
    }

    // ============================================================
    // YAW RESET
    // ============================================================

    /**
     * Reset the SDK yaw origin of one IMU.
     *
     * This does not recalibrate the gyroscope's bias.
     */
    private boolean resetYaw(IMU sensor) {
        if (sensor == null) {
            return false;
        }

        try {
            sensor.resetYaw();
            return true;
        } catch (RuntimeException e) {
            return false;
        }
    }

    /**
     * Reset BOTH IMUs and clear heading tracking state.
     *
     * Returns number of successfully reset sensors.
     */
    private int resetBothYaws() {
        boolean controlOK = resetYaw(controlImu);
        boolean expansionOK = resetYaw(expansionImu);

        // Never use an IMU that failed to reset as though
        // it had the same zero origin as the other.
        controlQuarantined =
                controlImu != null && !controlOK;
        expansionQuarantined =
                expansionImu != null && !expansionOK;

        previousControlYaw = Double.NaN;
        previousExpansionYaw = Double.NaN;
        previousReadTimeNs = 0;

        controlYaw = Double.NaN;
        expansionYaw = Double.NaN;
        heading = Double.NaN;
        disagreement = Double.NaN;
        headingSource = "RESETTING";

        return (controlOK ? 1 : 0)
                + (expansionOK ? 1 : 0);
    }

    // ============================================================
    // WALL ALIGNMENT RESET
    // ============================================================

    /**
     * Begin a reset request without blocking the OpMode thread.
     *
     * The robot's drive motors will stop immediately.
     */
    private void requestWallReset(long now) {
        wallResetPending = true;

        wallRequestedNs = now;
        wallStillSinceNs = 0;
        wallSensorMask = 0;

        wallInitialControl = Double.NaN;
        wallInitialExpansion = Double.NaN;

        wallStatus = "Waiting for robot to settle...";
    }

    /**
     * Called every loop while wall reset is pending.
     *
     * Require a stable heading for 300 ms.
     * Then re-zero both IMUs and signal success.
     *
     * If the robot keeps rotating, abort after two seconds.
     */
    private void updateWallReset(long now) {

        int mask =
                (valid(controlYaw) ? 1 : 0)
                        | (valid(expansionYaw) ? 2 : 0);

        if (mask == 0) {
            wallResetPending = false;
            wallStatus = "FAILED: no readable IMU";
            return;
        }

        // If sensor availability changed, restart the stillness timer.
        if (mask != wallSensorMask || wallStillSinceNs == 0) {
            startStillnessWindow(now, mask);

        } else {
            boolean controlMoved =
                    (mask & 1) != 0
                            && Math.abs(wrap(
                            controlYaw - wallInitialControl))
                            > WALL_STILL_TOLERANCE;

            boolean expansionMoved =
                    (mask & 2) != 0
                            && Math.abs(wrap(
                            expansionYaw - wallInitialExpansion))
                            > WALL_STILL_TOLERANCE;

            if (controlMoved || expansionMoved) {
                startStillnessWindow(now, mask);
            }
        }

        if (now - wallStillSinceNs >= WALL_STILL_NS) {

            int successful = resetBothYaws();
            wallResetPending = false;

            if (successful > 0) {
                wallStatus = successful == 2
                        ? "Wall zero SUCCESS (both IMUs)"
                        : "Wall zero SUCCESS (one IMU only)";

                // Short tactile confirmation on Gamepad 1.
                try {
                    gamepad1.rumble(160);
                } catch (RuntimeException ignored) {
                    // Lack of rumble support does not affect driving.
                }

            } else {
                wallStatus =
                        "FAILED: IMU reset did not succeed";
            }

        } else if (now - wallRequestedNs >= WALL_TIMEOUT_NS) {

            wallResetPending = false;
            wallStatus =
                    "FAILED: robot not still; press BACK again";

        } else {
            wallStatus =
                    "Hold still: zeroing after 0.30 s...";
        }
    }

    private void startStillnessWindow(long now, int mask) {
        wallSensorMask = mask;
        wallStillSinceNs = now;

        wallInitialControl = controlYaw;
        wallInitialExpansion = expansionYaw;
    }

    // ============================================================
    // DUAL-IMU HEADING ESTIMATION
    // ============================================================

    private void updateHeading(long now) {

        controlYaw = readYaw(controlImu);
        expansionYaw = readYaw(expansionImu);

        boolean bothRead =
                valid(controlYaw) && valid(expansionYaw);

        disagreement = bothRead
                ? Math.abs(wrap(controlYaw - expansionYaw))
                : Double.NaN;

        // Detect an isolated abrupt sensor discontinuity.
        // A sensor quarantined here stays excluded until reset.
        if (bothRead
                && valid(previousControlYaw)
                && valid(previousExpansionYaw)
                && previousReadTimeNs != 0
                && now - previousReadTimeNs < 150_000_000L
                && disagreement > MAX_IMU_DISAGREEMENT) {

            double deltaControl = Math.abs(wrap(
                    controlYaw - previousControlYaw));

            double deltaExpansion = Math.abs(wrap(
                    expansionYaw - previousExpansionYaw));

            if (deltaControl > SUDDEN_JUMP
                    && deltaExpansion < OTHER_IMU_STILL) {

                controlQuarantined = true;

            } else if (deltaExpansion > SUDDEN_JUMP
                    && deltaControl < OTHER_IMU_STILL) {

                expansionQuarantined = true;
            }
        }

        previousControlYaw = controlYaw;
        previousExpansionYaw = expansionYaw;
        previousReadTimeNs = now;

        boolean controlValid =
                valid(controlYaw) && !controlQuarantined;

        boolean expansionValid =
                valid(expansionYaw) && !expansionQuarantined;

        // Explicit driver sensor selection has priority.
        if (headingPreference == HeadingPreference.CONTROL
                && controlValid) {

            heading = controlYaw;
            headingSource = "CONTROL (chosen)";

        } else if (
                headingPreference == HeadingPreference.EXPANSION
                        && expansionValid) {

            heading = expansionYaw;
            headingSource = "EXPANSION (chosen)";

        } else if (controlValid
                && expansionValid
                && disagreement < MAX_IMU_DISAGREEMENT) {

            // Circular average with a tapered secondary contribution.
            // Correctly handles crossings at +180/-180 degrees.
            //
            // Both readings contribute when in good agreement.
            // If they start diverging, fall back smoothly
            // toward the Control Hub rather than averaging blindly.

            double taper = Math.min(
                    1.0,
                    (MAX_IMU_DISAGREEMENT - disagreement)
                            / BLEND_TAPER
            );

            heading = wrap(
                    controlYaw
                            + 0.5 * taper * wrap(
                            expansionYaw - controlYaw)
            );

            headingSource = "FUSED";

        } else if (controlValid) {

            heading = controlYaw;
            headingSource = "CONTROL";

        } else if (expansionValid) {

            heading = expansionYaw;
            headingSource = "EXPANSION";

        } else {

            heading = Double.NaN;
            headingSource = "NONE";
        }
    }

    private double readYaw(IMU sensor) {
        if (sensor == null) {
            return Double.NaN;
        }

        try {
            double yaw = sensor
                    .getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            return valid(yaw) ? wrap(yaw) : Double.NaN;

        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    // ============================================================
    // FIELD-CENTRIC DRIVE
    // ============================================================

    private void driveFromGamepad() {

        double fieldRight =
                applyDeadband(gamepad1.left_stick_x);

        double fieldForward =
                applyDeadband(-gamepad1.left_stick_y);

        double turnStick =
                applyDeadband(gamepad1.right_stick_x);

        // Blended linear/cubic turning curve.
        // Fine at low inputs, full power at maximum input.
        double turn =
                0.55 * turnStick
                        + 0.45 * turnStick
                        * turnStick * turnStick;

        double right = fieldRight;
        double forward = fieldForward;

        // Field-centric transformation.
        // If LB is held or heading unavailable, drive robot-centric.
        if (!gamepad1.left_bumper && valid(heading)) {

            double c = Math.cos(heading);
            double s = Math.sin(heading);

            right =
                    fieldRight * c + fieldForward * s;

            forward =
                    -fieldRight * s + fieldForward * c;
        }

        right *= STRAFE_COMPENSATION;

        double scale = gamepad1.right_bumper
                ? PRECISION_SCALE : 1.0;

        driveMecanum(forward, right, turn, scale);
    }

    /**
     * Standard mecanum power mixing with normalization.
     *
     * Positive forward = robot forward.
     * Positive right   = robot right.
     * Positive turn    = clockwise command.
     */
    private void driveMecanum(
            double forward,
            double right,
            double turn,
            double scale) {

        double fl = forward + right + turn;
        double fr = forward - right - turn;
        double bl = forward - right + turn;
        double br = forward + right - turn;

        double denominator = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        frontLeft.setPower(scale * fl / denominator);
        frontRight.setPower(scale * fr / denominator);
        backLeft.setPower(scale * bl / denominator);
        backRight.setPower(scale * br / denominator);
    }

    // ============================================================
    // MATH UTILITIES
    // ============================================================

    private static double applyDeadband(double input) {
        double magnitude = Math.abs(input);

        if (magnitude <= DEADBAND) {
            return 0.0;
        }

        return Math.copySign(
                (magnitude - DEADBAND) / (1.0 - DEADBAND),
                input
        );
    }

    private static boolean valid(double value) {
        return !Double.isNaN(value)
                && !Double.isInfinite(value);
    }

    private static double wrap(double radians) {
        return Math.atan2(
                Math.sin(radians),
                Math.cos(radians)
        );
    }

    private static String formatDegrees(double radians) {
        return valid(radians)
                ? String.format(
                "%.1f deg",
                Math.toDegrees(radians))
                : "N/A";
    }
}
