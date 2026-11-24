package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

/*
 * ============================================================================
 *  BIOBUZZ TELEOP  (2026-2027)
 * ============================================================================
 *
 *  GAMEPAD 1 (driver)
 *    Left stick ........ drive (FIELD CENTRIC: up = away from you, left = your left)
 *    Right stick X ..... rotate
 *    Right trigger ..... hold for SLOW MODE
 *    OPTIONS ........... reset heading (point the INTAKE straight away from you first)
 *
 *    Dev mode only (DEV_MODE = true):
 *      X / B ........... pollen RPM +250 / -250
 *      Y / A ........... nectar RPM +250 / -250
 *
 *  GAMEPAD 2 (operator)
 *    Left stick Y ...... intake (down = intake in, up = push pieces back out)
 *    Left trigger ...... toggle POLLEN outtake
 *                          ON:  spins up, opens gate when at speed
 *                          OFF: closes gate, stops wheel
 *    Right trigger ..... toggle NECTAR outtake (same as pollen)
 *    X ................. toggle pollen gate
 *    B ................. toggle nectar gate
 *
 *  Gp2 rumbles on the LEFT side when pollen is at speed, RIGHT side for nectar.
 * ============================================================================
 */
@TeleOp(name = "BioBuzz TeleOp ULTIMATE", group = "TeleOp")
public class UltimateBIOBUZZTeleOp1 extends LinearOpMode {

    // =========================================================================
    //  DEV MODE  - set to false before competition
    // =========================================================================
    private static final boolean DEV_MODE = true;

    // =========================================================================
    //  HARDWARE CONFIG NAMES  - must match your Robot Configuration exactly
    // =========================================================================
    private static final String FRONT_LEFT_NAME     = "frontLeft";
    private static final String FRONT_RIGHT_NAME    = "frontRight";
    private static final String BACK_LEFT_NAME      = "backLeft";
    private static final String BACK_RIGHT_NAME     = "backRight";
    private static final String INTAKE_NAME         = "intake";
    private static final String POLLEN_OUTTAKE_NAME = "pollenOuttake";
    private static final String NECTAR_OUTTAKE_NAME = "nectarOuttake";
    private static final String POLLEN_GATE_NAME    = "pollenGate";
    private static final String NECTAR_GATE_NAME    = "nectarGate";
    private static final String IMU_NAME            = "imu";

    // =========================================================================
    //  DRIVE
    // =========================================================================
    // Standard goBILDA mecanum: left side reversed. If the robot spins when you
    // push forward, or a wheel goes the wrong way, flip that motor here.
    private static final DcMotorSimple.Direction FRONT_LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction BACK_LEFT_DIR   = DcMotorSimple.Direction.REVERSE;
    private static final DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction BACK_RIGHT_DIR  = DcMotorSimple.Direction.FORWARD;

    private static final double SLOW_MODE_SCALE   = 0.35; // speed while gp1 right trigger is held
    private static final double STRAFE_CORRECTION = 1.1;  // mecanum strafes slower than it drives; 1.0 = off
    private static final double STICK_DEADZONE    = 0.05;

    // =========================================================================
    //  IMU  - Control Hub: logo facing LEFT, USB ports facing UP
    // =========================================================================
    private static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    private static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

    // =========================================================================
    //  INTAKE
    // =========================================================================
    private static final DcMotorSimple.Direction INTAKE_DIR = DcMotorSimple.Direction.FORWARD; // flip if stick-down spits pieces out
    private static final double INTAKE_MAX_POWER = 1.0;

    // =========================================================================
    //  OUTTAKE FLYWHEELS  (goBILDA 5203 6000 RPM, 1:1, 96mm Rhino wheel)
    // =========================================================================
    private static final DcMotorSimple.Direction POLLEN_OUTTAKE_DIR = DcMotorSimple.Direction.FORWARD;
    private static final DcMotorSimple.Direction NECTAR_OUTTAKE_DIR = DcMotorSimple.Direction.FORWARD;

    private static final double TICKS_PER_REV = 28.0;   // 6000 RPM Yellow Jacket encoder, at the motor shaft

    // Competition RPMs. Tune in dev mode, then copy the numbers from telemetry here.
    private static final double POLLEN_RPM = 4500;
    private static final double NECTAR_RPM = 4500;

    private static final double RPM_STEP = 250;          // dev mode step per button press
    private static final double MIN_RPM  = 500;
    private static final double MAX_RPM  = 6000;         // real-world top speed is ~5400 RPM unloaded

    private static final double READY_TOLERANCE_RPM = 150; // "at speed" = within this many RPM of target
    private static final double SPINUP_TIMEOUT_SEC  = 2.5; // open the gate anyway after this long

    // ---- Velocity PIDF (runs on the hub itself) ----
    // Starting values from the standard FTC tuning method:
    //   1. MAX_TICKS_PER_SEC: run the motor at full power and read its velocity
    //      (dev mode shows it as RPM; ticks/sec = RPM * 28 / 60). ~5400 RPM -> 2520.
    //   2. F = 32767 / MAX_TICKS_PER_SEC
    //   3. P = 0.1 * F,  I = 0.1 * P,  D = 0
    //   If it overshoots/oscillates, lower P. If it sits below target, raise F a bit.
    private static final double MAX_TICKS_PER_SEC = 2520.0;
    private static final double OUTTAKE_F = 32767.0 / MAX_TICKS_PER_SEC;
    private static final double OUTTAKE_P = 0.1 * OUTTAKE_F;
    private static final double OUTTAKE_I = 0.1 * OUTTAKE_P;
    private static final double OUTTAKE_D = 0.0;

    // =========================================================================
    //  GATES  (goBILDA 2000-0025-0003 Speed servo, 300 degree range)
    // =========================================================================
    // We widen the hub's PWM range to the servo's full 500-2500us, so
    // position 0.0 -> 1.0 covers the whole 300 degrees (60 degrees = 0.2).
    private static final double SERVO_RANGE_DEG = 300.0;
    private static final double SERVO_PWM_MIN_US = 500;
    private static final double SERVO_PWM_MAX_US = 2500;

    // CLOSED = up (default), OPEN = down ~60 degrees.
    // If a gate swings UP when it should open, change its OPEN value to 210.
    private static final double POLLEN_GATE_CLOSED_DEG = 150;
    private static final double POLLEN_GATE_OPEN_DEG   = 90;
    private static final double NECTAR_GATE_CLOSED_DEG = 150;
    private static final double NECTAR_GATE_OPEN_DEG   = 90;

    private static final double TRIGGER_THRESHOLD = 0.5;

    // =========================================================================
    //  HARDWARE
    // =========================================================================
    private DcMotorEx frontLeft, frontRight, backLeft, backRight, intake;
    private IMU imu;
    private VoltageSensor battery;
    private List<LynxModule> hubs;

    private Outtake pollen, nectar;

    // Gamepad snapshots for "just pressed" detection
    private final Gamepad currGp1 = new Gamepad();
    private final Gamepad prevGp1 = new Gamepad();
    private final Gamepad currGp2 = new Gamepad();
    private final Gamepad prevGp2 = new Gamepad();

    // Values kept for telemetry
    private double headingDeg = 0;
    private boolean slowMode = false;
    private double flPower, frPower, blPower, brPower;
    private double intakePower = 0;
    private double batteryVolts = 0;
    private final ElapsedTime batteryTimer = new ElapsedTime();

    // =========================================================================
    //  OUTTAKE STATE MACHINE (one instance each for pollen and nectar)
    // =========================================================================
    private enum OuttakeState { OFF, SPINNING_UP, FIRING }

    private class Outtake {
        final String label;
        final DcMotorEx motor;
        final ServoImplEx gate;
        final double gateClosedPos, gateOpenPos;
        final boolean rumbleLeftSide;

        double targetRpm;
        OuttakeState state = OuttakeState.OFF;
        boolean gateOpen = false;

        final ElapsedTime spinUpTimer = new ElapsedTime();
        double lastSpinUpSec = -1;
        boolean lastSpinUpTimedOut = false;

        Outtake(String label, DcMotorEx motor, ServoImplEx gate,
                double closedDeg, double openDeg, double startRpm, boolean rumbleLeftSide) {
            this.label = label;
            this.motor = motor;
            this.gate = gate;
            this.gateClosedPos = degToServoPos(closedDeg);
            this.gateOpenPos = degToServoPos(openDeg);
            this.targetRpm = startRpm;
            this.rumbleLeftSide = rumbleLeftSide;
        }

        /** Trigger pressed: OFF -> start, anything else -> stop. */
        void toggle() {
            if (state == OuttakeState.OFF) start();
            else stop();
        }

        /** Spin up first, gate opens in update() once at speed. */
        void start() {
            setGate(false);              // keep pieces off the wheel until it's moving
            spinUpTimer.reset();
            state = OuttakeState.SPINNING_UP;
            motor.setVelocity(rpmToTicks(targetRpm));
        }

        /** Close gate, let the wheel coast down. */
        void stop() {
            setGate(false);
            motor.setPower(0);
            state = OuttakeState.OFF;
        }

        void toggleGate() {
            setGate(!gateOpen);
        }

        void setGate(boolean open) {
            gateOpen = open;
            gate.setPosition(open ? gateOpenPos : gateClosedPos);
        }

        void changeRpm(double delta) {
            targetRpm = Range.clip(targetRpm + delta, MIN_RPM, MAX_RPM);
            if (state != OuttakeState.OFF) {
                motor.setVelocity(rpmToTicks(targetRpm)); // apply live while running
            }
        }

        /** Call every loop. */
        void update() {
            if (state != OuttakeState.SPINNING_UP) return;

            boolean ready = isAtSpeed();
            boolean timedOut = spinUpTimer.seconds() >= SPINUP_TIMEOUT_SEC;
            if (ready || timedOut) {
                lastSpinUpSec = spinUpTimer.seconds();
                lastSpinUpTimedOut = !ready;
                setGate(true);
                state = OuttakeState.FIRING;
                if (rumbleLeftSide) gamepad2.rumble(1.0, 0.0, 250);
                else                gamepad2.rumble(0.0, 1.0, 250);
            }
        }

        double currentRpm() {
            return ticksToRpm(motor.getVelocity());
        }

        boolean isAtSpeed() {
            return Math.abs(currentRpm() - targetRpm) <= READY_TOLERANCE_RPM;
        }

        void addTelemetry() {
            String stateText;
            switch (state) {
                case SPINNING_UP:
                    stateText = String.format("SPINNING UP (%.1fs)", spinUpTimer.seconds());
                    break;
                case FIRING:
                    stateText = "ON";
                    break;
                default:
                    stateText = "OFF";
            }
            double rpm = currentRpm();

            telemetry.addLine("--- " + label + " ---");
            telemetry.addData("State", stateText);
            telemetry.addData("RPM", "%.0f / %.0f target (err %+.0f)", rpm, targetRpm, rpm - targetRpm);
            telemetry.addData("At speed", state == OuttakeState.OFF ? "-" : (isAtSpeed() ? "YES" : "no"));
            telemetry.addData("Gate", "%s (pos %.2f)", gateOpen ? "OPEN" : "CLOSED", gate.getPosition());
            if (lastSpinUpSec < 0) {
                telemetry.addData("Last spin-up", "-");
            } else {
                telemetry.addData("Last spin-up", "%.2f s%s", lastSpinUpSec,
                        lastSpinUpTimedOut ? "  (TIMED OUT - opened anyway)" : "");
            }
            if (DEV_MODE) {
                telemetry.addData("Motor current", "%.2f A", motor.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Wheel power", "%.2f", motor.getPower());
            }
        }
    }

    // =========================================================================
    //  MAIN
    // =========================================================================
    @Override
    public void runOpMode() {
        initHardware();

        // ---------- INIT LOOP ----------
        while (opModeInInit()) {
            snapshotGamepads();
            if (justPressed(currGp1.options, prevGp1.options)) {
                imu.resetYaw();
                gamepad1.rumble(200);
            }
            headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            telemetry.addLine("BioBuzz TeleOp  " + (DEV_MODE ? "[DEV MODE]" : "[COMPETITION]"));
            telemetry.addLine("Point the INTAKE straight away from you,");
            telemetry.addLine("then press gp1 OPTIONS to zero the heading.");
            telemetry.addData("Heading", "%.1f deg", headingDeg);
            telemetry.addData("Pollen RPM", "%.0f", pollen.targetRpm);
            telemetry.addData("Nectar RPM", "%.0f", nectar.targetRpm);
            telemetry.update();
        }
        if (isStopRequested()) return;

        // Servos only move once PLAY is pressed
        pollen.setGate(false);
        nectar.setGate(false);
        batteryVolts = battery.getVoltage();
        batteryTimer.reset();

        ElapsedTime loopTimer = new ElapsedTime();

        // ---------- TELEOP LOOP ----------
        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            snapshotGamepads();

            handleDrive();
            handleIntake();
            handleOuttakes();

            pollen.update();
            nectar.update();

            if (batteryTimer.seconds() > 0.5) {   // voltage reads are slow, don't do it every loop
                batteryVolts = battery.getVoltage();
                batteryTimer.reset();
            }

            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();
            addTelemetry(loopMs);
            telemetry.update();
        }

        pollen.stop();
        nectar.stop();
    }

    // =========================================================================
    //  INIT
    // =========================================================================
    private void initHardware() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // Drive
        frontLeft  = initDriveMotor(FRONT_LEFT_NAME,  FRONT_LEFT_DIR);
        frontRight = initDriveMotor(FRONT_RIGHT_NAME, FRONT_RIGHT_DIR);
        backLeft   = initDriveMotor(BACK_LEFT_NAME,   BACK_LEFT_DIR);
        backRight  = initDriveMotor(BACK_RIGHT_NAME,  BACK_RIGHT_DIR);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, INTAKE_NAME);
        intake.setDirection(INTAKE_DIR);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Outtakes
        DcMotorEx pollenMotor = initOuttakeMotor(POLLEN_OUTTAKE_NAME, POLLEN_OUTTAKE_DIR);
        DcMotorEx nectarMotor = initOuttakeMotor(NECTAR_OUTTAKE_NAME, NECTAR_OUTTAKE_DIR);
        ServoImplEx pollenGate = initGateServo(POLLEN_GATE_NAME);
        ServoImplEx nectarGate = initGateServo(NECTAR_GATE_NAME);

        pollen = new Outtake("POLLEN (LT toggles)", pollenMotor, pollenGate,
                POLLEN_GATE_CLOSED_DEG, POLLEN_GATE_OPEN_DEG, POLLEN_RPM, true);
        nectar = new Outtake("NECTAR (RT toggles)", nectarMotor, nectarGate,
                NECTAR_GATE_CLOSED_DEG, NECTAR_GATE_OPEN_DEG, NECTAR_RPM, false);

        // IMU
        imu = hardwareMap.get(IMU.class, IMU_NAME);
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(IMU_LOGO_DIR, IMU_USB_DIR)));
        imu.resetYaw();

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    private DcMotorEx initDriveMotor(String name, DcMotorSimple.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    private DcMotorEx initOuttakeMotor(String name, DcMotorSimple.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, no braking spike
        m.setVelocityPIDFCoefficients(OUTTAKE_P, OUTTAKE_I, OUTTAKE_D, OUTTAKE_F);
        return m;
    }

    private ServoImplEx initGateServo(String name) {
        ServoImplEx s = hardwareMap.get(ServoImplEx.class, name);
        s.setPwmRange(new PwmControl.PwmRange(SERVO_PWM_MIN_US, SERVO_PWM_MAX_US));
        return s;
    }

    // =========================================================================
    //  GAMEPAD 1 - DRIVE
    // =========================================================================
    private void handleDrive() {
        if (justPressed(currGp1.options, prevGp1.options)) {
            imu.resetYaw();
            gamepad1.rumble(200);
        }

        double y  = -deadzone(currGp1.left_stick_y);  // up on the stick is negative
        double x  =  deadzone(currGp1.left_stick_x);
        double rx =  deadzone(currGp1.right_stick_x);

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        headingDeg = Math.toDegrees(heading);

        // Rotate the stick vector by -heading: field frame -> robot frame
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
        rotX *= STRAFE_CORRECTION;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        slowMode = currGp1.right_trigger > TRIGGER_THRESHOLD;
        double scale = slowMode ? SLOW_MODE_SCALE : 1.0;

        flPower = (rotY + rotX + rx) / denominator * scale;
        blPower = (rotY - rotX + rx) / denominator * scale;
        frPower = (rotY - rotX - rx) / denominator * scale;
        brPower = (rotY + rotX - rx) / denominator * scale;

        frontLeft.setPower(flPower);
        backLeft.setPower(blPower);
        frontRight.setPower(frPower);
        backRight.setPower(brPower);
    }

    // =========================================================================
    //  GAMEPAD 2 - INTAKE
    // =========================================================================
    private void handleIntake() {
        // Stick DOWN is positive in the SDK -> positive = intake in
        intakePower = deadzone(currGp2.left_stick_y) * INTAKE_MAX_POWER;
        intake.setPower(intakePower);
    }

    // =========================================================================
    //  GAMEPAD 2 - OUTTAKES, GATES, DEV MODE RPM
    // =========================================================================
    private void handleOuttakes() {
        boolean ltNow  = currGp2.left_trigger  > TRIGGER_THRESHOLD;
        boolean ltPrev = prevGp2.left_trigger  > TRIGGER_THRESHOLD;
        boolean rtNow  = currGp2.right_trigger > TRIGGER_THRESHOLD;
        boolean rtPrev = prevGp2.right_trigger > TRIGGER_THRESHOLD;

        // Triggers toggle the outtakes on/off
        if (ltNow && !ltPrev) pollen.toggle();
        if (rtNow && !rtPrev) nectar.toggle();

        // Gate toggles (gp2, always)
        if (justPressed(currGp2.x, prevGp2.x)) pollen.toggleGate();
        if (justPressed(currGp2.b, prevGp2.b)) nectar.toggleGate();

        // Dev mode RPM tuning (gp1)
        if (DEV_MODE) {
            if (justPressed(currGp1.x, prevGp1.x)) pollen.changeRpm(+RPM_STEP);
            if (justPressed(currGp1.b, prevGp1.b)) pollen.changeRpm(-RPM_STEP);
            if (justPressed(currGp1.y, prevGp1.y)) nectar.changeRpm(+RPM_STEP);
            if (justPressed(currGp1.a, prevGp1.a)) nectar.changeRpm(-RPM_STEP);
        }
    }

    // =========================================================================
    //  TELEMETRY
    // =========================================================================
    private void addTelemetry(double loopMs) {
        telemetry.addLine("=== BIOBUZZ " + (DEV_MODE ? "[DEV MODE]" : "[COMPETITION]") + " ===");

        telemetry.addLine("--- DRIVE (gp1) ---");
        telemetry.addData("Heading", "%.1f deg  (OPTIONS = reset)", headingDeg);
        telemetry.addData("Slow mode", slowMode ? "ON (%.0f%%)" : "off", SLOW_MODE_SCALE * 100);
        telemetry.addData("Stick", "strafe %.2f  fwd %.2f  turn %.2f",
                deadzone(currGp1.left_stick_x), -deadzone(currGp1.left_stick_y), deadzone(currGp1.right_stick_x));
        telemetry.addData("Wheels", "FL %.2f  FR %.2f  BL %.2f  BR %.2f", flPower, frPower, blPower, brPower);

        telemetry.addLine("--- INTAKE (gp2 left stick) ---");
        String intakeDir = intakePower > 0 ? "IN" : (intakePower < 0 ? "OUT" : "off");
        telemetry.addData("Intake", "%.2f  %s", intakePower, intakeDir);
        if (DEV_MODE) {
            telemetry.addData("Intake current", "%.2f A", intake.getCurrent(CurrentUnit.AMPS));
        }

        pollen.addTelemetry();
        nectar.addTelemetry();

        telemetry.addLine("--- SYSTEM ---");
        telemetry.addData("Battery", "%.2f V", batteryVolts);
        telemetry.addData("Loop", "%.0f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);

        if (DEV_MODE) {
            telemetry.addLine("--- DEV ---");
            telemetry.addData("PIDF", "P %.3f  I %.3f  D %.3f  F %.3f", OUTTAKE_P, OUTTAKE_I, OUTTAKE_D, OUTTAKE_F);
            telemetry.addData("Ready band", "+/- %.0f RPM, timeout %.1f s", READY_TOLERANCE_RPM, SPINUP_TIMEOUT_SEC);
            telemetry.addLine("gp1 X/B: pollen RPM +/-   gp1 Y/A: nectar RPM +/-");
            telemetry.addData("Copy into code", "POLLEN_RPM = %.0f, NECTAR_RPM = %.0f",
                    pollen.targetRpm, nectar.targetRpm);
        }
    }

    // =========================================================================
    //  HELPERS
    // =========================================================================
    private void snapshotGamepads() {
        prevGp1.copy(currGp1);
        prevGp2.copy(currGp2);
        currGp1.copy(gamepad1);
        currGp2.copy(gamepad2);
    }

    private static boolean justPressed(boolean now, boolean before) {
        return now && !before;
    }

    private static double deadzone(double v) {
        return Math.abs(v) < STICK_DEADZONE ? 0.0 : v;
    }

    private static double rpmToTicks(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    private static double ticksToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    private static double degToServoPos(double deg) {
        return Range.clip(deg / SERVO_RANGE_DEG, 0.0, 1.0);
    }
}
