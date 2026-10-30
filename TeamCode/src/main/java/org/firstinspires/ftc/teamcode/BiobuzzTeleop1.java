package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Biobuzz Teleop 1", group = "Linear Opmode")
public class BiobuzzTeleop1 extends LinearOpMode {

    // Drive motors
    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive drive;

    private GamepadEx driverOp, toolOp;

    // Both motors are intakes now (the old "outtake" motor is the second intake)
    private MotorEx intakeMotor, intake2Motor;

    // Flip these if a motor spins the wrong way. The two motors drive the same
    // roller through 45 degree gears from opposite sides, so one is reversed by default.
    private static final boolean INTAKE_REVERSED = false;
    private static final boolean INTAKE2_REVERSED = true;

    @Override
    public void runOpMode() {
        frontLeft = new Motor(hardwareMap, "frontLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backLeft = new Motor(hardwareMap, "backLeft");
        backRight = new Motor(hardwareMap, "backRight");

        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        driverOp = new GamepadEx(gamepad1);
        toolOp = new GamepadEx(gamepad2);

        intakeMotor = new MotorEx(hardwareMap, "intake");
        intake2Motor = new MotorEx(hardwareMap, "outtake"); // same config name as MecanumTeleOp

        intakeMotor.setInverted(INTAKE_REVERSED);
        intake2Motor.setInverted(INTAKE2_REVERSED);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            driverOp.readButtons();
            toolOp.readButtons();

            // Gamepad 1: left stick strafe/forward, right stick turn
            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    -driverOp.getLeftY(),
                    driverOp.getRightX()
            );

            // Gamepad 2: left stick down = intake in, up = push out
            double intakePower = toolOp.getLeftY();
            intakeMotor.set(intakePower);
            intake2Motor.set(intakePower);

            // TODO: 2 servo gates and 1 outtake motor controls

            telemetry.addData("Status", "Running");
            telemetry.addData("Drive", "X:%.2f Y:%.2f R:%.2f",
                    driverOp.getLeftX(), -driverOp.getLeftY(), driverOp.getRightX());
            telemetry.addData("Intake Power", "%.2f", intakePower);
            telemetry.addData("Intake 1 RPM", "%.0f", intakeMotor.getVelocity());
            telemetry.addData("Intake 2 RPM", "%.0f", intake2Motor.getVelocity());
            telemetry.update();
        }
    }
}
