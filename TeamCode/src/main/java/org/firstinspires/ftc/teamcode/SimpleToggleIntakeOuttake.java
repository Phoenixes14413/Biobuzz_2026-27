package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SimpleToggleIntakeOuttake", group = "Simple")
public class SimpleToggleIntakeOuttake extends OpMode {
    private DcMotor intakeMotor;
    private DcMotor outtakeMotor;
    private boolean intakeOn = false;
    private boolean outtakeOn = false;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        intakeMotor.setPower(0);
        outtakeMotor.setPower(0);
    }

    @Override
    public void loop() {
        boolean aPressed = gamepad1.a;
        boolean bPressed = gamepad1.b;

        // Toggle intake motor on/off with A
        if (aPressed && !lastA) {
            intakeOn = !intakeOn;
            intakeMotor.setPower(intakeOn ? 0.15 : 0.0);
        }
        lastA = aPressed;

        // Toggle outtake motor on/off with B
        if (bPressed && !lastB) {
            outtakeOn = !outtakeOn;
            outtakeMotor.setPower(outtakeOn ? -0.15 : 0.0);
        }
        lastB = bPressed;

        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Outtake On", outtakeOn);
        telemetry.update();
    }
}
