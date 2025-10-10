package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "StraferMecanumTeleOp", group = "TeleOp")
public class StraferMecanumTeleOp extends OpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse right side motors for correct mecanum drive
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Strafer Mecanum TeleOp Initialized");
    }

    @Override
    public void loop() {
    // Controls
    double rotation = gamepad1.left_stick_x;    // Turning (unchanged)
    // Invert the strafing X and Y axes per request (option B)
    double strafe   = -gamepad1.right_stick_x;  // Sideways (inverted)
    double forward  = gamepad1.right_stick_y;   // Forward/Backward (inverted Y)

        // Mecanum drive calculations
        double fl = forward + strafe + rotation;
        double bl = forward - strafe + rotation;
        double fr = forward - strafe - rotation;
        double br = forward + strafe - rotation;

        // Normalize powers so no value exceeds +/-1
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        // Set motor powers
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);

        // Telemetry
        telemetry.addData("Front Left Power", fl);
        telemetry.addData("Back Left Power", bl);
        telemetry.addData("Front Right Power", fr);
        telemetry.addData("Back Right Power", br);
        telemetry.update();
    }
}
