package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
public class driveChain extends OpMode {

    public static double MAX_SPEED   = 1.0;
    public static double SLOW_SPEED  = 0.4;
    public static double DEADZONE    = 0.05;
    public static double STRAFE_GAIN = 1.1;   // mecanum strafes slower than it drives; compensate

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Ready. F310 switch on X, press START + A to bind as gamepad1.");
    }

    @Override
    public void loop() {
        double axial   = deadzone(-gamepad1.left_stick_y);              // stick Y is inverted
        double lateral = deadzone(gamepad1.left_stick_x) * STRAFE_GAIN;
        double yaw     = deadzone(gamepad1.right_stick_x);

        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        // Normalize so no wheel exceeds 1.0 while keeping the ratios
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));
        double speed = gamepad1.right_bumper ? SLOW_SPEED : MAX_SPEED;

        frontLeft.setPower (fl / max * speed);
        frontRight.setPower(fr / max * speed);
        backLeft.setPower  (bl / max * speed);
        backRight.setPower (br / max * speed);

        telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "FULL");
        telemetry.addData("Axial / Lateral / Yaw", "%.2f / %.2f / %.2f", axial, lateral, yaw);
        telemetry.addData("FL / FR", "%.2f / %.2f", fl / max * speed, fr / max * speed);
        telemetry.addData("BL / BR", "%.2f / %.2f", bl / max * speed, br / max * speed);
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }
}

