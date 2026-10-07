package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp
public class launching_test3 extends OpMode {

    public static double TICKS_PER_REV = 28;      // goBILDA 6000rpm 裸馬達
    public static double MAX_RPM       = 5400;    // 6000 的 90%
    public static double READY_TOL     = 100;     // 誤差在 ±100 RPM 內算到位

    // 飛輪 PIDF 起始值，F ≈ 32767 / 最大 ticks每秒(2800) ≈ 11.7
    public static double kP = 10.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 11.7;

    private DcMotorEx motor;
    private double presetRatio = 0;
    private boolean wasReady = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);   // 飛輪自己滑停
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        motor.setVelocityPIDFCoefficients(kP, kI, kD, kF);   // Dashboard 改了立即生效

        if (gamepad1.a) presetRatio = 1.0;
        if (gamepad1.x) presetRatio = 0.75;
        if (gamepad1.y) presetRatio = 0.5;
        if (gamepad1.b) presetRatio = 0.25;
        if (gamepad1.right_bumper) presetRatio = 0;

        double ratio;
        if (gamepad1.right_bumper) {
            ratio = 0;
        } else if (gamepad1.right_trigger > 0.05) {
            ratio = gamepad1.right_trigger;
        } else {
            ratio = presetRatio;
        }

        double targetRpm = ratio * MAX_RPM;
        motor.setVelocity(targetRpm / 60.0 * TICKS_PER_REV);

        double actualRpm = motor.getVelocity() / TICKS_PER_REV * 60.0;
        boolean ready = targetRpm > 0 && Math.abs(actualRpm - targetRpm) < READY_TOL;

        if (ready && !wasReady) gamepad1.rumble(200);   // 剛到位時震一下
        wasReady = ready;

        telemetry.addData("Status", ready ? ">>> READY <<<" : (targetRpm > 0 ? "spinning up..." : "stopped"));
        telemetry.addData("Target RPM", targetRpm);    // 用數字（不是字串），Dashboard 才能畫圖
        telemetry.addData("Actual RPM", actualRpm);
        telemetry.addData("Error", targetRpm - actualRpm);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setVelocity(0);
    }
}