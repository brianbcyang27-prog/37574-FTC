package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class launching extends OpMode {

    private DcMotor motor;
    private double presetPower = 0;   // A/X/Y/B 設定的速度，按 RB 歸零

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Ready");
    }

    @Override
    public void loop() {
        if (gamepad1.a) presetPower = 1.0;
        if (gamepad1.x) presetPower = 0.75;
        if (gamepad1.y) presetPower = 0.5;
        if (gamepad1.b) presetPower = 0.25;
        if (gamepad1.right_bumper) presetPower = 0;

        double power;
        if (gamepad1.right_bumper) {
            power = 0;                              // RB 停，優先權最高
        } else if (gamepad1.right_trigger > 0.05) {
            power = gamepad1.right_trigger;         // 扣 RT 時照 RT 深度，按到底 = 全速
        } else {
            power = presetPower;                    // 沒扣 RT 就用按鈕設定的速度
        }

        motor.setPower(power);

        telemetry.addData("RT", "%.2f", gamepad1.right_trigger);
        telemetry.addData("Preset", "%.2f", presetPower);
        telemetry.addData("Power", "%.2f", power);
        telemetry.update();
    }

    @Override
    public void stop() {
        motor.setPower(0);
    }
}