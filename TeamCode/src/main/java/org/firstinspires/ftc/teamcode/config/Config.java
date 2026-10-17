package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** 只保存目前底盤需要的設定。硬體名稱必須與 Robot Configuration 完全一致。 */
public final class Config {
    private Config() { }

    // 與 Driver Station 的 Robot Configuration 名稱一致（大小寫也要相同）。
    public static final String LF_NAME = "FL";
    public static final String RF_NAME = "FR";
    public static final String LB_NAME = "BL";
    public static final String RB_NAME = "BR";

    // 沿用 driveChain.java 的方向；第一次上車仍要架空逐輪確認。
    public static final DcMotorSimple.Direction LF_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RF_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction LB_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RB_DIRECTION = DcMotorSimple.Direction.FORWARD;

    // 第一次上車先用低速；確認四輪方向都正確後再調高（最大 1.0）。
    public static final double MAX_POWER = 1.0;
    // 按住 gamepad1 右肩鍵時，輸出再乘上這個比例，方便精細對位。
    public static final double SLOW_SCALE = 0.5;
    // 麥克納姆橫移比前進慢，稍微放大橫移輸入來補償。
    public static final double STRAFE_GAIN = 1.1;
    public static final double STICK_DEADBAND = 0.05;
}
