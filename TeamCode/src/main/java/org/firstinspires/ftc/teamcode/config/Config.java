package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** 只保存目前底盤需要的設定。硬體名稱必須與 Robot Configuration 完全一致。 */
public final class Config {
    private Config() { }

    // 與 driveChain.java 使用相同的 Robot Configuration 名稱。
    public static final String LF_NAME = "frontLeft";
    public static final String RF_NAME = "frontRight";
    public static final String LB_NAME = "backLeft";
    public static final String RB_NAME = "backRight";

    // 沿用 driveChain.java 的方向；第一次上車仍要架空逐輪確認。
    public static final DcMotorSimple.Direction LF_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RF_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction LB_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction RB_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final double MAX_POWER = 0.30;
    public static final double STICK_DEADBAND = 0.05;
}
