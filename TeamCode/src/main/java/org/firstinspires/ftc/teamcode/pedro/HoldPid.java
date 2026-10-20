package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.controllers.Controller;

/**
 * ============================================================================
 *  位置保持用的 PID 控制器 (實作 Pedro 的 Controller 介面)
 * ============================================================================
 *
 *  為什麼不用 Pedro 內建的 Controller.pid()：
 *    1. 內建的 PIDController 沒有抗積分飽和 (anti-windup)。定點保持時誤差會
 *       一直存在，積分會無限累加，等你把機器人推走再回來就會超調、來回振盪。
 *    2. 第一筆量測的微分會產生衝擊 (derivative kick)，這裡第一筆直接忽略。
 *    3. 這個版本把 P / I / D 三項的貢獻分別記在 lastP / lastI / lastD，
 *       可以直接畫在 Panels 上看比例對不對，跟 launching_PID 的調法一致。
 *
 *  誤差單位：
 *    - 前後 / 左右 (forward / strafe) ... 英寸 (inch)
 *    - 朝向 (heading) .................. 弧度 (radian)
 *
 *  接法：Foresight 會呼叫 calculate(0, error)，並在 reset() 時呼叫 reset()。
 * ============================================================================
 */
public class HoldPid implements Controller {

    // 面板可調 (由 opmode 每圈寫進來)
    public double kP;
    public double kI;
    public double kD;

    /** 只有 |誤差| 小於這個值才累積積分 (單位同誤差：英寸或弧度) */
    public double iBand = 6.0;
    /** 積分項的最大貢獻 (power 0~1)，防止 windup */
    public double iMax = 0.15;

    // 最後一筆各項貢獻 (給儀表板畫圖用)
    public double lastP = 0.0;
    public double lastI = 0.0;
    public double lastD = 0.0;
    public double lastOut = 0.0;

    // 內部狀態
    private double integral = 0.0;
    private double prevError = 0.0;
    private boolean hasPrev = false;
    private long lastNano = 0L;

    public HoldPid(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    @Override
    public double calculate(double target, double error) {
        long now = System.nanoTime();
        double dt = lastNano == 0L ? 0.0 : (now - lastNano) / 1e9;
        lastNano = now;

        // --- P ---
        double p = kP * error;

        // --- I (只在 iBand 內累積，並限制貢獻大小) ---
        if (kI != 0.0 && Math.abs(error) <= iBand) {
            integral += error * dt;
            double cap = iMax / Math.abs(kI);
            integral = clamp(integral, -cap, cap);
        }
        double i = kI * integral;

        // --- D (第一筆不微分，避免剛 arm 的瞬間踢一下) ---
        double d = 0.0;
        if (kD != 0.0 && hasPrev && dt > 1e-4) {
            d = kD * (error - prevError) / dt;
        }
        prevError = error;
        hasPrev = true;

        double out = p + i + d;

        lastP = p;
        lastI = i;
        lastD = d;
        lastOut = out;
        return out;
    }

    @Override
    public double calculate(double target, double error, double derivative) {
        // Foresight 的定點保持只會呼叫兩參數版本，這裡維持一致
        return calculate(target, error);
    }

    @Override
    public void reset() {
        integral = 0.0;
        prevError = 0.0;
        hasPrev = false;
        lastNano = 0L;
        lastP = lastI = lastD = lastOut = 0.0;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
