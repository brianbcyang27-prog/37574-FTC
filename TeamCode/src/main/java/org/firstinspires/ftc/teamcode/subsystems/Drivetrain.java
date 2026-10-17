package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.framework.OpBehavior;

import com.qualcomm.robotcore.hardware.DcMotor;

/** 四輪麥克納姆底盤。ManualControl 提供目標，本類在 Loop() 寫入馬達。 */
public final class Drivetrain extends OpBehavior {
    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;

    private double forward;
    private double strafe;
    private double turn;
    private boolean slowMode;

    @Override
    public void Init() {
        lf = m_object.hardwareMap.get(DcMotor.class, Config.LF_NAME);
        rf = m_object.hardwareMap.get(DcMotor.class, Config.RF_NAME);
        lb = m_object.hardwareMap.get(DcMotor.class, Config.LB_NAME);
        rb = m_object.hardwareMap.get(DcMotor.class, Config.RB_NAME);

        lf.setDirection(Config.LF_DIRECTION);
        rf.setDirection(Config.RF_DIRECTION);
        lb.setDirection(Config.LB_DIRECTION);
        rb.setDirection(Config.RB_DIRECTION);

        for (DcMotor motor : new DcMotor[]{lf, rf, lb, rb}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // 馬達模式會保留上一個 OpMode 的設定（例如 RUN_TO_POSITION），這裡明確改回直接給電。
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        stopMotors();
    }

    @Override
    public void Start() {
        setDrive(0, 0, 0);
        stopMotors();
    }

    /** 三個輸入分別是前後、橫移、旋轉，範圍都是 -1 到 1。 */
    public void setDrive(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe * Config.STRAFE_GAIN;
        this.turn = turn;
    }

    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    @Override
    public void Loop() {
        double lfPower = forward + strafe + turn;
        double rfPower = forward - strafe - turn;
        double lbPower = forward - strafe + turn;
        double rbPower = forward + strafe - turn;

        // 斜推或同時旋轉時保持四輪比例：最大的那顆輪子縮到 1，其餘等比例縮小。
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(lfPower), Math.abs(rfPower)),
                Math.max(Math.abs(lbPower), Math.abs(rbPower))));
        double power = Config.MAX_POWER * (slowMode ? Config.SLOW_SCALE : 1.0) / max;

        lf.setPower(power * lfPower);
        rf.setPower(power * rfPower);
        lb.setPower(power * lbPower);
        rb.setPower(power * rbPower);

        m_object.telemetry.addData("Drive", slowMode ? "SLOW" : "NORMAL");
        m_object.telemetry.addData("Forward / Strafe / Turn", "%.2f / %.2f / %.2f",
                forward, strafe, turn);
        m_object.telemetry.addData("FL / FR", "%.2f / %.2f", power * lfPower, power * rfPower);
        m_object.telemetry.addData("BL / BR", "%.2f / %.2f", power * lbPower, power * rbPower);
    }

    @Override
    public void Stop() {
        setDrive(0, 0, 0);
        stopMotors();
    }

    private void stopMotors() {
        if (lf != null) lf.setPower(0);
        if (rf != null) rf.setPower(0);
        if (lb != null) lb.setPower(0);
        if (rb != null) rb.setPower(0);
    }
}
