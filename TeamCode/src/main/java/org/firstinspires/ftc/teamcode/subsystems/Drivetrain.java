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

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        this.strafe = strafe;
        this.turn = turn;
    }

    @Override
    public void Loop() {
        // 斜推或同時旋轉時保持四輪比例，並限制最大輸出。
        double scale = Math.max(1.0,
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        double power = Config.MAX_POWER / scale;

        lf.setPower(power * (forward + strafe + turn));
        rf.setPower(power * (forward - strafe - turn));
        lb.setPower(power * (forward - strafe + turn));
        rb.setPower(power * (forward + strafe - turn));
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
