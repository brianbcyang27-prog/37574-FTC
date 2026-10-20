package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.HoldPid;

/**
 * ============================================================================
 *  定點保持 (Position Hold / 回到原點) —— Pedro Pathing
 * ============================================================================
 *
 *  行為：
 *    1. 按 A       → 把「現在的座標」鎖起來 (arm)。此時機器人會待在這個點。
 *    2. 有人把機器人推走 → 一放開手把，它就會自己走回剛剛鎖住的那個點、那個朝向。
 *    3. 推著走的時候按住搖桿 → 暫時接手駕駛 (座標依然鎖著，放手就回來)。
 *    4. 按 RB     → 緊急解除 + 斷電 (跟飛輪那支的 RB 手感一樣)，要再按 A 才會重鎖。
 *
 *  座標系 (Pedro)：前方 = +X，左方 = +Y，逆時針為正。
 *
 *  ★ 前提：機器人要能知道自己的位置 (odometry pod + IMU)。
 *      沒跑過 AutoTune 的話，「回到原點」會有偏移 —— 打開
 *      http://192.168.43.1:10158 跑 Two Wheel Tuner 與 Foresight Tuner。
 *
 *  面板上只有三個可調變數 (Panels :8001 / FTC Dashboard :8080)：
 *      kP / kI / kD  →  平移 (前後 + 左右) 的 PID。朝向用 Constants 裡固定的 P。
 *      ★ 這三顆會同時套用到 forward 與 strafe 兩個軸。
 *
 *  手把 (Logitech F310，X 模式)：
 *    A ........... 鎖定目前位置 (開始保持)
 *    RB .......... 解除鎖定 + 斷電 (機器人不會再自己動)
 *    START ....... 清掉 PID 積分 / 錯誤統計
 *    左搖桿 ...... 暫時接手駕駛 (鎖定中也可用，放手就回到原點)
 *    到位時手把震一下
 * ============================================================================
 */
@Config        // FTC Dashboard :8080
@Configurable  // Panels        :8001
@TeleOp(name = "Hold Position", group = "Drive")
public class HoldPosition extends OpMode {

    // =========================================================================
    //  1. 面板可調參數 —— 只有這三顆會出現在面板上
    // =========================================================================
    /** 比例項 [power / 英寸] ← 回到原點的速度主要靠這個 (起手 0.30) */
    public static double kP = 0.30;
    /** 積分項 [power / (英寸*秒)] ← 消掉穩態誤差，先用 0，要加從 0.01 開始 */
    public static double kI = 0.0;
    /** 微分項 [power / (英寸/秒)] ← 通常保持 0，量測有雜訊會發抖 */
    public static double kD = 0.0;

    // =========================================================================
    //  2. 固定常數 (不出現在面板)
    // =========================================================================
    /** 搖桿死區 */
    private static final double DEADZONE = 0.05;
    /** 手動駕駛時的最大 power (1.0 = 全速) */
    private static final double MANUAL_POWER = 0.7;
    /** 平移誤差小於這個距離算「回到位」(英寸) */
    private static final double HOLD_TOL_IN = 1.5;
    /** 朝向誤差小於這個角度算「回到位」(度) */
    private static final double HOLD_TOL_DEG = 6.0;

    // =========================================================================
    //  3. 執行期狀態
    // =========================================================================
    private Follower follower;

    private TelemetryManager panels;
    private Telemetry ftcTelemetry;

    /** 鎖定的目標點 (null = 沒有鎖定) */
    private Pose home = null;

    private boolean prevA, prevB, prevStart, prevRb;
    private boolean settled = false;
    private double peakErrIn = 0.0;

    // =========================================================================
    //  init / loop / stop
    // =========================================================================

    @Override
    public void init() {
        // Panels (8001) + Driver Station + FTC Dashboard (8080) 同時輸出
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        ftcTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(0, 0, 0));   // 起點當作 (0, 0, 0)

        panels.addData("Status", "init done — press A to lock position");
        panels.update(ftcTelemetry);
    }

    @Override
    public void loop() {
        // ---- 1. 手把：先處理按鈕 ------------------------------------------
        boolean a = gamepad1.a;
        boolean rb = gamepad1.right_bumper;
        boolean b = gamepad1.b;
        boolean start = gamepad1.start;

        if (a && !prevA) arm();                     // 鎖定目前座標
        if ((rb && !prevRb) || (b && !prevB)) disarm();  // 解除 + 斷電
        if (start && !prevStart) {                  // 清統計 / 清積分
            Constants.resetHoldPids();
            peakErrIn = 0.0;
            settled = false;
        }
        prevA = a;
        prevB = b;
        prevRb = rb;
        prevStart = start;

        // ---- 2. 搖桿 ------------------------------------------------------
        //     FTC 的 stick_y 往上推是負值，所以要取負號才是「前進」
        double fwd = deadzone(-gamepad1.left_stick_y);
        double lat = deadzone(gamepad1.left_stick_x);
        double yaw = deadzone(gamepad1.right_stick_x);
        boolean driving = Math.abs(fwd) > 0 || Math.abs(lat) > 0 || Math.abs(yaw) > 0;

        // ---- 3. 面板 PID 寫進控制器 (每圈，改面板立即生效) ----------------
        applyPanelPid();

        // ---- 4. 決定這一圈要「駕駛」還是「保持」 ---------------------------
        if (driving) {
            // 暫時接手：座標照樣鎖著，放手就會回去
            follower.manual(fwd * MANUAL_POWER, lat * MANUAL_POWER, yaw * MANUAL_POWER);
        } else if (home != null) {
            if (!follower.holding()) {
                // 從手動切回保持：清掉積分再回去，避免帶著移動時累積的誤差衝回去
                Constants.resetHoldPids();
                follower.hold(home, true);   // true = 套用 hold scaling，比較不振盪
            }
        } else {
            // 沒鎖定：就是一般 teleop (回中會煞車)
            follower.manual(0, 0, 0);
        }

        follower.update();

        // ---- 5. 誤差統計 + 到位震動 ---------------------------------------
        Pose pose = follower.pose();
        updateSettle(pose);

        // ---- 6. 儀表板 -----------------------------------------------------
        HoldPid fp = Constants.forwardPid;
        HoldPid sp = Constants.strafePid;
        double errIn = errorInches(pose);
        double errDeg = Math.toDegrees(errorHeading(pose));

        panels.addData("Status", statusText(errIn, errDeg));
        panels.addData("Mode", follower.mode().name());
        panels.addData("ErrIn", errIn);
        panels.addData("ErrDeg", errDeg);
        panels.addData("PoseX", pose.x());
        panels.addData("PoseY", pose.y());
        panels.addData("PoseDeg", Math.toDegrees(pose.heading()));
        if (home != null) {
            panels.addData("HomeX", home.x());
            panels.addData("HomeY", home.y());
            panels.addData("HomeDeg", Math.toDegrees(home.heading()));
        }
        panels.addData("PeakErrIn", peakErrIn);
        panels.addData("kP", kP);
        panels.addData("kI", kI);
        panels.addData("kD", kD);
        // P / I / D 三項各自的貢獻，畫成折線圖最好調
        panels.addData("P_out", fp.lastP);
        panels.addData("I_out", fp.lastI);
        panels.addData("D_out", fp.lastD);
        panels.addData("StrafeP_out", sp.lastP);
        double[] wp = ((Mecanum) follower.drivetrain).wheelPowers;
        panels.addData("Wheels", String.format("FL %.2f FR %.2f BL %.2f BR %.2f", wp[0], wp[1], wp[2], wp[3]));
        panels.update(ftcTelemetry);
    }

    @Override
    public void stop() {
        follower.stop();   // 斷電，機器人不再自己動
    }

    // =========================================================================
    //  鎖定 / 解除
    // =========================================================================

    /** 鎖定「現在」的座標，開始保持 */
    private void arm() {
        Pose p = follower.pose();
        home = new Pose(p.x(), p.y(), p.heading());   // 複製一份，不要抓到 localizer 的即時物件
        Constants.resetHoldPids();
        settled = false;
        peakErrIn = 0.0;
        follower.hold(home, true);
        gamepad1.rumble(200);
    }

    /** 解除鎖定 + 斷電 (要按 A 才會重新供電保持) */
    private void disarm() {
        home = null;
        settled = false;
        Constants.resetHoldPids();
        follower.stop();
        gamepad1.rumble(400);
    }

    /** 把面板的 kP / kI / kD 同步到前後、左右兩顆 PID (朝向用 Constants 的固定值) */
    private void applyPanelPid() {
        Constants.forwardPid.kP = kP;
        Constants.forwardPid.kI = kI;
        Constants.forwardPid.kD = kD;
        Constants.strafePid.kP = kP;
        Constants.strafePid.kI = kI;
        Constants.strafePid.kD = kD;
    }

    // =========================================================================
    //  誤差 / 狀態
    // =========================================================================

    private double errorInches(Pose pose) {
        if (home == null) return 0.0;
        return Math.hypot(home.x() - pose.x(), home.y() - pose.y());
    }

    /** 角度誤差，回傳 -PI ~ PI */
    private double errorHeading(Pose pose) {
        if (home == null) return 0.0;
        double d = home.heading() - pose.heading();
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        return d;
    }

    private void updateSettle(Pose pose) {
        if (home == null) {
            settled = false;
            return;
        }
        double errIn = errorInches(pose);
        double errDeg = Math.toDegrees(errorHeading(pose));
        peakErrIn = Math.max(peakErrIn, errIn);

        boolean inTol = errIn <= HOLD_TOL_IN && Math.abs(errDeg) <= HOLD_TOL_DEG;
        if (inTol && !settled) {
            settled = true;
            gamepad1.rumble(200);   // 回到位了，震一下
        } else if (!inTol) {
            settled = false;
        }
    }

    private String statusText(double errIn, double errDeg) {
        if (home == null) {
            return "NOT LOCKED — press A";
        }
        boolean inTol = errIn <= HOLD_TOL_IN && Math.abs(errDeg) <= HOLD_TOL_DEG;
        return inTol ? ">>> HOLDING <<<" : ">>> RETURNING <<<";
    }

    private double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }
}
