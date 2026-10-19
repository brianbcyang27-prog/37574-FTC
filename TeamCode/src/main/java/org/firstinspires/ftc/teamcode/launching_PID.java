package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.Properties;

/**
 * ============================================================================
 *  飛輪發射器 (Shooter / Flywheel) 速度控制  --  自寫 PIDF 版本
 * ============================================================================
 *
 *  設計目標：球打上飛輪後轉速被拉下來，要「用最短時間」把轉速拉回目標值。
 *  為了縮短恢復時間，這支程式做了幾件關鍵的事：
 *
 *   1. 前饋 (kF / kS) ── 先用「目標轉速 / MAX_RPM」估出維持轉速需要的 power 直接
 *      送出去，PID 只需要補剩下的差額。擾動發生時，誤差會立刻讓輸出打飽和，
 *      等於用最大力量把轉速追回來，這是恢復速度的來源。
 *
 *   2. 抗積分飽和 (anti-windup, I_BAND_RPM / I_MAX) ── 只有在接近目標時才累積
 *      積分。轉速被大幅拉低的那一瞬間不累積，才不會在回穩時積分過頭造成超調。
 *
 *   3. 微分作用在「量測值」上 (derivative on measurement, kD) ── 不是對誤差微分，
 *      所以目標轉速跳變時不會產生微分衝擊 (derivative kick)。
 *
 *   4. 電壓補償 (VOLTAGE_COMP) ── 電池越沒電，同樣 power 得到的轉速越低。
 *      這裡用實際匯流排電壓換算，讓同一組 PID 參數在電池掉到 11V 時依然準。
 *
 *   5. 自算轉速 (USE_TICK_DELTA_VELOCITY) ── hub 內建的 getVelocity() 是用一段
 *      固定的取樣視窗算出來的，延遲較大；回饋越慢，恢復就越慢。這裡改用編碼器
 *      位置差分自己算，視窗可調 (VEL_WINDOW_MS)，讓控制器更早知道轉速掉了。
 *
 *   6. 恢復時間量測 (RecoveryMs) ── 程式會自動偵測「轉速被打下來」的瞬間並計時，
 *      直到回到容許範圍為止，把 ms 數值畫在儀表板上。這樣調參就有客觀數字可以
 *      比大小，而不是「感覺好像快一點」。
 *
 *  ---------------------------------------------------------------------------
 *  調參介面 (兩個都可以用，同一組變數)：
 *    Panels        http://192.168.43.1:8001          ← 主要，圖表最好用
 *    FTC Dashboard http://192.168.43.1:8080/dash     ← 備用
 *  (Mac 連上 Control Hub 的 Wi-Fi 後即可開啟，不需要 USB)
 *
 *  ★ 面板上的可調變數，其他全部是固定常數，不會出現在面板：
 *        SPEED ....... 一般速度 (0.0 ~ 1.0)，目標 = SPEED * MAX_RPM
 *        TARGET_RPM .. 直接指定目標轉速；> 0 時優先使用，= 0 時改用 SPEED
 *        kP / kI / kD  PID 三個增益
 *        kF / kS ..... 前饋：kF 乘上 (目標 / MAX_RPM)，kS 是起步墊的固定 power
 *        MAX_REVERSE . 降速時反向出力的上限 (0 ~ 1)，0 = 只靠摩擦滑停
 *
 *  ★ 按下 START (▶) 時 SPEED 與 TARGET_RPM 都會歸零，飛輪不會自己轉起來。
 *    請在開始之後再用 D-Pad 上 或 面板設定速度。
 *
 *  調參順序建議：
 *    (0) 先調前饋：kP、kI、kD 都設 0，設一個 TARGET_RPM (例如 4000)，調 kF
 *        直到 ActualRPM 大致貼住 TargetRPM。轉速偏低就加大 kF，偏高就減小。
 *        前饋準了，PID 只需要補一點點，恢復也會更快。
 *    (1) 再加 kP：從 0.0015 開始往上加，直到「手指撥一下飛輪」能很快回來、
 *        且不來回震盪。kP 是恢復速度的主要來源。
 *    (2) 加一點 kI (0.001~0.005) 消掉穩態誤差。太大會在球打上去之後超調。
 *    (3) kD 通常保持 0。要加的話從 0.0002 開始「非常小」，因為 hub 的轉速
 *        量測有雜訊，kD 放太大會讓馬達發抖。
 *    (4) 每次調完，用 START 鍵歸零 RecoveryMs 統計，重新打幾顆球比較數字。
 *    看圖時注意 Saturated：= 1 表示輸出已經頂到上限，PID 再加大也追不回來。
 *
 *  手把 (Logitech F310，切到 XInput / X 模式)：
 *    D-Pad 上 ....... 速度 +5% (按住會連續增加)；TARGET_RPM 模式時 +100 RPM
 *    D-Pad 下 ....... 速度 -5% (按住會連續減少)；TARGET_RPM 模式時 -100 RPM
 *    RB ............. 緊急切斷：SPEED、TARGET_RPM 歸零、飛輪立刻斷電
 *                     (要再按 D-Pad 上才會重新供電)
 *    START .......... 歸零 RecoveryMs / DipDepth / Shots 統計
 *    BACK ........... 立刻存檔目前的參數 (手把震一下)
 *    到位時手把震一下
 *
 *  參數存檔：kP / kI / kD / kF / kS / MAX_REVERSE 會存在 Control Hub 的
 *  /sdcard/FIRST/launching_pid.properties。按 STOP 或 BACK 時存檔，下次 INIT 自動讀回，
 *  重開機、重新上傳程式都會保留。想回到程式裡寫的預設值，刪掉這個檔案即可。
 * ============================================================================
 */
@Config        // FTC Dashboard :8080  (備用)
@Configurable  // Panels        :8001  (主要調參介面)
@TeleOp(name = "Launching PID", group = "Shooter")
public class launching_PID extends OpMode {

    // =========================================================================
    //  1. 面板可調參數 —— 只有這些會出現在 Panels / Dashboard 上
    // =========================================================================
    /** 一般速度比例 0.0 ~ 1.0。面板滑桿 或 手把 D-Pad 上/下 都可以調。按 START 時歸零 */
    public static double SPEED = 0.0;
    /** 直接指定目標轉速 [RPM]。> 0 時優先使用 (上限 MAX_RPM)，= 0 時改用 SPEED。按 START 時歸零 */
    public static double TARGET_RPM = 0.0;
    /** 比例項 [power / RPM]  ← 恢復速度主要靠這個 */
    public static double kP = 0.0015;
    /** 積分項 [power / (RPM*秒)] ← 只負責修掉穩態誤差，保持小 */
    public static double kI = 0.0020;
    /** 微分項 [power / (RPM/秒)] ← 量測有雜訊，通常保持 0 */
    public static double kD = 0.0;
    /** 前饋：power = kF * (目標轉速 / MAX_RPM)。kF = 1.0 表示前饋就等於比例 power */
    public static double kF = 1.00;
    /** 靜摩擦補償：只要有目標轉速就先墊一點 power，讓飛輪比較容易起步 */
    public static double kS = 0.03;
    /**
     * 反向出力上限 (power，0 ~ 1)。降速時由 PID 自己決定要不要反向出力把轉速壓下來，
     * 這裡只是上限：越大降速越快，但電流越大、齒輪/皮帶衝擊越大。
     * 0 = 不反向出力，只靠摩擦自然滑停。
     */
    public static double MAX_REVERSE = 0.30;

    // =========================================================================
    //  2. 固定常數 (全部 private，不出現在面板)
    // =========================================================================
    /** 硬體設定裡那顆飛輪馬達的名字 */
    private static final String MOTOR_NAME = "motor";
    /** goBILDA 6000rpm 裸馬達：馬達軸一圈 28 個 encoder count */
    private static final double TICKS_PER_REV = 28.0;
    /** 飛輪定義上的最高轉速 (SPEED 的 100%)，先保守抓 6000 的 90% */
    private static final double MAX_RPM = 5400.0;

    /** D-Pad 每按一次 (或連續重複時) 調整的速度比例 */
    private static final double SPEED_STEP = 0.05;
    /** TARGET_RPM 模式下，D-Pad 每按一次調整的轉速 (RPM) */
    private static final double RPM_STEP = 100.0;
    /** D-Pad 按住不放時，每隔多久自動再加/減一次 (ms) */
    private static final double DPAD_REPEAT_MS = 200.0;
    /** 程式改了面板變數後，最快多久把新值推回網頁一次 (ms)，避免太頻繁拖慢 loop */
    private static final double PANEL_SYNC_MIN_MS = 150.0;

    /**
     * 調好的參數 (kP/kI/kD/kF/kS/MAX_REVERSE) 存在 Control Hub 上的這個檔案。
     * INIT 時自動讀回來，STOP 或按 BACK 時存檔，重開機、重新上傳程式都會保留。
     * 想回到程式裡寫的預設值：刪掉這個檔案 (/sdcard/FIRST/launching_pid.properties)。
     */
    private static final File TUNING_FILE = new File(AppUtil.FIRST_FOLDER, "launching_pid.properties");

    /** true = 改回 SDK 內建速度環 (舊做法)，用來跟自己寫的版本 A/B 比較 */
    private static final boolean USE_SDK_VELOCITY = false;
    /** 只有誤差在 ±這個值以內才累積積分 (防大擾動時 windup) */
    private static final double I_BAND_RPM = 250.0;
    /** 積分項的最大貢獻 (power) */
    private static final double I_MAX = 0.25;
    /** 輸出上限 (power) */
    private static final double MAX_DUTY = 1.00;
    /** 輸出斜率限制 [power/秒]，0 = 不限制 (恢復最快，但起步電流較大) */
    private static final double MAX_DUTY_RATE = 0.0;

    /** SDK 內建速度環用的 PIDF (只有 USE_SDK_VELOCITY = true 時才生效) */
    private static final double SDK_P = 10.0;
    private static final double SDK_I = 0.0;
    private static final double SDK_D = 0.0;
    private static final double SDK_F = 11.7;

    /** 電壓補償開關 */
    private static final boolean VOLTAGE_COMP = true;
    /** 電壓補償的基準電壓 (V)，通常用充滿電時的電壓 */
    private static final double NOMINAL_VOLTAGE = 12.5;

    /** true = 用編碼器位置差分自算轉速 (延遲比 hub 內建小)，false = 用 getVelocity() */
    private static final boolean USE_TICK_DELTA_VELOCITY = true;
    /** 差分視窗 (ms)。越大越平滑但越慢；6000rpm 下 50ms 約 126 個 count，雜訊約 ±1% */
    private static final double VEL_WINDOW_MS = 50.0;
    /** 低通濾波強度 (0~1，1 = 不濾波)。1 反應最快但最抖 */
    private static final double VEL_SMOOTH = 0.65;

    /** 誤差在 ±這個 RPM 內算到位 */
    private static final double READY_TOL_RPM = 100.0;
    /** 要連續維持這麼久才算真的穩 (避免只是路過) */
    private static final double READY_HOLD_MS = 150.0;
    /** 轉速掉超過這個值就開始計時恢復 */
    private static final double DIP_THRESHOLD_RPM = 120.0;
    /** 目標轉速上升斜率 [RPM/秒]，避免起步瞬間拉爆電流 */
    private static final double TARGET_SLEW_UP = 9000.0;
    /** 目標轉速下降斜率 [RPM/秒] */
    private static final double TARGET_SLEW_DOWN = 9000.0;

    // =========================================================================
    //  執行期狀態
    // =========================================================================
    private DcMotorEx motor;
    private VoltageSensor voltageSensor;

    private TelemetryManager panels;
    private Telemetry ftcTelemetry;

    private final ElapsedTime clock = new ElapsedTime();      // 開機後累計時間 (ms)，不要 reset
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime readyTimer = new ElapsedTime();
    private final ElapsedTime dipTimer = new ElapsedTime();

    private final Pidf pidf = new Pidf();

    private double targetRpm = 0.0;
    private double lastDesiredRpm = 0.0;     // 上一圈手把 / 面板要求的轉速，用來偵測「改速度」
    private boolean sdkModeApplied = false;  // 目前馬達跑在哪個模式
    private double lastSdkP = Double.NaN, lastSdkI = Double.NaN, lastSdkD = Double.NaN, lastSdkF = Double.NaN;

    // 手把：D-Pad 調速 + RB 緊急切斷
    private boolean prevDpadUp, prevDpadDown, prevCut, prevStart;
    private final ElapsedTime dpadRepeatTimer = new ElapsedTime();

    // 轉速估算
    private boolean velInit = false;
    private boolean velTickDeltaMode = false;
    private double velWindowStart = 0.0;
    private int velWindowStartPos = 0;
    private double rawTps = 0.0;
    private double smoothTps = 0.0;
    // 轉速變化率 [RPM/秒]，給 kD 用。只在轉速真的更新時重算，用實際取樣間隔來除
    private double measuredRateRpmPerSec = 0.0;
    private double prevSampleRpm = 0.0;
    private double prevSampleMs = 0.0;
    private boolean hasPrevSample = false;

    // 面板同步：手把 / 程式改了 SPEED、TARGET_RPM 後，要推回 Panels / Dashboard 網頁
    private boolean panelDirty = false;
    private final ElapsedTime panelSyncTimer = new ElapsedTime();

    // 參數存檔
    private String tuningStatus = "";
    private boolean prevBack = false;

    // 到位 / 恢復量測
    private boolean settled = false;
    private boolean armed = false;
    private boolean inDip = false;
    private double dipMinRpm = 0.0;
    private double lastRecoveryMs = 0.0;
    private double worstRecoveryMs = 0.0;
    private double lastDipDepthRpm = 0.0;
    private int shots = 0;

    // =========================================================================
    //  init / loop / stop
    // =========================================================================

    @Override
    public void init() {
        // Panels (8001) + Driver Station + FTC Dashboard (8080) 同時輸出
        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        ftcTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotor.Direction.FORWARD);
        // 飛輪讓它自然滑停，不要煞車 (煞車會發熱、也會讓恢復時間變長)
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(USE_SDK_VELOCITY);

        for (VoltageSensor s : hardwareMap.voltageSensor) {
            voltageSensor = s;
            break;
        }

        // 讀回上次存的參數，並推到網頁上，讓面板顯示的是實際使用的值
        loadTuning();
        syncPanelsNow();

        loopTimer.reset();
        clock.reset();
        panels.addData("Status", "init done");
        panels.addData("Tuning", tuningStatus);
        panels.addData("kP / kI / kD", String.format("%.5f / %.5f / %.5f", kP, kI, kD));
        panels.addData("kF / kS / MAX_REVERSE", String.format("%.3f / %.3f / %.2f", kF, kS, MAX_REVERSE));
        panels.update(ftcTelemetry);
    }

    @Override
    public void start() {
        // SPEED / TARGET_RPM 是 static，會保留上一次執行或面板設定的值。
        // 一律從 0 開始，避免按下 START 飛輪就自己衝到高速。
        SPEED = 0.0;
        TARGET_RPM = 0.0;
        targetRpm = 0.0;
        lastDesiredRpm = 0.0;
        pidf.reset();
        loopTimer.reset();
        panelDirty = true;   // 讓網頁上的 SPEED / TARGET_RPM 也顯示歸零
    }

    @Override
    public void loop() {
        double nowMs = clock.milliseconds();
        double dt = Math.max(loopTimer.seconds(), 1e-4);
        loopTimer.reset();

        double voltage = readVoltage();

        // ---- 1. 手把 / 面板：決定希望的目標轉速 desiredRpm ----------------
        double desiredRpm = readGamepadTargetRpm();

        // 使用者改了速度 (手把或面板)：重新開始到位判斷，
        // 否則升速時的誤差會被誤算成「球打上去」，污染 RecoveryMs / Shots
        if (Math.abs(desiredRpm - lastDesiredRpm) > 1e-3) {
            armed = false;
            inDip = false;
            settled = false;
            readyTimer.reset();
        }
        lastDesiredRpm = desiredRpm;

        // ---- 2. 目標轉速 + 斜率限制 (保護電流，也讓量測可重現) -----------
        if (desiredRpm <= 1e-3) {
            targetRpm = 0.0;
        } else if (targetRpm <= 1e-3) {
            targetRpm = Math.min(desiredRpm, TARGET_SLEW_UP * dt);
        } else {
            double lo = TARGET_SLEW_DOWN > 0 ? targetRpm - TARGET_SLEW_DOWN * dt : desiredRpm;
            double hi = TARGET_SLEW_UP > 0 ? targetRpm + TARGET_SLEW_UP * dt : desiredRpm;
            targetRpm = clamp(desiredRpm, lo, hi);
        }

        // ---- 3. 讀轉速 ---------------------------------------------------
        double actualTps = readVelocityTps(nowMs);
        double actualRpm = tpsToRpm(actualTps);

        // ---- 4. 算出要給多少 power ---------------------------------------
        double output;
        if (USE_SDK_VELOCITY) {
            if (!sdkModeApplied) setDriveMode(true);
            // 參數有變才寫進 hub，避免每圈都送 I2C 指令
            if (SDK_P != lastSdkP || SDK_I != lastSdkI || SDK_D != lastSdkD || SDK_F != lastSdkF) {
                motor.setVelocityPIDFCoefficients(SDK_P, SDK_I, SDK_D, SDK_F);
                lastSdkP = SDK_P;
                lastSdkI = SDK_I;
                lastSdkD = SDK_D;
                lastSdkF = SDK_F;
            }
            if (targetRpm <= 1e-3) {
                motor.setPower(0);
            } else {
                motor.setVelocity(rpmToTps(targetRpm));
            }
            // SDK 模式下看不到實際輸出，這條線只是前饋估計值，方便對照
            output = feedForward(targetRpm);
            pidf.reset();
        } else {
            if (sdkModeApplied) setDriveMode(false);
            if (targetRpm <= 1e-3) {
                pidf.reset();
                motor.setPower(0);
                output = 0;
            } else {
                output = pidf.update(targetRpm, actualRpm, measuredRateRpmPerSec, dt, voltage);
                motor.setPower(output);
            }
        }

        // ---- 5. 到位判斷 + 恢復時間量測 ----------------------------------
        double error = targetRpm - actualRpm;
        updateReadyAndRecovery(targetRpm, actualRpm, error);

        // ---- 6. 儀表板 ---------------------------------------------------
        // Panels 的 Graph 會自動抓「名字: 數字」的格式畫成折線圖
        panels.addData("TargetRPM", targetRpm);
        panels.addData("ActualRPM", actualRpm);
        panels.addData("ErrorRPM", error);
        panels.addData("Power", output);
        panels.addData("Saturated", output >= MAX_DUTY - 1e-3 ? 1 : 0);
        panels.addData("Voltage", voltage);
        panels.addData("Speed", clamp(SPEED, 0.0, 1.0));
        panels.addData("Mode", TARGET_RPM > 0 ? "TARGET_RPM" : "SPEED");
        panels.addData("LoopMs", dt * 1000.0);
        panels.addData("Status", statusText(targetRpm));
        panels.addData("Tuning", tuningStatus);
        panels.addData("Recovering", inDip ? 1 : 0);
        panels.addData("RecoveryMs", lastRecoveryMs);
        panels.addData("WorstMs", worstRecoveryMs);
        panels.addData("DipDepthRPM", lastDipDepthRpm);
        panels.addData("Shots", shots);
        panels.update(ftcTelemetry);

        // ---- 8. 手把 / 程式改過的變數推回網頁，讓兩邊顯示一致 ----------
        syncPanelsIfDirty();
    }

    /**
     * 面板 → 程式：程式每圈都直接讀 static 變數，網頁改了下一圈就生效。
     * 程式 → 面板：網頁不會自己重新讀值，所以手把 / 歸零改了變數之後，
     * 要主動叫 Panels 和 Dashboard 重新讀一次這個 class 的變數並推給網頁。
     * 有節流 (PANEL_SYNC_MIN_MS)，按住 D-Pad 時也不會每圈都推。
     */
    private void syncPanelsIfDirty() {
        if (!panelDirty || panelSyncTimer.milliseconds() < PANEL_SYNC_MIN_MS) return;
        syncPanelsNow();
    }

    /** 立刻把目前的變數值推給 Panels / Dashboard 網頁 (不受節流限制) */
    private void syncPanelsNow() {
        panelDirty = false;
        panelSyncTimer.reset();
        try {
            PanelsConfigurables.refreshClass(launching_PID.class);
        } catch (Exception ignored) {
            // Panels 被關掉或尚未載入時略過，不影響控制
        }
        try {
            FtcDashboard.getInstance().updateConfig();
        } catch (Exception ignored) {
            // 同上
        }
    }

    @Override
    public void stop() {
        motor.setPower(0);
        targetRpm = 0;
        // 停止時自動存檔，下次 INIT 會讀回來
        saveTuning();
    }

    // =========================================================================
    //  參數存檔 / 讀檔
    // =========================================================================

    private void loadTuning() {
        if (!TUNING_FILE.exists()) {
            tuningStatus = "沒有存檔，使用程式預設值";
            return;
        }
        Properties p = new Properties();
        try (FileInputStream in = new FileInputStream(TUNING_FILE)) {
            p.load(in);
            kP = readDouble(p, "kP", kP);
            kI = readDouble(p, "kI", kI);
            kD = readDouble(p, "kD", kD);
            kF = readDouble(p, "kF", kF);
            kS = readDouble(p, "kS", kS);
            MAX_REVERSE = readDouble(p, "MAX_REVERSE", MAX_REVERSE);
            tuningStatus = "已讀取上次存的參數";
        } catch (Exception e) {
            tuningStatus = "讀取存檔失敗，使用程式預設值: " + e.getMessage();
        }
    }

    private void saveTuning() {
        Properties p = new Properties();
        p.setProperty("kP", Double.toString(kP));
        p.setProperty("kI", Double.toString(kI));
        p.setProperty("kD", Double.toString(kD));
        p.setProperty("kF", Double.toString(kF));
        p.setProperty("kS", Double.toString(kS));
        p.setProperty("MAX_REVERSE", Double.toString(MAX_REVERSE));
        try (FileOutputStream out = new FileOutputStream(TUNING_FILE)) {
            p.store(out, "launching_PID tuned values");
            tuningStatus = "已存檔";
        } catch (Exception e) {
            tuningStatus = "存檔失敗: " + e.getMessage();
        }
    }

    private static double readDouble(Properties p, String key, double fallback) {
        String v = p.getProperty(key);
        if (v == null) return fallback;
        try {
            double d = Double.parseDouble(v.trim());
            return Double.isNaN(d) || Double.isInfinite(d) ? fallback : d;
        } catch (NumberFormatException e) {
            return fallback;
        }
    }

    // =========================================================================
    //  手把
    // =========================================================================

    /**
     * 手把 / 面板 → 目前希望的目標轉速 (RPM)。
     *   TARGET_RPM > 0 時直接用它 (面板輸入 RPM)，否則用 SPEED * MAX_RPM。
     *   D-Pad 上/下 ... TARGET_RPM 模式加減 100 RPM，否則加減 SPEED 5% (按住連續重複)
     *   RB ............ 緊急切斷：SPEED、TARGET_RPM 歸零，飛輪立刻斷電
     *   START ......... 歸零恢復時間統計
     * SPEED / TARGET_RPM 本身是面板變數，所以手把按了面板數字也會跟著動。
     */
    private double readGamepadTargetRpm() {
        // (a) D-Pad 上/下：調整速度。剛按下時動一次，按住每 200ms 重複一次
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean firstPress = (up && !prevDpadUp) || (down && !prevDpadDown);
        if ((up || down) && (firstPress || dpadRepeatTimer.milliseconds() >= DPAD_REPEAT_MS)) {
            int dir = (up && !down) ? 1 : ((down && !up) ? -1 : 0);
            if (TARGET_RPM > 0) {
                TARGET_RPM = clamp(TARGET_RPM + dir * RPM_STEP, 0.0, MAX_RPM);
                // 減到 0 就整個停下來，不要掉回 SPEED 模式又用舊的 SPEED 轉起來
                if (TARGET_RPM <= 0) SPEED = 0.0;
            } else {
                SPEED = clamp(SPEED + dir * SPEED_STEP, 0.0, 1.0);
            }
            if (dir != 0) panelDirty = true;
            dpadRepeatTimer.reset();
        }
        prevDpadUp = up;
        prevDpadDown = down;

        // (b) RB 緊急切斷：壓住期間強制歸零 → 同一圈 targetRpm 歸零、power = 0。
        //     這是鎖住的 (latched)：放開 RB 不會復電，要按 D-Pad 上才會重新供電。
        if (gamepad1.right_bumper) {
            if (!prevCut) {
                gamepad1.rumble(400);
                panelDirty = true;
            }
            SPEED = 0.0;
            TARGET_RPM = 0.0;
        }
        prevCut = gamepad1.right_bumper;

        // (c) START 歸零恢復時間統計，方便比較調參前後
        if (gamepad1.start && !prevStart) {
            lastRecoveryMs = 0;
            worstRecoveryMs = 0;
            lastDipDepthRpm = 0;
            shots = 0;
        }
        prevStart = gamepad1.start;

        // (d) BACK 立刻存檔目前的 kP/kI/kD/kF/kS/MAX_REVERSE (STOP 時也會自動存)
        if (gamepad1.back && !prevBack) {
            saveTuning();
            gamepad1.rumble(200);
        }
        prevBack = gamepad1.back;

        if (TARGET_RPM > 0) return clamp(TARGET_RPM, 0.0, MAX_RPM);
        return clamp(SPEED, 0.0, 1.0) * MAX_RPM;
    }

    // =========================================================================
    //  轉速量測
    // =========================================================================

    /**
     * 回傳 ticks/sec。
     * USE_TICK_DELTA_VELOCITY = true 時用編碼器位置差分自己算，視窗 VEL_WINDOW_MS；
     * 否則直接用 DcMotorEx.getVelocity() (hub 內建，延遲較大)。
     */
    private double readVelocityTps(double nowMs) {
        // 切換量測方式時重新初始化差分視窗
        if (velTickDeltaMode != USE_TICK_DELTA_VELOCITY) {
            velTickDeltaMode = USE_TICK_DELTA_VELOCITY;
            velInit = false;
        }

        if (!USE_TICK_DELTA_VELOCITY) {
            smoothTps = motor.getVelocity();
            rawTps = smoothTps;
            updateMeasuredRate(nowMs);
            return smoothTps;
        }

        int pos = motor.getCurrentPosition();
        if (!velInit) {
            velInit = true;
            velWindowStart = nowMs;
            velWindowStartPos = pos;
            rawTps = 0;
            smoothTps = 0;
            hasPrevSample = false;
            measuredRateRpmPerSec = 0.0;
            return 0;
        }

        double elapsedMs = nowMs - velWindowStart;
        if (elapsedMs >= VEL_WINDOW_MS) {
            double dTicks = pos - velWindowStartPos;
            rawTps = dTicks / (elapsedMs / 1000.0);
            velWindowStart = nowMs;
            velWindowStartPos = pos;
            // 低通濾波：讓回饋訊號平滑一點，避免 kD 把雜訊放大
            smoothTps = smoothTps + clamp(VEL_SMOOTH, 0.05, 1.0) * (rawTps - smoothTps);
            // 只有轉速真的更新時才重算變化率；兩次更新之間沿用上一次的值。
            // (舊版每圈都對「沒更新的轉速」微分，大部分時間是 0、更新那圈突然暴衝)
            updateMeasuredRate(nowMs);
        }
        return smoothTps;
    }

    /** 用「這次取樣 - 上次取樣」除以實際取樣間隔，算出轉速變化率 [RPM/秒] */
    private void updateMeasuredRate(double nowMs) {
        double rpm = tpsToRpm(smoothTps);
        if (hasPrevSample && nowMs - prevSampleMs > 1e-3) {
            measuredRateRpmPerSec = (rpm - prevSampleRpm) / ((nowMs - prevSampleMs) / 1000.0);
        } else {
            measuredRateRpmPerSec = 0.0;
        }
        prevSampleRpm = rpm;
        prevSampleMs = nowMs;
        hasPrevSample = true;
    }

    private double tpsToRpm(double tps) {
        return tps / TICKS_PER_REV * 60.0;
    }

    private double rpmToTps(double rpm) {
        return rpm / 60.0 * TICKS_PER_REV;
    }

    private double readVoltage() {
        double v = voltageSensor == null ? NOMINAL_VOLTAGE : voltageSensor.getVoltage();
        return v > 1.0 ? v : NOMINAL_VOLTAGE;
    }

    private void setDriveMode(boolean sdkVelocity) {
        motor.setPower(0);
        motor.setMode(sdkVelocity ? DcMotor.RunMode.RUN_USING_ENCODER
                                  : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sdkModeApplied = sdkVelocity;
        pidf.reset();
        velInit = false;
    }

    // =========================================================================
    //  到位判斷 / 恢復時間量測
    // =========================================================================

    private void updateReadyAndRecovery(double target, double actual, double error) {
        if (target <= 1e-3) {
            settled = false;
            armed = false;
            inDip = false;
            readyTimer.reset();
            return;
        }

        boolean inTol = Math.abs(error) <= READY_TOL_RPM;

        // (a) 偵測「球打上飛輪」：已經穩定過 (armed)，現在轉速掉超過門檻
        if (!inDip && armed && error > DIP_THRESHOLD_RPM) {
            inDip = true;
            dipTimer.reset();
            dipMinRpm = actual;
            settled = false;
            armed = false;
            readyTimer.reset();
        }

        // (b) 正在恢復：記錄掉到多低，回到容許範圍就停止計時
        if (inDip) {
            dipMinRpm = Math.min(dipMinRpm, actual);
            if (inTol) {
                inDip = false;
                lastRecoveryMs = dipTimer.milliseconds();
                lastDipDepthRpm = target - dipMinRpm;
                if (lastRecoveryMs > worstRecoveryMs) worstRecoveryMs = lastRecoveryMs;
                shots++;
            }
        }

        // (c) 到位：必須連續維持 READY_HOLD_MS 才算真的穩
        if (!inDip && inTol) {
            if (readyTimer.milliseconds() >= READY_HOLD_MS) {
                if (!settled) {
                    settled = true;
                    gamepad1.rumble(200);        // 剛到位震一下
                }
                armed = true;
            }
        } else {
            settled = false;
            readyTimer.reset();
        }
    }

    private String statusText(double target) {
        if (target <= 1e-3) return "stopped";
        if (inDip) return ">>> RECOVERING <<<";
        if (settled) return ">>> READY <<<";
        return "spinning up...";
    }

    // =========================================================================
    //  自寫 PIDF 控制器
    // =========================================================================

    /** 前饋：維持目標轉速「大概」需要的 power */
    private static double feedForward(double targetRpm) {
        if (targetRpm <= 1e-3) return 0.0;
        return kF * (targetRpm / MAX_RPM) + kS;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    /**
     * 前饋 + PID。
     *  - 微分作用在量測值上 (derivative on measurement)：目標跳變時不會有衝擊。
     *    變化率由 readVelocityTps() 依實際取樣間隔算好傳進來
     *  - 抗積分飽和：只在 |error| < I_BAND_RPM 時累積，且限制積分貢獻大小
     *  - 電壓補償：out *= NOMINAL_VOLTAGE / 實際電壓
     *  - 選用的輸出斜率限制 (MAX_DUTY_RATE)
     *  - 降速時允許反向出力 (由 PID 決定)，但最多 MAX_REVERSE，避免電流與衝擊太大
     */
    private static class Pidf {
        private double integral = 0.0;
        private double lastOutput = 0.0;

        void reset() {
            integral = 0.0;
            lastOutput = 0.0;
        }

        double update(double targetRpm, double measuredRpm, double measuredRateRpmPerSec,
                      double dt, double voltage) {
            if (targetRpm <= 1e-3) {
                reset();
                return 0.0;
            }

            double error = targetRpm - measuredRpm;

            // --- 前饋 ---
            double out = feedForward(targetRpm);

            // --- 積分 (抗飽和) ---
            if (Math.abs(error) < I_BAND_RPM) {
                integral += error * dt;
            }
            double iLimit = I_MAX / Math.max(kI, 1e-9);
            integral = clamp(integral, -iLimit, iLimit);

            // --- PID (微分對量測值) ---
            out += kP * error + kI * integral - kD * measuredRateRpmPerSec;

            // --- 電壓補償 ---
            if (VOLTAGE_COMP && voltage > 1.0) {
                out *= NOMINAL_VOLTAGE / voltage;
            }

            // --- 輸出斜率限制 (0 = 不限制) ---
            if (MAX_DUTY_RATE > 1e-6 && dt > 1e-6) {
                double maxStep = MAX_DUTY_RATE * dt;
                out = clamp(out, lastOutput - maxStep, lastOutput + maxStep);
            }
            out = clamp(out, -clamp(MAX_REVERSE, 0.0, MAX_DUTY), MAX_DUTY);

            lastOutput = out;
            return out;
        }
    }
}
