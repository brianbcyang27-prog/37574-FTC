package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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

/**
 * ============================================================================
 *  飛輪發射器 (Shooter / Flywheel) 速度控制  --  自寫 PIDF 版本
 * ============================================================================
 *
 *  設計目標：球打上飛輪後轉速被拉下來，要「用最短時間」把轉速拉回目標值。
 *  為了縮短恢復時間，這支程式做了幾件關鍵的事：
 *
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
 *  調參順序建議：
 *    (1) 先只調 kF：把 kI、kD 設 0，看 TargetRPM 與 ActualRPM 是否幾乎重疊在
 *        全速附近。差多少就把 kF 往上/往下修 (kF = 1.0 代表前饋剛好等於比例 power)。
 *    (2) 再加 kP：從 0.001 開始往上加，直到「手指撥一下飛輪」能很快回來、且
 *        不來回震盪。kP 是恢復速度的主要來源。
 *    (3) 最後加一點 kI 消掉穩態誤差，但保持 kI 很小 (0.001~0.005)。太大會在
 *        球打上去之後超調。
 *    (4) kD 通常保持 0。要加的話從 0.0002 開始「非常小」，因為 hub 的轉速
 *        量測有雜訊，kD 放太大會讓馬達發抖。
 *    (5) 每次調完，用 Start 鍵歸零 RecoveryMs 統計，重新打幾顆球比較數字。
 *
 *  手把 (Logitech F310，切到 XInput / X 模式)：
 *    A/X/Y/B ........ 設定基準速度 100% / 75% / 50% / 25%
 *    左or右搖桿 Y ... 連續微調 (往上推加速、往下推減速，放手保持)
 *    D-Pad 上/下 .... 一段 +/-5%      D-Pad 左/右 ... 微調 +/-1%
 *    LB ............. 回到基準速度 (取消微調)
 *    RB ............. 立刻停止 (最高優先、並歸零)
 *    RT ............. 壓住時直接照板機深度給速度 (暫時全速)
 *    START .......... 歸零 RecoveryMs / DipDepth / Shots 統計
 *    到位時手把震一下
 * ============================================================================
 */
@Config        // FTC Dashboard :8080  (備用)
@Configurable  // Panels        :8001  (主要調參介面)
@TeleOp(name = "Launching PID", group = "Shooter")
public class launching_PID extends OpMode {

    // =========================================================================
    //  1. 硬體 / 目標轉速
    // =========================================================================
    /** 硬體設定裡那顆飛輪馬達的名字 */
    public static String MOTOR_NAME = "motor";
    /** goBILDA 6000rpm 裸馬達：馬達軸一圈 28 個 encoder count */
    public static double TICKS_PER_REV = 28.0;
    /** 飛輪定義上的最高轉速 (目標比例的 100%)，先保守抓 6000 的 90% */
    public static double MAX_RPM = 5400.0;

    // =========================================================================
    //  2. 手把速度設定
    // =========================================================================
    public static double PRESET_FULL = 1.00;   // A
    public static double PRESET_HIGH = 0.75;   // X
    public static double PRESET_MID  = 0.50;   // Y
    public static double PRESET_LOW  = 0.25;   // B
    /** 搖桿推到底時，每秒改變的速度比例 (0.6 = 約 1.6 秒從 0 推到全速) */
    public static double STICK_RATE = 0.60;
    /** D-Pad 上/下 每按一次調整的比例 */
    public static double DPAD_COARSE = 0.05;
    /** D-Pad 左/右 每按一次調整的比例 */
    public static double DPAD_FINE = 0.01;

    // =========================================================================
    //  3. 自寫 PIDF 參數
    // =========================================================================
    /** true = 改回 SDK 內建速度環 (舊做法)，用來跟自己寫的版本 A/B 比較 */
    public static boolean USE_SDK_VELOCITY = false;

    /** 前饋：power = kF * (目標轉速 / MAX_RPM)。kF = 1.0 表示前饋就等於比例 power */
    public static double kF = 1.00;
    /** 靜摩擦補償：只要有目標轉速就先墊一點 power，讓飛輪比較容易起步 */
    public static double kS = 0.03;
    /** 比例項 [power / RPM]  ← 恢復速度主要靠這個 */
    public static double kP = 0.0015;
    /** 積分項 [power / (RPM*秒)] ← 只負責修掉穩態誤差，保持小 */
    public static double kI = 0.0020;
    /** 微分項 [power / (RPM/秒)] ← 量測有雜訊，通常保持 0 */
    public static double kD = 0.0;
    /** 只有誤差在 ±這個值以內才累積積分 (防大擾動時 windup) */
    public static double I_BAND_RPM = 250.0;
    /** 積分項的最大貢獻 (power) */
    public static double I_MAX = 0.25;
    /** 輸出上限 (power) */
    public static double MAX_DUTY = 1.00;
    /** 輸出斜率限制 [power/秒]，0 = 不限制 (恢復最快，但起步電流較大) */
    public static double MAX_DUTY_RATE = 0.0;

    /** SDK 內建速度環用的 PIDF (只有 USE_SDK_VELOCITY = true 時才生效) */
    public static double SDK_P = 10.0;
    public static double SDK_I = 0.0;
    public static double SDK_D = 0.0;
    public static double SDK_F = 11.7;

    // =========================================================================
    //  4. 電壓補償
    // =========================================================================
    public static boolean VOLTAGE_COMP = true;
    /** 電壓補償的基準電壓 (V)，通常用充滿電時的電壓 */
    public static double NOMINAL_VOLTAGE = 12.5;

    // =========================================================================
    //  5. 轉速量測
    // =========================================================================
    /** true = 用編碼器位置差分自算轉速 (延遲比 hub 內建小)，false = 用 getVelocity() */
    public static boolean USE_TICK_DELTA_VELOCITY = true;
    /** 差分視窗 (ms)。越大越平滑但越慢；6000rpm 下 50ms 約 126 個 count，雜訊約 ±1% */
    public static double VEL_WINDOW_MS = 50.0;
    /** 低通濾波強度 (0~1，1 = 不濾波)。1 反應最快但最抖 */
    public static double VEL_SMOOTH = 0.65;

    // =========================================================================
    //  6. 到位判斷 / 恢復時間量測
    // =========================================================================
    /** 誤差在 ±這個 RPM 內算到位 */
    public static double READY_TOL_RPM = 100.0;
    /** 要連續維持這麼久才算真的穩 (避免只是路過) */
    public static double READY_HOLD_MS = 150.0;
    /** 轉速掉超過這個值就開始計時恢復 */
    public static double DIP_THRESHOLD_RPM = 120.0;
    /** 目標轉速上升斜率 [RPM/秒]，避免起步瞬間拉爆電流 */
    public static double TARGET_SLEW_UP = 9000.0;
    /** 目標轉速下降斜率 [RPM/秒] */
    public static double TARGET_SLEW_DOWN = 9000.0;

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
    private boolean sdkModeApplied = false;  // 目前馬達跑在哪個模式
    private double lastSdkP = Double.NaN, lastSdkI = Double.NaN, lastSdkD = Double.NaN, lastSdkF = Double.NaN;

    // 手把：微調與邊緣偵測
    private double presetRatio = 0.0;
    private double trim = 0.0;
    private boolean prevDpadUp, prevDpadDown, prevDpadLeft, prevDpadRight, prevStart;

    // 轉速估算
    private boolean velInit = false;
    private boolean velTickDeltaMode = false;
    private double velWindowStart = 0.0;
    private int velWindowStartPos = 0;
    private double rawTps = 0.0;
    private double smoothTps = 0.0;

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

        loopTimer.reset();
        clock.reset();
        panels.addData("Status", "init done");
        panels.update(ftcTelemetry);
    }

    @Override
    public void loop() {
        double nowMs = clock.milliseconds();
        double dt = Math.max(loopTimer.seconds(), 1e-4);
        loopTimer.reset();

        double voltage = readVoltage();

        // ---- 1. 手把：決定希望的速度比例 desiredRatio --------------------
        double desiredRatio = readGamepadRatio(dt);

        // ---- 2. 目標轉速 + 斜率限制 (保護電流，也讓量測可重現) -----------
        double desiredRpm = desiredRatio * MAX_RPM;
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
                output = pidf.update(targetRpm, actualRpm, dt, voltage);
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
        panels.addData("Voltage", voltage);
        panels.addData("Ratio", desiredRatio);
        panels.addData("Trim", trim);
        panels.addData("Status", statusText(targetRpm));
        panels.addData("Recovering", inDip ? 1 : 0);
        panels.addData("RecoveryMs", lastRecoveryMs);
        panels.addData("WorstMs", worstRecoveryMs);
        panels.addData("DipDepthRPM", lastDipDepthRpm);
        panels.addData("Shots", shots);
        panels.update(ftcTelemetry);
    }

    @Override
    public void stop() {
        motor.setPower(0);
        targetRpm = 0;
    }

    // =========================================================================
    //  手把
    // =========================================================================

    /**
     * 把 F310 的輸入換算成 0~1 的速度比例。
     * 邏輯：presetRatio (按鈕設定的基準) + trim (搖桿/D-Pad 微調)，RT 直接覆蓋，RB 強制歸零。
     */
    private double readGamepadRatio(double dt) {
        // (a) A/X/Y/B 設定基準速度，同時把微調歸零
        if (gamepad1.a) {
            presetRatio = PRESET_FULL;
            trim = 0;
        } else if (gamepad1.x) {
            presetRatio = PRESET_HIGH;
            trim = 0;
        } else if (gamepad1.y) {
            presetRatio = PRESET_MID;
            trim = 0;
        } else if (gamepad1.b) {
            presetRatio = PRESET_LOW;
            trim = 0;
        }

        // (b) 兩支搖桿都可以當「油門微調」：往上推加速、往下推減速、放手保持。
        //     FTC 的 stick_y 往上推是負值，所以要取負號。
        double stick = Math.abs(gamepad1.left_stick_y) > Math.abs(gamepad1.right_stick_y)
                ? gamepad1.left_stick_y
                : gamepad1.right_stick_y;
        if (Math.abs(stick) > 0.08) {
            trim -= stick * STICK_RATE * dt;
        }

        // (c) D-Pad：上/下一段 5%，左/右微調 1% (只在「剛按下」時觸發一次)
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        if (up && !prevDpadUp) trim += DPAD_COARSE;
        if (down && !prevDpadDown) trim -= DPAD_COARSE;
        if (right && !prevDpadRight) trim += DPAD_FINE;
        if (left && !prevDpadLeft) trim -= DPAD_FINE;
        prevDpadUp = up;
        prevDpadDown = down;
        prevDpadLeft = left;
        prevDpadRight = right;

        // (d) LB 回到基準速度 (取消微調)
        if (gamepad1.left_bumper) {
            trim = 0;
        }

        // (e) 組合
        double ratio = clamp(presetRatio + trim, 0.0, 1.0);

        // (f) RB 緊急停止 / RT 直接照板機深度 (這兩個優先權最高)
        if (gamepad1.right_bumper) {
            presetRatio = 0;
            trim = 0;
            ratio = 0;
        } else if (gamepad1.right_trigger > 0.05) {
            ratio = clamp(gamepad1.right_trigger, 0.0, 1.0);
        }

        // (g) START 歸零恢復時間統計，方便比較調參前後
        if (gamepad1.start && !prevStart) {
            lastRecoveryMs = 0;
            worstRecoveryMs = 0;
            lastDipDepthRpm = 0;
            shots = 0;
        }
        prevStart = gamepad1.start;

        return ratio;
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
            return smoothTps;
        }

        int pos = motor.getCurrentPosition();
        if (!velInit) {
            velInit = true;
            velWindowStart = nowMs;
            velWindowStartPos = pos;
            rawTps = 0;
            smoothTps = 0;
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
        }
        return smoothTps;
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
     *  - 微分作用在量測值上 (derivative on measurement)：目標跳變時不會有衝擊
     *  - 抗積分飽和：只在 |error| < I_BAND_RPM 時累積，且限制積分貢獻大小
     *  - 電壓補償：out *= NOMINAL_VOLTAGE / 實際電壓
     *  - 選用的輸出斜率限制 (MAX_DUTY_RATE)
     */
    private static class Pidf {
        private double integral = 0.0;
        private double lastMeasured = 0.0;
        private double lastOutput = 0.0;
        private boolean hasLast = false;

        void reset() {
            integral = 0.0;
            lastOutput = 0.0;
            hasLast = false;
        }

        double update(double targetRpm, double measuredRpm, double dt, double voltage) {
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

            // --- 微分：對量測值微分 ---
            double dMeasured = 0.0;
            if (hasLast && dt > 1e-6) {
                dMeasured = (measuredRpm - lastMeasured) / dt;
            }
            lastMeasured = measuredRpm;
            hasLast = true;

            // --- PID ---
            out += kP * error + kI * integral - kD * dMeasured;

            // --- 電壓補償 ---
            if (VOLTAGE_COMP && voltage > 1.0) {
                out *= NOMINAL_VOLTAGE / voltage;
            }

            // --- 輸出斜率限制 (0 = 不限制) ---
            if (MAX_DUTY_RATE > 1e-6 && dt > 1e-6) {
                double maxStep = MAX_DUTY_RATE * dt;
                out = clamp(out, lastOutput - maxStep, lastOutput + maxStep);
            }
            out = clamp(out, -MAX_DUTY, MAX_DUTY);

            lastOutput = out;
            return out;
        }
    }
}
