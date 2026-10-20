package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * ============================================================================
 *  Pedro Pathing 的機器人設定 (drivetrain + localizer + Foresight 演算法)
 * ============================================================================
 *
 *  這支檔案只做一件事：把「你的硬體」告訴 Pedro，最後回傳一個 Follower。
 *
 *  ★★ 上場前一定要做的兩件事 (否則位置保持不會準)：
 *      (1) AutoTune  : 打開 http://192.168.43.1:10158 → 跑 Mecanum Tuner /
 *                      Two Wheel Tuner / Foresight Tuner，把結果貼回本檔。
 *      (2) 檢查下面標了 TODO 的 pod 名稱與 offsets。
 *
 *  座標系 (Pedro)：前方 = +X，左方 = +Y，逆時針為正角 (弧度)。
 * ============================================================================
 */
public class Constants {

    // =========================================================================
    //  1. 底盤 (mecanum) —— 馬達名稱要跟 hardware map 一致
    // =========================================================================
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.backLeftName.set("backLeft");
        c.frontRightName.set("frontRight");
        c.backRightName.set("backRight");

        // 與 driveChain.java 的方向設定保持一致
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);

        // 手把回中時煞車 (off = 溜車)，定點保持時比較不會被推著滑
        c.manualBrakeMode.set(true);
    });

    // =========================================================================
    //  2. 定位 (兩顆 odometry pod + Control Hub 內建 IMU)
    // =========================================================================
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        // TODO ★ 這兩個是「odometry pod 編碼器插在哪個馬達 port」的 hardware map 名稱
        //          例：前方 pod 插在 frontLeft 的 encoder port → "frontLeft"
        c.xPodName.set("frontLeft");   // 前後 (FORWARD) pod
        c.yPodName.set("backRight");   // 左右 (STRAFE)  pod

        c.imuName.set("imu");          // Control Hub IMU 通常就叫 "imu"
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,      // TODO 依 Control Hub 實際朝向
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD))); // TODO

        // TODO ★ 以機器人旋轉中心為原點，pod 的位置 (英寸)。左為 +，前為 +
        //          跑 Two Wheel Tuner 會自動算出來，先用手量的值也行
        c.xPodOffset.set(0.0);         // 前後 pod 的「前後」偏移 (y 方向) ← TODO
        c.yPodOffset.set(0.0);         // 左右 pod 的「左右」偏移 (x 方向) ← TODO

        // TODO ★ 每個 count 等於多少英寸 (跑 Two Wheel Tuner 自動產生)
        //          goBILDA 5mm odometry pod ≈ 2048 ticks/rev，直徑 24mm → 0.0370 in/tick
        c.forwardTicksToInches.set(0.0370);
        c.strafeTicksToInches.set(0.0370);

        // TODO 跑 Two Wheel Tuner 的 Direction 驗證後貼過來 (FORWARD = 1 / REVERSE = -1)
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.FORWARD);
    });

    // =========================================================================
    //  3. 定點保持用的 PID (面板上只調這三顆，見 HoldPosition.java)
    //     ★ 一定要宣告在 foresightConfig 之前，靜態初始化才會先建好
    // =========================================================================
    /** 前後 / 左右 平移 PID (誤差單位：英寸) */
    public static final HoldPid forwardPid = new HoldPid(0.30, 0.0, 0.0);
    public static final HoldPid strafePid = new HoldPid(0.30, 0.0, 0.0);
    /** 朝向 P (誤差單位：弧度)。弧度值很小，所以 kP 要比英寸大一個數量級 */
    public static final HoldPid headingPid = new HoldPid(5.0, 0.0, 0.0);

    /** 每次 arm (鎖定新座標) 時把三顆 PID 的積分清零 */
    public static void resetHoldPids() {
        forwardPid.reset();
        strafePid.reset();
        headingPid.reset();
    }

    // =========================================================================
    //  4. Foresight 演算法
    //     下面 brake/coast/速度等數值是「官方文件範例機器人」的值，僅供起手。
    //     跑完 Foresight Tuner 後請整段換成 AutoTune 產生的結果。
    // =========================================================================
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // --- 回授 (你要調的 PID 就接在這裡) ---
        c.forwardTranslational.set(forwardPid);
        c.strafeTranslational.set(strafePid);
        c.headingFeedback.set(headingPid);
        c.headingStaticFF.set(Controller.staticFeedforward(0.0));

        // --- 速度前饋 / 煞車 (跑 Foresight Tuner 自動產生) ---
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));

        c.maxAchievableForwardVelocity.set(72.72923108818539);   // in/s  TODO AutoTune
        c.maxAchievableStrafeVelocity.set(52.34323936525474);    // in/s  TODO AutoTune
        c.naturalForwardDeceleration.set(85.01144677379789);     // in/s² TODO AutoTune
        c.naturalStrafeDeceleration.set(104.49787535782846);     // in/s² TODO AutoTune

        // --- 定點保持時的輸出縮放 (hold(pose, true) 才會套用) ---
        c.holdPointTranslationalScaling.set(0.45);   // 平移回復力打折，避免振盪
        c.holdPointHeadingScaling.set(0.35);         // 朝向回復力打折
    });

    // =========================================================================
    //  5. 建立 Follower —— opmode 裡一行搞定: follower = Constants.create(hardwareMap);
    // =========================================================================
    public static Follower create(HardwareMap h) {
        return new Follower(
                new TwoWheelLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
