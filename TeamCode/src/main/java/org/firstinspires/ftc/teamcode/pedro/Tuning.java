package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;
import org.firstinspires.ftc.teamcode.pedro.procedures.TwoWheelTuner;

/**
 * ============================================================================
 *  Pedro AutoTune —— 打開 http://192.168.43.1:10158 就會看到下面這些流程
 * ============================================================================
 *
 *  建議照這個順序跑 (每跑完一個，把產生的程式碼貼回 Constants.java)：
 *
 *    1. Mecanum Tuner   確認四顆馬達的名字與方向 (driveChain 的設定可當起手)
 *    2. Two Wheel Tuner 自動量出 pod 的 ticks-to-inches、方向、offsets
 *                       ★ 一定要跑，否則定點保持的「回到原點」不會準
 *    3. Foresight Tuner 自動量出 max velocity / deceleration / 煞車係數
 *    4. Tests           Hold Test 可以直接測「定點保持」，Localization Test
 *                       檢查推動機器人時座標是不是正確 (前進 +X、左移 +Y)
 *
 *  規則：@Tuner 方法必須是「public static、零參數、回傳 Procedure」。
 * ============================================================================
 */
public class Tuning {

    /** 底盤方向 / 馬達名稱確認 */
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    /** 兩顆 odometry pod 的解析度、方向、offsets (最重要的一個) */
    @Tuner
    public static Procedure twoWheelTuner() {
        return new TwoWheelTuner();
    }

    /** Foresight 演算法 (速度前饋 / 煞車 / 最大速度) */
    @Tuner
    public static Procedure foresightTuner() {
        return new ForesightTuner(
                hardwareMap -> new TwoWheelLocalizer(hardwareMap, Constants.localizerConfig),
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig));
    }

    /** 各項測試：Hold Test / Localization Test / Line Test ... */
    @Tuner
    public static Procedure tests() {
        return new Tests(
                hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig),
                hardwareMap -> new TwoWheelLocalizer(hardwareMap, Constants.localizerConfig),
                () -> new Foresight(Constants.foresightConfig));
    }
}
