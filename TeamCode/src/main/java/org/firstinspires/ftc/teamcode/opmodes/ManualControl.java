package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.config.Config;
import org.firstinspires.ftc.teamcode.framework.OpObject;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** 唯一可在 Driver Station 選取的模式：只控制底盤。 */
@TeleOp(name = "FTC37574 Basic Drive", group = "FTC37574")
public final class ManualControl extends OpObject {
    private Drivetrain drivetrain;

    @Override
    public void Init() {
        drivetrain = new Drivetrain();
        RegisterBehavior(drivetrain);
    }

    @Override
    public void Start() {
        drivetrain.setDrive(0, 0, 0);
    }

    @Override
    public void Loop() {
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x);
        double turn = deadband(gamepad1.right_stick_x);
        drivetrain.setDrive(forward, strafe, turn);
    }

    @Override
    public void Stop() {
        drivetrain.setDrive(0, 0, 0);
    }

    private static double deadband(double value) {
        return Math.abs(value) < Config.STICK_DEADBAND ? 0 : value;
    }
}
