package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Run Forever", group = "Test")
public class ServoRunForever extends OpMode {

    // ==================== Adjustable Parameters ====================

    /**
     * Continuous-rotation servo power while running (0.0 to 1.0 magnitude).
     * Direction is handled separately by the B button.
     * 0.3 here is equivalent to the old 0.65 position value.
     *
     * Start slowly and test with the mechanism clear.
     */
    public double runPower = 0.3;

    /**
     * Power used when stopped. Should be 0.0.
     * Nudge slightly (e.g. +/-0.02) only if the servo creeps while stopped.
     */
    public double stopPower = 0.0;

    /** Name configured in the FTC Robot Configuration (as a Continuous Rotation Servo). */
    private static final String SERVO_NAME = "servo";

    // ==================== Runtime State ====================

    /** Set when the device is configured as a Continuous Rotation Servo. */
    private CRServo crServo;

    /** Set when the device is configured as a standard Servo (driven via position 0..1). */
    private Servo stdServo;

    /** Non-null when no usable servo was found; shown on telemetry instead of crashing. */
    private String hardwareError;

    /** Whether the servo should currently rotate. */
    private boolean running = false;

    /** +1 or -1; swaps the direction of rotation. */
    private int direction = 1;

    // Button edge detection
    private boolean prevA = false;
    private boolean prevB = false;
    private boolean prevX = false;
    private boolean prevStart = false;

    @Override
    public void init() {
        crServo = hardwareMap.tryGet(CRServo.class, SERVO_NAME);
        if (crServo == null) {
            stdServo = hardwareMap.tryGet(Servo.class, SERVO_NAME);
        }
        if (crServo == null && stdServo == null) {
            hardwareError = "No servo named '" + SERVO_NAME + "' in the active configuration";
        } else if (crServo != null) {
            crServo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        applyPower(clampPower(stopPower));

        if (hardwareError != null) {
            telemetry.addData("ERROR", hardwareError);
        }
        telemetry.addData("Status", "Initialized - press PLAY to begin");
        telemetry.addData("Servo Type", crServo != null ? "Continuous Rotation" : stdServo != null ? "Standard (position)" : "NONE");
        telemetry.addData("Servo", SERVO_NAME);
        telemetry.addData("Run Power", runPower);
        telemetry.addData("Stop Power", stopPower);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Allow direction to be selected before PLAY.
        boolean b = gamepad1.b;

        if (b && !prevB) {
            direction = -direction;
        }

        prevB = b;

        telemetry.addData("Status", "Ready");
        telemetry.addData("Direction", direction > 0 ? "Direction 1" : "Direction 2");
        telemetry.addData("Hint", "B = reverse direction");
        telemetry.update();
    }

    @Override
    public void start() {
        // Begin rotating when PLAY is pressed.
        running = true;
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean start = gamepad1.start;

        // A: pause or resume
        if (a && !prevA) {
            running = !running;
        }

        // B: reverse direction
        if (b && !prevB) {
            direction = -direction;
        }

        // X: stop rotation
        if (x && !prevX) {
            running = false;
        }

        // START: resume rotation
        if (start && !prevStart) {
            running = true;
        }

        prevA = a;
        prevB = b;
        prevX = x;
        prevStart = start;

        double power;
        if (running) {
            double magnitude = Math.max(0.0, Math.min(1.0, runPower));
            power = clampPower(direction * magnitude);
        } else {
            power = clampPower(stopPower);
        }
        applyPower(power);

        if (hardwareError != null) {
            telemetry.addData("ERROR", hardwareError);
        }
        telemetry.addData("Status", running ? "RUNNING" : "STOPPED");
        telemetry.addData("Direction", direction > 0 ? "Direction 1" : "Direction 2");
        telemetry.addData("Commanded Power", power);
        telemetry.addData("Run Power Setting", runPower);
        telemetry.addData("Stop Power Setting", stopPower);
        telemetry.addData("Controls", "A = pause/resume | B = reverse");
        telemetry.addData("Controls ", "X = stop | START = run");
        telemetry.update();
    }

    @Override
    public void stop() {
        applyPower(0.0);
    }

    /** Sends power to whichever servo type is configured; -1..1 maps to position 0..1 on a standard servo. */
    private void applyPower(double power) {
        if (crServo != null) {
            crServo.setPower(power);
        } else if (stdServo != null) {
            stdServo.setPosition(0.5 + power / 2.0);
        }
    }

    private static double clampPower(double p) {
        return Math.max(-1.0, Math.min(1.0, p));
    }
}