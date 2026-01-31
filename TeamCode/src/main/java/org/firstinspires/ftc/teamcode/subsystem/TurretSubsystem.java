package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class TurretSubsystem implements Subsystem {

    public static final TurretSubsystem INSTANCE = new TurretSubsystem();


    /* ---------------- Hardware ---------------- */

    // Motor reversed so +power = right, -power = left
    public final MotorEx turretMotor =
            new MotorEx("turret").reversed().brakeMode();

    /* ---------------- Modes ---------------- */
    public enum TurretMode {
        ANGLE,
        VISION,
        VISIONFF,
        IDLE
    }

    public TurretMode mode = TurretMode.IDLE;

    /* ---------------- Angle Control ---------------- */

    private final ControlSystem angleController;

    /* ---------------- Vision PID ---------------- */


    //constants 1/15/26 working decent
    public static double kP = 0.03;
    public static double kI = 0.001;
    public static double kD = 0.001;

    public double integralSum = 0.0;
    public double lastError = 0.0;
    public double lastTime = 0.0;

    public static double kP2 = 0.012;     // lower than before
    public static double kI2 = 0.0;      // start OFF
    public static double kD2 = 0.0;      // start OFF

    public static double kS2 = .02;     // tune this first
    public static double I_MAX2 = 0.3;

    private double ffIntegral = 0.0;
    private double ffLastError = 0.0;
    private double ffLastTime = 0.0;
    private DoubleSupplier txSupplier = () -> 0.0;

    /* ---------------- Constants ---------------- */

    private static final int TICKS_TOLERANCE = 10;
    private static final double TICKS_PER_REV = 145.1;
    private static final double GEAR_RATIO = 5.0;

    private static final double MAX_POWER = 0.5;
    private static final double MIN_POWER = 0.05;
    private static final double TX_TOLERANCE = 0.5;

    private double desiredPower = 0.0;

    /* ---------------- Constructor ---------------- */

    private TurretSubsystem() {
        angleController = ControlSystem.builder()
                .posPid(0.004, 0.0, 0.0)
                .basicFF(1)
                .build();



    }

    /* ---------------- Angle Utilities ---------------- */

    private double angleToTicks(double degrees) {
        return degrees / 360.0 * TICKS_PER_REV * GEAR_RATIO;
    }

    /* ---------------- Commands ---------------- */

    public Command runToAngle(double degrees) {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = TurretMode.ANGLE;
                    angleController.setGoal(
                            new KineticState(angleToTicks(degrees))
                    );
                })
                .setIsDone(() ->
                        Math.abs(
                                angleController.getGoal().getPosition()
                                        - turretMotor.getCurrentPosition()
                        ) < TICKS_TOLERANCE
                )
                .setStop(interrupted -> mode = TurretMode.IDLE)
                .requires(this)
                .named("TurretToAngle");
    }

    public boolean isAiming() {
        return mode == TurretMode.VISION;
    }

    public void stopTurret() {
        mode = TurretMode.IDLE;
        turretMotor.setPower(0.0);
    }

    public Command homeTurret() {
        return runToAngle(0).named("HomeTurret");
    }

    public Command aimWithVision(DoubleSupplier txSupplier) {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = TurretMode.VISION;
                    this.txSupplier = txSupplier;

                    integralSum = 0.0;
                    lastError = 0.0;
                    lastTime = System.nanoTime() / 1e9;
                })
                .setIsDone(() -> false)
                .setStop(interrupted -> mode = TurretMode.IDLE)
                .requires(this)
                .named("TurretVisionAim");
                }

        public Command aimWithVisionFF(DoubleSupplier txSupplier) {
            return new LambdaCommand()
                    .setStart(() -> {
                        mode = TurretMode.VISIONFF;
                    this.txSupplier = txSupplier;

                    ffIntegral = 0.0;
                    ffLastError = 0.0;
                    ffLastTime = System.nanoTime() / 1e9;
                })
                .setIsDone(() -> false)
                .setStop(interrupted -> mode = TurretMode.IDLE)
                .requires(this)
                .named("TurretVisionAimFF");
        }

    /* ---------------- Vision PID ---------------- */

    public double visionPID(double error) {

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.02;

        double P = kP * error;

        integralSum += error * dt;
        double I = kI * integralSum;

        double derivative = (error - lastError) / dt;
        double D = kD * derivative;

        lastError = error;
        lastTime = currentTime;

        double output = P + I + D;

        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        // Minimum power uses ERROR sign
        if (Math.abs(error) > TX_TOLERANCE &&
                Math.abs(output) < MIN_POWER) {
            output = Math.copySign(MIN_POWER, error);
        }

        // Deadband
        if (Math.abs(error) <= TX_TOLERANCE) {
            output = 0.0;
            integralSum = 0.0;
        }

        return output;
    }


    //this is a new vision controller for testing
    public double visionController(double error) {

        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - ffLastTime;
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        /* ---------- PID ---------- */

        // Proportional
        double P = kP2 * error;

        // Integral (clamped)
        ffIntegral += error * dt;
        ffIntegral = Math.max(-I_MAX2, Math.min(I_MAX2, ffIntegral));
        double I = kI2 * ffIntegral;


        // Derivative (optional — usually small or zero for vision)
        double derivative = (error - ffLastError) / dt;
        double D = kD2 * derivative;

        /* ---------- Feedforward ---------- */

        // Static friction compensation
        double ff = 0.0;
        if (Math.abs(error) > TX_TOLERANCE) {
            ff = kS2 * Math.signum(error);
        }

        /* ---------- Combine ---------- */

        double output = P + I + D + ff;

        output = Math.max(-MAX_POWER, Math.min(MAX_POWER, output));

        /* ---------- Deadband ---------- */

        if (Math.abs(error) <= TX_TOLERANCE) {
            output = 0.0;
            ffIntegral = 0.0;
        }

        ffLastError = error;
        ffLastTime = currentTime;

        return output;
    }

    /* ---------------- Periodic ---------------- */

    @Override
    public void periodic() {

        switch (mode) {

            case ANGLE:
                desiredPower = angleController.calculate(
                        turretMotor.getState()
                );
                break;

            case VISION:
                //latest update 1/14/26 1:50pm - made error + tx not negative tx
                double tx = txSupplier.getAsDouble();
                desiredPower = visionPID(tx);
                break;

            case VISIONFF:
                double txFF = txSupplier.getAsDouble();
                desiredPower = visionController(txFF);
                break;

            case IDLE:
            default:
                desiredPower = 0.0;
                break;
        }

        turretMotor.setPower(desiredPower);
    }

    /* ---------------- Telemetry ---------------- */

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }

    public TurretMode getMode() {
        return mode;
    }
}
