package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import java.util.function.DoubleSupplier;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.interpolators.ConstantInterpolator;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class TurretSubsystem implements Subsystem {

    public static final TurretSubsystem INSTANCE = new TurretSubsystem();

    /* ---------------- Hardware ---------------- */
    // Motor reversed so +power = right, -power = left
    public final MotorEx turretMotor = new MotorEx("turret").reversed().brakeMode();

    /* ---------------- Modes ---------------- */
    public enum TurretMode {
        ANGLE, VISION, IDLE
    }
    public TurretMode mode = TurretMode.IDLE;

    /* ---------------- Angle Control ---------------- */
    private ControlSystem angleController;

    /* ---------------- Vision PID ---------------- */
    //constants 1/15/26 working decent
    public static double kP = 0.03;
    public static double kI = 0.001;
    public static double kD = 0.001;
    public double integralSum = 0.0;
    public double lastError = 0.0;
    public double lastTime = 0.0;
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
    private TurretSubsystem() { }

    @Override
    public void initialize() {
        turretMotor.zero();
        angleController = ControlSystem.builder()
                .posPid(0.1, 0.0, 0.0)
                .basicFF(0)
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
                .setIsDone(() -> Math.abs(
                        angleController.getGoal().getPosition() - turretMotor.getCurrentPosition()
                ) < TICKS_TOLERANCE)
                .setStop(interrupted -> mode = TurretMode.IDLE)
                .requires(this)
                .named("TurretToAngle");
    }

    public Command runToTicks(double ticks) {
        return new LambdaCommand()
                .setStart(() -> {
                    angleController.reset();
                    mode = TurretMode.ANGLE;
                    angleController.setGoal(new KineticState(ticks));
                })
                .setIsDone(() -> Math.abs(
                        angleController.getGoal().getPosition() - turretMotor.getCurrentPosition()
                ) < TICKS_TOLERANCE)
                .setStop(interrupted -> mode = TurretMode.IDLE)
                .requires(this)
                .named("TurretToTicks");
    }

    public boolean isAiming() {
        return mode == TurretMode.VISION;
    }

    public void stopTurret() {
        mode = TurretMode.IDLE;
        //turretMotor.setPower(0.0);
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

    public Command resetTicks() {
        return new LambdaCommand()
                .setStart(turretMotor::zero)
                .setIsDone(() -> true)
                .requires(this)
                .named("ResetTicks");
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
        if (Math.abs(error) > TX_TOLERANCE && Math.abs(output) < MIN_POWER) {
            output = Math.copySign(MIN_POWER, error);
        }

        // Deadband
        if (Math.abs(error) <= TX_TOLERANCE) {
            output = 0.0;
            integralSum = 0.0;
        }

        return output;
    }

    /* ---------------- Periodic ---------------- */
    @Override
    public void periodic() {
        switch (mode) {
            case ANGLE:
                double rawPower = angleController.calculate(turretMotor.getState());
                desiredPower = Math.max(-.5, Math.min(.5, rawPower));
                break;
            case VISION:
                double tx = txSupplier.getAsDouble();
                desiredPower = visionPID(tx);
                break;
            case IDLE:
            default:
                turretMotor.setPower(0);
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
