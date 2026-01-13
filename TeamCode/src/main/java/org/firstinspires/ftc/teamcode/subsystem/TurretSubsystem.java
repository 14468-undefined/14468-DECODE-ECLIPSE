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

    private final MotorEx turretMotor =
            new MotorEx("turret").reversed().brakeMode();

    /* ---------------- Modes ---------------- */

    public enum TurretMode {
        ANGLE,
        VISION,
        IDLE
    }

    private TurretMode mode = TurretMode.IDLE;

    /* ---------------- Angle Control ---------------- */

    private final ControlSystem angleController;

    /* ---------------- Vision PID ---------------- */

    public static double kP = 0.01;
    public static double kI = 0.0;//.001
    public static double kD = 0.0;//.002

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    private DoubleSupplier txSupplier = () -> 0.0;

    /* ---------------- Constants ---------------- */

    private static final int TICKS_TOLERANCE = 10;
    private static final double TICKS_PER_REV = 145.1;
    private static final double GEAR_RATIO = 5.0;//TODO: Correct?

    private double desiredPower = 0.0;

    /* ---------------- Constructor ---------------- */

    private TurretSubsystem() {
        angleController = ControlSystem.builder()
                .posPid(0.1, 0.0, 0.001)
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

    /* ---------------- Vision PID ---------------- */

    private double visionPID(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.02;

        // PID constants (starting safe values)


        // Proportional
        double P = kP * error;

        // Integral
        integralSum += error * dt;
        double I = kI * integralSum;

        // Derivative
        double derivative = (error - lastError) / dt;
        double D = kD * derivative;

        lastError = error;
        lastTime = currentTime;


        // Raw PID output
        double output = P + I + D;

        // Clamp to motor limits
        output = Math.max(-.5, Math.min(.5, output));//TODO: change?

        // Minimum power to overcome static friction
        double minPower = 0.05;
        if (Math.abs(output) < minPower && Math.abs(error) > 0.5) {
            output = Math.copySign(minPower, output);
        }

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
                // Negate if direction is reversed
                desiredPower = visionPID(txSupplier.getAsDouble());
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
