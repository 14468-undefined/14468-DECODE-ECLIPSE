package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class TurretSubsystem implements Subsystem {

    public enum TurretMode {
        ANGLE,
        VISION
    }

    private TurretMode mode = TurretMode.ANGLE;

    private MotorEx turretMotor;
    private ColorfulTelemetry telemetry;

    private ControlSystem angleController;

    private static final int TICKS_TOLERANCE = 10;//TODO: EDIT
    private static final double TICKS_PER_REV = 28;//TODO: EDIT
    private static final double GEAR_RATIO = 1.0;//TODO: EDIT

    private DoubleSupplier txSupplier = () -> 0.0;

    // Vision PID state
    private double kP = 0.02;
    private double kI = 0.0;
    private double kD = 0.003;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public TurretSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {
        this.telemetry = telemetry;

        turretMotor = new MotorEx("turret");
        turretMotor.brakeMode();
    }

    @Override
    public void initialize() {
        angleController = ControlSystem.builder()
                .posPid(0.1, 0.0, 0.001)
                .build();
    }

    public double angleToTicks(double degrees) {
        return degrees / 360.0 * TICKS_PER_REV * GEAR_RATIO;
    }

    /* ---------------- Angle Control ---------------- */

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
                .requires(this)
                .named("TurretToAngle");
    }

    public Command homeTurret() {
        return runToAngle(0).named("HomeTurret");
    }

    /* ---------------- Vision Control ---------------- */

    public Command aimWithVision(DoubleSupplier txSupplier) {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = TurretMode.VISION;
                    this.txSupplier = txSupplier;

                    integralSum = 0;
                    lastError = 0;
                    lastTime = System.nanoTime() / 1e9;
                })
                .setIsDone(() -> false)
                .requires(this)
                .named("TurretVisionAim");
    }

    private double turretPID(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.02;

        double P = kP * error;

        double potentialIntegral = integralSum + error * dt;
        if (Math.abs(potentialIntegral * kI) < 1.0) {
            integralSum = potentialIntegral;
        }
        double I = kI * integralSum;

        double derivative = (error - lastError) / dt;
        double D = kD * derivative;

        lastError = error;
        lastTime = currentTime;

        double output = P + I + D;
        output = Math.max(-1.0, Math.min(1.0, output));

        if (Math.abs(error) < 0.5) {
            output = 0;
            integralSum = 0;
        }

        return output;
    }

    /* ---------------- Periodic ---------------- */

    @Override
    public void periodic() {

        if (mode == TurretMode.ANGLE) {
            turretMotor.setPower(
                    angleController.calculate(turretMotor.getState())
            );
        } else {
            double tx = txSupplier.getAsDouble();
            turretMotor.setPower(turretPID(tx));
        }
    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }
}
