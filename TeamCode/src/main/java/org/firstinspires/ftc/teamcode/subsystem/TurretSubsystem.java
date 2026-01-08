package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

@Config
public class TurretSubsystem implements Subsystem {

    public static final TurretSubsystem INSTANCE = new TurretSubsystem();


    private final MotorEx turretMotor = new MotorEx("turret").brakeMode();//.reversed();


    private TurretSubsystem() {


        angleController = ControlSystem.builder()
                .posPid(0.1, 0.0, 0.001)
                .build();



    }

    public enum TurretMode {
        ANGLE,
        VISION
    }

    private TurretMode mode = TurretMode.ANGLE;


    private ColorfulTelemetry telemetry;

    private ControlSystem angleController;

    private static final int TICKS_TOLERANCE = 10;
    private static final double TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 1.0;

    private static final double MAX_TICKS_RIGHT = 0;
    private static final double MAX_TICKS_LEFT = 0;

    private DoubleSupplier txSupplier = () -> 0.0;

    // Vision PID state
    private static double kP = 0.02;
    private static double kI = 0.0;
    private static double kD = 0.003;

    private double integralSum = 0;
    private double lastError = 0;
    private double lastTime = 0;



    // ---------------- Initialization ----------------


    // ---------------- Angle Control ----------------
    public double angleToTicks(double degrees) {
        return degrees / 360.0 * TICKS_PER_REV * GEAR_RATIO;
    }

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

    // ---------------- Vision Control ----------------
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


    /*public Command setTurretPower(double p) {
        return new LambdaCommand()
                .setStart(() -> {
                    turretMotor.setPower(p);
                })
                .setIsDone(() -> false)
                .requires(this)
                .named("TurretVisionAim");
    }

     */


    public Command set4 = new SetPower(turretMotor, .4);
    public Command setneg4 = new SetPower(turretMotor, -.4);
    public Command set0 = new SetPower(turretMotor, 0);
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

    // ---------------- Periodic ----------------
    @Override
    public void periodic() {


        /* if (mode == TurretMode.ANGLE) {
            turretMotor.setPower(
                    angleController.calculate(turretMotor.getState())
            );
        } else {
            double tx = txSupplier.getAsDouble();
            turretMotor.setPower(turretPID(tx));
        }
        //TODO: FIX
         */





    }

    public double getTurretPosition() {
        return turretMotor.getCurrentPosition();
    }
}
