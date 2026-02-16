package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

@Config
public class ShooterSubsystem implements Subsystem {

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    // ===== Tunables =====
    public static double TARGET_RPM = 0;
    public static double TARGET_REVERSE_RPM = 2500;

    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_REV = 28.0;

    public static double kP = 0.001;//.001
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.000179;//.000194
    public static double kS = 0.09;//.09

    private static final double IDEAL_VOLTAGE = 13.8;
    private static final double MAX_VOLTAGE_COMP = 1.20;

    // ===== Hardware =====
    private MotorEx shooterLeft;
    private MotorEx shooterRight;

    // ===== State =====
    private double targetRPM = 0.0;
    private double batteryVoltage = 13.8;

    // PID state
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    private ShooterSubsystem() {}

    @Override
    public void initialize() {
        shooterLeft = new MotorEx("shooterLeft").reversed();
        shooterRight = new MotorEx("shooterRight");

        shooterRight.setPower(0);
        shooterLeft.setPower(0);
        TARGET_RPM = 0;

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /* ================= PIDF ================= */

    private double pidfRPM(double targetRPM, double currentRPM) {
        double now = System.nanoTime() / 1e9;
        double dt = now - lastTime;
        if (dt <= 0) dt = 0.02;

        double error = targetRPM - currentRPM;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;

        double pid =
                (kP * error) +
                        (kI * integralSum) +
                        (kD * derivative);

        double voltageComp = IDEAL_VOLTAGE / batteryVoltage;
        voltageComp = Math.min(voltageComp, MAX_VOLTAGE_COMP);

        double ff =
                (kV * voltageComp * targetRPM) +
                        (Math.signum(targetRPM) * kS);

        lastError = error;
        lastTime = now;

        return clamp(pid + ff);
    }

    private double clamp(double val) {
        return Math.max(-1.0, Math.min(1.0, val));
    }

    /* ================= Commands (unchanged API) ================= */

    public Command setTargetRPM(double rpm) {
        return new LambdaCommand()
                .setStart(() -> TARGET_RPM = rpm)
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Shooter Target RPM");
    }

    public void voltageCompensate(double voltage) {
        if (voltage > 0) {
            batteryVoltage = voltage;
        }
    }

    public Command setShooterPIDF(double kv, double kp, double kd, double ki, double ks) {
        return new LambdaCommand()
                .setStart(() -> {
                    kV = kv;
                    kP = kp;
                    kD = kd;
                    kI = ki;
                    kS = ks;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Shooter PIDF");
    }

    public void setTargetRPMDirect(double rpm) {
        TARGET_RPM = rpm;
        targetRPM = rpm;
    }

    public void spinPls() {
        targetRPM = TARGET_RPM;
    }

    public Command spinAndHold() {
        return new LambdaCommand()
                .setStart(() -> targetRPM = TARGET_RPM)
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command spin() {
        return new LambdaCommand()
                .setStart(() -> targetRPM = TARGET_RPM)
                .setIsDone(() -> true)
                .requires(this)
                .named("Spin Shooter");
    }

    public Command spinReverse() {
        return new LambdaCommand()
                .setStart(() -> targetRPM = -TARGET_REVERSE_RPM)
                .setIsDone(() -> true)
                .requires(this)
                .named("Spin Shooter Reverse");
    }

    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    targetRPM = 0.0;
                    integralSum = 0.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Shooter");
    }

    /* ================= Telemetry ================= */

    public double RPMtoTPS(double rpm) {
        return ((rpm / GEAR_RATIO) * TICKS_PER_REV) / 60.0;
    }

    public double getRPM() {

        double leftVel = -shooterLeft.getVelocity();
        double rightVel = shooterRight.getVelocity();
        double avgTPS;
        if (leftVel < 1 && rightVel > 100) {
             avgTPS = rightVel;
        } else if (rightVel < 1 && leftVel > 100) {
            avgTPS = leftVel;
        } else {
            // Normal case: average both
            avgTPS = (leftVel + rightVel) / 2.0;
        }
        return (avgTPS * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public double getLeftRPM() {
        return ((-shooterLeft.getVelocity()) * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public double getRightRPM() {
        return ((shooterRight.getVelocity()) * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public boolean isAtTargetSpeed() {
        double rpm = getRPM();
        return rpm > (TARGET_RPM - 10) && rpm < (TARGET_RPM + 350) && rpm != 0;
    }

    public double getTargetRPM() {
        return TARGET_RPM;
    }

    public double getLeftPower() {
        return shooterLeft.getPower();
    }

    public double getRightPower() {
        return shooterRight.getPower();
    }

    /* ================= Loop ================= */

    @Override
    public void periodic() {
        // If target is zero, do NOT run PID — let flywheel coast
        if (Math.abs(targetRPM) < 1e-6) {
            shooterLeft.setPower(0.0);
            shooterRight.setPower(0.0);
            return;
        }

        double currentRPM = getRPM();
        double power = pidfRPM(targetRPM, currentRPM);

        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }
}
