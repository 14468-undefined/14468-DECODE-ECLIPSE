package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.subsystems.SubsystemGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class ShooterSubsystem implements Subsystem {

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    private ShooterSubsystem() {}

    // --- Hardware ---
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;

    // --- PID / Shooter Constants ---
    public static double TARGET_RPM = 3500.0;
    private static double MOTOR_RPM = 6000;
    private static double GEAR_RATIO = 1;
    private static double TICKS_PER_REV = 28;

    private boolean active;

    // --- PIDF Coefficients ---
    public static double kP = 20;
    public static double kI = 0.0;
    public static double kD = 5.0;
    public static double kF = 24.0;

    private boolean initialized = false;

    // --- HardwareMap init ---
    public void initHardware(HardwareMap hardwareMap) {
        if (initialized) return;

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();
        active = Math.abs(TARGET_RPM) > 0;

        initialized = true;
    }

    // --- PIDF ---
    public void setShooterPIDF(double kf, double kp, double kd, double ki) {
        kP = kp;
        kI = ki;
        kD = kd;
        kF = kf;
        applyPIDF();
    }

    public void applyPIDF() {
        if (!initialized) return;
        shooterLeft.setVelocityPIDFCoefficients(kP, kI, kD, kF);
        shooterRight.setVelocityPIDFCoefficients(kP, kI, kD, kF);
    }

    // --- Shooter control ---
    public void setTargetRPM(double targetRPM) { TARGET_RPM = targetRPM; }
    public double getTargetRPM() { return TARGET_RPM; }

    public void setMotorRPM(double motorRPM) { MOTOR_RPM = motorRPM; }
    public void setGearRatio(double gearRatio) { GEAR_RATIO = gearRatio; }
    public double getGearRatio() { return GEAR_RATIO; }
    public void setTicksPerRev(double ticks) { TICKS_PER_REV = ticks; }
    public double getTicksPerRev() { return TICKS_PER_REV; }

    public void spin() {
        if (!initialized) return;

        double targetTPS = ((TARGET_RPM / GEAR_RATIO) * TICKS_PER_REV) / 60;
        shooterLeft.setVelocity(targetTPS);
        shooterRight.setVelocity(targetTPS);
        active = Math.abs(TARGET_RPM) > 0;
    }

    public void eStop() {
        if (!initialized) return;
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }

    public double getShooterVelocity() {
        if (!initialized) return 0;
        double avgTPS = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
        return (avgTPS * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public double getMotorVoltage() {
        if (!initialized) return 0;
        double leftAmps = shooterLeft.getCurrent(CurrentUnit.AMPS);
        double rightAmps = shooterRight.getCurrent(CurrentUnit.AMPS);
        return (leftAmps + rightAmps) / 2.0;
    }

    public boolean isActive() { return active; }

    public boolean isAtTargetSpeed() {
        double vel = getShooterVelocity();
        return (vel > TARGET_RPM - 0 && vel < TARGET_RPM + 350 && vel != 0);
    }

    @Override
    public void initialize() {
        spin(); // optional: spin immediately if you want
    }

    @Override
    public void periodic() {
        // optional: update PIDF or monitor velocity
    }
}
