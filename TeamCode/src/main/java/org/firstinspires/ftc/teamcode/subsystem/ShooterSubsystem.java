package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class ShooterSubsystem implements Subsystem {

    public static double TARGET_RPM = 3500.0;
    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_REV = 28.0;

    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;

    private boolean active;

    private final MotorEx shooterLeft = new MotorEx("shooterLeft");
    private final MotorEx shooterRight = new MotorEx("shooterRight");

    private ControlSystem controller;

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private ShooterSubsystem() {
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(kV)
                .build();
        controller.setGoal(new KineticState(0, 0));
    }

    public void applyPIDF() {
        controller = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(kV)
                .build();
    }

    public Command setShooterPIDF(double kv, double kp, double kd, double ki) {
        return new LambdaCommand()
                .setStart(() -> {
                    kV = kv;
                    kP = kp;
                    kI = ki;
                    kD = kd;
                    applyPIDF();
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Shooter PIDF");
    }

    public Command spin(double targetRPM) {
        return new LambdaCommand()
                .setStart(() -> controller.setGoal(new KineticState(0, RPMtoTPS(targetRPM))))
                .setIsDone(() -> true)
                .requires(this)
                .named("Spin Shooter");
    }



    public Command setLeft1 = new SetPower(shooterLeft, 1);
    public Command setRight1 = new SetPower(shooterRight, 1);
    public Command setLeftNeg1 = new SetPower(shooterLeft, -1);
    public Command setRightNeg1 = new SetPower(shooterRight, -1);
    public Command setLeft0 = new SetPower(shooterLeft, 0);
    public Command setRight0 = new SetPower(shooterRight, 0);



    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Shooter");
    }

    public double RPMtoTPS(double rpm) {
        TARGET_RPM = rpm;
        return ((TARGET_RPM / GEAR_RATIO) * TICKS_PER_REV) / 60.0;
    }

    public double getRPM() {
        double avgTPS = (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
        return (avgTPS * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public boolean isAtTargetSpeed() {
        double vel = getRPM();
        return vel > (TARGET_RPM - 10) && vel < (TARGET_RPM + 350) && vel != 0;
    }

    public double getTargetRPM(){
        return TARGET_RPM;
    }

    public double getLeftPower(){
        return shooterLeft.getPower();
    }
    public double getRightPower(){
        return shooterRight.getPower();
    }

    @Override
    public void periodic() {
        shooterLeft.setPower(controller.calculate(shooterLeft.getState()));
        shooterRight.setPower(controller.calculate(shooterRight.getState()));
    }
}
