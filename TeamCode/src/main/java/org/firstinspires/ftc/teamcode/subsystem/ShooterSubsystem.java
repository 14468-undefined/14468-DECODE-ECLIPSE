package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.VoltageCompensatingMotor;

@Config
public class ShooterSubsystem implements Subsystem {

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();




    //RPM close - 2550


    /**
     *
     * 1. tune kS
     * 2. get to target vel with kV
     * 3. add a lot of kP (at least .001 maybe .01) - specifically,
     * increase it until it isnt overshooting muych in the recovery phase
     *
     */


    //2/11/26 - both encoders
    //

    public static double TARGET_RPM = 3500.0;//3150 = far - new 2/5, 2550 = close
    public static double TARGET_REVERSE_RPM = 2500;
    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_REV = 28.0;

    private static final double IDEAL_VOLTAGE = 13.8;
    private static final double MAX_VOLTAGE_COMP = 1.20;

    private double batteryVoltage = 13.8;
    private double compensatedKV = kV;



    private boolean shooterEnabled = false;
    //2500-2450 rpm close
    //3520 rpm far

    //values as of 2/4
    public static double kP = 0.000058;//0.00014
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.00040;//0.00041
    public static double kS = 0.0;//.07


    private double lastKP, lastKI, lastKD, lastKV, lastKS;

    private double lastCompensatedKV;

    private ControlSystem controller;

    private MotorEx shooterLeft;
    private MotorEx shooterRight;

    private double manualLeft = 0.0;
    private double manualRight = 0.0;


    private ShooterSubsystem() {}

    @Override
    public void initialize() {
        shooterLeft = new MotorEx("shooterLeft").reversed();
        shooterRight = new MotorEx("shooterRight");

        //VoltageCompensatingMotor shooterLeft = new VoltageCompensatingMotor("shooterLeft").

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        controller = ControlSystem.builder()
                .velPid(kP, kI, kD)
                .basicFF(kV, 0, kS)
                .build();

        controller.setGoal(new KineticState(0, 0));

        lastKP = kP;
        lastKI = kI;
        lastKD = kD;
        lastKV = kV;
        lastKS = kS;
    }

    // ONLY rebuilds if values changed
    public void maybeUpdatePIDF() {
        if (kP != lastKP || kI != lastKI || kD != lastKD || kV != lastKV || kS != lastKS) {
            double comp = IDEAL_VOLTAGE / batteryVoltage;
            comp = Math.min(comp, MAX_VOLTAGE_COMP);
            compensatedKV = kV * comp;

            controller = ControlSystem.builder()
                    .velPid(kP, kI, kD)
                    .basicFF(kV, 0, kS)
                    .build();

            if (shooterEnabled) {
                controller.setGoal(new KineticState(0, RPMtoTPS(TARGET_RPM)));
            }


            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
            lastKV = kV;
            lastKS = kS;
        }
    }



    public Command setTargetRPM(double rpm) {
        return new LambdaCommand()
                .setStart(() -> {
                    TARGET_RPM = rpm;   // store only
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Shooter Target RPM");
    }

    public void voltageCompensate(double voltage) {
        if (voltage <= 0) return;

        batteryVoltage = voltage;

        double comp = IDEAL_VOLTAGE / batteryVoltage;
        comp = Math.min(comp, MAX_VOLTAGE_COMP);

        compensatedKV = kV * comp;

        // Rebuild controller if FF changed
        if (Math.abs(compensatedKV - lastKV) > 1e-6) {
            controller = ControlSystem.builder()
                    .velPid(kP, kI, kD)
                    .basicFF(compensatedKV)
                    .build();

            controller.setGoal(new KineticState(0, RPMtoTPS(TARGET_RPM)));

            lastKV = compensatedKV;
        }
    }

    /* ===== existing Commands (unchanged) ===== */

    public Command setShooterPIDF(double kv, double kp, double kd, double ki, double ks) {
        return new LambdaCommand()
                .setStart(() -> {
                    kV = kv;
                    kP = kp;
                    kI = ki;
                    kD = kd;
                    kS = ks;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Shooter PIDF");
    }

    public void setTargetRPMDirect(double rpm) {
        TARGET_RPM = rpm;

            controller.setGoal(
                    new KineticState(0, RPMtoTPS(TARGET_RPM))
            );

    }


    public void spinPls(){
        shooterEnabled = true;
        controller.setGoal(
                new KineticState(0, RPMtoTPS(TARGET_RPM))
        );
    }


    public Command spinAndHold() {
        return new LambdaCommand()
                .setStart(() -> {

                    controller.setGoal(
                            new KineticState(0, RPMtoTPS(TARGET_RPM))
                    );
                })
                .setIsDone(() -> true)
                .requires(this);
    }

    public Command spin() {
        return new LambdaCommand()
                .setStart(() -> {

                    shooterEnabled = true;
                    controller.setGoal(
                            new KineticState(0, RPMtoTPS(TARGET_RPM))
                    );
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Spin Shooter");
    }

    public Command spinReverse() {
        return new LambdaCommand()
                .setStart(() -> {
                    shooterEnabled = true;
                    controller.setGoal(
                            new KineticState(0, RPMtoTPS(-TARGET_REVERSE_RPM))
                    );
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Spin Shooter Reverse");
    }

    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    shooterEnabled = false;
                    controller.setGoal(
                            new KineticState(0, RPMtoTPS(0))
                    );
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Shooter");
    }





    public double RPMtoTPS(double rpm) {
        return ((rpm / GEAR_RATIO) * TICKS_PER_REV) / 60.0;
    }

    public double getRPM() {
        double avgTPS = (-shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
        return (avgTPS * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public double getLeftRPM(){
        return ((-shooterLeft.getVelocity()) * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }

    public double getRightRPM(){
        return ((shooterRight.getVelocity()) * 60.0 / TICKS_PER_REV) * GEAR_RATIO;
    }


    public boolean isAtTargetSpeed() {
        double vel = getRPM();
        return vel > (TARGET_RPM - 10) && vel < (TARGET_RPM + 350) && vel != 0;
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

    @Override
    public void periodic() {

        shooterLeft.setPower(controller.calculate(shooterLeft.getState()));
        shooterRight.setPower(controller.calculate(shooterRight.getState()));

    }
}
