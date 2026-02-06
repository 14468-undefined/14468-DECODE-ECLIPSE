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



    public double TARGET_RPM = 3500.0;//3150 = far - new 2/5, 2550 = close
    public static double TARGET_REVERSE_RPM = 2000;
    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_REV = 28.0;


    //2500-2450 rpm close
    //3520 rpm far

    //values as of 2/4
    public static double kP = 0.00014;//0.00014
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.00041;//0.00041

    private double lastKP, lastKI, lastKD, lastKV;

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
                .basicFF(kV)
                .build();

        controller.setGoal(new KineticState(0, 0));

        lastKP = kP;
        lastKI = kI;
        lastKD = kD;
        lastKV = kV;
    }

    // ONLY rebuilds if values changed
    public void maybeUpdatePIDF() {
        if (kP != lastKP || kI != lastKI || kD != lastKD || kV != lastKV) {
            controller = ControlSystem.builder()
                    .velPid(kP, kI, kD)
                    .basicFF(kV)
                    .build();

            controller.setGoal(new KineticState(0, RPMtoTPS(TARGET_RPM)));

            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
            lastKV = kV;
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


    /* ===== existing Commands (unchanged) ===== */

    public Command setShooterPIDF(double kv, double kp, double kd, double ki) {
        return new LambdaCommand()
                .setStart(() -> {
                    kV = kv;
                    kP = kp;
                    kI = ki;
                    kD = kd;
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
                    controller.setGoal(
                            new KineticState(0, RPMtoTPS(0))
                    );
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Shooter");
    }

    /*public Command setLeft1() {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = ShooterMode.MANUAL;
                    manualLeft = 1.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Shooter Left +1");
    }

    public Command setRight1() {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = ShooterMode.MANUAL;
                    manualRight = 1.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Shooter Right +1");
    }

    public Command setLeftNeg1() {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = ShooterMode.MANUAL;
                    manualLeft = -1.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Shooter Left -1");
    }

    public Command setRightNeg1() {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = ShooterMode.MANUAL;
                    manualRight = -1.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Shooter Right -1");
    }

    public Command setLeft0() {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = ShooterMode.MANUAL;
                    manualLeft = 0.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Shooter Left 0");
    }

    public Command setRight0() {
        return new LambdaCommand()
                .setStart(() -> {
                    mode = ShooterMode.MANUAL;
                    manualRight = 0.0;
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Shooter Right 0");
    }

     */



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
