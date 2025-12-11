package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class IntakeSubsystem implements Subsystem {

    MotorEx intakeMotor;
    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    private double intakePower = 1;
    private double reverseIntakePower = .5;


    public IntakeSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        intakeMotor = new MotorEx("intake");

        intakeMotor.floatMode();
        intakeMotor.reverse();
    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }



    public Command setIntakePower(double power) {
        return new LambdaCommand()
                .setStart(() -> intakePower = power)
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Intake Power");
    }
    public Command setReverseIntakePower(double power) {
        return new LambdaCommand()
                .setStart(() -> reverseIntakePower = power)
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Intake Power");
    }

    public Command intake() {
        return new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(intakePower))
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Intake Power");
    }

    public Command intakeReverse() {
        return new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(-reverseIntakePower))
                .setIsDone(() -> true)
                .requires(this)
                .named("Intake Reverse");
    }

    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> intakeMotor.setPower(0))
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Intake");
    }


    public double getIntakePower(){
        return intakePower;
    }
    public double getReverseIntakePower(){
        return reverseIntakePower;
    }



    public void printTelemetry(ColorfulTelemetry t){
        t.addData("Motor Power", intakeMotor.getPower());
        t.addData("Intake Power Setting", intakePower);
        t.addData("Outtake Power Setting", reverseIntakePower);
        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
