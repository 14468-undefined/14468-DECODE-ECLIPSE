package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class TurretSubsystem implements Subsystem {

    MotorEx turretMotor;
    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;
    private double intakePower = 1;
    private double reverseIntakePower = .5;


    private static double ROT_TOLERANCE = 2;//in degrees


    public TurretSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        turretMotor = new MotorEx("turret");

        turretMotor.brakeMode();
        //turretMotor.reverse();

    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }



    public Command autoAim(double aprilTagRotation) {
        return new LambdaCommand()
                .setStart(() -> {
                    //TODO - make a custom PIDF - based on rot set the power to turret motor til its 0

                    if(Math.abs(aprilTagRotation) < 2){
                        turretMotor.setPower(0);
                    }
                })

                .setIsDone(() -> false)
                .requires(this)
                .named("Set Intake Power");
    }




    public double getIntakePower(){
        return intakePower;
    }
    public double getTurretTicks(){
        return turretMotor.getRawTicks();
    }
    public double getTurretRotation(){
        double turretRotation = 0;
        //positive = angles positive
        //negative = angles negative
        return turretRotation;
    }







    public void printTelemetry(ColorfulTelemetry t){
        t.addData("Motor Power", turretMotor.getPower());
        t.addData("Intake Power Setting", intakePower);
        t.addData("Outtake Power Setting", reverseIntakePower);
        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
