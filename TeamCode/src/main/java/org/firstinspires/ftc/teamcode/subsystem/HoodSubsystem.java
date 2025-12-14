package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class HoodSubsystem implements Subsystem {

    ServoEx hood;
    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;

    //TODO: change these
    private static double MIN_ANGLE = 30;
    private static double MAX_ANGLE = 50;





    public HoodSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        hood = hardwareMap.get(ServoEx.class, "hood");

    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }

    public double angleToPose(double angle) {
        angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
        double pose = (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
        return pose;
    }

    public Command setHoodAngle(double degrees) {
        return new LambdaCommand()
                .setStart(() -> {
                    hood.setPosition(angleToPose(degrees));

                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Hood Angle");
    }




    public double getHoodPose(){
        return hood.getPosition();
    }



    public void printTelemetry(ColorfulTelemetry t){

        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
