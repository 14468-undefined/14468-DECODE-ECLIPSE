package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class GateSubsystem implements Subsystem {

    ServoEx gateLeft;
    ServoEx gateRight;
    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;

    private final double LEFT_GATE_OPEN = 0;
    private final double LEFT_GATE_CLOSED = 0;
    private final double RIGHT_GATE_OPEN = 0;
    private final double RIGHT_GATE_CLOSED = 0;



    public GateSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        gateLeft = hardwareMap.get(ServoEx.class, "gateLeft");
        gateRight = hardwareMap.get(ServoEx.class, "gateRight");
    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }



    public Command closeGates() {
        return new LambdaCommand()
                .setStart(() -> {
                    gateLeft.setPosition(LEFT_GATE_CLOSED);
                    gateRight.setPosition(RIGHT_GATE_CLOSED);

                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Close Gates");
    }
    public Command openGates() {
        return new LambdaCommand()
                .setStart(() -> {
                    gateLeft.setPosition(LEFT_GATE_OPEN);
                    gateRight.setPosition(RIGHT_GATE_OPEN);

                })
                .setIsDone(() -> true)
                .requires(this);
    }



    public double getLeftGatePose(){
        return gateLeft.getPosition();
    }
    public double getRightGatePose(){
        return gateRight.getPosition();
    }



    public void printTelemetry(ColorfulTelemetry t){

        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
