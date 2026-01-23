package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class GateSubsystem implements Subsystem {


    //ServoEx gateRight;
    ColorfulTelemetry cTelemetry;


    private final double GATE_OPEN = .32;
    private final double GATE_CLOSED = .93;
    //private final double RIGHT_GATE_OPEN = 0;
    //private final double RIGHT_GATE_CLOSED = 0;


    public static final GateSubsystem INSTANCE = new GateSubsystem();

    private GateSubsystem() {
    }


    private ServoEx gate = new ServoEx("gate");
    //gateRight = hardwareMap.get(ServoEx.class, "gateRight");


    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }




    public Command setGatePos(double pose) {
        return new LambdaCommand()
                .setStart(() -> gate.setPosition(pose))
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Intake Power");
    }
    public Command closeGate = new SetPosition(gate, GATE_CLOSED).requires(this);
    public Command openGate = new SetPosition(gate, GATE_OPEN).requires(this);

    public double getGatePose(){
        return gate.getPosition();
    }




    public void printTelemetry(ColorfulTelemetry t){

        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
