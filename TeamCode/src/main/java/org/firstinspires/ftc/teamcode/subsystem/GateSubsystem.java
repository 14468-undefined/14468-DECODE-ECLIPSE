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

    ServoEx gate;
    //ServoEx gateRight;
    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;

    private final double GATE_OPEN = 0;
    private final double GATE_CLOSED = 0;
    //private final double RIGHT_GATE_OPEN = 0;
    //private final double RIGHT_GATE_CLOSED = 0;



    public GateSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        gate = hardwareMap.get(ServoEx.class, "gate");
        //gateRight = hardwareMap.get(ServoEx.class, "gateRight");

    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
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
