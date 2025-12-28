package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class LEDSubsystem implements Subsystem {


    //ServoEx gateRight;
    ColorfulTelemetry cTelemetry;


    double currentPose;



    public static final LEDSubsystem INSTANCE = new LEDSubsystem();

    private LEDSubsystem() {
    }


    private ServoEx LED = new ServoEx("LED");
    //gateRight = hardwareMap.get(ServoEx.class, "gateRight");



    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }






    public double getLEDColor(){
        return currentPose;
    }

    public Command setColor(LEDColor color) {
        return new LambdaCommand()
                .setStart(() -> {
                    LED.setPosition(color.position);
                })
                .setIsDone(()-> true)
                .requires(this)
                .named("SetLEDColor");
    }

    public enum LEDColor {
        OFF(0.0),

        RED(0.28),

        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),//light green
        GREEN(0.5),
        UNDEFINEDBLUE(.57),
        AZURE(0.555),//blue green
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0);


        public final double position;

        LEDColor(double position) {
            this.position = position;
        }
    }
    public void printTelemetry(ColorfulTelemetry t){

        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)d
        currentPose = LED.getPosition();
    }
}
