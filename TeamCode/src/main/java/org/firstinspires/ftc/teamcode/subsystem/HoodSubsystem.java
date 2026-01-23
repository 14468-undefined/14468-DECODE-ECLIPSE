package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class HoodSubsystem implements Subsystem {

    public static final HoodSubsystem INSTANCE = new HoodSubsystem();
    private HoodSubsystem() {}

    private final ServoEx hood = new ServoEx("hood");

    // TODO: tune these
    private static final double MIN_ANGLE = 30;
    private static final double MAX_ANGLE = 50;

    private static final double HOOD_DOWN = .0311;
    private static final double HOOD_UP = .84;

    public double angleToPose(double angle) {
        angle = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
        return (angle - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE);
    }

    public Command setHoodAngle(double degrees) {
        return new LambdaCommand()
                .setStart(() -> hood.setPosition(angleToPose(degrees)))
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Hood Angle");
    }

    public Command setHoodPose(double pose) {///FOR TESTING
        return new LambdaCommand()
                .setStart(() -> hood.setPosition(pose))
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Hood pose");
    }

    public double getHoodPose() {
        return hood.getPosition();
    }

    @Override
    public void periodic() {
        // nothing needed unless you add closed-loop control
    }
}
