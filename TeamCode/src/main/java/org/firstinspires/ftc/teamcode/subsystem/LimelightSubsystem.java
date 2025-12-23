package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class LimelightSubsystem implements Subsystem {

    public static final LimelightSubsystem INSTANCE = new LimelightSubsystem();
    private LimelightSubsystem() {}

    private Limelight3A limelight;

    public double Tx;

    //constants
    private static final double ROT_TOLERANCE = 2;

    //TODO: gotta initialize hardware in the opmode init
    public void initHardware(HardwareMap hardwareMap) {
        if (limelight == null) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(8);
        }
    }


    public Command initializeLimelight() {
        return new LambdaCommand()
                .setStart(() -> limelight.start())
                .setIsDone(() -> true)
                .requires(this)
                .named("Initialize Limelight");
    }

    public Command stopLimelight() {
        return new LambdaCommand()
                .setStart(() -> limelight.stop())
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Limelight");
    }

    //methods
    public double getDistance() {
        if (!limelight.isRunning()) limelight.start();
        LLResult llResult = limelight.getLatestResult();
        return (llResult != null && llResult.isValid()) ? llResult.getTy() : Double.NaN;
    }

    public double getTx() {
        if (!limelight.isRunning()) limelight.start();
        LLResult llResult = limelight.getLatestResult();
        return (llResult != null && llResult.isValid()) ? llResult.getTx() : Double.NaN;
    }

    //teleem
    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine();
        t.update();
    }

    @Override
    public void initialize() {
        if (limelight != null) limelight.start();
    }

    @Override
    public void periodic() {

    }
}
