package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LimelightSubsystem implements Subsystem {

    public static final LimelightSubsystem INSTANCE = new LimelightSubsystem();

    private LimelightSubsystem() { }

    private Limelight3A limelight;

    // constants
    private static final double ROT_TOLERANCE = 2;

    // ------------------------
    // Commands
    // ------------------------

    public Command initializeLimelight() {
        return new LambdaCommand()
                .setStart(() -> {
                    if (limelight != null) limelight.start();
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Initialize Limelight");
    }

    public Command setPipeline(int pipeline) {
        return new LambdaCommand()
                .setStart(() -> {
                    if (limelight != null) limelight.pipelineSwitch(pipeline);
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Set Pipeline");
    }

    public Command stopLimelight() {
        return new LambdaCommand()
                .setStart(() -> {
                    if (limelight != null) limelight.stop();
                })
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Limelight");
    }

    // ------------------------
    // Methods
    // ------------------------

    public double getDistance() {
        if (limelight == null) return Double.NaN;
        if (!limelight.isRunning()) limelight.start();
        LLResult llResult = limelight.getLatestResult();
        return (llResult != null && llResult.isValid()) ? llResult.getTy() : Double.NaN;
    }

    public double getTx() {
        if (limelight == null) return Double.NaN;
        if (!limelight.isRunning()) limelight.start();
        LLResult llResult = limelight.getLatestResult();
        return (llResult != null && llResult.isValid()) ? llResult.getTx() : Double.NaN;
    }

    public boolean hasTarget() {
        if (limelight == null || !limelight.isRunning()) return false;
        LLResult result = limelight.getLatestResult();
        return result != null && result.isValid();
    }

    // ------------------------
    // Telemetry
    // ------------------------
    public void printTelemetry(ColorfulTelemetry t) {
        t.addLine();
        t.update();
    }

    // ------------------------
    // Subsystem lifecycle
    // ------------------------
    @Override
    public void initialize() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        if (limelight != null) {
            limelight.start();
            limelight.pipelineSwitch(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE); // default pipeline
        }
    }

    @Override
    public void periodic() {
        // no periodic needed right now
    }
}
