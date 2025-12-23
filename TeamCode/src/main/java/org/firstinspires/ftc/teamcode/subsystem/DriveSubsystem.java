package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.function.DoubleSupplier;

public class DriveSubsystem implements Subsystem {

    public static final DriveSubsystem INSTANCE = new DriveSubsystem();
    private DriveSubsystem() {}

    public MecanumDrive drive;
    private boolean initialized = false;

    // ---------------- Hardware Initialization ----------------
    public void initHardware(HardwareMap hwMap, Pose2d startPos) {
        if (initialized) return;

        drive = new MecanumDrive(hwMap, startPos);
        initialized = true;
    }

    //drive methods
    public double getHeading() {
        if (!initialized) return 0;
        return drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void driveFieldcentric(double xPow, double yPow, double rotPow, double speed) {
        if (!initialized) return;

        if (Math.abs(xPow) < 0.05 && Math.abs(yPow) < 0.05) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rotPow * speed));
            return;
        }

        double targetTheta = Math.atan2(Math.toRadians(yPow), Math.toRadians(xPow));
        double robotTheta = getHeading();
        double diffTheta = Math.toDegrees(targetTheta) - Math.toDegrees(robotTheta);
        xPow = Math.cos(Math.toRadians(diffTheta)) * speed;
        yPow = Math.sin(Math.toRadians(diffTheta)) * speed;
        rotPow = rotPow * speed;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPow, yPow), rotPow));
    }

    public void drive(double xPow, double yPow, double rotPow) {
        if (!initialized) return;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(xPow, yPow), rotPow));
    }

    public void rest() {
        if (!initialized) return;
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    // ---------------- Commands ----------------
    /*public Command getDriveCommand(DoubleSupplier xPow, DoubleSupplier yPow, DoubleSupplier rotPower) {
        return this.runEnd(() -> drive(xPow.getAsDouble(), yPow.getAsDouble(), rotPower.getAsDouble()),
                this::rest);
    }

    public Command getDriveFieldcentric(DoubleSupplier xPow, DoubleSupplier yPow, DoubleSupplier rotPower, double speed) {
        return this.runEnd(() -> driveFieldcentric(xPow.getAsDouble(), yPow.getAsDouble(), rotPower.getAsDouble(), speed),
                this::rest);
    }

     */


    public TrajectoryActionBuilder actionBuilder(Pose2d startPose) {
        if (!initialized) return null;
        return drive.actionBuilder(startPose);
    }

    public void updatePoseEstimate() {
        if (!initialized) return;
        drive.updatePoseEstimate();
    }

    public Pose2d getPose() {
        if (!initialized) return null;
        return drive.localizer.getPose();
    }

    // ---------------- Telemetry ----------------
    public void printTelemetry(ColorfulTelemetry t) {
        if (!initialized) return;

        t.addLine("");
        t.addLine("MOTOR POWERS");
        t.addLine(String.format("     %1$s     %2$s",
                Constants.Util.round(drive.leftFront.getPower(), 2),
                Constants.Util.round(drive.rightFront.getPower(), 2)));
        t.addLine(String.format("     %1$s     %2$s",
                Constants.Util.round(drive.leftBack.getPower(), 2),
                Constants.Util.round(drive.rightBack.getPower(), 2)));
        t.addLine();
        t.addLine("IMU STUFF");
        t.addLine("IMU (RAW) " + drive.lazyImu.get().getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        t.addLine("IMU (Modified) " + Math.toDegrees(drive.getHeading()));
        t.addLine("Initial Heading " + drive.initialHeading);
    }

    @Override
    public void periodic() {
        //pdate pose estimate, loggingg
    }
}
