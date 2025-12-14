package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

import dev.nextftc.core.commands.Command;

public class AutoAimCommand extends Command {

    HardwareMap hwMap;
    ColorfulTelemetry cTelemetry;
    BaseRobot robot;
    public AutoAimCommand(double camAngle) {

        requires(robot.hood, robot.shooter, robot.limelight, robot.turret);
        setInterruptible(true);
    }

    @Override
    public boolean isDone() {
        return false; // whether or not the command is done
    }

    @Override
    public void start() {

        ShotPoint shot = shooterInterpolator.interpolate(camAngle);

        robot.hood.setHoodAngle(shot.hoodDeg);
        robot.shooter.setTargetRPM(shot.rpm);
    }

    @Override
    public void update() {
        // executed on every update of the command
    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
    }
}