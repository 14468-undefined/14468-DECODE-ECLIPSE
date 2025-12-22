package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

public class AutoAimCommand extends Command {

    HardwareMap hwMap;
    ColorfulTelemetry cTelemetry;
    BaseRobot robot;

    ShotInterpolator shooterInterpolator = new ShotInterpolator();

    private DoubleSupplier TxSupplier;
    private DoubleSupplier camAngle;

    public AutoAimCommand(BaseRobot robot, DoubleSupplier camAngle, DoubleSupplier TxSupplier) {

        this.robot = robot;
        requires(robot.hood, robot.shooter, robot.limelight, robot.turret);
        setInterruptible(true);
        this.TxSupplier = TxSupplier;
        this.camAngle = camAngle;

    }

    @Override
    public boolean isDone() {
        return false; // whether or not the command is done
    }

    @Override
    public void start() {


    }

    @Override
    public void update() {
        /*
        this is the interpolation for both hood angle and RPM of the shooter
         */

        ShotPoint shot = shooterInterpolator.interpolate(camAngle.getAsDouble());

        robot.hood.setHoodAngle(shot.hoodDeg);
        robot.shooter.setTargetRPM(shot.rpm);


        /*
        this is the turret auto-aim logic
         */

        robot.turret.aimWithVision(() -> TxSupplier.getAsDouble());
    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
        robot.shooter.eStop();
    }
}