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

public class Shoot3Command extends Command {

    HardwareMap hwMap;
    ColorfulTelemetry cTelemetry;
    BaseRobot robot;



    public Shoot3Command(BaseRobot robot) {

        this.robot = robot;
        requires(robot.gate, robot.shooter);
        setInterruptible(true);


    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void start() {



    }

    @Override
    public void update() {
        /*
        this is the interpolation for both hood angle and RPM of the shooter
         */



    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
        robot.shooter.stop();
    }
}