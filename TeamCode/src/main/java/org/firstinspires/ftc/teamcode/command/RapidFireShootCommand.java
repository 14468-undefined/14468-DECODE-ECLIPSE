package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.function.DoubleSupplier;

import dev.nextftc.core.commands.Command;

public class RapidFireShootCommand extends Command {

    HardwareMap hwMap;
    ColorfulTelemetry cTelemetry;
    BaseRobot robot;

    Command waitTilAtTargetRPM;
    Command waitTilRPMDrop;
    Command waitTil3Shot;//for close zone when rapid fire
    double rpm;

    double camAngle;

    String shotZone = "";

    ShotInterpolator shooterInterpolator = new ShotInterpolator();

    public RapidFireShootCommand(BaseRobot robot, double shotTime) {


        this.robot = robot;
        requires(robot.gate, robot.shooter, robot.intake);
        setInterruptible(true);



        waitTilAtTargetRPM = new WaitUntil(robot.shooter::isAtTargetSpeed);
        waitTilRPMDrop = new WaitUntil(() -> !robot.shooter.isAtTargetSpeed());//TODO: since this is js once on init, will it mess up? do i need to make it RPM dropped instead of RPM less than target???

        waitTil3Shot = new Delay(shotTime);//only for close rapid fire
    }

    @Override
    public boolean isDone() {
        return false;
    }

    @Override
    public void start() {


            new SequentialGroup(
                    robot.shooter.spin(),
                    waitTilAtTargetRPM,
                    robot.intake.intake(),
                    waitTil3Shot


            ).schedule();



    }

    @Override
    public void update() {



        camAngle = robot.limelight.getDistance();

        ShotPoint shot = shooterInterpolator.interpolate(camAngle);
        rpm = shot.rpm;


    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
        robot.shooter.stop();
    }
}