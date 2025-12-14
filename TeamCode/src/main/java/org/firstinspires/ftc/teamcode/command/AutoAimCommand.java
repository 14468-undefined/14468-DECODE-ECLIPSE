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

    ShotInterpolator shooterInterpolator = new ShotInterpolator();

    double Tx;

    public AutoAimCommand(double camAngle, double Tx) {

        requires(robot.hood, robot.shooter, robot.limelight, robot.turret);
        setInterruptible(true);
        this.Tx = Tx;
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
        double camAngle = robot.limelight.getDistance();//in angle (Ty)
        ShotPoint shot = shooterInterpolator.interpolate(camAngle);

        robot.hood.setHoodAngle(shot.hoodDeg);
        robot.shooter.setTargetRPM(shot.rpm);


        /*
        this is the turret auto-aim logic
         */

        robot.turret.setTurretPower(robot.turret.turretPID(Tx));
    }

    @Override
    public void stop(boolean interrupted) {
        // executed when the command ends
        robot.shooter.eStop();
    }
}