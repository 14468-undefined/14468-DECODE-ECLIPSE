package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "BlueNear12")
public class BlueNear12 extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(9.0, 111.0, Math.toRadians(-90.0));
    private final Pose2d scorePose = new Pose2d(16.0, 128.0, Math.toRadians(-45.0));
    private final Pose2d shotPoseOnLine = new Pose2d(0, 0, Math.toRadians(0));
    HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command shootFirst3;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public BlueNear12() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                robot.gate.openGate,
                new ParallelGroup(
                        //TODO: aim with vision with updating Tx
                ),
                new Delay(0.5),
                new ParallelGroup(

                )
        );
    }

    @Override
    public void onInit(){

        drive = new MecanumDrive(hwMap, startPose);

        shootFirst3 = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                .stopAndAdd(new AutoAimCommand(robot))

                .build();

        driveCommand = drive.commandBuilder(startPose)
                .strafeToSplineHeading(scorePose.position, scorePose.heading)
                .afterTime(.5, robot.gate.openGate)
                .stopAndAdd(robot.intake.intake())
                .build();

    }
    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }


}