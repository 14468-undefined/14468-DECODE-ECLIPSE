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
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "BlueNear12")
public class BlueNear12 extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(9.0, 111.0, Math.toRadians(-90.0));
    private final Pose2d scorePose = new Pose2d(16.0, 128.0, Math.toRadians(-45.0));
    private final Pose2d shotPoseOnLine = new Pose2d(0, 0, Math.toRadians(0));
    private final Pose2d firstPileIntakeLineUp = new Pose2d(0, 0, 0);
    private final Pose2d firstPileIntakeEnd = new Pose2d(0,0,0);
    HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command shootFirst3;
    Command intakeFirstPile;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public BlueNear12() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }

    private Command shoot3() {
        return new SequentialGroup(
                robot.gate.openGate,
                new ParallelGroup(
                        //TODO: aim with vision with updating Tx
                        //new InstantCommand(robot.shooter::spin)
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

        intakeFirstPile = drive.commandBuilder(robot.drive.getPose())//TODO - update this so it acc gets the last pose and not the init pose
                .stopAndAdd(robot.gate.closeGate)
                .strafeToLinearHeading(firstPileIntakeLineUp.position, firstPileIntakeLineUp.heading)
                .stopAndAdd(robot.intake.intake())
                .strafeToConstantHeading(firstPileIntakeEnd.position)
                .stopAndAdd(robot.intake.stop())
                //stopAndAdd(spinup shooter)
                .build();


    }
    @Override
    public void onStartButtonPressed() {
        shootFirst3.schedule();
        intakeFirstPile.schedule();

    }


}