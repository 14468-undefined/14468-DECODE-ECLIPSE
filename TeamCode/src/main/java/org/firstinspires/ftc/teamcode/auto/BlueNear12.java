package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    private final Pose2d shotPoseOnLine = new Pose2d(0, 0, Math.toRadians(0));

    HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;
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

        autoCommand = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                .stopAndAdd(new AutoAimCommand(robot))

                .stopAndAdd(robot.gate.closeGate)

                .fresh()//update pose estimate

                //FIRST PILE----------------------------------------------
                .strafeToLinearHeading(new Vector2d(0, 0), 0)//go to first pile
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(0, 0))//intake
                .strafeToConstantHeading(new Vector2d(0, 0))//back up
                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                .afterTime(1.0, new AutoAimCommand(robot))

                .stopAndAdd(shoot3())
                .stopAndAdd(robot.shooter.stop())

                .stopAndAdd(robot.gate.closeGate)//close gate

                //SECOND PILE --------------------------------------
                .strafeToLinearHeading(new Vector2d(0,0), 0)//go to second pile
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(0,0))//intake pile
                .strafeToConstantHeading(new Vector2d(0,0))//back up
                .stopAndAdd(robot.intake.stop())

                //set the hood and rpm to a estimated value in case the ll fails
                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .afterTime(1.0, new AutoAimCommand(robot))

                .stopAndAdd(shoot3())
                .stopAndAdd(robot.shooter.stop())
                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}