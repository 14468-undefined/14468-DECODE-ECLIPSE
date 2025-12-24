package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.command.Shoot3Command;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(name = "BN12DA3")
public class BN12DA3 extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(9.0, 111.0, Math.toRadians(-90.0));
    private final Pose2d shotPoseOnLine = new Pose2d(0, 0, Math.toRadians(0));

    HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public BN12DA3() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }





    @Override
    public void onInit(){

        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hwMap, startPose);

        autoAimCommand = new AutoAimCommand(robot);
        autoCommand = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                .stopAndAdd(autoAimCommand)
                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.gate.closeGate)

                //.fresh()//update pose estimate

                //FIRST PILE----------------------------------------------
                .strafeToLinearHeading(new Vector2d(0, 0), 0)//go to first pile
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(0, 0))//intake
                .strafeToConstantHeading(new Vector2d(0, 0))//back up
                .strafeToConstantHeading(new Vector2d(0, 0))//line up a gate dump
                .strafeToConstantHeading(new Vector2d(0, 0))//dump gate
                .strafeToConstantHeading(new Vector2d(0, 0))//back up
                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .afterTime(1.0, new AutoAimCommand(robot))

                .stopAndAdd(shoot3Command)
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

                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.shooter.stop())
                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}