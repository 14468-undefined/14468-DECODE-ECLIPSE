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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.util.Constants;

/**
 * auto naming scheme explained:
 *
 * First: B/R - Blue/Red aliances
 * Second: N/F - Near/Far side from goal
 * Third: # Of Artifacts - (i.e. 12)
 * Fourth: indicates whether or not there is a
 * gate dump, and if so, where it is located.
 * ND: No Dump
 * DA3: Dump after 3 shot (preloads)
 * DA6: Dump after 6 shot (preloads + 1 pile)
 *
 * Example:*
 * BN12DA3 - Blue Near 12 Artifacts with a gate dump after preloads shot
 */
@Autonomous(name = "BF9ND")
public class BF9ND extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(9.0, 111.0, Math.toRadians(-90.0));
    private final Pose2d shotPose = new Pose2d(0, 0, Math.toRadians(0));

    HardwareMap hwMap;
    MecanumDrive drive;

    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public BF9ND() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }





    @Override
    public void onInit(){

        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.FAR_ZONE, 0);//shot time is irrelevant here bc its far zone
        drive = new MecanumDrive(hwMap, startPose);
        autoAimCommand = new AutoAimCommand(robot);

        autoCommand = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPose.position, shotPose.heading)
                .stopAndAdd(autoAimCommand)//aim turret and set hood
                .stopAndAdd(shoot3Command)//shoot 3

                .stopAndAdd(robot.gate.closeGate)//close gate


                //.fresh()//update pose estimate

                //FIRST PILE----------------------------------------------
                .strafeToLinearHeading(new Vector2d(0, 0), 0)//go to first pile
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(0, 0))//intake
                .strafeToConstantHeading(new Vector2d(0, 0))//back up
                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))//set hood angle to an estimate
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//set RPM to an estimate so it can start spinning up

                .strafeToLinearHeading(shotPose.position, shotPose.heading)//go to shoot pose
                .afterTime(1.0, autoAimCommand)//start auto aiming after 1 second

                .stopAndAdd(shoot3Command)//shoot 3
                .stopAndAdd(robot.shooter.stop())//stop flywheel

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

                .strafeToLinearHeading(shotPose.position, shotPose.heading)//go to shoot pose
                .afterTime(4.0, autoAimCommand)//after 4 seconds start auto aiming

                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.shooter.stop())
                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}