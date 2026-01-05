package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Autonomous(name = "RN12DA6")
public class RN12DA6 extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(-61, 40, Math.toRadians(180));
    private final Pose2d shotPoseOnLine = new Pose2d(-24,24, Math.toRadians(90));//go shoot


    HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RN12DA6() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }



    @Override
    public void onInit(){

        robot.limelight.initHardware(hwMap, "RED");
        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hwMap, startPose);

        autoAimCommand = new AutoAimCommand(robot);
        autoCommand = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                .stopAndAdd(autoAimCommand)
                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.gate.closeGate)


                //FIRST PILE----------------------------------------------
                .strafeToConstantHeading(new Vector2d(-11.2, 25.4))


                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(-11.2, 54.5), new TranslationalVelConstraint(15))//intake
                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .stopAndAdd(autoAimCommand)

                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.shooter.stop())

                .stopAndAdd(robot.gate.closeGate)//close gate

                //SECOND PILE --------------------------------------
                .strafeToConstantHeading(new Vector2d(12, 29))//line up intake

                .strafeToConstantHeading(new Vector2d(12, 61), new TranslationalVelConstraint(15))//intake
                .strafeToConstantHeading(new Vector2d(12, 48))//back up

                .stopAndAdd(robot.intake.stop())

                //GATE DUMP
                .strafeToConstantHeading(new Vector2d(1.4, 48))//line up
                .strafeToConstantHeading(new Vector2d(1.4, 55))//gate dump

                //set the hood and rpm to a estimated value in case the ll fails
                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .stopAndAdd(autoCommand)

                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)


                //PILE 3
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(35.8, 29))//go to motif
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake
                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .stopAndAdd(autoCommand)

                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)

                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}