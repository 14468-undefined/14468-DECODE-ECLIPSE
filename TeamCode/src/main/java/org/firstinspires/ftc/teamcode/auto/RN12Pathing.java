package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.command.Shoot3Command;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
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
@Autonomous(name = "RN12Pathing")
public class RN12Pathing extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(-61, 40, Math.toRadians(180));
    private final Pose2d shotPoseOnLine = new Pose2d(-24,24, Math.toRadians(90));//go shoot


    HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;


    Shoot3Command shoot3Command;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RN12Pathing() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private Command auto() {
        return new SequentialGroup(
                //Actions.runBlocking(robot.drive.actionBuilder(robot.drive.drive.getPose()));
        );
    }



    @Override
    public void onInit(){


        //robot.limelight.initHardware(hwMap, "RED");
        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hwMap, new Pose2d(-61, 40, Math.toRadians(180)));


        autoCommand = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)



                //FIRST PILE----------------------------------------------
                .strafeToConstantHeading(new Vector2d(-11.2, 25.4))



                .strafeToConstantHeading(new Vector2d(-11.2, 54.5), new TranslationalVelConstraint(15))//intake

                //gate dump
                .strafeToConstantHeading(new Vector2d(-11.2, 48))
                .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump



                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose


                //SECOND PILE --------------------------------------
                .strafeToConstantHeading(new Vector2d(12, 29))//line up intake

                .strafeToConstantHeading(new Vector2d(12, 61), new TranslationalVelConstraint(15))//intake
                .strafeToConstantHeading(new Vector2d(12, 48))//back up


                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose


                //PILE 3

                .strafeToConstantHeading(new Vector2d(35.8, 29))//go to motif
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake


                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose


                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}