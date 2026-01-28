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
@Autonomous(name = "RF12ND")
public class RF12ND extends NextFTCOpMode {
//hunter is my goat
    private final Pose2d startPose = new Pose2d(9.0, 111.0, Math.toRadians(-90.0));
    private final Pose2d shotPose = new Pose2d(65, 15, Math.toRadians(90));//go to shoot pose


    //HardwareMap hwMap;
    MecanumDrive drive;

    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;

    double HOOD_ANGLE_FAR_ESTIMATE = 0;
    double RPM_FAR_ESTIMATE = 0;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RF12ND() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }





    @Override
    public void onInit(){
        //robot.limelight.initHardware(hwMap, "RED");

        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.FAR_ZONE, 0);//shot time is irrelevant here bc its far zone
        drive = new MecanumDrive(hardwareMap, startPose);
        autoAimCommand = new AutoAimCommand(robot);


        autoCommand = drive.commandBuilder(startPose)

                //.strafeToLinearHeading(shotPose.position, shotPose.heading)
                //.stopAndAdd(robot.turret.runToAngle(-45))//so it can see the tag
                //.stopAndAdd(autoAimCommand)//aim turret and set hood
                //.stopAndAdd(shoot3Command)//shoot 3

                .stopAndAdd(robot.gate.closeGate)//close gate


                //CORNER----------------------------------------------
                .strafeToSplineHeading(new Vector2d(53, 57), Math.toRadians(70))//line up for HP zone balls
                .stopAndAdd(robot.intake.intake())
                .strafeToSplineHeading(new Vector2d(61, 60), Math.toRadians(60), new TranslationalVelConstraint(5))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(65, 60), Math.toRadians(90), new TranslationalVelConstraint(5))//line up for HP zone balls

                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_FAR_ESTIMATE))//set hood angle to an estimate
                .stopAndAdd(robot.shooter.spin(RPM_FAR_ESTIMATE))//set RPM to an estimate so it can start spinning up

                .strafeToLinearHeading(shotPose.position, shotPose.heading)//go to shoot pose
                //.stopAndAdd(autoAimCommand)//start auto aiming after 1 second

                //.stopAndAdd(shoot3Command)//shoot 3
                .stopAndAdd(robot.shooter.stop())//stop flywheel

                .stopAndAdd(robot.gate.closeGate)//close gate

                //FIRST PILE --------------------------------------
                .strafeToSplineHeading(new Vector2d(35.8, 29), Math.toRadians(90))//go to motif 1
                .stopAndAdd(robot.intake.intake())
                .strafeToConstantHeading(new Vector2d(35.8, 61), new TranslationalVelConstraint(15))//intake
                .stopAndAdd(robot.intake.stop())

                //set the hood and rpm to a estimated value in case the ll fails
                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_FAR_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_FAR_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPose.position, shotPose.heading)//go to shoot pose
                .stopAndAdd(autoAimCommand)

                .stopAndAdd(shoot3Command)
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.openGate)

                //CORNER GATE DUMPS--------------------------------
                .strafeToSplineHeading(new Vector2d(64, 45), Math.toRadians(90))//line up for HP zone balls
                .stopAndAdd(robot.intake.intake())
                .strafeToConstantHeading(new Vector2d(64, 60))
                .strafeToSplineHeading(new Vector2d(60, 59.5), Math.toRadians(120))
                .strafeToConstantHeading(new Vector2d(40,59.5), new TranslationalVelConstraint(10))
                .strafeToSplineHeading(new Vector2d(60, 59.5), Math.toRadians(90), new TranslationalVelConstraint(10))//line up for HP zone balls

                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_FAR_ESTIMATE))//set hood angle to an estimate
                .stopAndAdd(robot.shooter.spin(RPM_FAR_ESTIMATE))//set RPM to an estimate so it can start spinning up

                .strafeToLinearHeading(shotPose.position, shotPose.heading)//go to shoot pose
                .stopAndAdd(autoAimCommand)//start auto aiming after 1 second

                .stopAndAdd(shoot3Command)//shoot 3
                .stopAndAdd(robot.shooter.stop())//stop flywheel

                .stopAndAdd(robot.gate.closeGate)//close gate


                .stopAndAdd(robot.turret.homeTurret())//for teleop in case we need to home then so motor knows its pose
                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}