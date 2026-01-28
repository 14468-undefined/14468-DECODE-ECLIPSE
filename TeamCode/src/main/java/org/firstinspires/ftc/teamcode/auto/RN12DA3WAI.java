package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

//WAI = With auto aim
@Autonomous(name = "RN12DA3WAI")
public class RN12DA3WAI extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(-61, 40, Math.toRadians(180));
    private final Pose2d shotPoseOnLine = new Pose2d(-24,24, Math.toRadians(90));//go shoot


    //HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RN12DA3WAI() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private Command autoAim() {
        return new SequentialGroup(
                // Aim turret using Limelight, then continue
                new Command() {

                    private static final double TX_TOLERANCE = 1.0;

                    {
                        requires(
                                BaseRobot.INSTANCE.turret,
                                BaseRobot.INSTANCE.limelight
                        );
                    }

                    @Override
                    public void update() {
                        if (!BaseRobot.INSTANCE.limelight.hasTarget()) {
                            return;
                        }

                        BaseRobot.INSTANCE.turret
                                .aimWithVision(BaseRobot.INSTANCE.limelight::getTx);
                    }

                    @Override
                    public boolean isDone() {
                        if (!BaseRobot.INSTANCE.limelight.hasTarget()) return false;
                        return Math.abs(BaseRobot.INSTANCE.limelight.getTx()) < TX_TOLERANCE;
                    }

                    @Override
                    public void stop(boolean interrupted) {

                    }
                }
        );
    }




    @Override
    public void onInit(){

        //robot.limelight.initHardware(hwMap, "RED");
        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hardwareMap, startPose);

        autoAimCommand = new AutoAimCommand(robot);
        autoCommand = drive.commandBuilder(startPose)
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                //.stopAndAdd(autoAimCommand)
                //.stopAndAdd(shoot3Command)
                .stopAndAdd(robot.gate.closeGate)

                .stopAndAdd(autoAim())

                //FIRST PILE----------------------------------------------
                .strafeToConstantHeading(new Vector2d(-11.2, 25.4))


                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(-13, 54.5), new TranslationalVelConstraint(15))//intake
                .stopAndAdd(robot.intake.stop())

                //gate dump
                .strafeToConstantHeading(new Vector2d(-3, 48))
                .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump

                .stopAndAdd(robot.gate.openGate)
                //.stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                //.stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .stopAndAdd(autoAim())

                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())

                .stopAndAdd(robot.gate.closeGate)//close gate

                //SECOND PILE --------------------------------------
                .strafeToConstantHeading(new Vector2d(10, 29))//line up intake

                .stopAndAdd(robot.intake.intake())
                .strafeToConstantHeading(new Vector2d(10, 61), new TranslationalVelConstraint(15))//intake
                .strafeToConstantHeading(new Vector2d(10, 48))//back up

                .stopAndAdd(robot.intake.stop())


                //set the hood and rpm to a estimated value in case the ll fails
                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                //.stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                //.stopAndAdd(autoCommand)

                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)


                //PILE 3
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(35.8, 29))//go to motif
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake
                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                .stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                //.stopAndAdd(autoCommand)

                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)

                .build();


    }
    @Override
    public void onStartButtonPressed() {
        autoCommand.schedule();

    }


}