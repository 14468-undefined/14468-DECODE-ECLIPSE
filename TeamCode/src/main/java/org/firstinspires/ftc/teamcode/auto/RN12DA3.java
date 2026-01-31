package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.command.Shoot3Command;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.TurretSubsystem;
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
@Autonomous(name = "RN12DA3")
public class RN12DA3 extends NextFTCOpMode {

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

    double SHOOTING_DELAY = 3;//seconds
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RN12DA3() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private Command autoAimWithPID() {
        return new Command() {

            private static final double TX_TOLERANCE = 4.0;

            // Auto-specific PID constants
            private final double AUTO_kP = 0.03;
            private final double AUTO_kI = 0;//.002
            private final double AUTO_kD = 0;//.002

            // Store original PID constants to restore after auto
            private double original_kP;
            private double original_kI;
            private double original_kD;

            {
                requires(BaseRobot.INSTANCE.turret, BaseRobot.INSTANCE.limelight);
            }

            @Override
            public void start() {
                // Save current PID constants
                original_kP = BaseRobot.INSTANCE.turret.kP;
                original_kI = BaseRobot.INSTANCE.turret.kI;
                original_kD = BaseRobot.INSTANCE.turret.kD;

                // Override with auto PID constants
                BaseRobot.INSTANCE.turret.kP = AUTO_kP;
                BaseRobot.INSTANCE.turret.kI = AUTO_kI;
                BaseRobot.INSTANCE.turret.kD = AUTO_kD;

                // Reset PID state
                BaseRobot.INSTANCE.turret.integralSum = 0.0;
                BaseRobot.INSTANCE.turret.lastError = 0.0;
                BaseRobot.INSTANCE.turret.lastTime = System.nanoTime() / 1e9;

                // Set turret mode
                BaseRobot.INSTANCE.turret.mode = TurretSubsystem.TurretMode.VISION;
            }

            @Override
            public void update() {
                if (!BaseRobot.INSTANCE.limelight.hasTarget()) {
                    BaseRobot.INSTANCE.turret.stopTurret();
                    return;
                }

                // Fetch TX in real time
                double tx = BaseRobot.INSTANCE.limelight.getTx();
                double power = BaseRobot.INSTANCE.turret.visionPID(tx); // PID calculation
                BaseRobot.INSTANCE.turret.turretMotor.setPower(power);
            }

            @Override
            public boolean isDone() {
                if (!BaseRobot.INSTANCE.limelight.hasTarget()) return false;
                return Math.abs(BaseRobot.INSTANCE.limelight.getTx()) < TX_TOLERANCE;
            }

            @Override
            public void stop(boolean interrupted) {
                BaseRobot.INSTANCE.turret.stopTurret();

                // Restore original PID constants
                BaseRobot.INSTANCE.turret.kP = original_kP;
                BaseRobot.INSTANCE.turret.kI = original_kI;
                BaseRobot.INSTANCE.turret.kD = original_kD;
            }
        };
    }









    @Override
    public void onInit(){
        robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE);
        //robot.limelight.initHardware(hwMap, "RED");
        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hardwareMap, startPose);

        autoAimCommand = new AutoAimCommand(robot);
        autoCommand = drive.commandBuilder(startPose)
                .stopAndAdd(robot.turret.runToAngle(-45))
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)
                //.stopAndAdd(autoAimCommand)
                //.stopAndAdd(shoot3Command)
                .stopAndAdd(robot.gate.closeGate)

                .stopAndAdd(autoAimWithPID())
                .stopAndAdd(robot.gate.openGate)
                .stopAndAdd(robot.intake.intake())
                .stopAndAdd(robot.shooter.spin())

                .waitSeconds(SHOOTING_DELAY)//WAIT

                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)


                //.stopAndAdd(robot.turret.runToAngle(0))

                //SECOND PILE --------------------------------------
                .strafeToConstantHeading(new Vector2d(8, 29))//line up intake



                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(8, 61), new TranslationalVelConstraint(15))//intake
                .strafeToConstantHeading(new Vector2d(8, 48))//back up

                .stopAndAdd(robot.intake.stop())


                //gate dump
                //.strafeToConstantHeading(new Vector2d(-3, 48))
                //.strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump

                .stopAndAdd(robot.gate.openGate)
                //.stopAndAdd(robot.hood.setHoodAngle(HOOD_ANGLE_CLOSE_ESTIMATE))
                //.stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))

                .stopAndAdd(robot.shooter.spin())
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                //.stopAndAdd(robot.turret.runToAngle(-45))
                .stopAndAdd(autoAimWithPID())
                //.stopAndAdd(robot.turret.runToAngle(0))


                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY)
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.intake.stop())
                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())

                .stopAndAdd(robot.gate.closeGate)//close gate

                //GATE INTAKE --------------------------------------
                .strafeToLinearHeading(new Vector2d(8, 48), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(7, 62))//gate dump

                .strafeToConstantHeading(new Vector2d(16, 66))
                //.strafeToConstantHeading(new Vector2d(13, 60))
                .stopAndAdd(robot.intake.intake())//start intaking
                .waitSeconds(3)

                .strafeToConstantHeading(new Vector2d(15, 40))
                .stopAndAdd(robot.intake.stop())



                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                //.stopAndAdd(robot.turret.runToAngle(-45))
                .stopAndAdd(autoAimWithPID())
                //.stopAndAdd(robot.turret.runToAngle(0))


                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY)
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.intake.stop())
                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())

                .stopAndAdd(robot.gate.closeGate)//close gate


                //FIRST PILE----------------------------------------------
                .strafeToConstantHeading(new Vector2d(-15, 25.4))


                .stopAndAdd(robot.intake.intake())

                .strafeToConstantHeading(new Vector2d(-15, 54.5), new TranslationalVelConstraint(15))//intake
                .strafeToConstantHeading(new Vector2d(-15, 48), new TranslationalVelConstraint(15))//intake

                .stopAndAdd(robot.intake.stop())



                //set the hood and rpm to a estimated value in case the ll fails

                //.stopAndAdd(robot.shooter.spin(RPM_CLOSE_ESTIMATE))//start spinning flywheel

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                //.stopAndAdd(robot.turret.runToAngle(-45))
                .stopAndAdd(autoAimWithPID())
                //.stopAndAdd(robot.turret.runToAngle(0))


                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY)
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.intake.stop())
                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())

                .stopAndAdd(robot.gate.closeGate)//close gate


                //PILE 3
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(35.8, 29))//go to motif
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake
                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.shooter.spin())

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .stopAndAdd(autoAimWithPID())

                //.stopAndAdd(shoot3Command)
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)

                .build();


    }
    @Override
    public void onStartButtonPressed() {
        robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE);
        autoCommand.schedule();


    }

    @Override
    public void onStop(){
        robot.shooter.stop();
        robot.intake.stop();
    }


}