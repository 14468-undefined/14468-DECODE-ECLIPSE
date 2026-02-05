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
@Autonomous(name = "RF12ND")
public class RF12ND extends NextFTCOpMode {
//hunter is my goat
    private final Pose2d startPose = new Pose2d(65, 15, Math.toRadians(90.0));
    private final Pose2d shotPose = new Pose2d(60, 15, Math.toRadians(90));//go to shoot pose



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
    public RF12ND() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private Command autoAimWithPID() {
        return new Command() {

            private static final double TX_TOLERANCE = 10;

            // Auto-specific PID constants
            private final double AUTO_kP = 0.02;
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
        //robot.limelight.initHardware(hwMap, "RED");

        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.FAR_ZONE, 0);//shot time is irrelevant here bc its far zone
        drive = new MecanumDrive(hardwareMap, startPose);
        autoAimCommand = new AutoAimCommand(robot);


        autoCommand = drive.commandBuilder(startPose)

                .stopAndAdd(robot.turret.resetTicks())
                .stopAndAdd(robot.hood.setHoodPose(.84))
                .stopAndAdd(robot.shooter.setTargetRPM(3120))
                .stopAndAdd(robot.intake.setIntakePower(.3))
                .stopAndAdd(robot.shooter.spin())
                .strafeToConstantHeading(new Vector2d(60, 15))
                .stopAndAdd(robot.turret.runToTicks(-152))
                .stopAndAdd(robot.gate.openGate)

                .stopAndAdd(autoAimWithPID())

                .waitSeconds(3)
                .stopAndAdd(robot.intake.intake())


                .waitSeconds(SHOOTING_DELAY)//WAIT

                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)


                //CORNER----------------------------------------------
                .strafeToSplineHeading(new Vector2d(53, 57), Math.toRadians(70))//line up for HP zone balls
                .stopAndAdd(robot.intake.intake())
                .strafeToSplineHeading(new Vector2d(61, 60), Math.toRadians(60), new TranslationalVelConstraint(5))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(65, 60), Math.toRadians(90), new TranslationalVelConstraint(5))//line up for HP zone balls

                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.shooter.spin())


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


                .stopAndAdd(robot.shooter.spin())

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


                .stopAndAdd(robot.shooter.spin())

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
        robot.turret.resetTicks();
        autoCommand.schedule();


    }


}