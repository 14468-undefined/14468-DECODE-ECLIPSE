package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
@Autonomous(name = "RF6")
public class RF6 extends NextFTCOpMode {
//hunter is my goat
    private final Pose2d startPose = new Pose2d(65, 15, Math.toRadians(90.0));
    private final Pose2d shotPose = new Pose2d(60, 15, Math.toRadians(90));//go to shoot pose


    //private Command autoAimHoldCmd;


    //HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;


    //1,3,5  //40.5, 13.7, heading: -47.5
    //2 //45, 9.5, heading: -45
    //4 //46.3, 6.5, heading: -43.6
    //6 //46, 2.6, heading:-25.6

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;

    double SHOOTING_DELAY = 3;//seconds
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RF6() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private boolean autoAimHoldEnabled = false;


    /*private Command autoAimWithPIDHOLD() {
        return new Command() {

            // Auto-specific PID constants
            private final double AUTO_kP = 0.025;
            private final double AUTO_kI = 0;
            private final double AUTO_kD = 0;

            // Store original PID constants
            private double original_kP;
            private double original_kI;
            private double original_kD;

            // Separate timing state so it doesn't clash
            private double lastTime;

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
                lastTime = System.nanoTime() / 1e9;
                BaseRobot.INSTANCE.turret.lastTime = lastTime;

                // Set turret mode
                BaseRobot.INSTANCE.turret.mode = TurretSubsystem.TurretMode.VISION;
            }

            @Override
            public void update() {
                if (!BaseRobot.INSTANCE.limelight.hasTarget()) {
                    BaseRobot.INSTANCE.turret.stopTurret();
                    return;
                }

                // Continuously PID on TX
                double tx = BaseRobot.INSTANCE.limelight.getTx();
                double power = BaseRobot.INSTANCE.turret.visionPID(tx);
                BaseRobot.INSTANCE.turret.turretMotor.setPower(power);
            }

            @Override
            public boolean isDone() {
                // HOLD command — never finishes unless canceled
                return false;
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

     */
    private Command autoAimWithPIDFirst() {
        return new Command() {

            private static final double TX_TOLERANCE = 6;

            // Auto-specific PID constants
            private final double AUTO_kP = 0.022;
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
                double power = BaseRobot.INSTANCE.turret.visionPID(tx + 20); // PID calculation //was 13
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

    private Command autoAimWithPIDSecond() {
        return new Command() {

            private static final double TX_TOLERANCE = 6;

            // Auto-specific PID constants
            private final double AUTO_kP = 0.022;
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
                double power = BaseRobot.INSTANCE.turret.visionPID(tx - 8); // PID calculation //was 13
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

        //autoAimHoldCmd = autoAimWithPIDHOLD();



        //robot.limelight.initHardware(hwMap, "RED");

        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.FAR_ZONE, 0);//shot time is irrelevant here bc its far zone
        drive = new MecanumDrive(hardwareMap, startPose);
        autoAimCommand = new AutoAimCommand(robot);


        autoCommand = drive.commandBuilder(startPose)

                .stopAndAdd(robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE))
                .stopAndAdd(robot.turret.resetTicks())
                //.stopAndAdd(robot.hood.setHoodPose(.84))
                .stopAndAdd(robot.shooter.setTargetRPM(3200))//was 3150

                .stopAndAdd(robot.shooter.spin())
                .strafeToConstantHeading(new Vector2d(60, 15))
                //.stopAndAdd(robot.turret.runToTicks(-152))
                .stopAndAdd(robot.gate.openGate)

                .stopAndAdd(autoAimWithPIDFirst())

                .waitSeconds(3.5)//was 3.5

                .stopAndAdd(robot.intake.setIntakePower(.7))
                .stopAndAdd(robot.intake.intake())

                .waitSeconds(2)
                .stopAndAdd(autoAimWithPIDFirst())

                .stopAndAdd(robot.intake.setIntakePower(1))
                .waitSeconds(1)


                /*OLD DELAYED SHOOTING SEQUENCE---------
                .waitSeconds(.6)//WAIT
                .stopAndAdd(robot.intake.stop())
                .waitSeconds(.5)
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(.3)

                .stopAndAdd(robot.intake.stop())
                .waitSeconds(1.3)
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(1)
                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.intake.setIntakePower(1))
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)
                //OLD DELAYED SHOOTING SEQUENCE---------
                 */


                //NEW CORNER-----------------------------
                .stopAndAdd(robot.gate.closeGate)
                .waitSeconds(1)
                .stopAndAdd(robot.intake.setIntakePower(1))
                .stopAndAdd(robot.intake.intake())

                .strafeToLinearHeading(new Vector2d(47, 63), Math.toRadians(42.5))//1
                .strafeToLinearHeading(new Vector2d(55.5, 61), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(51.3, 55.5), Math.toRadians(42.5))//1
                .strafeToLinearHeading(new Vector2d(58.5, 61.3), Math.toRadians(46.4))
                .strafeToLinearHeading(new Vector2d(51.3, 55.5), Math.toRadians(42.5))//1
                .strafeToLinearHeading(new Vector2d(63, 63), Math.toRadians(68.4))
                .strafeToLinearHeading(new Vector2d(63, 64), Math.toRadians(90))
               // .stopAndAdd(robot.intake.stop())
                //.stopAndAdd(robot.gate.openGate)
                //CORNER----------------------------------------------
                /*.strafeToSplineHeading(new Vector2d(53, 57), Math.toRadians(70))//line up for HP zone balls
                .stopAndAdd(robot.intake.intake())

                .strafeToSplineHeading(new Vector2d(61, 60), Math.toRadians(60), new TranslationalVelConstraint(5))//line up for HP zone balls
                .strafeToSplineHeading(new Vector2d(65, 60), Math.toRadians(90), new TranslationalVelConstraint(5))//line up for HP zone balls

                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.intake.setIntakePower(.6))

                 */




                //.endTrajectory()
                .strafeToLinearHeading(shotPose.position, shotPose.heading)//go to shoot pose
                //.afterTime(1.2, robot.intake.stop())
                //.afterTime(1.2, robot.gate.openGate)
                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.gate.openGate)
                .stopAndAdd(robot.gate.openGate)

                .stopAndAdd(autoAimWithPIDSecond( ))

                //.waitSeconds(3)

                .waitSeconds(2)
                .stopAndAdd(robot.intake.setIntakePower(.7))
                .stopAndAdd(robot.intake.intake())

                .waitSeconds(1.5)
                //.stopAndAdd(autoAimWithPID())

                .stopAndAdd(robot.intake.setIntakePower(1))
                .waitSeconds(2)

                .stopAndAdd(robot.gate.closeGate)
                .waitSeconds(1)



                .stopAndAdd(robot.shooter.stop())

                /*
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
                .stopAndAdd(autoAimWithPID())

                .waitSeconds(3)

                .stopAndAdd(robot.gate.openGate)
                .stopAndAdd(robot.intake.setIntakePower(.6))
                .stopAndAdd(robot.intake.intake())


                .waitSeconds(SHOOTING_DELAY-2)//WAIT
                .stopAndAdd(robot.intake.stop())
                .waitSeconds(.5)
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(1)

                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.intake.setIntakePower(1))
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)


                 */
                .strafeToLinearHeading(new Vector2d(55, 63), Math.toRadians(90))//1
                .strafeToConstantHeading(new Vector2d(58, 52))//1
                .strafeToConstantHeading(new Vector2d(58, 63))//1
                .strafeToConstantHeading(new Vector2d(58, 52))//1
                .strafeToConstantHeading(new Vector2d(63, 63))//1




                .build();


    }
    @Override
    public void onStartButtonPressed() {
        robot.shooter.setTargetRPM(0);
        robot.shooter.enabled = true;
        robot.turret.resetTicks();
        autoCommand.schedule();


    }

    @Override
    public void onUpdate() {



    }


    @Override
    public void onStop(){
        robot.shooter.enabled = false;
    }

}