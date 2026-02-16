package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
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
 * First: B/R - Blue/Red alliances
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
@Autonomous(name = "BN12DA3DA6New")
public class BN12DA3DA6NEW extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(-61, -40, Math.toRadians(180));
    //private final Pose2d shotPoseOnLine = new Pose2d(-14,14, Math.toRadians(90));//go shoot
    //private final Pose2d shotPoseOnLine = new Pose2d(-2.55,8.5, Math.toRadians(90));//go shoot
    private final Pose2d shotPoseOnLine = new Pose2d(-29,-23, Math.toRadians(270));//go shoot

    private final Pose2d shotPoseOffLine = new Pose2d(-40, -16, Math.toRadians(284));
    //-29, 23
    //-2.55, 7.10

    //HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;
    private ControlSystem controller;



    private Limelight3A limelight;
    private boolean limelightStarted = false;

    private double voltage;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;


    private final ElapsedTime autoTimer = new ElapsedTime();
    private static final double AUTO_AIM_DELAY = 3.5; // seconds


    double SHOOTING_DELAY = 3;//seconds
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public BN12DA3DA6NEW() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private Command autoAimWithPID() {
        return new Command() {

            private static final double TX_TOLERANCE = 8;

            // Auto-specific PID constants
            private final double AUTO_kP = 0.023;
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

        robot.shooter.stop();
        controller = ControlSystem.builder()
                .posPid(.01, 0, 0)
                .build();


        //robot.turret.bypassPeriodic = true;
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        robot.turret.turretMotor.setCurrentPosition(0);


        robot.limelight.setPipeline(0).schedule();
        //robot.limelight.initHardware(hwMap, "RED");
        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hardwareMap, startPose);

        autoTimer.reset();


        autoAimCommand = new AutoAimCommand(robot);
        autoCommand = drive.commandBuilder(startPose)
                //BEFORE START----------------------------

                .stopAndAdd(robot.intake.setIntakePower(1))
                .stopAndAdd(robot.shooter.setTargetRPM(2400))
                //.stopAndAdd(robot.hood.setHoodPose(.6))
                .stopAndAdd(robot.gate.openGate)
                .stopAndAdd(robot.shooter.spin())
                //BEFORE START----------------------------

                //DRIVE TO SHOT POSE-----------------------
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading, new TranslationalVelConstraint(100))//go to shoot pose
                //DRIVE TO SHOT POSE-----------------------

                .stopAndAdd(autoAimWithPID())

                //SHOT SEQUENCE------------------------------
                .waitSeconds(.3)
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY-1.3)//WAIT
                .stopAndAdd(robot.intake.stop())
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)
                .stopAndAdd(robot.shooter.setTargetRPM(2390))
                //SHOT SEQUENCE------------------------------


                //INTAKE FIRST PILE--------------------------
                .stopAndAdd(robot.intake.setIntakePower(1))
                .stopAndAdd(robot.intake.intake())
                .strafeToConstantHeading(new Vector2d(-13,  -20), new TranslationalVelConstraint(100))//intake
                .strafeToConstantHeading(new Vector2d(-13, -47), new TranslationalVelConstraint(100),  new ProfileAccelConstraint(-100,100))//intake
                .stopAndAdd(robot.shooter.spin())
                //.strafeToConstantHeading(new Vector2d(-13, 48), new TranslationalVelConstraint(100))//intake
                //.strafeToLinearHeading(new Vector2d(0, 59), n 180)//intake
                .strafeToConstantHeading(new Vector2d(0, -48), new TranslationalVelConstraint(100))//intake
                .strafeToConstantHeading(new Vector2d(0, -59), new TranslationalVelConstraint(100))//intake
                .stopAndAdd(robot.intake.stop())

                //.strafeToConstantHeading(new Vector2d(-13, 48), new TranslationalVelConstraint(100))//intake

                //INTAKE FIRST PILE--------------------------


                //SHOT SEQUENCE------------------------------
                .stopAndAdd(robot.gate.openGate)
                .stopAndAdd(robot.intake.setIntakePower(1))
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading, new TranslationalVelConstraint(100),  new ProfileAccelConstraint(-100,100))//go to shoot pose
                .stopAndAdd(autoAimWithPID())
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY-1.9)
                .stopAndAdd(robot.intake.stop())
                //SHOT SEQUENCE------------------------------



                //INTAKE SECOND PILE--------------------------
                .stopAndAdd(robot.gate.closeGate)//close gate
                .stopAndAdd(robot.intake.setIntakePower(1))
                .strafeToConstantHeading(new Vector2d(11, -20), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))//line up intake
                .stopAndAdd(robot.intake.intake())//start intaking
                //.strafeToConstantHeading(new Vector2d(11, 61), new TranslationalVelConstraint(100))//intake
                .strafeToConstantHeading(new Vector2d(11, -56), new TranslationalVelConstraint(100),  new ProfileAccelConstraint(-40, 40))//back up
                .strafeToConstantHeading(new Vector2d(11, -50), new TranslationalVelConstraint(100))//back up


                .stopAndAdd(robot.intake.stop())

                .strafeToConstantHeading(new Vector2d(0, -48), new TranslationalVelConstraint(100))//intake
                .strafeToConstantHeading(new Vector2d(0, -59), new TranslationalVelConstraint(100))//intake

                //INTAKE FIRST PILE--------------------------



                //SHOT SEQUENCE------------------------------
                .stopAndAdd(robot.gate.openGate)
                .stopAndAdd(robot.shooter.spin())
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading,new TranslationalVelConstraint(100),new ProfileAccelConstraint(-100,100))//go to shoot pose


                .stopAndAdd(autoAimWithPID())

                .stopAndAdd(robot.intake.setIntakePower(1))
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY-1.5)
                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.gate.closeGate)//close gate
                //SHOT SEQUENCE------------------------------


                //INTAKE THIRD PILE--------------------------
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(33, -16), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100,100))//go to motif
                .strafeToConstantHeading(new Vector2d(33, -54), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-40, 40))//back up
                .strafeToConstantHeading(new Vector2d(33, -45), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-100, 100))//intake
                //.strafeToConstantHeading((new Vector2d(33, 10)), new TranslationalVelConstraint(100))//go to motif
                //.splineToConstantHeading(new Vector2d(38, 59), Math.PI / 2, new TranslationalVelConstraint(100))
                //.splineToConstantHeading(new Vector2d(35, 55), -Math.PI / 2, new TranslationalVelConstraint(100))

                .stopAndAdd(robot.intake.stop())
                //INTAKE THIRD PILE--------------------------


                //SHOT SEQUENCE------------------------------
                .stopAndAdd(robot.shooter.spin())
                .strafeToLinearHeading(shotPoseOffLine.position, shotPoseOffLine.heading, new TranslationalVelConstraint(100) )//go to shoot pose
                //.strafeToLinearHeading(new Vector2d(-36.7, 16), 90, new TranslationalVelConstraint(100))//intake

                .stopAndAdd(autoAimWithPID())

                .stopAndAdd(robot.gate.openGate)
                .waitSeconds(.5)
                .stopAndAdd(robot.intake.setIntakePower(.84))
                .stopAndAdd(robot.intake.intake())
                .waitSeconds(SHOOTING_DELAY-.8)//WAIT
                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)
                //SHOT SEQUENCE------------------------------

                //.strafeToConstantHeading(new Vector2d(0,  23))

                .build();


    }
    @Override
    public void onStartButtonPressed() {
        robot.limelight.setPipeline(0).schedule();
        autoCommand.schedule();


    }

    @Override
    public void onUpdate(){



        //controller.setGoal(new KineticState(-90));

        //double power = controller.calculate(robot.turret.turretMotor.getState());

        //robot.turret.turretMotor.setPower(power);



        //-------------------------------
        //TODO: maybe add the voltage stuff here
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        //robot.shooter.voltageCompensate(voltage);
        //robot.shooter.maybeUpdatePIDF();
        telemetry.addData("VOLTAGE", voltage);

        //robot.turret.runToTicks(-56).schedule();




        telemetry.update();
    }
    @Override
    public void onStop(){
        robot.shooter.stop();
        robot.intake.stop();
    }


}