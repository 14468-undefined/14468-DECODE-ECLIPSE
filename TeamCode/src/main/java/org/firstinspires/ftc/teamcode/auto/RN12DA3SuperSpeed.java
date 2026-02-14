package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
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

import java.util.function.DoubleSupplier;


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
@Autonomous(name = "RN12DA3SuperSpeed")
public class RN12DA3SuperSpeed extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(-61, 40, Math.toRadians(180));
    //private final Pose2d shotPoseOnLine = new Pose2d(-14,14, Math.toRadians(90));//go shoot
    //private final Pose2d shotPoseOnLine = new Pose2d(-2.55,8.5, Math.toRadians(90));//go shoot
    private final Pose2d shotPoseOnLine = new Pose2d(-29,23, Math.toRadians(90));//go shoot

    //-29, 23
    //-2.55, 7.10

    //HardwareMap hwMap;
    MecanumDrive drive;
    Command driveCommand;
    Command autoCommand;
    Command autoAimCommand;

    Shoot3Command shoot3Command;

    private Limelight3A limelight;
    private boolean limelightStarted = false;

    private double voltage;

    double HOOD_ANGLE_CLOSE_ESTIMATE = 0;
    double RPM_CLOSE_ESTIMATE = 0;


    private final ElapsedTime autoTimer = new ElapsedTime();
    private static final double AUTO_AIM_DELAY = 3.5; // seconds


    double SHOOTING_DELAY = 3;//seconds
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public RN12DA3SuperSpeed() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private Command autoAimWithPID() {
        return new Command() {

            private static final double TX_TOLERANCE = 8;

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

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();



        robot.limelight.setPipeline(1);
        //robot.limelight.initHardware(hwMap, "RED");
        shoot3Command = new Shoot3Command(robot, Constants.FieldConstants.CLOSE_SHOT, 3);
        drive = new MecanumDrive(hardwareMap, startPose);

        autoTimer.reset();


        autoAimCommand = new AutoAimCommand(robot);
        autoCommand = drive.commandBuilder(startPose)
                .stopAndAdd(robot.intake.setIntakePower(1))

                .stopAndAdd(robot.shooter.setTargetRPM(3120))
                .stopAndAdd(robot.hood.setHoodPose(.6))
                .stopAndAdd(robot.shooter.spin())


                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading, new TranslationalVelConstraint(100))//go to shoot pose

                .stopAndAdd(robot.gate.openGate)

                //.stopAndAdd(autoAimWithPID())
                .waitSeconds(.3)

                .stopAndAdd(robot.intake.intake())


                .waitSeconds(SHOOTING_DELAY-1.2)//WAIT

                .stopAndAdd(robot.intake.stop())
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)

                .stopAndAdd(robot.shooter.setTargetRPM(2825))





                .stopAndAdd(robot.intake.setIntakePower(1))

                //FIRST PILE----------------------------------------------
                //.strafeToConstantHeading(new Vector2d(-13, 25.4))


                .stopAndAdd(robot.intake.intake())
                .strafeToConstantHeading(new Vector2d(-13, 30), new TranslationalVelConstraint(100))//intake

                .strafeToConstantHeading(new Vector2d(-13, 54.5), new TranslationalVelConstraint(100))//intake
                .stopAndAdd(robot.shooter.spin())
                .strafeToConstantHeading(new Vector2d(-13, 48), new TranslationalVelConstraint(100))//intake

                .stopAndAdd(robot.intake.stop())



                .stopAndAdd(robot.gate.openGate)



                .stopAndAdd(robot.intake.setIntakePower(1))
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading, new TranslationalVelConstraint(100))//go to shoot pose

                //.stopAndAdd(robot.shooter.setTargetRPM(3350))

                .stopAndAdd(robot.intake.intake())
                //.stopAndAdd(autoAimWithPID())




                .waitSeconds(SHOOTING_DELAY-1.3)
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.intake.stop())


                .stopAndAdd(robot.gate.closeGate)//close gate


                .stopAndAdd(robot.intake.setIntakePower(1))
                //SECOND PILE --------------------------------------
                .strafeToConstantHeading(new Vector2d(11, 24.5), new TranslationalVelConstraint(100))//line up intake



                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(11, 61), new TranslationalVelConstraint(100))//intake
                .strafeToConstantHeading(new Vector2d(11, 48), new TranslationalVelConstraint(100))//back up

                .stopAndAdd(robot.intake.stop())




                .stopAndAdd(robot.gate.openGate)


                .stopAndAdd(robot.shooter.spin())

                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose
                .stopAndAdd(robot.intake.setIntakePower(1))

                .stopAndAdd(robot.intake.intake())
                .stopAndAdd(autoAimWithPID())


                .waitSeconds(SHOOTING_DELAY-1.5)
                //.stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.intake.stop())


                .stopAndAdd(robot.gate.closeGate)//close gate


                //PILE 3
                .stopAndAdd(robot.intake.intake())//start intaking
                .strafeToConstantHeading(new Vector2d(33, 24),new TranslationalVelConstraint(100))//go to motif
                .strafeToConstantHeading(new Vector2d(33, 61), new TranslationalVelConstraint(100))//intake
                .stopAndAdd(robot.intake.stop())

                .stopAndAdd(robot.shooter.spin())


                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading, new TranslationalVelConstraint(100))//go to shoot pose
                .stopAndAdd(robot.gate.openGate)

                .stopAndAdd(autoAimWithPID())
                .waitSeconds(.5)

                .stopAndAdd(robot.intake.setIntakePower(.84))
                .stopAndAdd(robot.intake.intake())


                .waitSeconds(SHOOTING_DELAY-1.2)//WAIT

                .stopAndAdd(robot.intake.stop())
                .stopAndAdd(robot.shooter.stop())
                .stopAndAdd(robot.gate.closeGate)

                .build();


    }
    @Override
    public void onStartButtonPressed() {
        robot.limelight.setPipeline(1);
        autoCommand.schedule();


    }

    @Override
    public void onUpdate(){
        //TODO: maybe add the voltage stuff here
        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        //robot.shooter.voltageCompensate(voltage);
        //robot.shooter.maybeUpdatePIDF();
        telemetry.addData("VOLTAGE", voltage);



        if (!limelightStarted) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE);
            limelight.start();
            limelightStarted = true;
        }



        telemetry.addData("LL Running", limelight.isRunning());

        if (autoTimer.seconds() > 3){
            LLResult result = limelight.getLatestResult();

            DoubleSupplier txSupplier = () -> {
                LLResult r = limelight.getLatestResult();
                return (r != null && r.isValid()) ? r.getTx() : 0.0;
            };
            LLStatus status = limelight.getStatus();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                //telemetry.addData("Ty", ty);
                //telemetry.addData("Ta", ta);
                //telemetry.addData("Tx", tx);

                // only schedule once
                if (!robot.turret.isAiming()) {
                    robot.turret.aimWithVision(txSupplier).schedule();
                }
            } else {

                if(gamepad1.right_stick_button){
                    robot.turret.runToTicks(0).schedule();

                }
            /*telemetry.addData("Limelight", "No Targets");
            telemetry.addData("CPU", status.getCpu());
            telemetry.addData("Temp", status.getTemp());
            telemetry.addData("RAM", status.getRam());
            telemetry.addData("Pipeline Type", status.getPipelineType());

             */

                // stop turret if no target
                //robot.turret.stopTurret(); /TODO
            }
        }

        telemetry.update();
    }
    @Override
    public void onStop(){
        robot.shooter.stop();
        robot.intake.stop();
    }


}