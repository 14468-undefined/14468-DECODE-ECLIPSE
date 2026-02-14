package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
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
@Autonomous(name = "BNSUPERDUPER")
public class BNSUPERDUPERDUPERSPEED extends NextFTCOpMode {

    private final Pose2d startPose = new Pose2d(-61, -40, Math.toRadians(180));
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
    public BNSUPERDUPERDUPERSPEED() {


        addComponents(
                new SubsystemComponent(robot)
        );
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
                //BEFORE START----------------------------
                .strafeToLinearHeading(new Vector2d(-13,-16), Math.toRadians(-90))
                .waitSeconds(.1)
                .endTrajectory()
                .strafeToLinearHeading(new Vector2d(12.5,-26.5), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(12.5, -53), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60) )
                .endTrajectory()
                .setReversed(true)
                .splineTo(new Vector2d(-8,-14), Math.toRadians(-225))
                .waitSeconds(.1)

                .endTrajectory()
                .setReversed(false)
                .splineTo(new Vector2d(11.5,-58.75), Math.toRadians(-110), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
                .waitSeconds(1.25)
                .endTrajectory()
                .setReversed(true)

                .splineTo(new Vector2d(-8,-14), Math.toRadians(-225))
                .waitSeconds(.1)

                .endTrajectory()
                .setReversed(false)
                .splineTo(new Vector2d(11.5,-58.75), Math.toRadians(-110), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
                .waitSeconds(1.9)
                .endTrajectory()
                .setReversed(true)

                .splineTo(new Vector2d(-8,-14), Math.toRadians(-225))
                .waitSeconds(.1)

                .endTrajectory()
                .setReversed(false)
                .splineTo(new Vector2d(11.5,-58.75), Math.toRadians(-110), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60,60))
                .waitSeconds(1.9)
                .endTrajectory()
                .setReversed(true)

                .splineTo(new Vector2d(-8,-14), Math.toRadians(-225))
                .waitSeconds(.1)

                .endTrajectory()
                .setReversed(false)
                .strafeToLinearHeading(new Vector2d(-11,-28), Math.toRadians(-90))
                .strafeToConstantHeading(new Vector2d(-11, -53))
                .endTrajectory()
                .strafeToLinearHeading(new Vector2d(-38.5, -17.5), Math.toRadians(-45))

                .waitSeconds(1)
                .endTrajectory()
                .build();



    }
    @Override
    public void onStartButtonPressed() {
        robot.limelight.setPipeline(2);
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