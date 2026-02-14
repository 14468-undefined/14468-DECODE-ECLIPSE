package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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




//WAI = With auto aim
@Autonomous(name = "autototestifonupdateworks")
public class autoToTestIfOnUpdateWorks extends NextFTCOpMode {

    private double voltage;

    Command autoCommand;


    private Limelight3A limelight;
    private boolean limelightStarted = false;

    private final BaseRobot robot = BaseRobot.INSTANCE;
    public autoToTestIfOnUpdateWorks() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }




    @Override
    public void onInit(){

        double voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();


        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");




    }
    @Override
    public void onStartButtonPressed() {


    }

    @Override
    public void onUpdate(){
        //TODO: maybe add the voltage stuff here

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        robot.shooter.voltageCompensate(voltage);
        //robot.shooter.maybeUpdatePIDF();
        telemetry.addData("VOLTAGE", voltage);



        if (!limelightStarted) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE);
            limelight.start();
            limelightStarted = true;
        }


            telemetry.addData("LL Running", limelight.isRunning());

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
        telemetry.update();

    }
    @Override
    public void onStop(){
        robot.shooter.stop();
        robot.intake.stop();
    }


}