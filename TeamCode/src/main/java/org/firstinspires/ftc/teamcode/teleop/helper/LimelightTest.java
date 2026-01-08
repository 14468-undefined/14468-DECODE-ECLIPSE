package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import java.util.function.DoubleSupplier;


@TeleOp(name = "LimelightTest", group = "BB - helper")
public class LimelightTest extends NextFTCOpMode {

    private boolean limelightStarted = false;
    private Limelight3A limelight;


    private FtcDashboard dash;

    private final BaseRobot robot = BaseRobot.INSTANCE;
    public LimelightTest(){

        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
//        limelight.start();
//        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed(){



    }
    @Override
    public void onUpdate() {



        if (!limelightStarted) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            limelightStarted = true;
        }


        telemetry.addData("LL Running", limelight.isRunning());

        LLResult result = limelight.getLatestResult();
        LLStatus status = limelight.getStatus();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Ty", ty);
                telemetry.addData("Ta", ta);
                telemetry.addData("Tx", tx);

                // only schedule once
                if (!robot.turret.isAiming()) {
                    DoubleSupplier txSupplier = result::getTx;
                    robot.turret.aimWithVision(txSupplier).schedule();
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.addData("CPU", status.getCpu());
                telemetry.addData("Temp", status.getTemp());
                telemetry.addData("RAM", status.getRam());
                telemetry.addData("Pipeline Type", status.getPipelineType());

                // stop turret if no target
                robot.turret.stopTurret();
            }


        telemetry.update();
    }
}
