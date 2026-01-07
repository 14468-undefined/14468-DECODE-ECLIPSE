package org.firstinspires.ftc.teamcode.teleop.helper;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
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
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            DoubleSupplier txSupplier = result::getTx;

            robot.turret.aimWithVision(txSupplier);

        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        telemetry.update();
    }
}
