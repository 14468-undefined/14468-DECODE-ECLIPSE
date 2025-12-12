package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;


import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;


public class LimelightSubsystem implements Subsystem {

    public Limelight3A limelight;

    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;

    public IMU imu;

    DriveSubsystem drive;
    private static double ROT_TOLERANCE = 2;//in degrees


    public LimelightSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8);//PICK WHICH PIPELINE  (theres a coach pratt vid)

        /*IMU imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();//gets heading from imu
        limelight.updateRobotOrientation(orientation.getYaw());//tells limelight orientation
        LLResult llResult = limelight.getLatestResult();//pulls data from limelight

       /* if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();//llResult.getBotpose (without IMU)
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());

        }

        */



    }

    public Command initializeLimelight() {
        return new LambdaCommand()
                .setStart(() -> limelight.start())
                .setIsDone(() -> true)
                .requires(this)
                .named("Initialize Limelight");
    }

    public Command stopLimelight() {
        return new LambdaCommand()
                .setStart(() -> limelight.stop())
                .setIsDone(() -> true)
                .requires(this)
                .named("Stop Limelight");
    }


    public double getTx(){

        if(!limelight.isRunning()){
            limelight.start();
        }

        LLResult llResult = limelight.getLatestResult();//pulls data from limelight
        Pose3D botPose = llResult.getBotpose();

        double Tx = llResult.getTx();
        return Tx;

    }

    @Override
    public void initialize() {
        limelight.start();//TODO: should this go here?

    }

















    public void printTelemetry(ColorfulTelemetry t){
      t.addLine();
      t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
