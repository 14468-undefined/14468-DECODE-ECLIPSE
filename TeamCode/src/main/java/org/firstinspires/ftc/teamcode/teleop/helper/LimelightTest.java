package org.firstinspires.ftc.teamcode.teleop.helper;

import android.renderscript.ScriptGroup;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.robocol.Command;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.LEDSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

import static dev.nextftc.bindings.Bindings.*;

import java.util.function.DoubleSupplier;


@TeleOp(name = "LimelightTest", group = "BB - helper")

public class LimelightTest extends NextFTCOpMode {

    MecanumDrive drive;

    private final BaseRobot robot = BaseRobot.INSTANCE;

    {
        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private double Tx;


    HardwareMap hwMap;


    Limelight3A limelight;





    @Override
    public void onInit() {



        robot.initialize();

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0);//default blue goal i think


        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

    }

    @Override public void onWaitForStart() {
        //t.addLine("OpMode Initialized");
        //t.addLine("Waiting for start...");
    }
    @Override public void onStartButtonPressed() {





    }

    @Override
    public void onUpdate() {
        robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y, -gamepad1.right_stick_x), -gamepad1.right_stick_x));


            LLResult llResult = limelight.getLatestResult();

            if(llResult != null){
                Tx = llResult.getTx();
            }

        telemetry.addData("Tx", Tx);

    }
    @Override public void onStop() {}

}
