package org.firstinspires.ftc.teamcode.teleop.helper;

import android.renderscript.ScriptGroup;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.robocol.Command;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.subsystems.SubsystemGroup;
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


@TeleOp(name = "IntakeTeleop", group = "BB - helper")

public class IntakeTeleop extends NextFTCOpMode {

    MecanumDrive drive;

    private final BaseRobot robot = BaseRobot.INSTANCE;

    {
        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }



    HardwareMap hwMap;

    ColorfulTelemetry t;





    @Override
    public void onInit() {



        robot.initialize();



        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

    }

    @Override public void onWaitForStart() {
        //t.addLine("OpMode Initialized");
        //t.addLine("Waiting for start...");
    }
    @Override public void onStartButtonPressed() {




        Gamepads.gamepad2().x()
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesTrue(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad2().b()
                .whenBecomesTrue(robot.intake.intakeReverse())
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad1().x()
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesTrue(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad1().b()
                .whenBecomesTrue(robot.intake.intakeReverse())
                .whenBecomesFalse(robot.intake.stop());


        Gamepads.gamepad2().rightTrigger().atLeast(.1)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad2().leftTrigger().atLeast(.1)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.intake.stop());


        Gamepads.gamepad1().rightTrigger().atLeast(.1)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad1().leftTrigger().atLeast(.1)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.intake.stop());


    }

    @Override
    public void onUpdate() {
        robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y, -gamepad1.right_stick_x), -gamepad1.right_stick_x));


    }
    @Override public void onStop() {}

}
