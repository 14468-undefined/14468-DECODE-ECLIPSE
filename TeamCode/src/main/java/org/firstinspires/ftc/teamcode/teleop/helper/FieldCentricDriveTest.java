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
import dev.nextftc.hardware.driving.RobotCentric;
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


@TeleOp(name = "FieldCentricDriveTest" , group = "BB - Helper")

public class FieldCentricDriveTest extends NextFTCOpMode {

    MecanumDrive drive;


    {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }






    ColorfulTelemetry t;



    @Override
    public void onInit() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

    }

    @Override public void onWaitForStart() {
        t.addLine("OpMode Initialized");
        t.addLine("Waiting for start...");
    }
    @Override public void onStartButtonPressed() {

        t.addLine("field centric driving");

        Command FieldCentricDrive = drive.driverControlledCommand(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false

        );
        FieldCentricDrive.schedule();




    }

    @Override
    public void onUpdate() {

        //BindingManager.update(); // update button states

        /*g1LeftX = Gamepads.gamepad1().leftStickX().get();
        g1LeftY = Gamepads.gamepad1().leftStickY().get();
        g1RightX = Gamepads.gamepad1().rightStickX().get();

         */


        //BaseRobot.INSTANCE.periodic(); // update all subsystems - dont need to bc addComponents should do this automatically


        ///robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1LeftY, -g1LeftX), -g1RightX));
        //TODO: field or robot centric?
        //robot.drive.driveFieldcentric(g1LeftY, g1LeftX, -g1RightX, 1);


        /*if(robot.shooter.isAtTargetSpeed() && robot.shooter.getTargetRPM() > 0){//if at target speed which is > 0
            robot.LED.setColor(LEDSubsystem.LEDColor.GREEN);
        }
        else if (robot.shooter.getTargetRPM() > 0 && !robot.shooter.isAtTargetSpeed()){//if trying to spin but not up to speed
            robot.LED.setColor(LEDSubsystem.LEDColor.RED);
        }
        else {//if target RPM is 0
            robot.LED.setColor(LEDSubsystem.LEDColor.OFF);
        }

         */
    }
    @Override public void onStop() {
        //BindingManager.reset();
    }
}
