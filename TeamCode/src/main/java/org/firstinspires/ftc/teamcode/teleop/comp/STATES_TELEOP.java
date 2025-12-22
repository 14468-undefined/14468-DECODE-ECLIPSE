package org.firstinspires.ftc.teamcode.teleop.comp;

import android.renderscript.ScriptGroup;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.Command;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import static dev.nextftc.bindings.Bindings.*;

import java.util.function.DoubleSupplier;


@TeleOp(name = "STATES_TELEOP" , group = "AA - COMP")

public class STATES_TELEOP extends NextFTCOpMode {

    MecanumDrive drive;
    BaseRobot robot;
    {
        addComponents(/* vararg components */);
    }





    @Override public void onInit() {

        robot = new BaseRobot(hardwareMap, new Pose2d(0,0,0));
        robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));



    }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {

        DoubleSupplier TxSupplier = () -> robot.limelight.getTx();
        DoubleSupplier TySupplier = () -> robot.limelight.getDistance();


        AutoAimCommand AutoAimCommand = new AutoAimCommand(robot, TySupplier, TxSupplier);

        AutoAimCommand.schedule();




        Button gamepad1a = button(() -> gamepad1.a);


        Button g1RB = button(() -> gamepad1.right_bumper)
                .whenBecomesTrue(robot.turret.runToAngle(0));
        //TODO: this needs to be a new state that overrides all the other turret movements. it also needs to have rpm and hood

        Button g2x = button(() -> gamepad2.x)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.intake.stop());



        //.whenTrue: every loop when the button is true.
        //.whenFalse: every loop when the button is false.
        //.whenBecomesTrue: the first loop when the button is true, a.k.a. the rising edge.
        //.whenBecomesFalse: the first loop when the button is false, a.k.a. the falling edge.

        //button(() -> gamepad1.a)
        //.whenBecomesTrue(() -> runSomeCode())
        //.whenBecomesFalse(() -> runSomeMoreCode());

        //Button andButton = button1.and(button2); // true when both buttons are true
        //Button orButton = button1.or(button2); // true when at least one button is true
        //Button xorButton = button1.xor(button2); // true when exactly one button is true
        //Button notButton = button1.not(); // true when button1 is false



        /// READ THIS ABOUT LAYERS IN TELEOP https://nextftc.dev/bindings/buttons //i.e. endgame layer
    }
    @Override public void onUpdate() {



        robot.periodic();

        BindingManager.update();

        if(gamepad1.right_trigger>.1){
            //TODO - set shooter rpm
        }
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}
