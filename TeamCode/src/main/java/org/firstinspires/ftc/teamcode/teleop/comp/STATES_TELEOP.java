package org.firstinspires.ftc.teamcode.teleop.comp;

import android.renderscript.ScriptGroup;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robocol.Command;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.components.Component;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import static dev.nextftc.bindings.Bindings.*;

import java.util.function.DoubleSupplier;


@TeleOp(name = "STATES_TELEOP" , group = "AA - COMP")

public class STATES_TELEOP extends NextFTCOpMode {

    MecanumDrive drive;

    private final BaseRobot robot = BaseRobot.INSTANCE;

    {
        addComponents(
                new SubsystemComponent(robot)
        );
    }


    private double HOOD_ANGLE_FAR = 0;
    double g1LeftX;
    double g1LeftY;
    double g1RightX;




    String llWorking = "Limelight Working";
    String llNotWorking = "Limelight Not Working";


    @Override
    public void onInit() {
        robot.initialize();


        BindingManager.setLayer(llWorking);
    }

    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {





        AutoAimCommand autoAimCommand = new AutoAimCommand(robot);





        Button g1RB = button(() -> gamepad1.right_bumper).or(button(() -> gamepad1.left_bumper))
                .whenBecomesTrue(() -> BindingManager.setLayer(llWorking))
                .whenBecomesTrue(robot.turret.homeTurret())
                .whenBecomesTrue(robot.hood.setHoodAngle(HOOD_ANGLE_FAR));


        Button g1A = button(() -> gamepad1.right_bumper)
                .inLayer(llNotWorking)
                .whenBecomesTrue(() -> BindingManager.setLayer(llWorking))
                .whenBecomesTrue(robot.turret.homeTurret())
                .whenBecomesTrue(robot.hood.setHoodAngle(HOOD_ANGLE_FAR))
                .global();




        //TODO set rpm too
        //TODO another layer???
        //TODO: this needs to be a new state that overrides all the other turret movements. it also needs to have rpm and hood

        Button g2x = button(() -> gamepad2.x)
                .whenBecomesTrue(robot.intake::intake)
                .whenBecomesFalse(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad2().rightTrigger().atLeast(.1)
                .whenBecomesTrue(autoAimCommand)
                .whenBecomesFalse(autoAimCommand::cancel);


        Gamepads.gamepad2().leftTrigger().atLeast(.1)
                .whenBecomesTrue(robot.gate.openGate)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.gate.closeGate)
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
    @Override
    public void onUpdate() {

        g1LeftX = Gamepads.gamepad1().leftStickX().get();
        g1LeftY = Gamepads.gamepad1().leftStickY().get();
        g1RightX = Gamepads.gamepad1().rightStickX().get();


        BaseRobot.INSTANCE.periodic(); // update all subsystems
        BindingManager.update(); // update button states

        robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1LeftY, -g1LeftX), -g1RightX));

        robot.drive.driveFieldcentric(g1LeftY, g1LeftX, -g1RightX, 1);

    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}
