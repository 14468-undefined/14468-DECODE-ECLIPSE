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





        /*
        g1 RB/LB

        when in layer - Limelight Working
        - set layer to ll not working
        - home turret (cuz ll isnt working)
        - set hood angle to far constant (bc auto-aim distance doesn't work)

        when in layer - ll not working
        - set layer to limelight working

         */
        //TODO: this needs to override all the other turret movements and RPM and hood - when in this layer it shouldn't try to auto-aim
        Button g1RBorLB = button(() -> gamepad1.right_bumper).or(button(() -> gamepad1.left_bumper))
                .inLayer(llWorking)//when limelight is working
                .whenBecomesTrue(() -> BindingManager.setLayer(llNotWorking))
                .whenBecomesTrue(robot.turret.homeTurret())
                .whenBecomesTrue(robot.hood.setHoodAngle(HOOD_ANGLE_FAR))//default when ll not working

                .inLayer(llWorking)
                .whenBecomesTrue(() -> BindingManager.setLayer(llWorking))
                .global();




        /*
        g2x - intake normal
        - intake
        - close gate (so it doesn't shoot)
         */
        Button g2x = button(() -> gamepad2.x)
                .whenBecomesTrue(robot.intake::intake)
                .whenBecomesTrue(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());

        /*
        g2RT - auto aim
        - while true, auto aim
        - while false, don't
         */
        //TODO - should it hold pose when its false?
        Gamepads.gamepad2().rightTrigger().atLeast(.1)
                .whenBecomesTrue(autoAimCommand)
                .whenBecomesFalse(autoAimCommand::cancel);


        /*
        g2LT - shooting
        - open gate
        - run intake to transfer
        NOTE: Flywheel controlled separately through auto aim command using RT
         */
        Gamepads.gamepad2().leftTrigger().atLeast(.1)
                .whenBecomesTrue(robot.gate.openGate)
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesFalse(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());





        //.whenTrue: every loop when the button is true.
        //.whenFalse: every loop when the button is false.
        //.whenBecomesTrue: the first loop when the button is true, a.k.a. the rising edge.
        //.whenBecomesFalse: the first loop when the button is false, a.k.a. the falling edge.

        //Button andButton = button1.and(button2); // true when both buttons are true
        //Button orButton = button1.or(button2); // true when at least one button is true
        //Button xorButton = button1.xor(button2); // true when exactly one button is true
        //Button notButton = button1.not(); // true when button1 is false
    }

    @Override
    public void onUpdate() {

        g1LeftX = Gamepads.gamepad1().leftStickX().get();
        g1LeftY = Gamepads.gamepad1().leftStickY().get();
        g1RightX = Gamepads.gamepad1().rightStickX().get();


        BaseRobot.INSTANCE.periodic(); // update all subsystems
        BindingManager.update(); // update button states

        ///robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1LeftY, -g1LeftX), -g1RightX));
        //TODO: field or robot centric?
        robot.drive.driveFieldcentric(g1LeftY, g1LeftX, -g1RightX, 1);

    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}
