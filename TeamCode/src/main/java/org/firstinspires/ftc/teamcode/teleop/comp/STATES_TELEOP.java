package org.firstinspires.ftc.teamcode.teleop.comp;

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


@TeleOp(name = "STATES_TELEOP" , group = "AA - COMP")

public class STATES_TELEOP extends NextFTCOpMode {

    MecanumDrive drive;

    private final BaseRobot robot = BaseRobot.INSTANCE;

    {
        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }



    HardwareMap hwMap;
    private double HOOD_ANGLE_FAR = 0;



    ColorfulTelemetry t;


    String llWorking = "Limelight Working";
    String llNotWorking = "Limelight Not Working";


    @Override
    public void onInit() {



        robot.initialize();


        BindingManager.setLayer(llWorking);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

    }

    @Override public void onWaitForStart() {
        //t.addLine("OpMode Initialized");
        //t.addLine("Waiting for start...");
    }
    @Override public void onStartButtonPressed() {

       // t.addLine("x/b = intake controls");
        //t.addLine("a/y = april tag red/blue swap");

        /*Command robotCentricDrive = drive.driverControlledCommand(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                true
        );//robot centric is auto true
        robotCentricDrive.schedule();

         */
//TODO: Field Centric?

        robot.limelight.setPipeline(Constants.LimelightConstants.BLUE_GOAL_TAG_PIPELINE);//change

        Gamepads.gamepad1().a().whenBecomesTrue(robot.limelight.setPipeline(Constants.LimelightConstants.BLUE_GOAL_TAG_PIPELINE));
        Gamepads.gamepad1().y().whenBecomesTrue(robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE));









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
        Button g1RBorLB = button(Gamepads.gamepad1().rightBumper()).or(Gamepads.gamepad1().leftBumper())
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
        Gamepads.gamepad2().x()
                .whenBecomesTrue(robot.intake::intake)
                .whenBecomesTrue(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());

        Gamepads.gamepad2().b()
                        .whenBecomesTrue(robot.intake::intakeReverse)
                                .whenBecomesFalse(robot.intake.stop());
        /*
        g2RT - auto aim
        - while true, auto aim
        - while false, don't
         */
        //TODO - should it hold pose when its false?
        Gamepads.gamepad2().rightTrigger().atLeast(.1)
                .inLayer(llWorking)
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
    public void onUpdate() {}
    @Override public void onStop() {}

}
