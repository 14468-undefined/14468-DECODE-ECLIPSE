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
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.LEDSubsystem;

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
    double g1LeftX;
    double g1LeftY;
    double g1RightX;





    String llWorking = "Limelight Working";
    String llNotWorking = "Limelight Not Working";

    private MotorEx frontLeftMotor = new MotorEx("leftFront").brakeMode().
            reversed();
    private MotorEx frontRightMotor = new MotorEx("rightFront").brakeMode();
    private MotorEx backLeftMotor = new MotorEx("leftBack").brakeMode().reversed();
    private MotorEx backRightMotor = new MotorEx("rightBack").brakeMode();
    @Override
    public void onInit() {


        robot.initialize();
        //robot.limelight.initHardware(hwMap, "RED");

        BindingManager.setLayer(llWorking);


    }

    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {


        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();


        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                new FieldCentric(imu)
        );
        driverControlled.schedule();

//TODO: get this to work


        robot.intake.setIntakePower(1);
        robot.intake.intake();

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
