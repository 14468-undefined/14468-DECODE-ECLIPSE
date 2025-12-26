package org.firstinspires.ftc.teamcode.teleop.helper;

import android.renderscript.ScriptGroup;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

import static dev.nextftc.bindings.Bindings.*;


@TeleOp(name = "ShotTesting" , group = "AA - COMP")

public class ShotTesting extends NextFTCOpMode {

    ColorfulTelemetry cTelemetry;
    MecanumDrive drive;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    {
        addComponents(new SubsystemComponent(robot));
    }

    double distance = 0;
    double hoodAngle = 30;
    int shooterRPM = 3000;

    @Override public void onInit() {

        //robot = new BaseRobot(hardwareMap, new Pose2d(0,0,0));
        robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_y, -gamepad1.left_stick_x), -gamepad1.right_stick_x));


    }
    @Override public void onWaitForStart() {

    }
    @Override public void onStartButtonPressed() {



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

        Button gamepad1y = button(() -> gamepad1.y);
        gamepad1y.whenBecomesTrue(() -> {
            hoodAngle += 1;
        });
        Button gamepad1a = button(() -> gamepad1.a);
        gamepad1a.whenBecomesTrue(() -> {
            hoodAngle -= 1;
        });

        Button gamepad1x = button(() -> gamepad1.x);
        gamepad1x.whenBecomesTrue(() -> {
            shooterRPM -= 50;
        });
        Button gamepad1b = button(() -> gamepad1.b);
        gamepad1b.whenBecomesTrue(() -> {
            shooterRPM += 50;
        });



        /// READ THIS ABOUT LAYERS IN TELEOP https://nextftc.dev/bindings/buttons //i.e. endgame layer
    }
    @Override public void onUpdate() {

        robot.shooter.spin(shooterRPM);
        robot.hood.setHoodAngle(hoodAngle);

        distance = robot.limelight.getDistance();
        cTelemetry.addData("Distance", distance);

        cTelemetry.addData("hood angle", hoodAngle);
        cTelemetry.addData("Shooter RPM", shooterRPM);

        BindingManager.update();
    }
    @Override public void onStop() {
        BindingManager.reset();
    }
}
