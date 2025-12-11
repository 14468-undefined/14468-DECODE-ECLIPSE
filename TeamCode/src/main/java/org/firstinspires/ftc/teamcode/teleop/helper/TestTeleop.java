package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




@TeleOp(name = "LEAGUES_TELEOP" , group = "AA - COMP")
public class LeaguesTeleop extends LinearOpMode {


    double driveSpeed = 1;

    boolean shooterOn = false;

    int shooterRPM = 2135;




    @Override
    public void onInit() {



        robot.drive.setDefaultCommand(new RunCommand(()-> robot.drive.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(g1.getLeftY() * driveSpeed, -g1.getLeftX() * driveSpeed), -g1.getRightX() * driveSpeed)), robot.drive));




    }

    @Override
    public void onStart() {







    }

    @Override
    public void onLoop() {


        // Print intake telemetry every loopq
        pen.addData("Shooter RPM: ", robot.shooter.getShooterVelocity());
        pen.addData("Set RPM: ", robot.shooter.getTargetRPM());


    }

    @Override
    public void onStop() {
        robot.stopAll();
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
