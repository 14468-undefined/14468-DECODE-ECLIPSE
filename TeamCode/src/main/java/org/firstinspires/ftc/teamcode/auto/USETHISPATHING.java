package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import org.firstinspires.ftc.teamcode.util.SampleAuto;

import java.lang.reflect.MalformedParametersException;

@Autonomous(name="UseThisPathing")
public class USETHISPATHING extends SampleAuto {
    private MecanumDrive drive;

    //HardwareMap hwMap;


    private final Pose2d startPose = new Pose2d(-61, 40, Math.toRadians(180));
    private final Pose2d shotPoseOnLine = new Pose2d(-24,24, Math.toRadians(90));//go shoot

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void onInit() {

        drive = new MecanumDrive(hardwareMap, new Pose2d(-61, 40, Math.toRadians(180)));




    }

    @Override
    public void onStart() {

        //PARK
        Actions.runBlocking(drive.actionBuilder(drive.getPose())
                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)



                //FIRST PILE----------------------------------------------
                .strafeToConstantHeading(new Vector2d(-11.2, 25.4))



                .strafeToConstantHeading(new Vector2d(-11.2, 54.5), new TranslationalVelConstraint(15))//intake

                //gate dump
                .strafeToConstantHeading(new Vector2d(-11.2, 48))
                .strafeToSplineHeading(new Vector2d(1.4, 55), Math.toRadians(90))//gate dump



                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose


                //SECOND PILE --------------------------------------
                .strafeToConstantHeading(new Vector2d(12, 29))//line up intake

                .strafeToConstantHeading(new Vector2d(12, 61), new TranslationalVelConstraint(15))//intake
                .strafeToConstantHeading(new Vector2d(12, 48))//back up


                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose


                //PILE 3

                .strafeToConstantHeading(new Vector2d(35.8, 29))//go to motif
                .strafeToConstantHeading(new Vector2d(35.8, 61))//intake


                .strafeToLinearHeading(shotPoseOnLine.position, shotPoseOnLine.heading)//go to shoot pose


                .build());




    }

    @Override
    public void onStop() {

    }
}