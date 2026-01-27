package org.firstinspires.ftc.teamcode.teleop.helper;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;



@TeleOp(name = "AutoAimTest", group = "BB - helper")
public class AutoAimTest extends NextFTCOpMode {

    private boolean limelightStarted = false;





    MecanumDrive drive;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public AutoAimTest(){

        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }


    @Override
    public void onInit() {

    }


    @Override
    public void onStartButtonPressed(){


        Gamepads.gamepad1().leftTrigger().atLeast(.1).whenTrue(new AutoAimCommand(robot));
        Gamepads.gamepad1().rightTrigger().atLeast(.1).whenTrue(new AutoAimCommand(robot));

        Gamepads.gamepad2().leftTrigger().atLeast(.1).whenTrue(new AutoAimCommand(robot));
        Gamepads.gamepad2().rightTrigger().atLeast(.1).whenTrue(new AutoAimCommand(robot));




    }
    @Override
    public void onUpdate() {

    }
}
