package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "NextFTC Auto")
public class TestAuto extends NextFTCOpMode {


    HardwareMap hwMap;
    private final BaseRobot robot = BaseRobot.INSTANCE;
    public TestAuto() {


        addComponents(
                new SubsystemComponent(robot)
        );
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                robot.gate.openGate,
                new ParallelGroup(
                        //TODO: aim with vision with updating Tx
                ),
                new Delay(0.5),
                new ParallelGroup(

                )
        );
    }
    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }


}