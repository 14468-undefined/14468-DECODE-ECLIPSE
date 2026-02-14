package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.components.BulkReadComponent;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "FlywheelPIDTuner" , group = "TeleOp")
public class FlywheelPIDTuner extends NextFTCOpMode {

    ShooterSubsystem shooter;
    private FtcDashboard dash;

    private final BaseRobot robot = BaseRobot.INSTANCE;

    public FlywheelPIDTuner(){
        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onInit() {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        shooter = ShooterSubsystem.INSTANCE;

        telemetry.addLine("init done");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // nothing


        shooter.spin().schedule();



    }

    @Override
    public void onUpdate() {

        // only updates PID if values changed
        //shooter.maybeUpdatePIDF();

        shooter.spin().schedule();
        // directly set target (not a Command)
        //shooter.setTargetRPMDirect(ShooterSubsystem.TARGET_RPM);
        TelemetryPacket packet = new TelemetryPacket();

        packet.put("target_shooter_rpm", shooter.getTargetRPM());
        packet.put("current_shooter_rpm", shooter.getRPM());
        packet.put("left_power", shooter.getLeftPower());
        packet.put("right_power", shooter.getRightPower());
        packet.put("right_RPM", shooter.getRightRPM());
        packet.put("left_RPM", shooter.getLeftRPM());

        dash.sendTelemetryPacket(packet);
    }
}
