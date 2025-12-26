package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.ShooterSubsystem;

import dev.nextftc.ftc.NextFTCOpMode;


@TeleOp(name = "FlywheelPIDTuner" , group = "TeleOp")
public class FlywheelPIDTuner extends NextFTCOpMode {
    ShooterSubsystem shooter;
    private FtcDashboard dash;
    TelemetryPacket packet = new TelemetryPacket();

    @Override public void onInit() {
        dash = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        shooter = ShooterSubsystem.INSTANCE;


        telemetry.addLine("init done");
        telemetry.update();


    }
    @Override public void onStartButtonPressed(){

    }
    @Override public void onUpdate() {


        shooter.applyPIDF();
        shooter.spin(ShooterSubsystem.TARGET_RPM);

        packet.put("target_shooter_rpm", shooter.getTargetRPM());
        packet.put("current_shooter_rpm", shooter.getRPM());
        packet.put("left_power", shooter.getLeftPower());
        packet.put("right_power", shooter.getRightPower());
        dash.sendTelemetryPacket(packet);


    }
}