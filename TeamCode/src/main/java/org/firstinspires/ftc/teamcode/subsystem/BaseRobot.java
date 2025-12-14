package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.subsystems.SubsystemGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.util.AutoUtil;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;

public class BaseRobot extends SubsystemGroup {
//TODO - is this right


    public AutoUtil autoGenerator;

    public DriveSubsystem drive;
    public IntakeSubsystem intake;

    public GateSubsystem gate;
    public LimelightSubsystem limelight;
    public ShooterSubsystem shooter;
    public TurretSubsystem turret;
    public HoodSubsystem hood;


    public ColorfulTelemetry cTelemetry;


    public Telemetry telemetry;



    public BaseRobot(HardwareMap hwMap, Pose2d startPos){

        drive = new DriveSubsystem(hwMap, startPos);

        autoGenerator = new AutoUtil(drive);

        intake = new IntakeSubsystem(hwMap, cTelemetry);
        turret = new TurretSubsystem(hwMap, cTelemetry);
        shooter = new ShooterSubsystem(hwMap);
        limelight = new LimelightSubsystem(hwMap, cTelemetry);
        gate = new GateSubsystem(hwMap, cTelemetry);
        hood = new HoodSubsystem(hwMap, cTelemetry);


    }


    public void delay(double seconds){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < seconds){

        }

    }



    public void printTelemetry(ColorfulTelemetry t) {
        drive.printTelemetry(t);
        //intake.printTelemetry(t);
        //shooter.printTelemetry(telemetry);
        //webcamVision.printTelemetry(t);
        //huskyLensVision.printTelemetry(t);
        //transfer.printTelemetry(t);


    }

    @Override
    public void periodic() {

        drive.periodic();

        intake.periodic();
        turret.periodic();
        shooter.periodic();
        limelight.periodic();
        gate.periodic();
        hood.periodic();

    }

    public void stopAll(){
        CommandManager.INSTANCE.scheduleCommand(intake.stop());

    }




}