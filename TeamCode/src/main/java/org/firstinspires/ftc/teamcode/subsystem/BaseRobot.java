package org.firstinspires.ftc.teamcode.subsystem;


import dev.nextftc.core.subsystems.SubsystemGroup;

public class BaseRobot extends SubsystemGroup {
    public static final BaseRobot INSTANCE = new BaseRobot();

    public final DriveSubsystem drive = DriveSubsystem.INSTANCE;
    public final GateSubsystem gate = GateSubsystem.INSTANCE;
    public final HoodSubsystem hood = HoodSubsystem.INSTANCE;
    public final IntakeSubsystem intake = IntakeSubsystem.INSTANCE;
    public final LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;
    public final ShooterSubsystem shooter = ShooterSubsystem.INSTANCE;
    public final TurretSubsystem turret = TurretSubsystem.INSTANCE;
    public final LEDSubsystem LED = LEDSubsystem.INSTANCE;
    private BaseRobot() {

        super(
                DriveSubsystem.INSTANCE,
                GateSubsystem.INSTANCE,
                HoodSubsystem.INSTANCE,
                IntakeSubsystem.INSTANCE,
                LimelightSubsystem.INSTANCE,
                ShooterSubsystem.INSTANCE,
                TurretSubsystem.INSTANCE,
                LEDSubsystem.INSTANCE

        );
    }
}