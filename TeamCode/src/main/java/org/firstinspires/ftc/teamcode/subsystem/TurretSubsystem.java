package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class TurretSubsystem implements Subsystem {


    //TODO: logic for teleop
    /**
     * get limelight tx in the loop
     * get power through PID method in loop
     * set power from that method in the loop
     *
     */

    // PID gains
    double kP = 0.02;
    double kI = 0.0;
    double kD = 0.001;

    double integralSum = 0;
    double lastError = 0;
    double lastTime = System.nanoTime() / 1e9;


    MotorEx turretMotor;
    ColorfulTelemetry cTelemetry;
    private HardwareMap hardwareMap;


    private final double TICKS_PER_REV = 1;//TODO: Change //312 rPM?
    private final double GEAR_RATIO = 5;//20:400


    public TurretSubsystem(HardwareMap hardwareMap, ColorfulTelemetry telemetry) {

        this.cTelemetry = telemetry;

        turretMotor = new MotorEx("turret");

        turretMotor.brakeMode();
        //turretMotor.reverse();

    }
    @Override
    public void initialize() {
        // initialization logic (runs on init)
    }


    private double getTurretAngle() {
        double ticks = turretMotor.getCurrentPosition();
        return (ticks / (TICKS_PER_REV * GEAR_RATIO)) * 360.0;
    }

    public int angleToTicks(double angleDeg) {
        double ticks = (angleDeg / 360.0) * (TICKS_PER_REV * GEAR_RATIO);
        return (int) Math.round(ticks);
    }


    public Command setTurretPower(double p) {
        return new LambdaCommand()
                .setStart(() -> {
                    turretMotor.setPower(p);

                })
                .setIsDone(() -> false)
                .requires(this)
                .named("Set Turret Power");
    }
    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    turretMotor.setPower(0);
                })
                .setIsDone(() -> false)
                .requires(this)
                .named("Stop Turret");
    }

    public Command runToAngle(double angle) {
        return new LambdaCommand()
                .setStart(() -> {
//TODO - make run to position method
                    //run to this: angleToTicks(angle);

                })

                .setIsDone(() -> false)
                .requires(this)
                .named("Stop Turret");
    }







    public double turretPID(double error) {
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        if (dt <= 0) dt = 0.02; // guard

        double P = kP * error;

        // Conditional integration (anti-windup)
        double potentialIntegral = integralSum + error * dt;
        double maxIntegral = 10.0; // tune to avoid huge I term
        if (Math.abs(potentialIntegral * kI) < 1.0) { // only integrate if not saturated
            integralSum = potentialIntegral;
        }
        double I = kI * integralSum;

        double derivative = (error - lastError) / dt;
        double D = kD * derivative;

        lastError = error;
        lastTime = currentTime;

        double output = P + I + D;

        // Clip to motor input range -1..1
        output = Math.max(-1.0, Math.min(1.0, output));

        // Optional deadband: if very small, treat as 0
        if (Math.abs(error) < 0.5) {
            output = 0;
            integralSum = 0; // optional: reset integral when close
        }

        return output;
    }

    // In loop

    //double tx = limelight.getTx();
    //double power = turretPID(tx);
    //turretMotor.setPower(power);


    public double getTurretTicks(){
        return turretMotor.getRawTicks();
    }
    public double getTurretRotation(){
        double turretRotation = 0;
        //positive = angles positive
        //negative = angles negative
        return turretRotation;
    }







    public void printTelemetry(ColorfulTelemetry t){
        t.addData("Motor Power", turretMotor.getPower());

        t.update();
    }
    @Override
    public void periodic() {
        // periodic logic (runs every loop)
    }
}
