package org.firstinspires.ftc.teamcode.teleop.helper;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "Turret Angle CS Tuner", group = "Tuning")
public class TurretAngleControlSystemTuner extends LinearOpMode {

    /* ---------------- Dashboard Tunables ---------------- */

    public static double kP = 0.1;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double TARGET_TICKS = 0.0;

    public static double MAX_POWER = 0.5;

    /* ---------------- Hardware ---------------- */

    private MotorEx turretMotor;

    /* ---------------- Control System ---------------- */

    private ControlSystem controller;

    private double lastKP, lastKI, lastKD, lastKF;

    /* ---------------- Init ---------------- */


    @Override
    public void runOpMode() throws InterruptedException {


        turretMotor = new MotorEx("turret")
                .reversed()
                .brakeMode();

        turretMotor.zero();

        rebuildController();


        waitForStart();


        while(opModeIsActive()){

            // Rebuild controller if dashboard values changed
            if (kP != lastKP || kI != lastKI || kD != lastKD || kF != lastKF) {
                rebuildController();
            }

            // Update target live
            controller.setGoal(new KineticState(TARGET_TICKS));

            double power = controller.calculate(turretMotor.getState());

            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            turretMotor.setPower(power);

            /* ---------------- Telemetry ---------------- */

            telemetry.addData("Target (ticks)", TARGET_TICKS);
            telemetry.addData("Position (ticks)", turretMotor.getCurrentPosition());
            telemetry.addData("Error (ticks)",
                    TARGET_TICKS - turretMotor.getCurrentPosition());

            telemetry.addData("Power", power);

            telemetry.addData("kP", kP);
            telemetry.addData("kI", kI);
            telemetry.addData("kD", kD);
            telemetry.addData("kF", kF);

            telemetry.update();
        }
    }



    /* ---------------- Controller Utils ---------------- */

    private void rebuildController() {
        controller = ControlSystem.builder()
                .posPid(kP, kI, kD)
                .basicFF(kF)
                .build();

        controller.setGoal(new KineticState(TARGET_TICKS));

        lastKP = kP;
        lastKI = kI;
        lastKD = kD;
        lastKF = kF;
    }

    /* ---------------- Loop ---------------- */

}
