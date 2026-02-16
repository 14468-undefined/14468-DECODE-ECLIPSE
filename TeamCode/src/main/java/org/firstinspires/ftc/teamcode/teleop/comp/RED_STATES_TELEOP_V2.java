package org.firstinspires.ftc.teamcode.teleop.comp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.VoltageCompensatingMotor;
import dev.nextftc.hardware.powerable.SetPower;
import org.firstinspires.ftc.teamcode.command.AutoAimCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.BaseRobot;
import org.firstinspires.ftc.teamcode.util.ColorfulTelemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static dev.nextftc.bindings.Bindings.button;


@TeleOp(name = "RED_STATES_TELEOPV2" , group = "AA - COMP")

public class RED_STATES_TELEOP_V2 extends NextFTCOpMode {

    private Limelight3A limelight;
    private boolean limelightStarted = false;

    MecanumDrive drive;

    private final BaseRobot robot = BaseRobot.INSTANCE;

    public RED_STATES_TELEOP_V2(){
        addComponents(
                new SubsystemComponent(robot), BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private double voltage = 14;





    private double CLOSE_RPM = 2350;//close
    private double MID_RPM = 2650;//farthest spot in close zone
    private double FAR_RPM = 3250;//farthest spot

    private double CURRENT_RPM = 2650;


    HardwareMap hwMap;




    ColorfulTelemetry t;

    String llWorking = "Limelight Working";
    String llNotWorking = "Limelight Not Working";


    @Override
    public void onInit() {


        robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE).schedule();
        robot.shooter.stop();
        robot.turret.bypassPeriodic = false;

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();

        BindingManager.setLayer(llWorking);

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");

        drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

    }

    @Override public void onWaitForStart() {
        //t.addLine("OpMode Initialized");
        //t.addLine("Waiting for start...");

    }
    @Override public void onStartButtonPressed() {




        //robot.limelight.setPipeline(Constants.LimelightConstants.BLUE_GOAL_TAG_PIPELINE);//change

        //Gamepads.gamepad1().a().whenBecomesTrue(robot.limelight.setPipeline(Constants.LimelightConstants.BLUE_GOAL_TAG_PIPELINE));
        //Gamepads.gamepad1().y().whenBecomesTrue(robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE));

        //Gamepads.gamepad1().a().whenBecomesTrue(limelight.pipelineSwitch(Constants.LimelightConstants.BLUE_GOAL_TAG_PIPELINE));
        //Gamepads.gamepad1().y().whenBecomesTrue(robot.limelight.setPipeline(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE));


        //Gamepads.gamepad2().rightBumper().whenBecomesTrue(robot.gate.openGate);
        //Gamepads.gamepad2().leftBumper().whenBecomesTrue(robot.gate.closeGate);





        //GAMEPAD 1 ---------------- zone + drive


        //DRIVE
        Command robotCentricDrive = drive.driverControlledCommand(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                true
        );//robot centric is auto true
        robotCentricDrive.schedule();

        //ZONES------------------------------------------------------

        //far zone

        //new zones

        Gamepads.gamepad1().rightTrigger().atLeast(.1).whenBecomesTrue(() -> CURRENT_RPM += 100);
        Gamepads.gamepad1().leftTrigger().atLeast(.1).whenBecomesTrue(() -> CURRENT_RPM -= 100);
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> CURRENT_RPM += 50);
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> CURRENT_RPM -= 50);

        //mid rpm high hood
        //Gamepads.gamepad1().dpadUp().or(Gamepads.gamepad1().leftBumper())
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(robot.hood.setHoodPose(.84))//top
                .whenBecomesTrue(() -> CURRENT_RPM = MID_RPM);

        //close rpm low hood
        //Gamepads.gamepad1().dpadDown().or(Gamepads.gamepad1().rightBumper())
        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(() -> CURRENT_RPM = CLOSE_RPM)
                .whenBecomesTrue(robot.hood.setHoodPose(.0311));


        Gamepads.gamepad1().dpadRight().or(Gamepads.gamepad1().dpadLeft())
                .whenBecomesTrue(robot.hood.setHoodPose(.311));







        /*
        Gamepads.gamepad1().dpadUp().or(Gamepads.gamepad1().leftBumper())
                .whenBecomesTrue(robot.hood.setHoodPose(.84))//top
                //.whenBecomesTrue(robot.shooter.setTargetRPM(FAR_RPM))
                .whenBecomesTrue(robot.intake.setIntakePower(1));

        //close
        Gamepads.gamepad1().dpadDown().or(Gamepads.gamepad1().rightBumper()).or(Gamepads.gamepad1().rightTrigger().atLeast(.1))
                .whenBecomesTrue(robot.hood.setHoodPose(.0311))//mid
                .whenBecomesTrue(robot.shooter.setTargetRPM(CLOSE_RPM))
                        .whenBecomesTrue(robot.intake.setIntakePower(1));


        //mid
        Gamepads.gamepad1().dpadLeft().or(Gamepads.gamepad1().dpadRight())
                .whenBecomesTrue(robot.hood.setHoodPose(.311))//bottom //.46 is bottom
                .whenBecomesTrue(robot.shooter.setTargetRPM(MID_RPM))
                .whenBecomesTrue(robot.intake.setIntakePower(1));


         */

       /* Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(() -> visionEnabled = false)
                .whenBecomesTrue(robot.turret.runToAngle(0))
                .whenBecomesFalse(() -> visionEnabled = true);

        */


        //GAMEPAD 2 ------------------ everything else

        //intake normal
        Gamepads.gamepad2().x()
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesTrue(robot.gate.closeGate)
                .whenBecomesFalse(robot.intake.stop());


        //intake + transfer
        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(robot.intake.setIntakePower(.8))
                .whenBecomesTrue(robot.intake.intake())
                .whenBecomesTrue(robot.gate.openGate)
                .whenBecomesFalse(robot.intake.stop())
                .whenBecomesTrue(robot.intake.setIntakePower(1));

        //intake reverse
        Gamepads.gamepad2().b()
                .whenBecomesTrue(robot.intake.intakeReverse())
                .whenBecomesFalse(robot.intake.stop());


        //spin flywheel backwards full power


        Gamepads.gamepad2().rightTrigger().atLeast(.1)
                .whenBecomesTrue(robot.shooter.spin())
                .whenBecomesFalse(robot.shooter.stop());

        Gamepads.gamepad2().leftTrigger().atLeast(.1)
                .whenBecomesTrue(robot.shooter.spinReverse())
                .whenBecomesFalse(robot.shooter.stop());




        //OLD SHOOTER CONTROLS - JUST POWER - ARCHIVE


        /*Gamepads.gamepad2().y().whenBecomesTrue(robot.shooter.setRight1()).whenBecomesFalse(robot.shooter.setRight0());

        Gamepads.gamepad2().leftTrigger().atLeast(.1)

                .whenTrue(robot.shooter.setRightNeg1())
                .whenBecomesFalse(robot.shooter.setLeft0())
                .whenBecomesFalse(robot.shooter.setRight0());

        //spin flywheel forwards full power
        Gamepads.gamepad2().rightTrigger().atLeast(.1)
                .whenTrue(robot.shooter.setLeft1())
                .whenTrue(robot.shooter.setRight1())
                .whenBecomesFalse(robot.shooter.setLeft0())
                .whenBecomesFalse(robot.shooter.setRight0());

         */



    }

    @Override
    public void onUpdate() {

        voltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        robot.shooter.voltageCompensate(voltage);
        //robot.shooter.maybeUpdatePIDF();
        telemetry.addData("VOLTAGE", voltage);


        robot.shooter.TARGET_RPM = CURRENT_RPM;
        telemetry.addData("Actual RPM", robot.shooter.getRPM());
        telemetry.addData("Target RPM: ", CURRENT_RPM);

        if (!limelightStarted) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(Constants.LimelightConstants.RED_GOAL_TAG_PIPELINE);
            limelight.start();
            limelightStarted = true;
        }

        if(robot.shooter.isAtTargetSpeed()){
            gamepad2.rumble(1000);
            //gamepad2.rumble(1, 0, 2); //power left, power right, duration (ms)

        }

        if(gamepad2.x){
            //gamepad2.rumble(500);

        }


        telemetry.addData("LL Running", limelight.isRunning());

        LLResult result = limelight.getLatestResult();

        DoubleSupplier txSupplier = () -> {
            LLResult r = limelight.getLatestResult();
            if (r != null && r.isValid()) {
                double tx = r.getTx();


                double offset = 0.0;
                if (CURRENT_RPM >= FAR_RPM - 200) { // allow some tolerance
                    offset = 3;//positive here
                }

                return tx + offset;
            }
            return 0.0;
        };
        LLStatus status = limelight.getStatus();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            //telemetry.addData("Ty", ty);
            //telemetry.addData("Ta", ta);
            //telemetry.addData("Tx", tx);

            // only schedule once
            if (!robot.turret.isAiming()) {
                robot.turret.aimWithVision(txSupplier).schedule();
            }
        } else {

            if(gamepad1.right_stick_button){
                robot.turret.runToTicks(0).schedule();

            }

            /*telemetry.addData("Limelight", "No Targets");
            telemetry.addData("CPU", status.getCpu());
            telemetry.addData("Temp", status.getTemp());
            telemetry.addData("RAM", status.getRam());
            telemetry.addData("Pipeline Type", status.getPipelineType());

             */

            // stop turret if no target
            //robot.turret.stopTurret(); /TODO
        }


        telemetry.update();
    }


}
