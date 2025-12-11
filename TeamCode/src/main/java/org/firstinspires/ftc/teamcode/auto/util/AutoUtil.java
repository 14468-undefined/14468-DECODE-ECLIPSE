package org.firstinspires.ftc.teamcode.auto.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystem.DriveSubsystem;

public class AutoUtil {
    //Net side start (close to baskets)
    public static final int NET = 0;

    //Observation side start(farm from baskets)
    public static final int OBS = 1;

    public static final Vector2d goalScoring = new Vector2d(-48.5,48.8);
    public static final int redGoalAngle = 126;
    public static final int blueGoalAngle = 234;


    public static final Vector2d BLUEGATEDUMP = new Vector2d(1.4, -55);
    public static final Vector2d REDGATEDUMP = new Vector2d(1.4, 55);



    //Starting Pose2d from sides
    public static final Pose2d BLUENEARSTART = new Pose2d(-61, -40, Math.toRadians(180));
    public static final Pose2d REDNEARSTART = new Pose2d(-61, 40, Math.toRadians(180));

    public static final Pose2d REDFARSTART = new Pose2d(61, 16, Math.toRadians(180));
    public static final Pose2d BLUEFARSTART = new Pose2d(61, -16, Math.toRadians(180));

    //Base robot's drive subsystem
    private DriveSubsystem drive;

    public AutoUtil(DriveSubsystem drive) {
        this.drive=drive;
    }

    /*public Action getStartToNetAction(int start) {
        if(start==NET) {
            return drive.actionBuilder(NETSTART)
                    .strafeToLinearHeading(highBasketDeposit, Math.toRadians(45))
                    .build();
        }
        else {
            return drive.actionBuilder(OBSSTART)
                    .strafeToLinearHeading(new Vector2d(-75,-46), Math.toRadians(90))
                    .build();
        }
    }

    public Action getBackupToDepositAction(Pose2d pose) {
        return drive.actionBuilder(pose)
                .strafeToLinearHeading(highBasketDeposit, Math.toRadians(45))
                .build();
    }

     */




    public static void delay(double t) {
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.seconds() < t) {
        }
    }
    public static Action getDelayAction(double t) {
        return (q) -> {delay(t); return false;};
    }

}