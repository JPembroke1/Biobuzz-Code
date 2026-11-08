package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Flower;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotSettings;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@Autonomous(name = "00 - Blue Auto", group = "Main")
public class AbsolutelyAmazingBlueAuto extends OpMode {

    private List<LynxModule> allHubs;
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    private final PoseFactory p = PoseFactory.degrees();

    public static double SHOOT_SETTLE_SECONDS = 1.3;
    public static double SHOOT_FEED_SECONDS = 1.3;

    public double shooterReadyWaitStartTime = 0.0;
    public boolean shooterReadyTimeoutFired = false;
    public static double SHOOT_READY_TIMEOUT = 1;
    public static double TPS_READY_THRESHOLD = 0.95;
    private final Pose start = p.of(56, 8, 180);
    private final Pose point1 = p.of(8, 8, 180);
    private final Pose point2Start = p.of(8, 8, 180);
    private final Pose point2 = p.of(46, 120, 90);
    private final Pose point3Start = p.of(46, 120, 90);
    private final Pose point3 = p.of(14, 46, 180);
    private final Pose point3Control1 = p.of(20, 46, 0);
    private final Pose point4Start = p.of(14, 46, 180);
    private final Pose point4 = p.of(46, 122, 90);
    private final Pose point4Control1 = p.of(15, 46, 0);
    private final Pose point5Start = p.of(46, 122, 90);
    private final Pose point5 = p.of(14, 118, 0);

    private Path path1() {
        return line(start, point1).linear(start, point1);
    }

    private Path path2() {
        return line(point2Start, point2).linear(point2Start, point2);
    }

    private Path path3() {
        return curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }
    private Path path4() {
        return curve(point4Start, point4Control1, point4).linear(point4Start, point4);
    }

    private Path path5() {
        return line(point5Start, point5).linear(point5Start, point5);
    }

//    private final Pose start = p.of(8, 8, Math.toRadians(180));
//    private final Pose pickupFirstRow = p.of(56, 8, Math.toRadians(180));
//    private final Pose returnToShooter = p.of(8, 8, Math.toRadians(180));
//
//    private Path pathToCollectFirstRow() {
//        return line(start, pickupFirstRow).linear(start, pickupFirstRow);
//    }
//
//    private Path pathToReturnToShootFromFirstRow() {
//        return line(pickupFirstRow, returnToShooter).linear(pickupFirstRow, returnToShooter);
//    }


    public void init() {
        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(start);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();
        flower = new Flower();
        robotSettings = new RobotSettings();

        outtake.init(hardwareMap);
        intake.init(hardwareMap);
        flower.init(hardwareMap);
        robotSettings.init(hardwareMap);

        intake.stopIntake();
        outtake.stopOuttake();
        flower.flowerActivate(RobotConstants.flowerLiftPos);

    }

    public void init_loop() {
        clearBulkCache();

        PoseStorage.setBlue();

        Pose currentPose = follower.pose();
        PoseStorage.setPose(currentPose);
    }

    public void start() {
        //Scheduler.reset();

        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(start);

        turret.setForward();

        schedule(autoRoutine());

    }

    public void loop() {

        clearBulkCache();

        follower.update();
        Scheduler.execute();

        Pose currentPose = follower.pose();

        turret.aimTurret(currentPose);

        outtake.update();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }

    private Command autoRoutine() {
        return sequential(
//                followCollect(path1()),
//
                shootCycle()
//
//                followLowerFlower(path2()),
//
//                followRaiseFlower(path3()),
//
//                follow(follower, path4()),
//
//                follow(follower, path5())
        );
    }

    private Command followCollect(Path path) {
        return parallel(
                follow(follower, path),

                sequential(
                        instant(() -> {
                            intake.intake(
                                    RobotConstants.FRONT_INTAKE_SPEED,
                                    RobotConstants.BACK_INTAKE_SPEED
                            );
                        })
                )
        );
    }

    private Command followLowerFlower(Path path) {
        return parallel(
                follow(follower, path),

                sequential(
                        instant(() -> {
                            flower.flowerActivate(RobotConstants.flowerDropPos);
                        })
                )
        );
    }

    private Command followRaiseFlower(Path path) {
        return parallel(
                follow(follower, path),

                sequential(
                        waitSeconds(0.5),

                        instant(() -> {
                            flower.flowerActivate(RobotConstants.flowerLiftPos);
                        })

                )
        );
    }

    private Command shootCycle() {
        return sequential(
                instant(() -> {
                    startShooterReadyWait();

                    outtake.setTargetTPS(RobotConstants.OUTTAKE_TPS);

                    stopIntakeAndBlock();
                }),

                race(
                        waitUntil(this::shooterReadyOrTimedOut),
                        infinite(this::holdShooterAndBlock)
                ),

                race(
                        waitSeconds(SHOOT_SETTLE_SECONDS),
                        infinite(this::releaseBlocker)
                ),

                race(
                        waitSeconds(SHOOT_FEED_SECONDS),
                        infinite(this::runShootFeed)
                ),

                instant(this::stopIntakeAndBlock)
        );
    }

    private Command waitSeconds(double seconds) {
        return waitMs(seconds * 1000.0);
    }

    private void holdShooterAndBlock() {
        intake.stopIntake();
        robotSettings.block();
    }

    private void releaseBlocker() {
        robotSettings.release();
    }

    private void updateShooter() {
        outtake.update();
    }

    private void stopIntakeAndBlock() {
        intake.stopIntake();
        robotSettings.block();
    }

    private void runShootFeed() {

        robotSettings.release();
        intake.intake(
                RobotConstants.FRONT_INTAKE_SPEED,
                RobotConstants.BACK_INTAKE_SPEED
        );
    }

    private void startShooterReadyWait() {
        shooterReadyWaitStartTime = getRuntime();
        shooterReadyTimeoutFired = false;
    }

    private boolean shooterReadyOrTimedOut() {

        if (isShooterAtSpeed()) {
            return true;
        }

        if (hasShooterReadyTimedOut()) {
            shooterReadyTimeoutFired = true;
            return true;
        }

        return false;
    }

    private boolean hasShooterReadyTimedOut() {
        return (getRuntime() - shooterReadyWaitStartTime) > SHOOT_READY_TIMEOUT;
    }

    private boolean isShooterAtSpeed() {
        return outtake.isAtSpeed(TPS_READY_THRESHOLD);
    }


    private void robotPeriodic() {
        follower.update();

        Pose currentPose = follower.pose();
        PoseStorage.setPose(currentPose);

        intake.update();

        turret.aimTurret(currentPose);
    }


    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    private void clearBulkCache() {
        if (allHubs == null) {
            return;
        }

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

}
