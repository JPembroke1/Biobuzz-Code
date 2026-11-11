package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
@Autonomous(name = "01 - Flower Red Auto", group = "Main")
public class FlowerRedAuto extends OpMode {

    private List<LynxModule> allHubs;
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Route / mechanism values unchanged from the supplied auto unless marked CHANGED.
    public static double PRELOAD_TURRET_POSITION = 0.0;
    public static double FAR_WALL_TURRET_POSITION = 0.30;
    public static double SECOND_FLOWER_TURRET_POSITION = 0.40;
    public static double TRAVEL_TURRET_POSITION = 0.50;
    public static double AUTO_TURRET_TRIM = 0.0;

    // CHANGED: 1.00 -> 0.85. Paths 4 and 6 translate and rotate at the same time, and full
    // speed saturates the mecanum motors so heading and drive correction fight each other.
    public static double DRIVE_SPEED = 0.85;
    public static double FLOWER_APPROACH_SPEED = 0.70;
    public static double FLOWER_COLLECT_SPEED = 0.35;
    public static double WALL_APPROACH_SPEED = 0.35;
    public static double PARK_SPEED = 1.00;

    public static double TURRET_SETTLE_SECONDS = 0.20;
    public static double SHOOT_READY_TIMEOUT_SECONDS = 0.70;
    public static double BLOCK_RELEASE_SECONDS = 0.40;
    public static double PRELOAD_FEED_SECONDS = 1.00;
    public static double FLOWER_FEED_SECONDS = 1.60;

    public static double FIRST_FLOWER_INTAKE_HOLD_SECONDS = 0.75;
    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.45;
    public static double FLOWER_REVERSE_CLEAR_SECONDS = 0.35;
    public static double FLOWER_LIFT_DELAY_SECONDS = 0.20;

    public static double TPS_READY_FRACTION = 0.95;

    public static double PATH1_TIMEOUT_SECONDS = 4.0;
    public static double PATH2_TIMEOUT_SECONDS = 3.0;
    public static double PATH3_TIMEOUT_SECONDS = 3.0;
    public static double PATH4_TIMEOUT_SECONDS = 5.0;
    public static double PATH5_TIMEOUT_SECONDS = 5.0;
    public static double PATH6_TIMEOUT_SECONDS = 4.0;
    public static double PATH7_TIMEOUT_SECONDS = 2.5;
    public static double PATH8_TIMEOUT_SECONDS = 5.0;

    // NEW: path completion tuning.
    // Ignore "done" for this long after a path starts, so a stale atParametricEnd from the
    // previous path can't finish the new one instantly.
    public static double MIN_PATH_RUNTIME_SECONDS = 0.05;
    // Finish when this close to the end point AND heading is close (don't wait for t = 1).
    public static double END_POSITION_TOLERANCE_IN = 1.5;
    public static double END_HEADING_TOLERANCE_DEG = 10.0;
    // Wall stall exit (only legs flagged stallExit, i.e. path 5): if within
    // STALL_EXIT_MAX_DISTANCE_IN of the target and moving less than STALL_MOVE_TOLERANCE_IN
    // for STALL_TIME_SECONDS, treat the leg as finished instead of pushing until timeout.
    public static double STALL_EXIT_MAX_DISTANCE_IN = 8.0;
    public static double STALL_MOVE_TOLERANCE_IN = 0.3;
    public static double STALL_TIME_SECONDS = 0.30;

    // NEW: shot sequencing.
    // The blocker starts opening once the turret has settled and the shooter is above this
    // fraction of target, instead of waiting for TPS_READY_FRACTION first. Feeding still waits
    // for TPS_READY_FRACTION (or its timeout). Set this >= TPS_READY_FRACTION to get the old
    // "ready, then release" order back.
    public static double PRE_RELEASE_TPS_FRACTION = 0.80;

    // NEW: start moving the turret to its shot position before the robot arrives (far wall shot:
    // within PRE_AIM_DISTANCE_IN of the end of path 5; second flower shot: during the intake
    // hold) so TURRET_SETTLE_SECONDS has already elapsed by the time the shot starts.
    // Set false if the turret can't safely move while the robot is driving/intaking.
    public static boolean PRE_AIM_TURRET = true;
    public static double PRE_AIM_DISTANCE_IN = 20.0;

    // NEW: telemetry is throttled so the loop stays short and consistent.
    public static double TELEMETRY_INTERVAL_SECONDS = 0.25;

    private double activeTurretPosition = 0.50;
    private int shotCount = 0;
    private double shooterReadyStartTime = 0.0;
    private boolean shooterReadyTimeoutFired = false;
    private boolean anyPathTimeoutFired = false;
    private boolean currentPathTimedOut = false;
    private String lastTimedOutPath = "NONE";
    private String lastStallExitPath = "NONE";

    // NEW: per-path tracking state and loop timing.
    private double pathStartTime = 0.0;
    private Pose stallRefPose = null;
    private double stallRefTime = 0.0;
    private double lastLoopTime = 0.0;
    private double loopMs = 0.0;
    private double maxLoopMs = 0.0;
    private double lastTelemetryTime = -1.0;

    // NEW: shot state machine and turret/leg tracking.
    private enum ShootPhase { WAIT, RELEASE, FEED, DONE }
    private ShootPhase shootPhase = ShootPhase.DONE;
    private double shootStartTime = 0.0;
    private double releaseStartTime = 0.0;
    private double feedStartTime = 0.0;
    private double shootFeedSeconds = 0.0;
    private double turretMoveTime = 0.0;
    private boolean legDone = false;

    // Route coordinates intentionally unchanged.
    private final Pose start = poseFactory.of(52.3532, 8.0, 180.0);
    private final Pose path1End = poseFactory.of(22.4811, 45.4390, 180.0);
    private final Pose point2Start = poseFactory.of(22.4811, 45.4390, 180.0);
    private final Pose point2 = poseFactory.of(12.7950, 45.9688, 170.0);
    private final Pose point3 = poseFactory.of(22.3555, 46.1482, -178.9251);
    private final Pose point4Start = poseFactory.of(22.3555, 46.1482, 180.0);
    private final Pose point4 = poseFactory.of(22.6871, 94.7054, 90.0);
    private final Pose point5Start = poseFactory.of(22.6871, 94.7054, 90.0);
    private final Pose point5 = poseFactory.of(23.7064, 130.1432, 90.0);
    private final Pose point6Start = poseFactory.of(23.7064, 130.1432, 90.0);
    private final Pose point6 = poseFactory.of(42.0, 104.1657, 90.0);
    private final Pose point7Start = poseFactory.of(42.0, 104.1657, 90.0);
    private final Pose point7 = poseFactory.of(42.0, 127.0, 80.0);
    private final Pose point8Start = poseFactory.of(42.0, 127.0, 80.0);
    private final Pose point8 = poseFactory.of(5.0, 116.1719, 0.0);

    /**
     * Description of one drive leg. The Path itself is built when the leg starts, from the
     * robot's real pose, rather than from the planned start point. That stops the follower
     * lurching to correct accumulated error at the start of every leg.
     */
    private static class Leg {
        final Pose plannedStart;   // fallback only, used if the follower has no pose
        final Pose end;
        final double speed;
        final boolean reverseTangent;
        final boolean stallExit;
        final double timeoutSeconds;
        final String name;

        Leg(Pose plannedStart, Pose end, double speed, boolean reverseTangent,
            boolean stallExit, double timeoutSeconds, String name) {
            this.plannedStart = plannedStart;
            this.end = end;
            this.speed = speed;
            this.reverseTangent = reverseTangent;
            this.stallExit = stallExit;
            this.timeoutSeconds = timeoutSeconds;
            this.name = name;
        }
    }

    private Leg leg1() {
        return new Leg(start, path1End, FLOWER_APPROACH_SPEED, false, false,
                PATH1_TIMEOUT_SECONDS, "PATH 1");
    }

    private Leg leg2() {
        return new Leg(point2Start, point2, FLOWER_COLLECT_SPEED, false, false,
                PATH2_TIMEOUT_SECONDS, "PATH 2");
    }

    private Leg leg3() {
        return new Leg(point2, point3, FLOWER_APPROACH_SPEED, true, false,
                PATH3_TIMEOUT_SECONDS, "PATH 3");
    }

    private Leg leg4() {
        return new Leg(point4Start, point4, DRIVE_SPEED, false, false,
                PATH4_TIMEOUT_SECONDS, "PATH 4");
    }

    private Leg leg5() {
        return new Leg(point5Start, point5, WALL_APPROACH_SPEED, false, true,
                PATH5_TIMEOUT_SECONDS, "PATH 5");
    }

    private Leg leg6() {
        return new Leg(point6Start, point6, DRIVE_SPEED, false, false,
                PATH6_TIMEOUT_SECONDS, "PATH 6");
    }

    private Leg leg7() {
        return new Leg(point7Start, point7, FLOWER_COLLECT_SPEED, false, false,
                PATH7_TIMEOUT_SECONDS, "PATH 7");
    }

    private Leg leg8() {
        return new Leg(point8Start, point8, PARK_SPEED, false, false,
                PATH8_TIMEOUT_SECONDS, "PATH 8");
    }

    /** Builds the path from where the robot actually is right now to the leg's end pose. */
    private Path buildPath(Leg leg) {
        Pose from = follower.pose();
        if (from == null) from = leg.plannedStart;

        if (leg.reverseTangent) {
            return line(from, leg.end).reverseTangent()
                    .with(Constants.foresightConfig.maxPathSpeed.at(leg.speed));
        }
        return line(from, leg.end).linear(from, leg.end)
                .with(Constants.foresightConfig.maxPathSpeed.at(leg.speed));
    }

    private Command followLeg(Leg leg) {
        return followLeg(leg, null);
    }

    /**
     * Follows a leg. onStart (optional) runs in the same instant the path starts, so mechanism
     * changes like deploying the flower or stopping the intake need no separate command.
     * Timeout, normal finish and stall exit are all handled here: no race/wait commands, and
     * the follower is released the moment the leg ends.
     */
    // Local adapter avoids the incompatible PedroCommands.follow(PathChain) overload.
    private Command followLeg(Leg leg, Runnable onStart) {
        return new CommandBuilder()
                .setStart(() -> {
                    if (onStart != null) onStart.run();
                    pathStartTime = getRuntime();
                    stallRefPose = null;
                    currentPathTimedOut = false;
                    legDone = false;
                    follower.follow(buildPath(leg));
                })
                .setDone(() -> {
                    boolean finished = isLegFinished(leg);
                    if (finished) {
                        legDone = true;
                        releaseFollower();
                    }
                    return finished;
                });
    }

    private double distanceToEnd(Leg leg) {
        Pose pose = follower.pose();
        if (pose == null) return Double.MAX_VALUE;
        return Math.hypot(leg.end.x() - pose.x(), leg.end.y() - pose.y());
    }

    private boolean isLegFinished(Leg leg) {
        double now = getRuntime();

        // Timeout, measured from when this leg actually started.
        if (now - pathStartTime >= leg.timeoutSeconds) {
            currentPathTimedOut = true;
            anyPathTimeoutFired = true;
            lastTimedOutPath = leg.name;
            return true;
        }

        // Guard against a stale atParametricEnd from the previous path.
        if (now - pathStartTime < MIN_PATH_RUNTIME_SECONDS) return false;

        if (follower.atParametricEnd()) return true;

        Pose pose = follower.pose();
        if (pose == null) return false;

        double dist = distanceToEnd(leg);

        // Close enough in position and heading: don't wait for t to reach 1.
        double headingErrDeg = Math.abs(angleDiffDeg(
                Math.toDegrees(leg.end.heading()), Math.toDegrees(pose.heading())));
        if (dist <= END_POSITION_TOLERANCE_IN && headingErrDeg <= END_HEADING_TOLERANCE_DEG) {
            return true;
        }

        // Wall stall exit: near the target but not moving means we're against the wall.
        if (leg.stallExit && dist <= STALL_EXIT_MAX_DISTANCE_IN) {
            if (stallRefPose == null) {
                stallRefPose = pose;
                stallRefTime = now;
            } else {
                double moved = Math.hypot(pose.x() - stallRefPose.x(),
                        pose.y() - stallRefPose.y());
                if (moved > STALL_MOVE_TOLERANCE_IN) {
                    stallRefPose = pose;
                    stallRefTime = now;
                } else if (now - stallRefTime >= STALL_TIME_SECONDS) {
                    lastStallExitPath = leg.name;
                    return true;
                }
            }
        } else {
            stallRefPose = null;
        }

        return false;
    }

    private double angleDiffDeg(double a, double b) {
        double d = (a - b) % 360.0;
        if (d > 180.0) d -= 360.0;
        if (d < -180.0) d += 360.0;
        return d;
    }

    /**
     * Stops the follower from continuing to drive toward the old end pose once a leg is over.
     * Without this the robot keeps pushing at the end point during shooting and intake.
     * If follower.stop() doesn't zero the drive in your Pedro version, swap this one line
     * for follower.breakFollowing().
     */
    private void releaseFollower() {
        follower.stop();
    }

    @Override
    public void init() {
        Scheduler.reset();
        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setRed();
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
        turret.init(hardwareMap);
        flower.init(hardwareMap);
        robotSettings.init(hardwareMap);

        Turret.velocityCompensationActive = false;
        intake.stopIntake();
        outtake.stopOuttake();
        robotSettings.block();
        flower.flowerActivate(RobotConstants.flowerLiftPos);

        resetRuntimeState();
        setTravelTurretPosition();
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        PoseStorage.setRed();
        intake.update();

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        telemetry.addLine("===== FLOWER RED AUTO, FAST =====");
        telemetry.addData("Start Pose", start);
        telemetry.addData("Flower Approach Speed", "%.2f", FLOWER_APPROACH_SPEED);
        telemetry.addData("Flower Collect Speed", "%.2f", FLOWER_COLLECT_SPEED);
        telemetry.addData("Wall Approach Speed", "%.2f", WALL_APPROACH_SPEED);
        telemetry.addData("First Hold", "%.2f sec", FIRST_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Second Hold", "%.2f sec", SECOND_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Far Wall Turret", "%.3f", FAR_WALL_TURRET_POSITION);
        telemetry.update();
    }

    @Override
    public void start() {
        clearBulkCache();
        PoseStorage.setRed();
        follower.setPose(start);
        PoseStorage.setPose(start);
        resetRuntimeState();
        stopIntakeAndBlock();
        setTravelTurretPosition();
        lastLoopTime = getRuntime();
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        double loopNow = getRuntime();
        loopMs = (loopNow - lastLoopTime) * 1000.0;
        lastLoopTime = loopNow;
        if (loopMs > maxLoopMs) maxLoopMs = loopMs;

        clearBulkCache();
        Scheduler.execute();
        follower.update();

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        turret.setFixedServoPosition(activeTurretPosition);
        intake.update();
        outtake.setHoodPosition(RobotConstants.HOOD_POSITION);
        outtake.update();

        // Throttled: building and sending ~60 telemetry lines every loop skews loop timing.
        if (lastTelemetryTime < 0.0 || loopNow - lastTelemetryTime >= TELEMETRY_INTERVAL_SECONDS) {
            lastTelemetryTime = loopNow;
            displayTelemetry(currentPose);
        }
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (follower != null) {
            Pose finalPose = follower.pose();
            if (finalPose != null) PoseStorage.setPose(finalPose);
            follower.stop();
        }

        PoseStorage.setRed();
        if (outtake != null) outtake.stopOuttake();
        if (intake != null) intake.stopIntake();
        if (robotSettings != null) robotSettings.block();
        if (flower != null) flower.flowerActivate(RobotConstants.flowerLiftPos);
        if (turret != null) turret.setCentre();
    }

    private Command autoRoutine() {
        // followTravel / parkAndLift set the travel turret position themselves, so no separate
        // "set travel turret" commands are needed between steps.
        return sequential(
                shootCycle(PRELOAD_TURRET_POSITION, PRELOAD_FEED_SECONDS),
                followTravel(leg1()),
                collectFlower(leg2(), FIRST_FLOWER_INTAKE_HOLD_SECONDS, Double.NaN),
                reverseAndLiftFlower(),
                followTravel(leg4()),
                followTravelPreAim(leg5(), FAR_WALL_TURRET_POSITION),
                shootCycle(FAR_WALL_TURRET_POSITION, FLOWER_FEED_SECONDS),
                followTravel(leg6()),
                collectFlower(leg7(), SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        SECOND_FLOWER_TURRET_POSITION),
                shootCycle(SECOND_FLOWER_TURRET_POSITION, FLOWER_FEED_SECONDS),
                parkAndLiftFlower(),
                instant(this::finishAutoMechanisms)
        );
    }

    /**
     * Flower and intake deploy at the moment the path starts (no separate deploy wait), then
     * the intake holds at the pickup. aimAfterPosition (NaN for none) pre-aims the turret during
     * the hold. The intake is left running; whatever runs next stops it.
     */
    private Command collectFlower(Leg leg, double intakeHoldSeconds, double aimAfterPosition) {
        return sequential(
                followLeg(leg, this::holdFlowerCollection),
                race(
                        waitSeconds(intakeHoldSeconds),
                        infinite(() -> {
                            holdFlowerCollection();
                            preAimTurret(aimAfterPosition);
                        })
                )
        );
    }

    private void holdFlowerCollection() {
        robotSettings.block();
        outtake.setFeederTPS(0.0);
        flower.flowerActivate(RobotConstants.flowerDropPos);
        intake.intake(
                RobotConstants.FRONT_INTAKE_SPEED,
                RobotConstants.BACK_INTAKE_SPEED
        );
    }

    private void preAimTurret(double position) {
        if (PRE_AIM_TURRET && !Double.isNaN(position)) setFixedTurretPosition(position);
    }

    // Backs out of the first flower with the intake still running (the next travel leg stops it),
    // so any remaining pollen is pulled out. The turret is already at travel position.
    private Command reverseAndLiftFlower() {
        return parallel(
                followLeg(leg3(), this::holdFlowerCollection),
                sequential(
                        waitSeconds(FLOWER_REVERSE_CLEAR_SECONDS),
                        instant(() -> flower.flowerActivate(
                                RobotConstants.flowerLiftPos
                        ))
                )
        );
    }

    private Command followTravel(Leg leg) {
        return followLeg(leg, () -> {
            stopIntakeAndBlock();
            setTravelTurretPosition();
        });
    }

    /** Travel leg that moves the turret to aimPosition when close to the end of the leg. */
    private Command followTravelPreAim(Leg leg, double aimPosition) {
        return sequential(
                instant(() -> {
                    stopIntakeAndBlock();
                    setTravelTurretPosition();
                    legDone = false;
                }),
                parallel(
                        followLeg(leg),
                        sequential(
                                waitUntil(() -> legDone
                                        || (PRE_AIM_TURRET
                                        && distanceToEnd(leg) <= PRE_AIM_DISTANCE_IN)),
                                instant(() -> preAimTurret(aimPosition))
                        )
                )
        );
    }

    private Command shootCycle(double turretPosition, double feedSeconds) {
        return sequential(
                instant(() -> beginShot(turretPosition, feedSeconds)),
                race(
                        infinite(this::runShotStateMachine),
                        waitUntil(() -> shootPhase == ShootPhase.DONE)
                )
        );
    }

    private void beginShot(double turretPosition, double feedSeconds) {
        shotCount++;
        setFixedTurretPosition(turretPosition);
        outtake.setTargetTPS(RobotConstants.OUTTAKE_TPS);
        startShooterReadyWait();
        stopIntakeAndBlock();
        shootStartTime = getRuntime();
        shootFeedSeconds = feedSeconds;
        shootPhase = ShootPhase.WAIT;
    }

    /**
     * One state machine instead of several chained wait commands:
     *   WAIT    turret settling and shooter spinning up run together; blocker closed.
     *   RELEASE blocker opens (starting before the shooter is fully at speed); no feeding yet.
     *   FEED    feeds once the blocker has had time to open AND the shooter is ready/timed out.
     *   DONE    everything stopped.
     */
    private void runShotStateMachine() {
        double now = getRuntime();

        if (shootPhase == ShootPhase.WAIT) {
            boolean turretSettled = now - turretMoveTime >= TURRET_SETTLE_SECONDS;
            if (turretSettled
                    && (isShooterAtFraction(PRE_RELEASE_TPS_FRACTION)
                    || shooterReadyOrTimedOut())) {
                shootPhase = ShootPhase.RELEASE;
                releaseStartTime = now;
            }
        }

        if (shootPhase == ShootPhase.RELEASE
                && now - releaseStartTime >= BLOCK_RELEASE_SECONDS
                && shooterReadyOrTimedOut()) {
            shootPhase = ShootPhase.FEED;
            feedStartTime = now;
        }

        if (shootPhase == ShootPhase.FEED && now - feedStartTime >= shootFeedSeconds) {
            shootPhase = ShootPhase.DONE;
            stopIntakeAndBlock();
            return;
        }

        switch (shootPhase) {
            case WAIT:
                holdShooterAndBlock();
                break;
            case RELEASE:
                intake.stopIntake();
                outtake.setFeederTPS(0.0);
                robotSettings.release();
                break;
            case FEED:
                runShootFeed();
                break;
            case DONE:
            default:
                break;
        }
    }

    private void holdShooterAndBlock() {
        intake.stopIntake();
        outtake.setFeederTPS(0.0);
        robotSettings.block();
    }

    private void runShootFeed() {
        robotSettings.release();
        intake.intake(
                RobotConstants.FRONT_FEED_SPEED,
                RobotConstants.BACK_FEED_SPEED
        );
        outtake.setFeederTPS(RobotConstants.FEEDER_TPS);
    }

    private void stopIntakeAndBlock() {
        intake.stopIntake();
        outtake.setFeederTPS(0.0);
        robotSettings.block();
    }

    private void startShooterReadyWait() {
        shooterReadyStartTime = getRuntime();
        shooterReadyTimeoutFired = false;
    }

    private boolean shooterReadyOrTimedOut() {
        if (isShooterAtSpeed()) return true;

        if (getRuntime() - shooterReadyStartTime >= SHOOT_READY_TIMEOUT_SECONDS) {
            shooterReadyTimeoutFired = true;
            return true;
        }

        return false;
    }

    private boolean isShooterAtFraction(double fraction) {
        double targetTPS = outtake.getTargetTPS();
        double currentTPS = outtake.getCurrentTPS();
        return targetTPS > 0.0 && currentTPS >= targetTPS * clamp(fraction, 0.0, 1.0);
    }

    private boolean isShooterAtSpeed() {
        return isShooterAtFraction(TPS_READY_FRACTION);
    }

    private Command parkAndLiftFlower() {
        return parallel(
                followLeg(leg8(), () -> {
                    stopIntakeAndBlock();
                    setTravelTurretPosition();
                }),
                sequential(
                        waitSeconds(FLOWER_LIFT_DELAY_SECONDS),
                        instant(() -> flower.flowerActivate(
                                RobotConstants.flowerLiftPos
                        ))
                )
        );
    }

    private void finishAutoMechanisms() {
        stopIntakeAndBlock();
        outtake.setTargetTPS(0.0);
        outtake.setFeederTPS(0.0);
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        setTravelTurretPosition();
    }

    // Records when the turret was last told to move, so settle time is counted from the real
    // move (including pre-aims) rather than from the start of each shot.
    private void moveTurretTo(double position) {
        double target = clamp(position, 0.0, 1.0);
        if (Math.abs(target - activeTurretPosition) > 1e-6) turretMoveTime = getRuntime();
        activeTurretPosition = target;
        turret.setFixedServoPosition(activeTurretPosition);
    }

    private void setFixedTurretPosition(double position) {
        moveTurretTo(position + AUTO_TURRET_TRIM);
    }

    private void setTravelTurretPosition() {
        moveTurretTo(TRAVEL_TURRET_POSITION);
    }

    private Command waitSeconds(double seconds) {
        return waitMs(Math.round(Math.max(0.0, seconds) * 1000.0));
    }

    private void resetRuntimeState() {
        activeTurretPosition = TRAVEL_TURRET_POSITION;
        shotCount = 0;
        shooterReadyStartTime = 0.0;
        shooterReadyTimeoutFired = false;
        anyPathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";
        lastStallExitPath = "NONE";
        pathStartTime = 0.0;
        stallRefPose = null;
        stallRefTime = 0.0;
        lastLoopTime = 0.0;
        loopMs = 0.0;
        maxLoopMs = 0.0;
        lastTelemetryTime = -1.0;
        shootPhase = ShootPhase.DONE;
        shootStartTime = 0.0;
        releaseStartTime = 0.0;
        feedStartTime = 0.0;
        shootFeedSeconds = 0.0;
        turretMoveTime = 0.0;
        legDone = false;
    }

    private void displayTelemetry(Pose currentPose) {
        telemetry.addLine("===== FLOWER RED AUTO, FAST =====");

        if (currentPose != null) {
            telemetry.addData(
                    "Pose",
                    "(%.1f, %.1f, %.1f deg)",
                    currentPose.x(),
                    currentPose.y(),
                    Math.toDegrees(currentPose.heading())
            );
        }

        telemetry.addLine("===== AUTO TIME =====");
        telemetry.addData("Runtime", "%.2f sec", getRuntime());
        telemetry.addData("Time Remaining", "%.2f sec", Math.max(0.0, 30.0 - getRuntime()));
        telemetry.addData("Loop", "%.1f ms (max %.1f)", loopMs, maxLoopMs);
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Shot Count", shotCount);
        telemetry.addData("Shot Phase", shootPhase);

        telemetry.addLine("===== PATH STATUS =====");
        telemetry.addData("Any Timeout", anyPathTimeoutFired);
        telemetry.addData("Last Timeout", lastTimedOutPath);
        telemetry.addData("Last Stall Exit", lastStallExitPath);
        telemetry.addData("Current Path Timed Out", currentPathTimedOut);

        telemetry.addLine("===== SPEED SETTINGS =====");
        telemetry.addData("Drive", "%.2f", DRIVE_SPEED);
        telemetry.addData("Flower Approach", "%.2f", FLOWER_APPROACH_SPEED);
        telemetry.addData("Flower Collect", "%.2f", FLOWER_COLLECT_SPEED);
        telemetry.addData("Wall Approach", "%.2f", WALL_APPROACH_SPEED);

        telemetry.addLine("===== FLOWER =====");
        telemetry.addData("First Hold", "%.2f sec", FIRST_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Second Hold", "%.2f sec", SECOND_FLOWER_INTAKE_HOLD_SECONDS);
        telemetry.addData("Reverse Clear", "%.2f sec", FLOWER_REVERSE_CLEAR_SECONDS);
        telemetry.addData(
                "First Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point2.x(), point2.y(), Math.toDegrees(point2.heading())
        );
        telemetry.addData(
                "Second Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point7.x(), point7.y(), Math.toDegrees(point7.heading())
        );

        telemetry.addLine("===== TURRET =====");
        telemetry.addData("Active Position", "%.3f", activeTurretPosition);
        telemetry.addData("Far Wall Position", "%.3f", FAR_WALL_TURRET_POSITION);
        telemetry.addData("Servo Position", "%.3f", Turret.lastServoPosition);
        telemetry.addData("Fixed Mode", Turret.lastFixedPositionMode);

        telemetry.addLine("===== SHOOTER =====");
        telemetry.addData(
                "Shooter TPS",
                "%.0f / %.0f",
                outtake.getCurrentTPS(),
                outtake.getTargetTPS()
        );
        telemetry.addData("Shooter Ready", isShooterAtSpeed());
        telemetry.addData("Ready Timeout", shooterReadyTimeoutFired);
        telemetry.addData(
                "Feeder TPS",
                "%.0f / %.0f",
                outtake.getFeederCurrentTPS(),
                outtake.getFeederTargetTPS()
        );
        telemetry.addData("Hood Base", "%.3f", outtake.getHoodBase());
        telemetry.addData("Hood Compensation", "%.4f", outtake.getHoodCompensation());
        telemetry.addData("Hood Command", "%.3f", outtake.getHoodPosition());

        telemetry.addLine("===== INTAKE =====");
        telemetry.addData("Front Power", "%.2f", intake.getFrontMotorPower());
        telemetry.addData("Back Power", "%.2f", intake.getBackMotorPower());
        telemetry.update();
    }

    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void clearBulkCache() {
        if (allHubs == null) return;
        for (LynxModule hub : allHubs) hub.clearBulkCache();
    }

    private double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}