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

/**
 * Flower Red Auto.
 *
 * Changes in this version (tuned values and route are otherwise unchanged):
 *
 *  1. Shooter spins up in start() instead of inside the first shot.
 *  2. Turret settle and shooter-ready waits now run at the same time.
 *  3. Turret moves to its shot position during the preceding drive. The settle
 *     wait is skipped when the turret is already commanded to the shot position.
 *  4. Preload shot has its own (longer) ready timeout because the shooter
 *     starts from rest.
 *  5. Path 7 has its own collect speed tunable (defaults to the old value).
 *  6. Telemetry is throttled to keep the loop fast.
 *
 * The turret is NOT moved in init(); it moves to the preload position during
 * the first shot, overlapped with shooter spin-up.
 */
@Configurable
@Autonomous(name = "05 - Op Flower Red Auto", group = "Main")
public class opFlowerRedAuto extends OpMode {

    private List<LynxModule> allHubs;
    private Follower follower;
    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Values intentionally unchanged from the supplied auto.
    public static double PRELOAD_TURRET_POSITION = 0.0;
    public static double FAR_WALL_TURRET_POSITION = 0.30;
    public static double SECOND_FLOWER_TURRET_POSITION = 0.40;
    public static double TRAVEL_TURRET_POSITION = 0.50;
    public static double AUTO_TURRET_TRIM = 0.0;

    /* Turret within this of the shot position needs no settle wait. */
    public static double TURRET_SETTLE_TOLERANCE = 0.01;

    public static double DRIVE_SPEED = 1.00;
    public static double FLOWER_APPROACH_SPEED = 0.70;
    public static double FLOWER_COLLECT_SPEED = 0.35;
    public static double SECOND_FLOWER_COLLECT_SPEED = 0.35;
    public static double WALL_APPROACH_SPEED = 0.35;
    public static double PARK_SPEED = 1.00;

    public static double TURRET_SETTLE_SECONDS = 0.20;
    public static double SHOOT_READY_TIMEOUT_SECONDS = 0.70;
    public static double PRELOAD_READY_TIMEOUT_SECONDS = 1.00;
    public static double BLOCK_RELEASE_SECONDS = 0.40;
    public static double PRELOAD_FEED_SECONDS = 1.00;
    public static double FLOWER_FEED_SECONDS = 1.60;

    public static double FLOWER_DEPLOY_SECONDS = 0.20;
    public static double FIRST_FLOWER_INTAKE_HOLD_SECONDS = 0.75;
    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.45;
    public static double FLOWER_REVERSE_CLEAR_SECONDS = 0.35;
    public static double FLOWER_LIFT_DELAY_SECONDS = 0.20;

    public static double TPS_READY_FRACTION = 0.95;

    /* Minimum time between telemetry updates. */
    public static double TELEMETRY_INTERVAL_SECONDS = 0.15;

    public static double PATH1_TIMEOUT_SECONDS = 4.0;
    public static double PATH2_TIMEOUT_SECONDS = 3.0;
    public static double PATH3_TIMEOUT_SECONDS = 3.0;
    public static double PATH4_TIMEOUT_SECONDS = 5.0;
    public static double PATH5_TIMEOUT_SECONDS = 5.0;
    public static double PATH6_TIMEOUT_SECONDS = 4.0;
    public static double PATH7_TIMEOUT_SECONDS = 2.5;
    public static double PATH8_TIMEOUT_SECONDS = 5.0;

    private double activeTurretPosition = 0.50;
    private int shotCount = 0;

    private double shooterReadyStartTime = 0.0;
    private double activeReadyTimeoutSeconds = 0.70;
    private boolean shooterReadyTimeoutFired = false;

    private boolean turretNeedsSettle = false;
    private double turretSettleStartTime = 0.0;

    private boolean anyPathTimeoutFired = false;
    private boolean currentPathTimedOut = false;
    private String lastTimedOutPath = "NONE";

    private double lastTelemetryTime = -1.0;

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

    private Path path1() {
        return line(start, path1End).linear(start, path1End)
                .with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_APPROACH_SPEED));
    }

    private Path path2() {
        return line(point2Start, point2).linear(point2Start, point2)
                .with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_COLLECT_SPEED));
    }

    private Path path3() {
        return line(point2, point3).reverseTangent()
                .with(Constants.foresightConfig.maxPathSpeed.at(FLOWER_APPROACH_SPEED));
    }

    private Path path4() {
        return line(point4Start, point4).linear(point4Start, point4)
                .with(Constants.foresightConfig.maxPathSpeed.at(DRIVE_SPEED));
    }

    private Path path5() {
        return line(point5Start, point5).linear(point5Start, point5)
                .with(Constants.foresightConfig.maxPathSpeed.at(WALL_APPROACH_SPEED));
    }

    private Path path6() {
        return line(point6Start, point6).linear(point6Start, point6)
                .with(Constants.foresightConfig.maxPathSpeed.at(DRIVE_SPEED));
    }

    private Path path7() {
        return line(point7Start, point7).linear(point7Start, point7)
                .with(Constants.foresightConfig.maxPathSpeed.at(SECOND_FLOWER_COLLECT_SPEED));
    }

    private Path path8() {
        return line(point8Start, point8).linear(point8Start, point8)
                .with(Constants.foresightConfig.maxPathSpeed.at(PARK_SPEED));
    }

    // Local adapter avoids the incompatible PedroCommands.follow(PathChain) overload.
    private Command followPath(Path path) {
        return new CommandBuilder()
                .setStart(() -> follower.follow(path))
                .setDone(follower::atParametricEnd);
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
        telemetry.addData("Ball Sensor", intake.isSensorTriggered());
        telemetry.addData("Ball Count", intake.getConfirmedBallCount());
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

        /*
         * Spin the shooter up immediately. The preload shot's turret move and
         * spin-up now overlap, and the shooter stays at speed for the rest of
         * the auto (it is only zeroed in finishAutoMechanisms()).
         */
        outtake.setTargetTPS(RobotConstants.OUTTAKE_TPS);

        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        clearBulkCache();
        Scheduler.execute();
        follower.update();

        Pose currentPose = follower.pose();
        if (currentPose != null) PoseStorage.setPose(currentPose);

        turret.setFixedServoPosition(activeTurretPosition);
        intake.update();
        outtake.setHoodPosition(RobotConstants.HOOD_POSITION);
        outtake.update();
        displayTelemetry(currentPose);
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
        return sequential(
                // Preload: turret moves 0.50 -> 0.0 while the shooter spins up.
                shootCycle(
                        PRELOAD_TURRET_POSITION,
                        PRELOAD_FEED_SECONDS,
                        PRELOAD_READY_TIMEOUT_SECONDS
                ),

                followTravel(path1(), PATH1_TIMEOUT_SECONDS, "PATH 1"),

                collectFlower(path2(), PATH2_TIMEOUT_SECONDS,
                        FIRST_FLOWER_INTAKE_HOLD_SECONDS, "PATH 2"),

                reverseAndLiftFlower(),

                followTravel(path4(), PATH4_TIMEOUT_SECONDS, "PATH 4"),

                // Turret moves to the wall shot position during this drive.
                followTravel(
                        path5(),
                        PATH5_TIMEOUT_SECONDS,
                        "PATH 5",
                        () -> setFixedTurretPosition(FAR_WALL_TURRET_POSITION)
                ),

                shootCycle(
                        FAR_WALL_TURRET_POSITION,
                        FLOWER_FEED_SECONDS,
                        SHOOT_READY_TIMEOUT_SECONDS
                ),

                followTravel(path6(), PATH6_TIMEOUT_SECONDS, "PATH 6"),

                // Pre-position for the second shot; collection gives it time.
                instant(() -> setFixedTurretPosition(SECOND_FLOWER_TURRET_POSITION)),

                collectFlower(path7(), PATH7_TIMEOUT_SECONDS,
                        SECOND_FLOWER_INTAKE_HOLD_SECONDS, "PATH 7"),

                shootCycle(
                        SECOND_FLOWER_TURRET_POSITION,
                        FLOWER_FEED_SECONDS,
                        SHOOT_READY_TIMEOUT_SECONDS
                ),

                parkAndLiftFlower(),
                instant(this::finishAutoMechanisms)
        );
    }

    private Command collectFlower(
            Path path,
            double timeoutSeconds,
            double intakeHoldSeconds,
            String pathName
    ) {
        return sequential(
                instant(() -> {
                    robotSettings.block();
                    outtake.setFeederTPS(0.0);
                    flower.flowerActivate(RobotConstants.flowerDropPos);
                    intake.intake(
                            RobotConstants.FRONT_INTAKE_SPEED,
                            RobotConstants.BACK_INTAKE_SPEED
                    );
                }),
                waitSeconds(FLOWER_DEPLOY_SECONDS),
                followWithTimeout(path, timeoutSeconds, pathName),
                race(
                        waitSeconds(intakeHoldSeconds),
                        infinite(this::holdFlowerCollection)
                ),
                instant(this::stopIntakeAndBlock)
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

    private Command reverseAndLiftFlower() {
        return sequential(
                instant(() -> {
                    stopIntakeAndBlock();
                    setTravelTurretPosition();
                }),
                parallel(
                        followWithTimeout(path3(), PATH3_TIMEOUT_SECONDS, "PATH 3"),
                        sequential(
                                waitSeconds(FLOWER_REVERSE_CLEAR_SECONDS),
                                instant(() -> flower.flowerActivate(
                                        RobotConstants.flowerLiftPos
                                ))
                        )
                ),
                instant(this::stopIntakeAndBlock)
        );
    }

    /** Travel with the turret at the travel position. */
    private Command followTravel(Path path, double timeoutSeconds, String pathName) {
        return followTravel(path, timeoutSeconds, pathName, this::setTravelTurretPosition);
    }

    /** Travel, running the supplied turret action at the start of the path. */
    private Command followTravel(
            Path path,
            double timeoutSeconds,
            String pathName,
            Runnable turretAction
    ) {
        return sequential(
                instant(() -> {
                    stopIntakeAndBlock();
                    turretAction.run();
                }),
                followWithTimeout(path, timeoutSeconds, pathName),
                instant(this::stopIntakeAndBlock)
        );
    }

    private Command followWithTimeout(Path path, double timeoutSeconds, String pathName) {
        return sequential(
                instant(() -> currentPathTimedOut = false),
                race(
                        followPath(path),
                        sequential(
                                waitSeconds(timeoutSeconds),
                                instant(() -> {
                                    currentPathTimedOut = true;
                                    anyPathTimeoutFired = true;
                                    lastTimedOutPath = pathName;
                                    follower.stop();
                                })
                        )
                )
        );
    }

    private Command shootCycle(
            double turretPosition,
            double feedSeconds,
            double readyTimeoutSeconds
    ) {
        return sequential(
                instant(() -> {
                    shotCount++;

                    /*
                     * Only wait for the turret if it has to move. If it was
                     * already commanded here during the drive, skip the wait.
                     */
                    double target = clamp(turretPosition + AUTO_TURRET_TRIM, 0.0, 1.0);
                    turretNeedsSettle =
                            Math.abs(target - activeTurretPosition) > TURRET_SETTLE_TOLERANCE;
                    turretSettleStartTime = getRuntime();

                    setFixedTurretPosition(turretPosition);
                    outtake.setTargetTPS(RobotConstants.OUTTAKE_TPS);
                    startShooterReadyWait(readyTimeoutSeconds);
                    stopIntakeAndBlock();
                }),

                // Turret settle and shooter spin-up run together.
                race(
                        waitUntil(this::turretSettledAndShooterReady),
                        infinite(this::holdShooterAndBlock)
                ),

                race(
                        waitSeconds(BLOCK_RELEASE_SECONDS),
                        infinite(this::releaseBlocker)
                ),
                race(
                        waitSeconds(feedSeconds),
                        infinite(this::runShootFeed)
                ),
                instant(this::stopIntakeAndBlock)
        );
    }

    private boolean turretSettledAndShooterReady() {
        boolean turretOk = !turretNeedsSettle
                || getRuntime() - turretSettleStartTime >= TURRET_SETTLE_SECONDS;

        return turretOk && shooterReadyOrTimedOut();
    }

    private void holdShooterAndBlock() {
        intake.stopIntake();
        outtake.setFeederTPS(0.0);
        robotSettings.block();
    }

    private void releaseBlocker() {
        robotSettings.release();
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

    private void startShooterReadyWait(double timeoutSeconds) {
        shooterReadyStartTime = getRuntime();
        activeReadyTimeoutSeconds = timeoutSeconds;
        shooterReadyTimeoutFired = false;
    }

    private boolean shooterReadyOrTimedOut() {
        if (isShooterAtSpeed()) return true;

        if (getRuntime() - shooterReadyStartTime >= activeReadyTimeoutSeconds) {
            shooterReadyTimeoutFired = true;
            return true;
        }

        return false;
    }

    private boolean isShooterAtSpeed() {
        double targetTPS = outtake.getTargetTPS();
        double currentTPS = outtake.getCurrentTPS();
        double readyFraction = clamp(TPS_READY_FRACTION, 0.0, 1.0);
        return targetTPS > 0.0 && currentTPS >= targetTPS * readyFraction;
    }

    private Command parkAndLiftFlower() {
        return sequential(
                instant(() -> {
                    stopIntakeAndBlock();
                    setTravelTurretPosition();
                }),
                parallel(
                        followWithTimeout(path8(), PATH8_TIMEOUT_SECONDS, "PATH 8"),
                        sequential(
                                waitSeconds(FLOWER_LIFT_DELAY_SECONDS),
                                instant(() -> flower.flowerActivate(
                                        RobotConstants.flowerLiftPos
                                ))
                        )
                ),
                instant(this::stopIntakeAndBlock)
        );
    }

    private void finishAutoMechanisms() {
        stopIntakeAndBlock();
        outtake.setTargetTPS(0.0);
        outtake.setFeederTPS(0.0);
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        setTravelTurretPosition();
    }

    private void setFixedTurretPosition(double position) {
        activeTurretPosition = clamp(position + AUTO_TURRET_TRIM, 0.0, 1.0);
        turret.setFixedServoPosition(activeTurretPosition);
    }

    private void setTravelTurretPosition() {
        activeTurretPosition = clamp(TRAVEL_TURRET_POSITION, 0.0, 1.0);
        turret.setFixedServoPosition(activeTurretPosition);
    }

    private Command waitSeconds(double seconds) {
        return waitMs(Math.round(Math.max(0.0, seconds) * 1000.0));
    }

    private void resetRuntimeState() {
        activeTurretPosition = TRAVEL_TURRET_POSITION;
        shotCount = 0;
        shooterReadyStartTime = 0.0;
        activeReadyTimeoutSeconds = SHOOT_READY_TIMEOUT_SECONDS;
        shooterReadyTimeoutFired = false;
        turretNeedsSettle = false;
        turretSettleStartTime = 0.0;
        anyPathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";
        lastTelemetryTime = -1.0;
        if (intake != null) intake.resetConfirmedBallCount();
    }

    private void displayTelemetry(Pose currentPose) {
        /* Throttled: formatting and sending ~40 lines every loop is costly. */
        double now = getRuntime();
        if (lastTelemetryTime >= 0.0
                && now - lastTelemetryTime < TELEMETRY_INTERVAL_SECONDS) {
            return;
        }
        lastTelemetryTime = now;

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
        telemetry.addData("Runtime", "%.2f sec", now);
        telemetry.addData("Time Remaining", "%.2f sec", Math.max(0.0, 30.0 - now));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Shot Count", shotCount);

        telemetry.addLine("===== PATH STATUS =====");
        telemetry.addData("Any Timeout", anyPathTimeoutFired);
        telemetry.addData("Last Timeout", lastTimedOutPath);
        telemetry.addData("Current Path Timed Out", currentPathTimedOut);

        telemetry.addLine("===== SPEED SETTINGS =====");
        telemetry.addData("Drive", "%.2f", DRIVE_SPEED);
        telemetry.addData("Flower Approach", "%.2f", FLOWER_APPROACH_SPEED);
        telemetry.addData("Flower Collect", "%.2f", FLOWER_COLLECT_SPEED);
        telemetry.addData("Second Flower Collect", "%.2f", SECOND_FLOWER_COLLECT_SPEED);
        telemetry.addData("Wall Approach", "%.2f", WALL_APPROACH_SPEED);

        telemetry.addLine("===== FLOWER =====");
        telemetry.addData("Deploy Time", "%.2f sec", FLOWER_DEPLOY_SECONDS);
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
        telemetry.addData("Needs Settle", turretNeedsSettle);
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
        telemetry.addData("Sensor Triggered", intake.isSensorTriggered());
        telemetry.addData("Ball Confirmed", intake.isBallConfirmed());
        telemetry.addData("Ball Count", intake.getConfirmedBallCount());
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