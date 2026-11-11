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

import org.firstinspires.ftc.teamcode.opModes.subClasses.ArduCam;
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
@Autonomous(
        name = "01 - Hive Tip Flower Red Path 5 Auto",
        group = "Main"
)
public class DelayedFlowerRedPath5Auto extends OpMode {

    // ---------------------------------------------------------------------
    // Hardware and subsystems
    // ---------------------------------------------------------------------

    private List<LynxModule> allHubs;

    private Follower follower;

    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;
    private ArduCam camera;

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    // ---------------------------------------------------------------------
    // Camera and HIVE-tip detection
    //
    // Under the field convention used by this autonomous:
    //
    // RED FAR = high-Y side toward (0, 144)
    //
    // Red begins with the audience-side CELL upward. The first completed
    // tip is confirmed when the RED FAR CELL becomes the active/upward
    // CELL.
    // ---------------------------------------------------------------------

    public static String CAMERA_NAME = "Webcam 1";

    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;

    public static boolean VISION_ENABLED = true;
    public static boolean STREAM_CAMERA_TO_PANELS = false;

    public static double HIVE_TIP_WAIT_SECONDS = 10.0;

    /*
     * Number of consecutive observations required before the tip is
     * accepted.
     */
    public static int HIVE_TIP_CONFIRM_FRAMES = 3;

    // ---------------------------------------------------------------------
    // Turret positions
    // ---------------------------------------------------------------------

    public static double FAR_WALL_TURRET_POSITION = 0.30;
    public static double SECOND_FLOWER_TURRET_POSITION = 0.40;
    public static double TRAVEL_TURRET_POSITION = 0.50;

    public static double AUTO_TURRET_TRIM = 0.0;

    // ---------------------------------------------------------------------
    // Drive speeds
    // ---------------------------------------------------------------------

    public static double DRIVE_SPEED = 1.00;
    public static double FLOWER_COLLECT_SPEED = 0.35;
    public static double PARK_SPEED = 1.00;

    // ---------------------------------------------------------------------
    // Shooting timing
    // ---------------------------------------------------------------------

    public static double TURRET_SETTLE_SECONDS = 0.20;

    public static double SHOOT_READY_TIMEOUT_SECONDS = 0.70;

    public static double BLOCK_RELEASE_SECONDS = 0.40;

    public static double PRELOAD_FEED_SECONDS = 1.00;
    public static double FLOWER_FEED_SECONDS = 1.60;

    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.45;

    public static double FLOWER_LIFT_DELAY_SECONDS = 0.20;

    public static double TPS_READY_FRACTION = 0.95;
    public static double PRE_RELEASE_TPS_FRACTION = 0.80;

    public static boolean PRE_AIM_TURRET = true;

    // ---------------------------------------------------------------------
    // Path timing and completion
    // ---------------------------------------------------------------------

    public static double PATH6_TIMEOUT_SECONDS = 4.0;
    public static double PATH7_TIMEOUT_SECONDS = 2.5;
    public static double PATH8_TIMEOUT_SECONDS = 5.0;

    public static double MIN_PATH_RUNTIME_SECONDS = 0.05;

    public static double END_POSITION_TOLERANCE_IN = 1.5;
    public static double END_HEADING_TOLERANCE_DEG = 10.0;

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    public static double TELEMETRY_INTERVAL_SECONDS = 0.25;

    // ---------------------------------------------------------------------
    // Mechanism and path state
    // ---------------------------------------------------------------------

    private double activeTurretPosition = 0.50;

    private int shotCount = 0;

    private double shooterReadyStartTime = 0.0;

    private boolean shooterReadyTimeoutFired = false;

    private boolean anyPathTimeoutFired = false;
    private boolean currentPathTimedOut = false;

    private String lastTimedOutPath = "NONE";

    private boolean autoFinished = false;

    private double pathStartTime = 0.0;
    private double turretMoveTime = 0.0;

    // ---------------------------------------------------------------------
    // HIVE wait state
    // ---------------------------------------------------------------------

    private double hiveTipWaitStartTime = 0.0;

    private boolean hiveTipWaitActive = false;

    private int hiveTipConfirmCount = 0;

    private boolean hiveTipConfirmed = false;
    private boolean hiveTipTimedOut = false;

    /*
     * After a HIVE timeout:
     *
     * - shooting is skipped;
     * - Paths 6 and 7 are skipped;
     * - flower collection is skipped;
     * - Path 8 still runs from the robot's actual current pose.
     */
    private boolean skipToPark = false;

    // ---------------------------------------------------------------------
    // Shooting state machine
    // ---------------------------------------------------------------------

    private enum ShootPhase {
        WAIT,
        RELEASE,
        FEED,
        DONE
    }

    private ShootPhase shootPhase =
            ShootPhase.DONE;

    private double releaseStartTime = 0.0;
    private double feedStartTime = 0.0;
    private double shootFeedSeconds = 0.0;

    // ---------------------------------------------------------------------
    // Loop timing
    // ---------------------------------------------------------------------

    private double lastLoopTime = 0.0;
    private double loopMs = 0.0;
    private double maxLoopMs = 0.0;

    private double lastTelemetryTime = -1.0;

    // ---------------------------------------------------------------------
    // Poses
    //
    // Start is the original Path 5 endpoint.
    // ---------------------------------------------------------------------

    private final Pose start =
            poseFactory.of(
                    23.7064,
                    130.1432,
                    90.0
            );

    private final Pose point6 =
            poseFactory.of(
                    42.0,
                    104.1657,
                    90.0
            );

    private final Pose point7Start =
            poseFactory.of(
                    42.0,
                    104.1657,
                    90.0
            );

    private final Pose point7 =
            poseFactory.of(
                    42.0,
                    127.0,
                    75.0
            );

    private final Pose point8Start =
            poseFactory.of(
                    42.0,
                    127.0,
                    75.0
            );

    private final Pose point8 =
            poseFactory.of(
                    5.0,
                    116.1719,
                    0.0
            );

    // ---------------------------------------------------------------------
    // Path leg
    // ---------------------------------------------------------------------

    private static class Leg {

        final Pose plannedStart;
        final Pose end;

        final double speed;
        final double timeoutSeconds;

        final String name;

        Leg(
                Pose plannedStart,
                Pose end,
                double speed,
                double timeoutSeconds,
                String name
        ) {

            this.plannedStart = plannedStart;
            this.end = end;

            this.speed = speed;
            this.timeoutSeconds = timeoutSeconds;

            this.name = name;
        }
    }

    private Leg leg6() {

        return new Leg(
                start,
                point6,
                DRIVE_SPEED,
                PATH6_TIMEOUT_SECONDS,
                "PATH 6"
        );
    }

    private Leg leg7() {

        return new Leg(
                point7Start,
                point7,
                FLOWER_COLLECT_SPEED,
                PATH7_TIMEOUT_SECONDS,
                "PATH 7"
        );
    }

    private Leg leg8() {

        return new Leg(
                point8Start,
                point8,
                PARK_SPEED,
                PATH8_TIMEOUT_SECONDS,
                "PATH 8"
        );
    }

    // ---------------------------------------------------------------------
    // Path creation
    //
    // The path begins from follower.pose() whenever possible.
    //
    // If the HIVE wait times out, Path 8 therefore becomes a direct path
    // from the starting area to the final parking pose.
    // ---------------------------------------------------------------------

    private Path buildPath(
            Leg leg
    ) {

        Pose from =
                follower.pose();

        if (from == null) {
            from = leg.plannedStart;
        }

        return line(
                from,
                leg.end
        )
                .linear(
                        from,
                        leg.end
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        leg.speed
                                )
                );
    }

    private Command followLeg(
            Leg leg
    ) {

        return followLeg(
                leg,
                null
        );
    }

    /**
     * Path 8 always executes.
     *
     * Paths 6 and 7 complete immediately when skipToPark is true.
     */
    private Command followLeg(
            Leg leg,
            Runnable onStart
    ) {

        return new CommandBuilder()
                .setStart(() -> {

                    currentPathTimedOut = false;

                    if (skipToPark
                            && !"PATH 8".equals(
                            leg.name
                    )) {

                        releaseFollower();

                        return;
                    }

                    if (onStart != null) {
                        onStart.run();
                    }

                    pathStartTime =
                            getRuntime();

                    follower.follow(
                            buildPath(leg)
                    );
                })
                .setDone(() -> {

                    if (skipToPark
                            && !"PATH 8".equals(
                            leg.name
                    )) {

                        return true;
                    }

                    boolean finished =
                            isLegFinished(leg);

                    if (finished) {
                        releaseFollower();
                    }

                    return finished;
                });
    }

    private boolean isLegFinished(
            Leg leg
    ) {

        double now =
                getRuntime();

        double timeout =
                Math.max(
                        0.0,
                        leg.timeoutSeconds
                );

        if (now - pathStartTime >= timeout) {

            currentPathTimedOut = true;
            anyPathTimeoutFired = true;
            lastTimedOutPath = leg.name;

            return true;
        }

        /*
         * Prevent a stale end flag from completing a newly started path.
         */
        if (now - pathStartTime
                < Math.max(
                0.0,
                MIN_PATH_RUNTIME_SECONDS
        )) {

            return false;
        }

        if (follower.atParametricEnd()) {
            return true;
        }

        Pose currentPose =
                follower.pose();

        if (currentPose == null) {
            return false;
        }

        double distance =
                Math.hypot(
                        leg.end.x()
                                - currentPose.x(),
                        leg.end.y()
                                - currentPose.y()
                );

        double headingErrorDegrees =
                Math.abs(
                        angleDiffDeg(
                                Math.toDegrees(
                                        leg.end.heading()
                                ),
                                Math.toDegrees(
                                        currentPose.heading()
                                )
                        )
                );

        return distance
                <= Math.max(
                0.0,
                END_POSITION_TOLERANCE_IN
        )
                && headingErrorDegrees
                <= Math.max(
                0.0,
                END_HEADING_TOLERANCE_DEG
        );
    }

    private double angleDiffDeg(
            double targetDegrees,
            double currentDegrees
    ) {

        double difference =
                (targetDegrees - currentDegrees)
                        % 360.0;

        if (difference > 180.0) {
            difference -= 360.0;
        }

        if (difference < -180.0) {
            difference += 360.0;
        }

        return difference;
    }

    private void releaseFollower() {

        if (follower != null) {
            follower.stop();
        }
    }

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    @Override
    public void init() {

        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setRed();
        PoseStorage.setPose(start);

        follower =
                Constants.create(
                        hardwareMap
                );

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

        initialiseCamera();

        Turret.velocityCompensationActive = false;

        intake.stopIntake();
        outtake.stopOuttake();

        robotSettings.block();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        resetRuntimeState();

        setTravelTurretPosition();
    }

    private void initialiseCamera() {

        camera = null;

        if (!VISION_ENABLED) {
            return;
        }

        camera = new ArduCam(
                hardwareMap,
                CAMERA_NAME
        ).setResolution(
                CAMERA_WIDTH,
                CAMERA_HEIGHT
        );

        camera.init();
        camera.resetHiveState();

        if (STREAM_CAMERA_TO_PANELS) {
            camera.streamToPanels();
        }
    }

    // ---------------------------------------------------------------------
    // INIT loop
    // ---------------------------------------------------------------------

    @Override
    public void init_loop() {

        clearBulkCache();

        PoseStorage.setRed();

        intake.update();

        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }

        telemetry.addLine(
                "===== RED PATH 5 HIVE-TIP AUTO ====="
        );

        telemetry.addData(
                "Start Pose",
                start
        );

        telemetry.addData(
                "HIVE Tip Timeout",
                "%.2f sec",
                HIVE_TIP_WAIT_SECONDS
        );

        telemetry.addData(
                "Required Tip Frames",
                HIVE_TIP_CONFIRM_FRAMES
        );

        telemetry.addData(
                "Tip Target",
                "RED FAR / high-Y / (0,144) side"
        );

        telemetry.addData(
                "Camera Available",
                camera != null
        );

        telemetry.addData(
                "Camera Streaming",
                camera != null
                        && camera.isStreaming()
        );

        if (camera != null) {

            telemetry.addData(
                    "Visible Clusters",
                    camera.getVisibleClusterDetails()
            );

            telemetry.addData(
                    "Active Red CELL",
                    camera.getActiveCell(
                            ArduCam.Alliance.RED
                    )
            );
        }

        telemetry.addData(
                "Far Wall Turret",
                "%.3f",
                FAR_WALL_TURRET_POSITION
        );

        telemetry.addData(
                "Second Flower Turret",
                "%.3f",
                SECOND_FLOWER_TURRET_POSITION
        );

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // Start
    // ---------------------------------------------------------------------

    @Override
    public void start() {

        clearBulkCache();

        PoseStorage.setRed();

        follower.setPose(start);
        PoseStorage.setPose(start);

        resetRuntimeState();

        stopIntakeAndBlock();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setTravelTurretPosition();

        lastLoopTime =
                getRuntime();

        schedule(
                autoRoutine()
        );
    }

    // ---------------------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------------------

    @Override
    public void loop() {

        double loopNow =
                getRuntime();

        loopMs =
                (loopNow - lastLoopTime)
                        * 1000.0;

        lastLoopTime =
                loopNow;

        if (loopMs > maxLoopMs) {
            maxLoopMs = loopMs;
        }

        clearBulkCache();

        /*
         * VisionPortal processing is asynchronous. This reads the most
         * recent completed frame before the scheduler checks whether the
         * HIVE wait command is complete.
         */
        updateHiveTipObservation();

        if (!autoFinished) {

            Scheduler.execute();
            follower.update();

        } else {

            maintainFinishedState();
        }

        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }

        if (!autoFinished) {

            /*
             * Preserve the original fixed-position autonomous turret
             * operation.
             */
            turret.setFixedServoPosition(
                    activeTurretPosition
            );

            intake.update();

            outtake.setHoodPosition(
                    RobotConstants.HOOD_POSITION
            );

            outtake.update();
        }

        if (lastTelemetryTime < 0.0
                || loopNow - lastTelemetryTime
                >= Math.max(
                0.05,
                TELEMETRY_INTERVAL_SECONDS
        )) {

            lastTelemetryTime =
                    loopNow;

            displayTelemetry(
                    currentPose
            );
        }
    }

    // ---------------------------------------------------------------------
    // HIVE tip observation
    // ---------------------------------------------------------------------

    /**
     * Confirms the first Red HIVE tip when RED FAR becomes the active
     * upward CELL for the configured number of consecutive frames.
     */
    private void updateHiveTipObservation() {

        if (!hiveTipWaitActive
                || camera == null
                || !camera.isEnabled()
                || hiveTipConfirmed
                || hiveTipTimedOut) {

            return;
        }

        camera.updateHive(
                ArduCam.Alliance.RED
        );

        ArduCam.HiveCell activeCell =
                camera.getActiveCell(
                        ArduCam.Alliance.RED
                );

        boolean redFarIsUp =
                activeCell != null
                        && activeCell.alliance
                        == ArduCam.Alliance.RED
                        && activeCell.side
                        == ArduCam.HiveSide.FAR;

        if (redFarIsUp) {

            hiveTipConfirmCount++;

        } else {

            hiveTipConfirmCount = 0;
        }

        int requiredFrames =
                Math.max(
                        1,
                        HIVE_TIP_CONFIRM_FRAMES
                );

        if (hiveTipConfirmCount
                >= requiredFrames) {

            hiveTipConfirmed = true;
            hiveTipWaitActive = false;
        }
    }

    /**
     * Completes when either:
     *
     * - RED FAR is confirmed upward; or
     * - HIVE_TIP_WAIT_SECONDS expires.
     */
    private Command waitForHiveTipOrTimeout() {

        return new CommandBuilder()
                .setStart(() -> {

                    hiveTipWaitStartTime =
                            getRuntime();

                    hiveTipWaitActive = true;

                    hiveTipConfirmCount = 0;

                    hiveTipConfirmed = false;
                    hiveTipTimedOut = false;

                    skipToPark = false;

                    if (camera != null) {
                        camera.resetHiveState();
                    }
                })
                .setDone(() -> {

                    if (hiveTipConfirmed) {

                        hiveTipWaitActive = false;

                        return true;
                    }

                    double timeout =
                            Math.max(
                                    0.0,
                                    HIVE_TIP_WAIT_SECONDS
                            );

                    if (getRuntime()
                            - hiveTipWaitStartTime
                            >= timeout) {

                        hiveTipWaitActive = false;
                        hiveTipTimedOut = true;
                        skipToPark = true;

                        prepareDirectPark();

                        return true;
                    }

                    return false;
                });
    }

    /**
     * Safely abandons all shot and collection actions before parking.
     */
    private void prepareDirectPark() {

        shootPhase =
                ShootPhase.DONE;

        stopIntakeAndBlock();

        outtake.setTargetTPS(0.0);
        outtake.setFeederTPS(0.0);
        outtake.stopFeeder();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setTravelTurretPosition();

        releaseFollower();
    }

    // ---------------------------------------------------------------------
    // Stop
    // ---------------------------------------------------------------------

    @Override
    public void stop() {

        Scheduler.reset();

        if (follower != null) {

            Pose finalPose =
                    follower.pose();

            if (finalPose != null) {

                PoseStorage.setPose(
                        finalPose
                );
            }

            follower.stop();
        }

        PoseStorage.setRed();

        if (camera != null) {

            camera.close();
            camera = null;
        }

        shutdownAllSystems();

        restoreBulkCaching();
    }

    // ---------------------------------------------------------------------
    // Autonomous sequence
    // ---------------------------------------------------------------------

    private Command autoRoutine() {

        return sequential(

                /*
                 * Prepare immediately:
                 *
                 * - turret at preload position;
                 * - shooter spinning;
                 * - blocker closed;
                 * - feeder stopped;
                 * - intake stopped.
                 */
                instant(
                        this::preparePreload
                ),

                /*
                 * Wait for the HIVE tip or timeout.
                 */
                race(
                        waitForHiveTipOrTimeout(),
                        infinite(
                                this::holdShooterAndBlock
                        )
                ),

                /*
                 * These execute normally after tip confirmation.
                 *
                 * After timeout they complete immediately without moving
                 * or feeding.
                 */
                shootCycle(
                        FAR_WALL_TURRET_POSITION,
                        PRELOAD_FEED_SECONDS
                ),

                followTravel(
                        leg6()
                ),

                collectFlower(
                        leg7(),
                        SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        SECOND_FLOWER_TURRET_POSITION
                ),

                shootCycle(
                        SECOND_FLOWER_TURRET_POSITION,
                        FLOWER_FEED_SECONDS
                ),

                /*
                 * Path 8 always runs.
                 *
                 * Because buildPath() starts at follower.pose(), timeout
                 * produces a direct path to point8.
                 */
                parkAndLiftFlower(),

                instant(
                        this::shutdownAllSystems
                )
        );
    }

    private void preparePreload() {

        stopIntakeAndBlock();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setFixedTurretPosition(
                FAR_WALL_TURRET_POSITION
        );

        outtake.setTargetTPS(
                RobotConstants.OUTTAKE_TPS
        );
    }

    // ---------------------------------------------------------------------
    // Travel and flower collection
    // ---------------------------------------------------------------------

    private Command followTravel(
            Leg leg
    ) {

        return followLeg(
                leg,
                () -> {

                    stopIntakeAndBlock();

                    setTravelTurretPosition();
                }
        );
    }

    private Command collectFlower(
            Leg leg,
            double intakeHoldSeconds,
            double aimAfterPosition
    ) {

        return sequential(

                followLeg(
                        leg,
                        () -> {

                            if (!skipToPark) {
                                holdFlowerCollection();
                            }
                        }
                ),

                /*
                 * Nested race is used so this remains compatible with Ivy
                 * versions whose race() overload accepts only two commands.
                 */
                race(
                        race(
                                waitSeconds(
                                        intakeHoldSeconds
                                ),
                                waitUntil(
                                        () -> skipToPark
                                )
                        ),
                        infinite(
                                () -> {

                                    if (!skipToPark) {

                                        holdFlowerCollection();

                                        preAimTurret(
                                                aimAfterPosition
                                        );
                                    }
                                }
                        )
                )
        );
    }

    private void holdFlowerCollection() {

        if (skipToPark) {
            return;
        }

        robotSettings.block();

        outtake.setFeederTPS(0.0);

        flower.flowerActivate(
                RobotConstants.flowerDropPos
        );

        intake.intake(
                RobotConstants.FRONT_INTAKE_SPEED,
                RobotConstants.BACK_INTAKE_SPEED
        );
    }

    private void preAimTurret(
            double position
    ) {

        if (skipToPark) {
            return;
        }

        if (PRE_AIM_TURRET
                && Double.isFinite(position)) {

            setFixedTurretPosition(
                    position
            );
        }
    }

    // ---------------------------------------------------------------------
    // Shooting
    // ---------------------------------------------------------------------

    private Command shootCycle(
            double turretPosition,
            double feedSeconds
    ) {

        return sequential(

                instant(
                        () -> beginShot(
                                turretPosition,
                                feedSeconds
                        )
                ),

                race(
                        infinite(
                                this::runShotStateMachine
                        ),
                        waitUntil(
                                () -> shootPhase
                                        == ShootPhase.DONE
                        )
                )
        );
    }

    private void beginShot(
            double turretPosition,
            double feedSeconds
    ) {

        if (skipToPark) {

            shootPhase =
                    ShootPhase.DONE;

            stopIntakeAndBlock();

            outtake.setTargetTPS(0.0);

            return;
        }

        shotCount++;

        setFixedTurretPosition(
                turretPosition
        );

        outtake.setTargetTPS(
                RobotConstants.OUTTAKE_TPS
        );

        startShooterReadyWait();

        stopIntakeAndBlock();

        shootFeedSeconds =
                Math.max(
                        0.0,
                        feedSeconds
                );

        shootPhase =
                ShootPhase.WAIT;
    }

    private void runShotStateMachine() {

        if (skipToPark) {

            shootPhase =
                    ShootPhase.DONE;

            stopIntakeAndBlock();

            outtake.setTargetTPS(0.0);

            return;
        }

        double now =
                getRuntime();

        if (shootPhase == ShootPhase.WAIT) {

            boolean turretSettled =
                    now - turretMoveTime
                            >= Math.max(
                            0.0,
                            TURRET_SETTLE_SECONDS
                    );

            boolean releaseAllowed =
                    isShooterAtFraction(
                            PRE_RELEASE_TPS_FRACTION
                    )
                            || shooterReadyOrTimedOut();

            if (turretSettled
                    && releaseAllowed) {

                shootPhase =
                        ShootPhase.RELEASE;

                releaseStartTime =
                        now;
            }
        }

        if (shootPhase == ShootPhase.RELEASE
                && now - releaseStartTime
                >= Math.max(
                0.0,
                BLOCK_RELEASE_SECONDS
        )
                && shooterReadyOrTimedOut()) {

            shootPhase =
                    ShootPhase.FEED;

            feedStartTime =
                    now;
        }

        if (shootPhase == ShootPhase.FEED
                && now - feedStartTime
                >= shootFeedSeconds) {

            shootPhase =
                    ShootPhase.DONE;

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
                outtake.stopFeeder();

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
        outtake.stopFeeder();

        robotSettings.block();
    }

    private void runShootFeed() {

        robotSettings.release();

        intake.intake(
                RobotConstants.FRONT_FEED_SPEED,
                RobotConstants.BACK_FEED_SPEED
        );

        outtake.setFeederTPS(
                RobotConstants.FEEDER_TPS
        );
    }

    private void stopIntakeAndBlock() {

        intake.stopIntake();

        outtake.setFeederTPS(0.0);
        outtake.stopFeeder();

        robotSettings.block();
    }

    private void startShooterReadyWait() {

        shooterReadyStartTime =
                getRuntime();

        shooterReadyTimeoutFired =
                false;
    }

    private boolean shooterReadyOrTimedOut() {

        if (isShooterAtSpeed()) {
            return true;
        }

        if (getRuntime()
                - shooterReadyStartTime
                >= Math.max(
                0.0,
                SHOOT_READY_TIMEOUT_SECONDS
        )) {

            shooterReadyTimeoutFired = true;

            return true;
        }

        return false;
    }

    private boolean isShooterAtFraction(
            double fraction
    ) {

        double targetTPS =
                outtake.getTargetTPS();

        double currentTPS =
                outtake.getCurrentTPS();

        return targetTPS > 0.0
                && Double.isFinite(currentTPS)
                && currentTPS
                >= targetTPS
                * clamp(
                fraction,
                0.0,
                1.0
        );
    }

    private boolean isShooterAtSpeed() {

        return isShooterAtFraction(
                TPS_READY_FRACTION
        );
    }

    // ---------------------------------------------------------------------
    // Final parking
    // ---------------------------------------------------------------------

    private Command parkAndLiftFlower() {

        return parallel(

                followLeg(
                        leg8(),
                        () -> {

                            stopIntakeAndBlock();

                            /*
                             * Do not keep the flywheel running during the
                             * final parking path.
                             */
                            outtake.setTargetTPS(0.0);

                            setTravelTurretPosition();
                        }
                ),

                sequential(

                        waitSeconds(
                                FLOWER_LIFT_DELAY_SECONDS
                        ),

                        instant(
                                () -> flower.flowerActivate(
                                        RobotConstants.flowerLiftPos
                                )
                        )
                )
        );
    }

    // ---------------------------------------------------------------------
    // End-of-auto safety
    // ---------------------------------------------------------------------

    private void shutdownAllSystems() {

        autoFinished = true;
        hiveTipWaitActive = false;

        if (follower != null) {
            follower.stop();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (outtake != null) {

            outtake.setTargetTPS(0.0);
            outtake.setFeederTPS(0.0);

            outtake.stopOuttake();
        }

        if (robotSettings != null) {
            robotSettings.block();
        }

        if (flower != null) {

            flower.flowerActivate(
                    RobotConstants.flowerLiftPos
            );
        }

        activeTurretPosition =
                TRAVEL_TURRET_POSITION;

        if (turret != null) {
            turret.setCentre();
        }
    }

    private void maintainFinishedState() {

        if (follower != null) {
            follower.stop();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (outtake != null) {

            outtake.setTargetTPS(0.0);
            outtake.setFeederTPS(0.0);
            outtake.stopFeeder();
        }

        if (robotSettings != null) {
            robotSettings.block();
        }
    }

    // ---------------------------------------------------------------------
    // Fixed autonomous turret commands
    // ---------------------------------------------------------------------

    private void moveTurretTo(
            double position
    ) {

        double target =
                clamp(
                        position,
                        0.0,
                        1.0
                );

        if (Math.abs(
                target - activeTurretPosition
        ) > 1e-6) {

            turretMoveTime =
                    getRuntime();
        }

        activeTurretPosition =
                target;

        turret.setFixedServoPosition(
                activeTurretPosition
        );
    }

    private void setFixedTurretPosition(
            double position
    ) {

        moveTurretTo(
                position
                        + AUTO_TURRET_TRIM
        );
    }

    private void setTravelTurretPosition() {

        moveTurretTo(
                TRAVEL_TURRET_POSITION
        );
    }

    // ---------------------------------------------------------------------
    // Command helpers and runtime reset
    // ---------------------------------------------------------------------

    private Command waitSeconds(
            double seconds
    ) {

        return waitMs(
                Math.round(
                        Math.max(
                                0.0,
                                seconds
                        )
                                * 1000.0
                )
        );
    }

    private void resetRuntimeState() {

        activeTurretPosition =
                TRAVEL_TURRET_POSITION;

        shotCount = 0;

        shooterReadyStartTime = 0.0;
        shooterReadyTimeoutFired = false;

        anyPathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";

        autoFinished = false;

        pathStartTime = 0.0;
        turretMoveTime = 0.0;

        shootPhase =
                ShootPhase.DONE;

        releaseStartTime = 0.0;
        feedStartTime = 0.0;
        shootFeedSeconds = 0.0;

        hiveTipWaitStartTime = 0.0;
        hiveTipWaitActive = false;

        hiveTipConfirmCount = 0;

        hiveTipConfirmed = false;
        hiveTipTimedOut = false;

        skipToPark = false;

        lastLoopTime = 0.0;
        loopMs = 0.0;
        maxLoopMs = 0.0;

        lastTelemetryTime = -1.0;

        if (camera != null) {
            camera.resetHiveState();
        }
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void displayTelemetry(
            Pose currentPose
    ) {

        telemetry.addLine(
                "===== RED PATH 5 HIVE-TIP AUTO ====="
        );

        if (currentPose != null) {

            telemetry.addData(
                    "Pose",
                    "(%.1f, %.1f, %.1f deg)",
                    currentPose.x(),
                    currentPose.y(),
                    Math.toDegrees(
                            currentPose.heading()
                    )
            );
        }

        telemetry.addLine(
                "===== AUTO TIME ====="
        );

        telemetry.addData(
                "Runtime",
                "%.2f sec",
                getRuntime()
        );

        telemetry.addData(
                "Time Remaining",
                "%.2f sec",
                Math.max(
                        0.0,
                        30.0 - getRuntime()
                )
        );

        telemetry.addData(
                "Loop",
                "%.1f ms (max %.1f)",
                loopMs,
                maxLoopMs
        );

        telemetry.addData(
                "Auto Finished",
                autoFinished
        );

        telemetry.addData(
                "Shot Count",
                shotCount
        );

        telemetry.addData(
                "Shot Phase",
                shootPhase
        );

        telemetry.addLine(
                "===== HIVE TIP WAIT ====="
        );

        double tipWaitElapsed =
                hiveTipWaitActive
                        || hiveTipConfirmed
                        || hiveTipTimedOut
                        ? Math.max(
                        0.0,
                        getRuntime()
                        - hiveTipWaitStartTime
                )
                        : 0.0;

        telemetry.addData(
                "Tip Target",
                "RED FAR / high-Y"
        );

        telemetry.addData(
                "Wait Active",
                hiveTipWaitActive
        );

        telemetry.addData(
                "Tip Wait",
                "%.2f / %.2f sec",
                tipWaitElapsed,
                HIVE_TIP_WAIT_SECONDS
        );

        telemetry.addData(
                "Tip Frames",
                "%d / %d",
                hiveTipConfirmCount,
                Math.max(
                        1,
                        HIVE_TIP_CONFIRM_FRAMES
                )
        );

        telemetry.addData(
                "Tip Confirmed",
                hiveTipConfirmed
        );

        telemetry.addData(
                "Tip Timed Out",
                hiveTipTimedOut
        );

        telemetry.addData(
                "Skip To Park",
                skipToPark
        );

        telemetry.addData(
                "Camera Available",
                camera != null
        );

        telemetry.addData(
                "Camera Streaming",
                camera != null
                        && camera.isStreaming()
        );

        if (camera != null) {

            telemetry.addData(
                    "Red HIVE Visible",
                    camera.isHiveVisible(
                            ArduCam.Alliance.RED
                    )
            );

            telemetry.addData(
                    "Active Red CELL",
                    camera.getActiveCell(
                            ArduCam.Alliance.RED
                    )
            );

            telemetry.addData(
                    "Visible Clusters",
                    camera.getVisibleClusterDetails()
            );
        }

        telemetry.addLine(
                "===== PATH STATUS ====="
        );

        telemetry.addData(
                "Any Timeout",
                anyPathTimeoutFired
        );

        telemetry.addData(
                "Last Timeout",
                lastTimedOutPath
        );

        telemetry.addData(
                "Current Path Timed Out",
                currentPathTimedOut
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.addLine(
                "===== TURRET ====="
        );

        telemetry.addData(
                "Active Position",
                "%.3f",
                activeTurretPosition
        );

        telemetry.addData(
                "Far Wall Position",
                "%.3f",
                FAR_WALL_TURRET_POSITION
        );

        telemetry.addData(
                "Second Flower Position",
                "%.3f",
                SECOND_FLOWER_TURRET_POSITION
        );

        telemetry.addData(
                "Servo Position",
                "%.3f",
                Turret.lastServoPosition
        );

        telemetry.addData(
                "Fixed Mode",
                Turret.lastFixedPositionMode
        );

        telemetry.addLine(
                "===== SHOOTER ====="
        );

        telemetry.addData(
                "Shooter TPS",
                "%.0f / %.0f",
                outtake.getCurrentTPS(),
                outtake.getTargetTPS()
        );

        telemetry.addData(
                "Shooter Ready",
                isShooterAtSpeed()
        );

        telemetry.addData(
                "Ready Timeout",
                shooterReadyTimeoutFired
        );

        telemetry.addData(
                "Feeder TPS",
                "%.0f / %.0f",
                outtake.getFeederCurrentTPS(),
                outtake.getFeederTargetTPS()
        );

        telemetry.addData(
                "Hood Base",
                "%.3f",
                outtake.getHoodBase()
        );

        telemetry.addData(
                "Hood Compensation",
                "%.4f",
                outtake.getHoodCompensation()
        );

        telemetry.addData(
                "Hood Command",
                "%.3f",
                outtake.getHoodPosition()
        );

        telemetry.addLine(
                "===== FLOWER ====="
        );

        telemetry.addData(
                "Second Hold",
                "%.2f sec",
                SECOND_FLOWER_INTAKE_HOLD_SECONDS
        );

        telemetry.addData(
                "Second Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point7.x(),
                point7.y(),
                Math.toDegrees(
                        point7.heading()
                )
        );

        telemetry.addLine(
                "===== INTAKE ====="
        );

        telemetry.addData(
                "Front Power",
                "%.2f",
                intake.getFrontMotorPower()
        );

        telemetry.addData(
                "Back Power",
                "%.2f",
                intake.getBackMotorPower()
        );

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // Bulk caching
    // ---------------------------------------------------------------------

    private void configureBulkCaching() {

        allHubs =
                hardwareMap.getAll(
                        LynxModule.class
                );

        for (LynxModule hub : allHubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
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

    private void restoreBulkCaching() {

        if (allHubs == null) {
            return;
        }

        for (LynxModule hub : allHubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }
    }

    // ---------------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------------

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(
                minimum,
                Math.min(
                        maximum,
                        value
                )
        );
    }
}