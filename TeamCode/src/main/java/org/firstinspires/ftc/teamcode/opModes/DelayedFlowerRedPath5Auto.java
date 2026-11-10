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
@Autonomous(
        name = "04 - Delayed Flower Red Path 5 Auto",
        group = "Main"
)
public class DelayedFlowerRedPath5Auto extends OpMode {


    // -------------------------------------------------------------------------
    // Hardware and subsystems
    // -------------------------------------------------------------------------

    private List<LynxModule> allHubs;

    private Follower follower;

    private Outtake outtake;
    private Intake intake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;


    // -------------------------------------------------------------------------
    // Pose factory
    // -------------------------------------------------------------------------

    private final PoseFactory poseFactory =
            PoseFactory.degrees();


    // -------------------------------------------------------------------------
    // Turret positions
    // -------------------------------------------------------------------------

    public static double FAR_WALL_TURRET_POSITION = 0.30;

    public static double SECOND_FLOWER_TURRET_POSITION = 0.40;

    public static double TRAVEL_TURRET_POSITION = 0.50;

    public static double AUTO_TURRET_TRIM = 0.0;


    // -------------------------------------------------------------------------
    // Path speeds
    // -------------------------------------------------------------------------

    public static double DRIVE_SPEED = 1.00;

    public static double FLOWER_COLLECT_SPEED = 0.35;

    public static double PARK_SPEED = 1.00;


    // -------------------------------------------------------------------------
    // Shooting timing
    // -------------------------------------------------------------------------

    /*
     * Delay before the first preload shooting sequence.
     */
    public static double PRELOAD_START_DELAY_SECONDS = 9.0;

    public static double TURRET_SETTLE_SECONDS = 0.20;

    public static double SHOOT_READY_TIMEOUT_SECONDS = 0.70;

    public static double BLOCK_RELEASE_SECONDS = 0.40;

    public static double PRELOAD_FEED_SECONDS = 1.00;

    public static double FLOWER_FEED_SECONDS = 1.60;


    // -------------------------------------------------------------------------
    // Flower timing
    // -------------------------------------------------------------------------

    public static double FLOWER_DEPLOY_SECONDS = 0.20;

    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.45;

    public static double FLOWER_LIFT_DELAY_SECONDS = 0.20;


    // -------------------------------------------------------------------------
    // Shooter readiness
    // -------------------------------------------------------------------------

    public static double TPS_READY_FRACTION = 0.95;


    // -------------------------------------------------------------------------
    // Path watchdogs
    // -------------------------------------------------------------------------

    public static double PATH6_TIMEOUT_SECONDS = 4.0;

    public static double PATH7_TIMEOUT_SECONDS = 2.5;

    public static double PATH8_TIMEOUT_SECONDS = 5.0;


    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    private double activeTurretPosition = 0.50;

    private int shotCount = 0;

    private double shooterReadyStartTime = 0.0;

    private boolean shooterReadyTimeoutFired = false;

    private boolean anyPathTimeoutFired = false;

    private boolean currentPathTimedOut = false;

    private boolean autoFinished = false;

    private String lastTimedOutPath = "NONE";


    // -------------------------------------------------------------------------
    // Starting pose
    //
    // This is the END of original Path 5.
    // It is also the START of Path 6.
    // -------------------------------------------------------------------------

    private final Pose start =
            poseFactory.of(
                    23.7064,
                    130.1432,
                    90.0
            );


    // -------------------------------------------------------------------------
    // Path 6
    // -------------------------------------------------------------------------

    private final Pose point6 =
            poseFactory.of(
                    42.0,
                    104.1657,
                    90.0
            );


    // -------------------------------------------------------------------------
    // Path 7
    // -------------------------------------------------------------------------

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


    // -------------------------------------------------------------------------
    // Path 8
    // -------------------------------------------------------------------------

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


    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------

    private Path path6() {

        return line(
                start,
                point6
        )
                .linear(
                        start,
                        point6
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        DRIVE_SPEED
                                )
                );
    }


    private Path path7() {

        return line(
                point7Start,
                point7
        )
                .linear(
                        point7Start,
                        point7
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        FLOWER_COLLECT_SPEED
                                )
                );
    }


    private Path path8() {

        return line(
                point8Start,
                point8
        )
                .linear(
                        point8Start,
                        point8
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        PARK_SPEED
                                )
                );
    }


    // -------------------------------------------------------------------------
    // Local Pedro Path adapter
    // -------------------------------------------------------------------------

    private Command followPath(
            Path path
    ) {

        return new CommandBuilder()
                .setStart(
                        () -> follower.follow(path)
                )
                .setDone(
                        follower::atParametricEnd
                );
    }


    // -------------------------------------------------------------------------
    // FTC lifecycle
    // -------------------------------------------------------------------------

    @Override
    public void init() {

        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setBlue();
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


        outtake.init(
                hardwareMap
        );

        intake.init(
                hardwareMap
        );

        turret.init(
                hardwareMap
        );

        flower.init(
                hardwareMap
        );

        robotSettings.init(
                hardwareMap
        );


        Turret.velocityCompensationActive =
                false;


        intake.stopIntake();

        outtake.stopOuttake();

        robotSettings.block();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );


        resetRuntimeState();

        setTravelTurretPosition();
    }


    @Override
    public void init_loop() {

        clearBulkCache();

        PoseStorage.setBlue();

        intake.update();


        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }


        telemetry.addLine(
                "===== BLUE PATH 5 DELAYED AUTO ====="
        );

        telemetry.addData(
                "Start Pose",
                start
        );

        telemetry.addData(
                "Preload Delay",
                "%.2f sec",
                PRELOAD_START_DELAY_SECONDS
        );

        telemetry.addData(
                "Start",
                "Original Path 5 endpoint"
        );

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

        telemetry.addData(
                "Ball Sensor",
                intake.isSensorTriggered()
        );

        telemetry.addData(
                "Ball Count",
                intake.getConfirmedBallCount()
        );

        telemetry.update();
    }


    @Override
    public void start() {

        clearBulkCache();

        PoseStorage.setBlue();


        follower.setPose(
                start
        );

        PoseStorage.setPose(
                start
        );


        resetRuntimeState();


        stopIntakeAndBlock();

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setTravelTurretPosition();


        schedule(
                autoRoutine()
        );
    }


    @Override
    public void loop() {

        clearBulkCache();


        /*
         * Do not keep running the autonomous mechanisms after the complete
         * shutdown command has executed.
         */
        if (!autoFinished) {

            Scheduler.execute();

            follower.update();

        } else {

            /*
             * Reinforce the safe stopped state every loop after autonomous
             * has completed.
             */
            maintainFinishedState();
        }


        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }


        /*
         * Only maintain the normal turret/shooter subsystem updates while the
         * autonomous routine is active.
         */
        if (!autoFinished) {

            turret.setFixedServoPosition(
                    activeTurretPosition
            );

            intake.update();

            outtake.setHoodPosition(
                    RobotConstants.HOOD_POSITION
            );

            outtake.update();
        }


        displayTelemetry(
                currentPose
        );
    }


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


        PoseStorage.setBlue();


        /*
         * FTC stop() is a second safety layer.
         *
         * The autonomous routine already shuts everything down after Path 8,
         * but this guarantees the same state if STOP is manually pressed.
         */
        shutdownAllSystems();
    }


    // -------------------------------------------------------------------------
    // Autonomous routine
    // -------------------------------------------------------------------------

    private Command autoRoutine() {

        return sequential(


                // -------------------------------------------------------------
                // Prepare for preload
                //
                // Position turret and start shooter immediately.
                // Blocker remains closed and intake/feed remain stopped.
                // -------------------------------------------------------------

                instant(
                        () -> {

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
                ),


                // -------------------------------------------------------------
                // 9 SECOND DELAY
                //
                // Shooter may spin up, but the preload cannot feed.
                // -------------------------------------------------------------

                race(

                        waitSeconds(
                                PRELOAD_START_DELAY_SECONDS
                        ),

                        infinite(
                                this::holdShooterAndBlock
                        )
                ),


                // -------------------------------------------------------------
                // Shoot preload
                //
                // Robot is physically at the old Path 5 endpoint.
                // -------------------------------------------------------------

                shootCycle(
                        FAR_WALL_TURRET_POSITION,
                        PRELOAD_FEED_SECONDS
                ),


                instant(
                        this::setTravelTurretPosition
                ),


                // -------------------------------------------------------------
                // Path 6
                //
                // Reverse/move away from far wall and line up with Flower 2.
                // -------------------------------------------------------------

                followTravel(
                        path6(),
                        PATH6_TIMEOUT_SECONDS,
                        "PATH 6"
                ),


                // -------------------------------------------------------------
                // Path 7
                //
                // Lower Flower and collect second Flower.
                // -------------------------------------------------------------

                collectFlower(
                        path7(),
                        PATH7_TIMEOUT_SECONDS,
                        SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        "PATH 7"
                ),


                // -------------------------------------------------------------
                // Second Flower shot
                // -------------------------------------------------------------

                shootCycle(
                        SECOND_FLOWER_TURRET_POSITION,
                        FLOWER_FEED_SECONDS
                ),


                instant(
                        this::setTravelTurretPosition
                ),


                // -------------------------------------------------------------
                // Path 8
                //
                // Park and raise Flower.
                // -------------------------------------------------------------

                parkAndLiftFlower(),


                // -------------------------------------------------------------
                // COMPLETE SHUTDOWN
                //
                // Only reached after Path 8 has completed or its watchdog
                // timeout has fired.
                // -------------------------------------------------------------

                instant(
                        this::shutdownAllSystems
                )
        );
    }


    // -------------------------------------------------------------------------
    // Flower collection
    // -------------------------------------------------------------------------

    private Command collectFlower(
            Path path,
            double timeoutSeconds,
            double intakeHoldSeconds,
            String pathName
    ) {

        return sequential(


                instant(
                        () -> {

                            robotSettings.block();

                            outtake.setFeederTPS(
                                    0.0
                            );

                            flower.flowerActivate(
                                    RobotConstants.flowerDropPos
                            );

                            intake.intake(
                                    RobotConstants.FRONT_INTAKE_SPEED,
                                    RobotConstants.BACK_INTAKE_SPEED
                            );
                        }
                ),


                waitSeconds(
                        FLOWER_DEPLOY_SECONDS
                ),


                followWithTimeout(
                        path,
                        timeoutSeconds,
                        pathName
                ),


                race(

                        waitSeconds(
                                intakeHoldSeconds
                        ),

                        infinite(
                                this::holdFlowerCollection
                        )
                ),


                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    private void holdFlowerCollection() {

        robotSettings.block();

        outtake.setFeederTPS(
                0.0
        );

        flower.flowerActivate(
                RobotConstants.flowerDropPos
        );

        intake.intake(
                RobotConstants.FRONT_INTAKE_SPEED,
                RobotConstants.BACK_INTAKE_SPEED
        );
    }


    // -------------------------------------------------------------------------
    // Normal travel
    // -------------------------------------------------------------------------

    private Command followTravel(
            Path path,
            double timeoutSeconds,
            String pathName
    ) {

        return sequential(


                instant(
                        () -> {

                            stopIntakeAndBlock();

                            setTravelTurretPosition();
                        }
                ),


                followWithTimeout(
                        path,
                        timeoutSeconds,
                        pathName
                ),


                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    // -------------------------------------------------------------------------
    // Path watchdog
    // -------------------------------------------------------------------------

    private Command followWithTimeout(
            Path path,
            double timeoutSeconds,
            String pathName
    ) {

        return sequential(


                instant(
                        () -> currentPathTimedOut = false
                ),


                race(

                        followPath(
                                path
                        ),

                        sequential(

                                waitSeconds(
                                        timeoutSeconds
                                ),

                                instant(
                                        () -> {

                                            currentPathTimedOut =
                                                    true;

                                            anyPathTimeoutFired =
                                                    true;

                                            lastTimedOutPath =
                                                    pathName;

                                            follower.stop();
                                        }
                                )
                        )
                )
        );
    }


    // -------------------------------------------------------------------------
    // Shooting cycle
    // -------------------------------------------------------------------------

    private Command shootCycle(
            double turretPosition,
            double feedSeconds
    ) {

        return sequential(


                // -------------------------------------------------------------
                // Start shooter / set turret
                // -------------------------------------------------------------

                instant(
                        () -> {

                            shotCount++;

                            setFixedTurretPosition(
                                    turretPosition
                            );

                            outtake.setTargetTPS(
                                    RobotConstants.OUTTAKE_TPS
                            );

                            startShooterReadyWait();

                            stopIntakeAndBlock();
                        }
                ),


                // -------------------------------------------------------------
                // Turret settling time
                // -------------------------------------------------------------

                race(

                        waitSeconds(
                                TURRET_SETTLE_SECONDS
                        ),

                        infinite(
                                this::holdShooterAndBlock
                        )
                ),


                // -------------------------------------------------------------
                // Wait for shooter speed or timeout
                // -------------------------------------------------------------

                race(

                        waitUntil(
                                this::shooterReadyOrTimedOut
                        ),

                        infinite(
                                this::holdShooterAndBlock
                        )
                ),


                // -------------------------------------------------------------
                // Release blocker
                // -------------------------------------------------------------

                race(

                        waitSeconds(
                                BLOCK_RELEASE_SECONDS
                        ),

                        infinite(
                                this::releaseBlocker
                        )
                ),


                // -------------------------------------------------------------
                // Feed balls
                // -------------------------------------------------------------

                race(

                        waitSeconds(
                                feedSeconds
                        ),

                        infinite(
                                this::runShootFeed
                        )
                ),


                // -------------------------------------------------------------
                // Stop feeder and intake after shot
                // -------------------------------------------------------------

                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    // -------------------------------------------------------------------------
    // Shooting mechanism actions
    // -------------------------------------------------------------------------

    private void holdShooterAndBlock() {

        intake.stopIntake();

        outtake.setFeederTPS(
                0.0
        );

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

        outtake.setFeederTPS(
                RobotConstants.FEEDER_TPS
        );
    }


    private void stopIntakeAndBlock() {

        intake.stopIntake();

        outtake.setFeederTPS(
                0.0
        );

        robotSettings.block();
    }


    // -------------------------------------------------------------------------
    // Shooter readiness
    // -------------------------------------------------------------------------

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
                >= SHOOT_READY_TIMEOUT_SECONDS) {

            shooterReadyTimeoutFired =
                    true;

            return true;
        }


        return false;
    }


    private boolean isShooterAtSpeed() {

        double targetTPS =
                outtake.getTargetTPS();

        double currentTPS =
                outtake.getCurrentTPS();

        double readyFraction =
                clamp(
                        TPS_READY_FRACTION,
                        0.0,
                        1.0
                );


        return targetTPS > 0.0
                && currentTPS
                >= targetTPS * readyFraction;
    }


    // -------------------------------------------------------------------------
    // Path 8: park and lift Flower
    // -------------------------------------------------------------------------

    private Command parkAndLiftFlower() {

        return sequential(


                instant(
                        () -> {

                            stopIntakeAndBlock();

                            setTravelTurretPosition();
                        }
                ),


                parallel(

                        /*
                         * Complete final parking path.
                         */
                        followWithTimeout(
                                path8(),
                                PATH8_TIMEOUT_SECONDS,
                                "PATH 8"
                        ),


                        /*
                         * Raise the Flower shortly after the parking path begins.
                         */
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
                ),


                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    // -------------------------------------------------------------------------
    // COMPLETE END-OF-AUTO SHUTDOWN
    //
    // Called immediately after the final parking path completes.
    // -------------------------------------------------------------------------

    private void shutdownAllSystems() {

        /*
         * Flag first so loop() no longer runs the normal mechanism updates.
         */
        autoFinished = true;


        // ---------------------------------------------------------------------
        // Stop drivetrain
        // ---------------------------------------------------------------------

        if (follower != null) {

            follower.stop();
        }


        // ---------------------------------------------------------------------
        // Stop intake
        // ---------------------------------------------------------------------

        if (intake != null) {

            intake.stopIntake();
        }


        // ---------------------------------------------------------------------
        // Stop shooter flywheel and feeder
        // ---------------------------------------------------------------------

        if (outtake != null) {

            outtake.setTargetTPS(
                    0.0
            );

            outtake.setFeederTPS(
                    0.0
            );

            outtake.stopOuttake();
        }


        // ---------------------------------------------------------------------
        // Close blocker
        // ---------------------------------------------------------------------

        if (robotSettings != null) {

            robotSettings.block();
        }


        // ---------------------------------------------------------------------
        // Raise Flower
        // ---------------------------------------------------------------------

        if (flower != null) {

            flower.flowerActivate(
                    RobotConstants.flowerLiftPos
            );
        }


        // ---------------------------------------------------------------------
        // Return turret to centre
        // ---------------------------------------------------------------------

        activeTurretPosition =
                TRAVEL_TURRET_POSITION;

        if (turret != null) {

            turret.setCentre();
        }
    }


    // -------------------------------------------------------------------------
    // Maintain finished state
    //
    // This prevents anything from accidentally restarting after the final path.
    // -------------------------------------------------------------------------

    private void maintainFinishedState() {

        if (follower != null) {

            follower.stop();
        }


        if (intake != null) {

            intake.stopIntake();
        }


        if (outtake != null) {

            outtake.setTargetTPS(
                    0.0
            );

            outtake.setFeederTPS(
                    0.0
            );
        }


        if (robotSettings != null) {

            robotSettings.block();
        }
    }


    // -------------------------------------------------------------------------
    // Turret control
    // -------------------------------------------------------------------------

    private void setFixedTurretPosition(
            double position
    ) {

        activeTurretPosition =
                clamp(
                        position + AUTO_TURRET_TRIM,
                        0.0,
                        1.0
                );

        turret.setFixedServoPosition(
                activeTurretPosition
        );
    }


    private void setTravelTurretPosition() {

        activeTurretPosition =
                clamp(
                        TRAVEL_TURRET_POSITION,
                        0.0,
                        1.0
                );

        turret.setFixedServoPosition(
                activeTurretPosition
        );
    }


    // -------------------------------------------------------------------------
    // Wait helper
    // -------------------------------------------------------------------------

    private Command waitSeconds(
            double seconds
    ) {

        return waitMs(
                Math.round(
                        Math.max(
                                0.0,
                                seconds
                        ) * 1000.0
                )
        );
    }


    // -------------------------------------------------------------------------
    // Runtime reset
    // -------------------------------------------------------------------------

    private void resetRuntimeState() {

        activeTurretPosition =
                TRAVEL_TURRET_POSITION;

        shotCount = 0;

        shooterReadyStartTime = 0.0;

        shooterReadyTimeoutFired = false;

        anyPathTimeoutFired = false;

        currentPathTimedOut = false;

        autoFinished = false;

        lastTimedOutPath = "NONE";


        if (intake != null) {

            intake.resetConfirmedBallCount();
        }
    }


    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void displayTelemetry(
            Pose currentPose
    ) {

        telemetry.addLine(
                "===== BLUE PATH 5 DELAYED AUTO ====="
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
                "Preload Delay",
                "%.2f sec",
                PRELOAD_START_DELAY_SECONDS
        );

        telemetry.addData(
                "Auto Finished",
                autoFinished
        );

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.addData(
                "Shot Count",
                shotCount
        );


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

        telemetry.addData(
                "Sensor Triggered",
                intake.isSensorTriggered()
        );

        telemetry.addData(
                "Ball Confirmed",
                intake.isBallConfirmed()
        );

        telemetry.addData(
                "Ball Count",
                intake.getConfirmedBallCount()
        );


        telemetry.update();
    }


    // -------------------------------------------------------------------------
    // Bulk caching
    // -------------------------------------------------------------------------

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


    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

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