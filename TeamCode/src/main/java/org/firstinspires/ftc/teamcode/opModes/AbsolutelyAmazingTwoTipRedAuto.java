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
        name = "01 - Red Auto Fixed Turret",
        group = "Main"
)
public class AbsolutelyAmazingTwoTipRedAuto extends OpMode {

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
    // Pedro
    // -------------------------------------------------------------------------

    private final PoseFactory p =
            PoseFactory.degrees();


    // -------------------------------------------------------------------------
    // Fixed turret positions
    //
    // 0.0 = full robot-right
    // 0.5 = centre, pointing robot-back
    // 1.0 = full robot-left
    // -------------------------------------------------------------------------

    public static double FIRST_SHOT_TURRET_POSITION = 0.0;
    public static double FINAL_SHOT_TURRET_POSITION = 0.30;
    public static double FLOWER_SHOT_TURRET_POSITION = 0.30;
    public static double TRAVEL_TURRET_POSITION = 0.50;

    public static double AUTO_TURRET_TRIM = 0.0;


    // -------------------------------------------------------------------------
    // Shooting timing
    // -------------------------------------------------------------------------

    public static double TURRET_SETTLE_SECONDS = 0.30;
    public static double SHOOT_SETTLE_SECONDS = 1.30;
    public static double SHOOT_FEED_SECONDS = 1.30;
    public static double FLOWER_SHOOT_FEED_SECONDS = 3.50;


    // -------------------------------------------------------------------------
    // Path speeds
    // -------------------------------------------------------------------------

    public static double DRIVE_SPEED = 1.00;
    public static double GARDEN_COLLECT_SPEED = 0.20;
    public static double FLOWER_COLLECT_SPEED = 0.50;


    // -------------------------------------------------------------------------
    // Path watchdogs
    // -------------------------------------------------------------------------

    public static double PATH1_TIMEOUT_SECONDS = 4.0;
    public static double PATH3_TIMEOUT_SECONDS = 4.0;
    public static double PATH5_TIMEOUT_SECONDS = 6.0;
    public static double PATH7_TIMEOUT_SECONDS = 4.0;


    // -------------------------------------------------------------------------
    // Shooter readiness
    // -------------------------------------------------------------------------

    public static double SHOOT_READY_TIMEOUT = 1.00;
    public static double TPS_READY_FRACTION = 0.95;


    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    private int shootCycleCount = 0;

    private double activeTurretPosition = 0.50;

    private double shooterReadyWaitStartTime = 0.0;
    private boolean shooterReadyTimeoutFired = false;

    private boolean pathTimeoutFired = false;
    private boolean currentPathTimedOut = false;

    private String lastTimedOutPath = "NONE";


    // -------------------------------------------------------------------------
    // Restored eight-path route
    // -------------------------------------------------------------------------

    private final Pose start =
            p.of(
                    52.0,
                    8.0,
                    180.0
            );

    private final Pose point1 =
            p.of(
                    8.0,
                    8.0,
                    180.0
            );

    private final Pose point2 =
            p.of(
                    15.0,
                    8.0,
                    185.0
            );

    private final Pose point3 =
            p.of(
                    8.0,
                    8.0,
                    180.0
            );

    private final Pose point4 =
            p.of(
                    15.0,
                    8.0,
                    180.0
            );

    private final Pose point5Start =
            p.of(
                    15.0,
                    8.0,
                    180.0
            );

    private final Pose point5 =
            p.of(
                    32.0,
                    129.0,
                    90.0
            );

    private final Pose point6Start =
            p.of(
                    32.0,
                    129.0,
                    90.0
            );

    private final Pose point6 =
            p.of(
                    46.0,
                    113.0,
                    90.0
            );

    private final Pose point7Start =
            p.of(
                    46.0,
                    113.0,
                    90.0
            );

    private final Pose point7 =
            p.of(
                    46.0,
                    133.0,
                    90.0
            );

    private final Pose point8Start =
            p.of(
                    46.0,
                    133.0,
                    90.0
            );

    private final Pose point8 =
            p.of(
                    11.0,
                    117.0,
                    0.0
            );


    // -------------------------------------------------------------------------
    // Paths
    // -------------------------------------------------------------------------

    private Path path1() {

        return line(
                start,
                point1
        )
                .linear(
                        start,
                        point1
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(GARDEN_COLLECT_SPEED)
                );
    }


    private Path path2() {

        return line(
                point1,
                point2
        )
                .reverseTangent()
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(GARDEN_COLLECT_SPEED)
                );
    }


    private Path path3() {

        return line(
                point2,
                point3
        )
                .tangent()
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(GARDEN_COLLECT_SPEED)
                );
    }


    private Path path4() {

        return line(
                point3,
                point4
        )
                .reverseTangent()
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(GARDEN_COLLECT_SPEED)
                );
    }


    private Path path5() {

        return line(
                point5Start,
                point5
        )
                .linear(
                        point5Start,
                        point5
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(DRIVE_SPEED)
                );
    }


    private Path path6() {

        return line(
                point6Start,
                point6
        )
                .linear(
                        point6Start,
                        point6
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(DRIVE_SPEED)
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
                                .at(FLOWER_COLLECT_SPEED)
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
                                .at(DRIVE_SPEED)
                );
    }


    // -------------------------------------------------------------------------
    // Local Pedro Path adapter
    //
    // This bypasses the incompatible PedroCommands.follow() PathChain API.
    // -------------------------------------------------------------------------

    private Command followPath(Path path) {

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
                Constants.create(hardwareMap);

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
            PoseStorage.setPose(currentPose);
        }

        telemetry.addLine(
                "===== BLUE AUTO, EIGHT PATH ====="
        );

        telemetry.addData(
                "Start Pose",
                start
        );

        telemetry.addData(
                "Garden Speed",
                "%.2f",
                GARDEN_COLLECT_SPEED
        );

        telemetry.addData(
                "Flower Speed",
                "%.2f",
                FLOWER_COLLECT_SPEED
        );

        telemetry.addData(
                "First Shot Turret",
                "%.3f",
                getTrimmedPosition(
                        FIRST_SHOT_TURRET_POSITION
                )
        );

        telemetry.addData(
                "Final Shot Turret",
                "%.3f",
                getTrimmedPosition(
                        FINAL_SHOT_TURRET_POSITION
                )
        );

        telemetry.addData(
                "Flower Shot Turret",
                "%.3f",
                getTrimmedPosition(
                        FLOWER_SHOT_TURRET_POSITION
                )
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

        follower.setPose(start);
        PoseStorage.setPose(start);

        resetRuntimeState();

        stopIntakeAndBlock();
        setTravelTurretPosition();

        schedule(
                autoRoutine()
        );
    }


    @Override
    public void loop() {

        clearBulkCache();

        Scheduler.execute();
        follower.update();

        Pose currentPose =
                follower.pose();

        if (currentPose != null) {
            PoseStorage.setPose(currentPose);
        }

        /*
         * Maintain the selected fixed turret position.
         */
        turret.setFixedServoPosition(
                activeTurretPosition
        );

        /*
         * Update ball sensor and counter.
         */
        intake.update();

        /*
         * Maintain hood position and update the shooter and feeder controllers.
         */
        outtake.setHoodPosition(
                RobotConstants.HOOD_POSITION
        );

        outtake.update();

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
                PoseStorage.setPose(finalPose);
            }

            follower.stop();
        }

        PoseStorage.setBlue();

        if (outtake != null) {
            outtake.stopOuttake();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (robotSettings != null) {
            robotSettings.block();
        }

        if (flower != null) {

            flower.flowerActivate(
                    RobotConstants.flowerLiftPos
            );
        }

        if (turret != null) {
            turret.setCentre();
        }
    }


    // -------------------------------------------------------------------------
    // Completed autonomous routine
    // -------------------------------------------------------------------------

    private Command autoRoutine() {

        return sequential(

                // -------------------------------------------------------------
                // Shot 1
                // -------------------------------------------------------------

                shootCycle(
                        FIRST_SHOT_TURRET_POSITION,
                        SHOOT_FEED_SECONDS
                ),

                instant(
                        this::setTravelTurretPosition
                ),


                // -------------------------------------------------------------
                // Path 1
                //
                // Flower DOWN.
                // Intake ON.
                // -------------------------------------------------------------

                followCollectAndLowerFlowerWithTimeout(
                        path1(),
                        PATH1_TIMEOUT_SECONDS,
                        "PATH 1"
                ),


                // -------------------------------------------------------------
                // Path 2
                //
                // Flower remains DOWN.
                // Intake restarts and remains ON.
                // -------------------------------------------------------------

                followCollect(
                        path2()
                ),


                // -------------------------------------------------------------
                // Path 3
                //
                // Flower remains DOWN.
                // Intake remains ON.
                // -------------------------------------------------------------

                followCollectWithTimeout(
                        path3(),
                        PATH3_TIMEOUT_SECONDS,
                        "PATH 3"
                ),


                // -------------------------------------------------------------
                // Path 4
                //
                // Flower remains DOWN.
                // Intake remains ON.
                // -------------------------------------------------------------

                followCollect(
                        path4()
                ),


                // -------------------------------------------------------------
                // Path 5
                //
                // Intake OFF.
                // Feeder OFF.
                // Blocker closed.
                // Flower UP as Path 5 begins.
                // -------------------------------------------------------------

                followPath5RaiseFlower(),


                // -------------------------------------------------------------
                // Shot 2
                // -------------------------------------------------------------

                shootCycle(
                        FINAL_SHOT_TURRET_POSITION,
                        SHOOT_FEED_SECONDS
                ),


                // -------------------------------------------------------------
                // Path 6
                //
                // Intake OFF.
                // Flower remains UP.
                // -------------------------------------------------------------

                followWithoutIntake(
                        path6()
                ),


                // -------------------------------------------------------------
                // Path 7
                //
                // Flower DOWN.
                // Intake ON.
                // -------------------------------------------------------------

                followCollectAndLowerFlowerWithTimeout(
                        path7(),
                        PATH7_TIMEOUT_SECONDS,
                        "PATH 7"
                ),


                // -------------------------------------------------------------
                // Flower shot
                // -------------------------------------------------------------

                shootCycle(
                        FLOWER_SHOT_TURRET_POSITION,
                        FLOWER_SHOOT_FEED_SECONDS
                ),

                instant(
                        this::setTravelTurretPosition
                ),


                // -------------------------------------------------------------
                // Path 8
                //
                // Intake OFF.
                // Flower raises during the park movement.
                // -------------------------------------------------------------

                followRaiseFlower(
                        path8()
                ),


                // -------------------------------------------------------------
                // Finish
                // -------------------------------------------------------------

                instant(
                        this::finishAutoMechanisms
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

                                            currentPathTimedOut = true;
                                            pathTimeoutFired = true;
                                            lastTimedOutPath = pathName;
                                        }
                                )
                        )
                ),

                instant(
                        () -> {

                            if (currentPathTimedOut) {
                                follower.stop();
                            }
                        }
                )
        );
    }


    // -------------------------------------------------------------------------
    // Collection helpers
    // -------------------------------------------------------------------------

    private Command followCollect(
            Path path
    ) {

        return sequential(

                instant(
                        this::startCollection
                ),

                followPath(
                        path
                )
        );
    }


    private Command followCollectWithTimeout(
            Path path,
            double timeoutSeconds,
            String pathName
    ) {

        return sequential(

                instant(
                        this::startCollection
                ),

                followWithTimeout(
                        path,
                        timeoutSeconds,
                        pathName
                )
        );
    }


    /**
     * Lowers the flower and starts collection before following the path.
     *
     * The flower remains down after this command completes.
     */
    private Command followCollectAndLowerFlowerWithTimeout(
            Path path,
            double timeoutSeconds,
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

                followWithTimeout(
                        path,
                        timeoutSeconds,
                        pathName
                ),

                /*
                 * Stop intake and feeder, but do not move the flower.
                 * The flower remains down.
                 */
                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    // -------------------------------------------------------------------------
    // Path 5
    //
    // Raise the flower when Path 5 begins.
    // -------------------------------------------------------------------------

    private Command followPath5RaiseFlower() {

        return sequential(

                instant(
                        () -> {

                            /*
                             * Stop intake collection.
                             */
                            intake.stopIntake();

                            /*
                             * Stop the separate feeder.
                             */
                            outtake.setFeederTPS(
                                    0.0
                            );

                            /*
                             * Close the blocker.
                             */
                            robotSettings.block();

                            /*
                             * Place turret at the safe travel position.
                             */
                            setTravelTurretPosition();

                            /*
                             * Raise the flower as Path 5 begins.
                             */
                            flower.flowerActivate(
                                    RobotConstants.flowerLiftPos
                            );
                        }
                ),

                followWithTimeout(
                        path5(),
                        PATH5_TIMEOUT_SECONDS,
                        "PATH 5"
                ),

                instant(
                        this::prepareForTravel
                )
        );
    }


    // -------------------------------------------------------------------------
    // Travel helper
    // -------------------------------------------------------------------------

    private Command followWithoutIntake(
            Path path
    ) {

        return sequential(

                instant(
                        this::prepareForTravel
                ),

                followPath(
                        path
                ),

                instant(
                        this::prepareForTravel
                )
        );
    }


    // -------------------------------------------------------------------------
    // Path 8
    //
    // Raise flower during park.
    // -------------------------------------------------------------------------

    private Command followRaiseFlower(
            Path path
    ) {

        return sequential(

                instant(
                        this::prepareForTravel
                ),

                parallel(

                        followPath(
                                path
                        ),

                        sequential(

                                waitSeconds(
                                        0.50
                                ),

                                instant(
                                        () -> flower.flowerActivate(
                                                RobotConstants.flowerLiftPos
                                        )
                                )
                        )
                ),

                instant(
                        this::prepareForTravel
                )
        );
    }


    // -------------------------------------------------------------------------
    // Mechanism states
    // -------------------------------------------------------------------------

    private void startCollection() {

        robotSettings.block();

        outtake.setFeederTPS(
                0.0
        );

        intake.intake(
                RobotConstants.FRONT_INTAKE_SPEED,
                RobotConstants.BACK_INTAKE_SPEED
        );
    }


    private void prepareForTravel() {

        stopIntakeAndBlock();
        setTravelTurretPosition();
    }


    private void stopIntakeAndBlock() {

        intake.stopIntake();

        outtake.setFeederTPS(
                0.0
        );

        robotSettings.block();
    }


    // -------------------------------------------------------------------------
    // Shoot cycle
    // -------------------------------------------------------------------------

    private Command shootCycle(
            double turretPosition,
            double feedSeconds
    ) {

        return sequential(

                instant(
                        () -> {

                            shootCycleCount++;

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

                /*
                 * Allow turret to settle.
                 */
                race(

                        waitSeconds(
                                TURRET_SETTLE_SECONDS
                        ),

                        infinite(
                                this::holdShooterAndBlock
                        )
                ),

                /*
                 * Wait for shooter speed or timeout.
                 */
                race(

                        waitUntil(
                                this::shooterReadyOrTimedOut
                        ),

                        infinite(
                                this::holdShooterAndBlock
                        )
                ),

                /*
                 * Release blocker before feeding.
                 */
                race(

                        waitSeconds(
                                SHOOT_SETTLE_SECONDS
                        ),

                        infinite(
                                this::releaseBlocker
                        )
                ),

                /*
                 * Feed pollen.
                 */
                race(

                        waitSeconds(
                                feedSeconds
                        ),

                        infinite(
                                this::runShootFeed
                        )
                ),

                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    private void holdShooterAndBlock() {

        /*
         * Shooter retains its target TPS.
         */
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


    private void finishAutoMechanisms() {

        stopIntakeAndBlock();

        outtake.setTargetTPS(
                0.0
        );

        outtake.setFeederTPS(
                0.0
        );

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        setTravelTurretPosition();
    }


    // -------------------------------------------------------------------------
    // Shooter readiness
    // -------------------------------------------------------------------------

    private void startShooterReadyWait() {

        shooterReadyWaitStartTime =
                getRuntime();

        shooterReadyTimeoutFired =
                false;
    }


    private boolean shooterReadyOrTimedOut() {

        if (isShooterAtSpeed()) {
            return true;
        }

        if (getRuntime()
                - shooterReadyWaitStartTime
                >= SHOOT_READY_TIMEOUT) {

            shooterReadyTimeoutFired = true;

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
    // Turret control
    // -------------------------------------------------------------------------

    private void setFixedTurretPosition(
            double position
    ) {

        activeTurretPosition =
                getTrimmedPosition(
                        position
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


    private double getTrimmedPosition(
            double position
    ) {

        return clamp(
                position + AUTO_TURRET_TRIM,
                0.0,
                1.0
        );
    }


    // -------------------------------------------------------------------------
    // Utility
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


    private void resetRuntimeState() {

        shootCycleCount = 0;

        shooterReadyWaitStartTime = 0.0;
        shooterReadyTimeoutFired = false;

        pathTimeoutFired = false;
        currentPathTimedOut = false;
        lastTimedOutPath = "NONE";

        if (intake != null) {
            intake.resetConfirmedBallCount();
        }
    }


    private void displayTelemetry(
            Pose pose
    ) {

        telemetry.addLine(
                "===== RED AUTO, EIGHT PATH ====="
        );

        if (pose != null) {

            telemetry.addData(
                    "Pose",
                    "(%.1f, %.1f, %.1f deg)",
                    pose.x(),
                    pose.y(),
                    Math.toDegrees(
                            pose.heading()
                    )
            );
        }

        telemetry.addData(
                "Follower Mode",
                follower.mode()
        );

        telemetry.addData(
                "Shoot Cycle",
                shootCycleCount
        );


        telemetry.addLine(
                "===== PATH WATCHDOG ====="
        );

        telemetry.addData(
                "Any Timeout",
                pathTimeoutFired
        );

        telemetry.addData(
                "Current Timeout",
                currentPathTimedOut
        );

        telemetry.addData(
                "Last Timeout",
                lastTimedOutPath
        );


        telemetry.addLine(
                "===== ROUTE SETTINGS ====="
        );

        telemetry.addData(
                "Garden Speed",
                "%.2f",
                GARDEN_COLLECT_SPEED
        );

        telemetry.addData(
                "Travel Speed",
                "%.2f",
                DRIVE_SPEED
        );

        telemetry.addData(
                "Flower Speed",
                "%.2f",
                FLOWER_COLLECT_SPEED
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
                "Hood Position",
                "%.3f",
                outtake.getHoodPosition()
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
                "Ball Sensor",
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