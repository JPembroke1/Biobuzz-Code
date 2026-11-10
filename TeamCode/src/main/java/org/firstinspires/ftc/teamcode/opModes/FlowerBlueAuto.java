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
        name = "02 - Flower Blue Auto",
        group = "Main"
)
public class FlowerBlueAuto extends OpMode {

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
    // Fixed turret positions
    //
    // 0.0 = full robot-right
    // 0.5 = centre, pointing robot-back
    // 1.0 = full robot-left
    // -------------------------------------------------------------------------

    public static double PRELOAD_TURRET_POSITION = 0.0;

    public static double FAR_WALL_TURRET_POSITION = 0.30;

    public static double SECOND_FLOWER_TURRET_POSITION = 0.40;

    public static double TRAVEL_TURRET_POSITION = 0.50;

    public static double AUTO_TURRET_TRIM = 0.0;


    // -------------------------------------------------------------------------
    // Optimized path speeds
    // -------------------------------------------------------------------------

    public static double DRIVE_SPEED = 1.00;

    /*
     * Increased from 0.55.
     */
    public static double FLOWER_APPROACH_SPEED = 0.70;

    /*
     * Increased from 0.25.
     *
     * This is still substantially slower than normal travel.
     */
    public static double FLOWER_COLLECT_SPEED = 0.35;

    /*
     * Increased from 0.25.
     */
    public static double WALL_APPROACH_SPEED = 0.35;

    public static double PARK_SPEED = 1.00;


    // -------------------------------------------------------------------------
    // Optimized shooting timing
    // -------------------------------------------------------------------------

    /*
     * Reduced from 0.30 seconds.
     */
    public static double TURRET_SETTLE_SECONDS = 0.20;

    /*
     * Maximum wait for shooter speed.
     */
    public static double SHOOT_READY_TIMEOUT_SECONDS = 0.70;

    /*
     * Time between releasing the blocker and beginning feed.
     *
     * Reduced from 1.30 seconds.
     */
    public static double BLOCK_RELEASE_SECONDS = 0.40;

    /*
     * Feed time for the single preload.
     */
    public static double PRELOAD_FEED_SECONDS = 1.00;

    /*
     * Feed time for pollen collected from each Flower.
     */
    public static double FLOWER_FEED_SECONDS = 1.60;


    // -------------------------------------------------------------------------
    // Optimized Flower timing
    // -------------------------------------------------------------------------

    /*
     * Time for the Flower mechanism to lower before the robot moves into the
     * Flower.
     */
    public static double FLOWER_DEPLOY_SECONDS = 0.20;

    /*
     * The first Flower was previously not receiving enough intake time.
     *
     * This remains longer than the second Flower hold.
     */
    public static double FIRST_FLOWER_INTAKE_HOLD_SECONDS = 0.75;

    /*
     * The robot shoots immediately after the second Flower, so this hold can
     * be shorter.
     */
    public static double SECOND_FLOWER_INTAKE_HOLD_SECONDS = 0.45;

    /*
     * Begin raising the Flower after reversing for this long on Path 3.
     */
    public static double FLOWER_REVERSE_CLEAR_SECONDS = 0.35;

    /*
     * Raise the Flower shortly after the parking path begins.
     */
    public static double FLOWER_LIFT_DELAY_SECONDS = 0.20;


    // -------------------------------------------------------------------------
    // Shooter readiness
    // -------------------------------------------------------------------------

    public static double TPS_READY_FRACTION = 0.95;


    // -------------------------------------------------------------------------
    // Path watchdog timeouts
    // -------------------------------------------------------------------------

    public static double PATH1_TIMEOUT_SECONDS = 4.0;
    public static double PATH2_TIMEOUT_SECONDS = 3.0;
    public static double PATH3_TIMEOUT_SECONDS = 3.0;
    public static double PATH4_TIMEOUT_SECONDS = 5.0;
    public static double PATH5_TIMEOUT_SECONDS = 5.0;
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

    private String lastTimedOutPath = "NONE";


    // -------------------------------------------------------------------------
    // Route poses
    // -------------------------------------------------------------------------

    private final Pose start =
            poseFactory.of(
                    52.3532,
                    8.0,
                    180.0
            );


    // -------------------------------------------------------------------------
    // Path 1
    //
    // Line up in front of the first Flower.
    // -------------------------------------------------------------------------

    private final Pose path1End =
            poseFactory.of(
                    22.4811,
                    45.4390,
                    180.0
            );


    // -------------------------------------------------------------------------
    // Path 2
    //
    // Approach the first Flower slightly off-centre.
    // -------------------------------------------------------------------------

    private final Pose point2Start =
            poseFactory.of(
                    22.4811,
                    45.4390,
                    180.0
            );

    private final Pose point2 =
            poseFactory.of(
                    12.7950,
                    45.9688,
                    170.0
            );


    // -------------------------------------------------------------------------
    // Path 3
    //
    // Reverse clear of the first Flower.
    // -------------------------------------------------------------------------

    private final Pose point3 =
            poseFactory.of(
                    22.3555,
                    46.1482,
                    -178.9251
            );


    // -------------------------------------------------------------------------
    // Path 4
    //
    // Long drive toward the far wall.
    // -------------------------------------------------------------------------

    private final Pose point4Start =
            poseFactory.of(
                    22.3555,
                    46.1482,
                    180.0
            );

    private final Pose point4 =
            poseFactory.of(
                    22.6871,
                    94.7054,
                    90.0
            );


    // -------------------------------------------------------------------------
    // Path 5
    //
    // Final controlled wall approach.
    // -------------------------------------------------------------------------

    private final Pose point5Start =
            poseFactory.of(
                    22.6871,
                    94.7054,
                    90.0
            );

    private final Pose point5 =
            poseFactory.of(
                    23.7064,
                    130.1432,
                    90.0
            );


    // -------------------------------------------------------------------------
    // Path 6
    //
    // Reverse from the wall and line up with the second Flower.
    // -------------------------------------------------------------------------

    private final Pose point6Start =
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


    // -------------------------------------------------------------------------
    // Path 7
    //
    // Collect from the second Flower.
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
                    80.0
            );


    // -------------------------------------------------------------------------
    // Path 8
    //
    // Park.
    // -------------------------------------------------------------------------

    private final Pose point8Start =
            poseFactory.of(
                    42.0,
                    127.0,
                    80.0
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

    private Path path1() {

        return line(
                start,
                path1End
        )
                .linear(
                        start,
                        path1End
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        FLOWER_APPROACH_SPEED
                                )
                );
    }


    private Path path2() {

        return line(
                point2Start,
                point2
        )
                .linear(
                        point2Start,
                        point2
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        FLOWER_COLLECT_SPEED
                                )
                );
    }


    private Path path3() {

        return line(
                point2,
                point3
        )
                .reverseTangent()
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        FLOWER_APPROACH_SPEED
                                )
                );
    }


    private Path path4() {

        return line(
                point4Start,
                point4
        )
                .linear(
                        point4Start,
                        point4
                )
                .with(
                        Constants.foresightConfig
                                .maxPathSpeed
                                .at(
                                        DRIVE_SPEED
                                )
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
                                .at(
                                        WALL_APPROACH_SPEED
                                )
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
    //
    // This avoids the incompatible PedroCommands.follow() PathChain signature.
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
                "===== FLOWER BLUE AUTO, FAST ====="
        );

        telemetry.addData(
                "Start Pose",
                start
        );

        telemetry.addData(
                "Flower Approach Speed",
                "%.2f",
                FLOWER_APPROACH_SPEED
        );

        telemetry.addData(
                "Flower Collect Speed",
                "%.2f",
                FLOWER_COLLECT_SPEED
        );

        telemetry.addData(
                "Wall Approach Speed",
                "%.2f",
                WALL_APPROACH_SPEED
        );

        telemetry.addData(
                "First Hold",
                "%.2f sec",
                FIRST_FLOWER_INTAKE_HOLD_SECONDS
        );

        telemetry.addData(
                "Second Hold",
                "%.2f sec",
                SECOND_FLOWER_INTAKE_HOLD_SECONDS
        );

        telemetry.addData(
                "Far Wall Turret",
                "%.3f",
                FAR_WALL_TURRET_POSITION
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

        setTravelTurretPosition();

        schedule(
                autoRoutine()
        );
    }


    @Override
    public void loop() {

        clearBulkCache();

        /*
         * Run Ivy first so a newly started path is configured before Pedro's
         * drivetrain update.
         */
        Scheduler.execute();

        follower.update();

        Pose currentPose =
                follower.pose();

        if (currentPose != null) {

            PoseStorage.setPose(
                    currentPose
            );
        }

        /*
         * Maintain the selected fixed turret command.
         */
        turret.setFixedServoPosition(
                activeTurretPosition
        );

        /*
         * Update the intake sensor and ball counter.
         */
        intake.update();

        /*
         * Maintain the hood base position.
         */
        outtake.setHoodPosition(
                RobotConstants.HOOD_POSITION
        );

        /*
         * Update shooter, feeder, voltage compensation and hood compensation.
         */
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

                PoseStorage.setPose(
                        finalPose
                );
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
    // Optimized autonomous routine
    // -------------------------------------------------------------------------

    private Command autoRoutine() {

        return sequential(

                // -------------------------------------------------------------
                // Preload shot
                // -------------------------------------------------------------

                shootCycle(
                        PRELOAD_TURRET_POSITION,
                        PRELOAD_FEED_SECONDS
                ),

                instant(
                        this::setTravelTurretPosition
                ),


                // -------------------------------------------------------------
                // Path 1
                //
                // Line up with the first Flower.
                // -------------------------------------------------------------

                followTravel(
                        path1(),
                        PATH1_TIMEOUT_SECONDS,
                        "PATH 1"
                ),


                // -------------------------------------------------------------
                // Path 2
                //
                // Lower Flower and collect.
                // Includes a 0.75-second stationary intake hold.
                // -------------------------------------------------------------

                collectFlower(
                        path2(),
                        PATH2_TIMEOUT_SECONDS,
                        FIRST_FLOWER_INTAKE_HOLD_SECONDS,
                        "PATH 2"
                ),


                // -------------------------------------------------------------
                // Path 3
                //
                // Reverse clear and raise Flower concurrently.
                // -------------------------------------------------------------

                reverseAndLiftFlower(),


                // -------------------------------------------------------------
                // Path 4
                //
                // Long drive begins without an additional Flower-lift delay.
                // -------------------------------------------------------------

                followTravel(
                        path4(),
                        PATH4_TIMEOUT_SECONDS,
                        "PATH 4"
                ),


                // -------------------------------------------------------------
                // Path 5
                //
                // Final controlled wall approach.
                // -------------------------------------------------------------

                followTravel(
                        path5(),
                        PATH5_TIMEOUT_SECONDS,
                        "PATH 5"
                ),


                // -------------------------------------------------------------
                // Far-wall shot
                //
                // Turret position = 0.30.
                // -------------------------------------------------------------

                shootCycle(
                        FAR_WALL_TURRET_POSITION,
                        FLOWER_FEED_SECONDS
                ),

                instant(
                        this::setTravelTurretPosition
                ),


                // -------------------------------------------------------------
                // Path 6
                //
                // Move in front of the second Flower.
                // -------------------------------------------------------------

                followTravel(
                        path6(),
                        PATH6_TIMEOUT_SECONDS,
                        "PATH 6"
                ),


                // -------------------------------------------------------------
                // Path 7
                //
                // Collect second Flower with shorter stationary hold.
                // -------------------------------------------------------------

                collectFlower(
                        path7(),
                        PATH7_TIMEOUT_SECONDS,
                        SECOND_FLOWER_INTAKE_HOLD_SECONDS,
                        "PATH 7"
                ),


                // -------------------------------------------------------------
                // Shoot immediately from second Flower position.
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
                // Park and lift Flower.
                // -------------------------------------------------------------

                parkAndLiftFlower(),


                // -------------------------------------------------------------
                // Finish
                // -------------------------------------------------------------

                instant(
                        this::finishAutoMechanisms
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

                // -------------------------------------------------------------
                // Lower Flower and start intake
                // -------------------------------------------------------------

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


                // -------------------------------------------------------------
                // Short deployment delay
                // -------------------------------------------------------------

                waitSeconds(
                        FLOWER_DEPLOY_SECONDS
                ),


                // -------------------------------------------------------------
                // Drive into Flower
                // -------------------------------------------------------------

                followWithTimeout(
                        path,
                        timeoutSeconds,
                        pathName
                ),


                // -------------------------------------------------------------
                // Stationary intake hold
                // -------------------------------------------------------------

                race(

                        waitSeconds(
                                intakeHoldSeconds
                        ),

                        infinite(
                                this::holdFlowerCollection
                        )
                ),


                // -------------------------------------------------------------
                // Stop intake but leave Flower down
                // -------------------------------------------------------------

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
    // Reverse from first Flower and lift concurrently
    // -------------------------------------------------------------------------

    private Command reverseAndLiftFlower() {

        return sequential(

                instant(
                        () -> {

                            stopIntakeAndBlock();

                            setTravelTurretPosition();
                        }
                ),

                parallel(

                        /*
                         * Reverse away using Path 3.
                         */
                        followWithTimeout(
                                path3(),
                                PATH3_TIMEOUT_SECONDS,
                                "PATH 3"
                        ),

                        /*
                         * Wait until clear, then begin raising the Flower while
                         * the robot is still moving.
                         */
                        sequential(

                                waitSeconds(
                                        FLOWER_REVERSE_CLEAR_SECONDS
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
    // Optimized shooting cycle
    // -------------------------------------------------------------------------

    private Command shootCycle(
            double turretPosition,
            double feedSeconds
    ) {

        return sequential(

                // -------------------------------------------------------------
                // Start shooter and position turret
                // -------------------------------------------------------------

                instant(
                        () -> {

                            shotCount++;

                            setFixedTurretPosition(
                                    turretPosition
                            );

                            /*
                             * The target is retained after the shot so the
                             * shooter stays spinning through autonomous.
                             */
                            outtake.setTargetTPS(
                                    RobotConstants.OUTTAKE_TPS
                            );

                            startShooterReadyWait();

                            stopIntakeAndBlock();
                        }
                ),


                // -------------------------------------------------------------
                // Short turret settle
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
                // Shooter ready or short timeout
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
                // Release blocker before feeding
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
                // Feed shot
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
                // Stop intake and feeder
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

        /*
         * Shooter flywheel retains its target TPS.
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
    // Park and lift Flower
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

                        followWithTimeout(
                                path8(),
                                PATH8_TIMEOUT_SECONDS,
                                "PATH 8"
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
                ),

                instant(
                        this::stopIntakeAndBlock
                )
        );
    }


    // -------------------------------------------------------------------------
    // Final mechanism state
    // -------------------------------------------------------------------------

    private void finishAutoMechanisms() {

        stopIntakeAndBlock();

        /*
         * Stop the shooter only after all shooting is complete.
         */
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
                "===== FLOWER BLUE AUTO, FAST ====="
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
                "===== SPEED SETTINGS ====="
        );

        telemetry.addData(
                "Drive",
                "%.2f",
                DRIVE_SPEED
        );

        telemetry.addData(
                "Flower Approach",
                "%.2f",
                FLOWER_APPROACH_SPEED
        );

        telemetry.addData(
                "Flower Collect",
                "%.2f",
                FLOWER_COLLECT_SPEED
        );

        telemetry.addData(
                "Wall Approach",
                "%.2f",
                WALL_APPROACH_SPEED
        );


        telemetry.addLine(
                "===== FLOWER ====="
        );

        telemetry.addData(
                "Deploy Time",
                "%.2f sec",
                FLOWER_DEPLOY_SECONDS
        );

        telemetry.addData(
                "First Hold",
                "%.2f sec",
                FIRST_FLOWER_INTAKE_HOLD_SECONDS
        );

        telemetry.addData(
                "Second Hold",
                "%.2f sec",
                SECOND_FLOWER_INTAKE_HOLD_SECONDS
        );

        telemetry.addData(
                "Reverse Clear",
                "%.2f sec",
                FLOWER_REVERSE_CLEAR_SECONDS
        );

        telemetry.addData(
                "First Pickup",
                "(%.1f, %.1f, %.1f deg)",
                point2.x(),
                point2.y(),
                Math.toDegrees(
                        point2.heading()
                )
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