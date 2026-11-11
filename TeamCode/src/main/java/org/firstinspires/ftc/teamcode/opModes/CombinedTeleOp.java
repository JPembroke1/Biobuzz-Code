package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp(name = "0. Combined TeleOp", group = "Main")
public class CombinedTeleOp extends OpMode {

    // ---------------------------------------------------------------------
    // Field and robot geometry
    // ---------------------------------------------------------------------

    public static double FIELD_SIZE_IN = 141.5;
    public static double FIELD_CENTER_IN = 70.75;

    public static double ROBOT_LENGTH_MM = 340.0;
    public static double ROBOT_WIDTH_MM = 340.0;

    // ---------------------------------------------------------------------
    // Goal coordinates
    // ---------------------------------------------------------------------

    public static double RED_LOWER_GOAL_X = 58.00;
    public static double RED_LOWER_GOAL_Y = 46.69;

    public static double RED_UPPER_GOAL_X = 58.00;
    public static double RED_UPPER_GOAL_Y = 94.81;

    public static double BLUE_LOWER_GOAL_X = 83.50;
    public static double BLUE_LOWER_GOAL_Y = 46.69;

    public static double BLUE_UPPER_GOAL_X = 83.50;
    public static double BLUE_UPPER_GOAL_Y = 94.81;

    // ---------------------------------------------------------------------
    // Automatic goal selection
    // ---------------------------------------------------------------------

    public static double GOAL_SWITCH_Y_IN = 70.75;
    public static double GOAL_HYSTERESIS_IN = 1.0;

    // ---------------------------------------------------------------------
    // Corner-reset headings
    // ---------------------------------------------------------------------

    public static double RED_LOWER_RESET_HEADING_DEG = 270.0;
    public static double RED_UPPER_RESET_HEADING_DEG = 90.0;

    public static double BLUE_LOWER_RESET_HEADING_DEG = 270.0;
    public static double BLUE_UPPER_RESET_HEADING_DEG = 90.0;

    // ---------------------------------------------------------------------
    // Drive settings
    // ---------------------------------------------------------------------

    public static double FORWARD_SIGN = -1.0;
    public static double STRAFE_SIGN = -1.0;
    public static double TURN_SIGN = -1.0;

    public static boolean CUBIC_STRAFE_TURN = false;

    // ---------------------------------------------------------------------
    // Camera settings
    // ---------------------------------------------------------------------

    public static String CAMERA_NAME = "Webcam 1";

    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;

    /*
     * Leave false for competition unless Panels preview is needed.
     */
    public static boolean STREAM_CAMERA_TO_PANELS = false;

    /*
     * Allows vision to be disabled from configuration without removing
     * the camera integration code.
     */
    public static boolean VISION_ENABLED = true;

    // ---------------------------------------------------------------------
    // Alliance selection
    // ---------------------------------------------------------------------

    public enum AllianceMode {
        STORED,
        RED,
        BLUE
    }

    public static AllianceMode ALLIANCE_MODE =
            AllianceMode.STORED;

    private PoseStorage.Alliance activeAlliance =
            PoseStorage.Alliance.UNKNOWN;

    // ---------------------------------------------------------------------
    // Subsystems
    // ---------------------------------------------------------------------

    private Follower follower;

    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;
    private ArduCam camera;

    private List<LynxModule> hubs;

    // ---------------------------------------------------------------------
    // Runtime state
    // ---------------------------------------------------------------------

    private boolean shooterActive = false;
    private boolean autoAimMode = true;
    private boolean flowerActive = false;

    private boolean poseEstablished = false;
    private boolean poseResetThisLoop = false;

    private boolean upperGoalSelected = false;
    private boolean goalSelectionInitialized = false;
    private boolean manualGoalMode = false;
    private boolean targetValid = false;

    private double goalX = Double.NaN;
    private double goalY = Double.NaN;
    private double distanceToGoalIn = Double.NaN;

    private long feedStartNanos = -1L;

    private boolean feederRequested = false;
    private boolean feederPermitted = false;

    // ---------------------------------------------------------------------
    // Button edge states
    // ---------------------------------------------------------------------

    private boolean previousFlowerButton = false;
    private boolean previousRegressionButton = false;

    private boolean previousAimButton = false;
    private boolean previousCentreButton = false;

    private boolean previousLowerResetButton = false;
    private boolean previousUpperResetButton = false;

    private boolean previousLowerGoalButton = false;
    private boolean previousUpperGoalButton = false;
    private boolean previousAutomaticGoalButton = false;

    private String status = "Initialising";

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    @Override
    public void init() {

        ALLIANCE_MODE = AllianceMode.STORED;

        follower = Constants.create(hardwareMap);

        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }

        intake = new Intake();
        intake.init(hardwareMap);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        Outtake.DISTANCE_REGRESSION_ENABLED = false;
        Outtake.HOOD_COMP_ENABLED = false;

        turret = new Turret();
        turret.init(hardwareMap);
        turret.setCentre();

        Turret.velocityCompensationActive = false;

        initialiseCamera();

        flower = new Flower();
        flower.init(hardwareMap);

        flowerActive = false;

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        robotSettings = new RobotSettings();
        robotSettings.init(hardwareMap);
        robotSettings.block();

        activeAlliance = resolveAlliance();

        Pose savedPose =
                PoseStorage.getPose();

        if (savedPose != null
                && validPose(savedPose)) {

            follower.setPose(savedPose);

            poseEstablished = true;

            status =
                    "Stored pose restored; verify robot has not moved";

        } else {

            poseEstablished = false;

            status =
                    "No stored pose; perform a corner reset";
        }

        shooterActive = false;
        autoAimMode = true;
        manualGoalMode = false;

        resetFeedTimer();

        displayTelemetry();
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

        if (gamepad1.dpad_left) {

            ALLIANCE_MODE = AllianceMode.RED;

        } else if (gamepad1.dpad_right) {

            ALLIANCE_MODE = AllianceMode.BLUE;

        } else if (gamepad1.dpad_up) {

            ALLIANCE_MODE = AllianceMode.STORED;
        }

        activeAlliance = resolveAlliance();

        Pose previewPose =
                poseEstablished
                        ? follower.pose()
                        : null;

        updateGoalFromPose(previewPose);

        /*
         * Do not move automatically before START.
         */
        turret.setCentre();

        flowerActive = false;

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        displayTelemetry();
    }

    // ---------------------------------------------------------------------
    // START
    // ---------------------------------------------------------------------

    @Override
    public void start() {

        /*
         * Alliance locks here.
         */
        activeAlliance = resolveAlliance();

        if (activeAlliance
                == PoseStorage.Alliance.RED) {

            PoseStorage.setRed();

        } else if (activeAlliance
                == PoseStorage.Alliance.BLUE) {

            PoseStorage.setBlue();
        }

        shooterActive = false;
        autoAimMode = true;
        flowerActive = false;

        flower.flowerActivate(
                RobotConstants.flowerLiftPos
        );

        manualGoalMode = false;

        upperGoalSelected = false;
        goalSelectionInitialized = false;
        targetValid = false;

        goalX = Double.NaN;
        goalY = Double.NaN;
        distanceToGoalIn = Double.NaN;

        poseResetThisLoop = false;

        /*
         * Fixed mode remains the safe startup mode.
         */
        Outtake.DISTANCE_REGRESSION_ENABLED = false;
        Outtake.HOOD_COMP_ENABLED = false;

        Turret.velocityCompensationActive = false;

        if (camera != null) {

            camera.resetHiveState();
            camera.setEnabled(true);
        }

        resetFeedTimer();

        previousFlowerButton =
                gamepad1.dpad_right;

        previousRegressionButton =
                gamepad1.circle;

        previousAimButton =
                gamepad1.square;

        previousCentreButton =
                gamepad2.square;

        previousLowerResetButton =
                gamepad2.dpad_down;

        previousUpperResetButton =
                gamepad2.dpad_up;

        previousLowerGoalButton =
                gamepad2.cross;

        previousUpperGoalButton =
                gamepad2.circle;

        previousAutomaticGoalButton =
                gamepad2.triangle;

        if (activeAlliance
                == PoseStorage.Alliance.RED) {

            status = "LOCKED ALLIANCE: RED";

        } else if (activeAlliance
                == PoseStorage.Alliance.BLUE) {

            status = "LOCKED ALLIANCE: BLUE";

        } else {

            status =
                    "UNKNOWN ALLIANCE - AUTO AIM DISABLED";

            autoAimMode = false;
            turret.setCentre();
        }
    }

    // ---------------------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------------------

    @Override
    public void loop() {

        clearBulkCache();

        poseResetThisLoop = false;

        handleCornerResets();

        handleShooterControls();
        handleFlowerControls();
        handleRegressionToggle();

        // -------------------------------------------------------------
        // Drive and update localization
        // -------------------------------------------------------------

        if (!poseResetThisLoop) {

            driveRobot();

        } else {

            follower.manual(
                    new DrivePowers(
                            0.0,
                            0.0,
                            0.0
                    )
            );
        }

        Pose currentPose =
                follower.pose();

        if (poseEstablished
                && validPose(currentPose)) {

            PoseStorage.setPose(currentPose);
        }

        // -------------------------------------------------------------
        // Select target
        // -------------------------------------------------------------

        handleManualGoalSelection();

        if (manualGoalMode) {

            applySelectedGoal();

        } else {

            updateGoalFromPose(currentPose);
        }

        handleTurretModeControls();

        // -------------------------------------------------------------
        // Update camera HIVE state
        // -------------------------------------------------------------

        updateCameraHiveState();

        // -------------------------------------------------------------
        // Pinpoint + camera fused turret aim
        // -------------------------------------------------------------

        updateTurretAim(currentPose);

        // -------------------------------------------------------------
        // Shooter regression distance
        // -------------------------------------------------------------

        outtake.setGoalDistance(
                targetValid
                        && poseEstablished
                        && Double.isFinite(distanceToGoalIn)
                        ? distanceToGoalIn
                        : Double.NaN
        );

        /*
         * Resolve shooter target and current regression status.
         */
        outtake.update();

        // -------------------------------------------------------------
        // Intake and feeding
        // -------------------------------------------------------------

        handleIntakeAndFeeder();

        /*
         * A newly requested positive feeder target takes effect during
         * the next outtake update. stopFeeder() still stops immediately.
         */
        intake.update();

        displayTelemetry();
    }

    private void updateCameraHiveState() {

        if (camera == null
                || !camera.isEnabled()) {

            return;
        }

        ArduCam.Alliance cameraAlliance =
                getCameraAlliance();

        if (cameraAlliance != null) {

            camera.updateHive(
                    cameraAlliance
            );
        }
    }

    // ---------------------------------------------------------------------
    // Shooter controls
    // ---------------------------------------------------------------------

    private void handleShooterControls() {

        if (gamepad1.right_bumper) {
            shooterActive = false;
        }

        if (gamepad1.cross) {
            shooterActive = true;
        }

        /*
         * Triangle takes priority over Cross.
         */
        if (gamepad1.triangle) {
            shooterActive = false;
        }

        outtake.setTargetTPS(
                shooterActive
                        ? Math.max(
                        0.0,
                        RobotConstants.OUTTAKE_TPS
                )
                        : 0.0
        );

        outtake.setHoodPosition(
                RobotConstants.HOOD_POSITION
        );
    }

    // ---------------------------------------------------------------------
    // Regression toggle
    // ---------------------------------------------------------------------

    private void handleRegressionToggle() {

        boolean pressed =
                gamepad1.circle;

        if (pressed
                && !previousRegressionButton) {

            Outtake.DISTANCE_REGRESSION_ENABLED =
                    !Outtake.DISTANCE_REGRESSION_ENABLED;

            stopFeederOnly();

            intake.stopIntake();
            robotSettings.block();

            resetFeedTimer();

            status =
                    Outtake.DISTANCE_REGRESSION_ENABLED
                            ? "REGRESSION ENABLED"
                            : "FIXED TPS MODE";
        }

        previousRegressionButton = pressed;
    }

    // ---------------------------------------------------------------------
    // Intake and feeder
    // ---------------------------------------------------------------------

    private void handleIntakeAndFeeder() {

        feederRequested = false;
        feederPermitted = false;

        // -------------------------------------------------------------
        // Collect
        // -------------------------------------------------------------

        if (gamepad1.right_bumper) {

            intake.intake(
                    RobotConstants.FRONT_INTAKE_SPEED,
                    RobotConstants.BACK_INTAKE_SPEED
            );

            robotSettings.block();

            stopFeederOnly();
            resetFeedTimer();

            return;
        }

        // -------------------------------------------------------------
        // Feed
        // -------------------------------------------------------------

        if (gamepad1.right_trigger > 0.3) {

            if (!feederAllowed()) {

                robotSettings.block();

                intake.stopIntake();

                stopFeederOnly();
                resetFeedTimer();

                return;
            }

            robotSettings.release();

            if (feedStartNanos < 0L) {

                feedStartNanos =
                        System.nanoTime();
            }

            double elapsedSeconds =
                    (System.nanoTime()
                            - feedStartNanos)
                            / 1_000_000_000.0;

            RobotConstants.timer =
                    elapsedSeconds;

            if (elapsedSeconds >= Math.max(
                    0.0,
                    RobotConstants.delayAmount
            )) {

                intake.intake(
                        RobotConstants.FRONT_FEED_SPEED,
                        RobotConstants.BACK_FEED_SPEED
                );

                feederRequested = true;
                feederPermitted = true;

                outtake.setFeederTPS(
                        RobotConstants.FEEDER_TPS
                );

            } else {

                intake.stopIntake();

                stopFeederOnly();
            }

            return;
        }

        // -------------------------------------------------------------
        // Reverse intake
        // -------------------------------------------------------------

        if (gamepad1.left_bumper) {

            intake.intake(
                    -RobotConstants.FRONT_INTAKE_SPEED,
                    -RobotConstants.BACK_INTAKE_SPEED
            );

            robotSettings.block();

            stopFeederOnly();
            resetFeedTimer();

            return;
        }

        // -------------------------------------------------------------
        // Nothing requested
        // -------------------------------------------------------------

        intake.stopIntake();

        robotSettings.block();

        stopFeederOnly();
        resetFeedTimer();
    }

    private void stopFeederOnly() {

        feederRequested = false;
        feederPermitted = false;

        outtake.setFeederTPS(0.0);
        outtake.stopFeeder();
    }

    private boolean feederAllowed() {

        if (!shooterActive) {
            return false;
        }

        if (!autoAimMode) {
            return false;
        }

        if (!poseEstablished
                || !targetValid) {

            return false;
        }

        /*
         * PINPOINT:
         * target must be reachable.
         *
         * FUSED:
         * target must be reachable and camera bearing must be aligned.
         */
        if (!turret.isAimReady()) {
            return false;
        }

        /*
         * Do not silently use fixed fallback TPS when regression was
         * deliberately enabled but is currently invalid.
         */
        if (Outtake.DISTANCE_REGRESSION_ENABLED
                && !outtake.isUsingDistanceRegression()) {

            return false;
        }

        if (!RobotConstants.FEED_REQUIRES_SPEED) {
            return true;
        }

        return outtake.isAtSpeed(
                RobotConstants.FEED_SPEED_TOLERANCE_TPS
        );
    }

    private void resetFeedTimer() {

        feedStartNanos = -1L;

        RobotConstants.timer = 0.0;
    }

    // ---------------------------------------------------------------------
    // Manual goal selection
    // ---------------------------------------------------------------------

    private void handleManualGoalSelection() {

        boolean lowerButton =
                gamepad2.cross;

        boolean upperButton =
                gamepad2.circle;

        boolean automaticButton =
                gamepad2.triangle;

        if (automaticButton
                && !previousAutomaticGoalButton) {

            manualGoalMode = false;
            goalSelectionInitialized = false;

            stopFeederOnly();
            resetFeedTimer();

            status =
                    "Automatic Y target selection enabled";
        }

        if (lowerButton
                && !previousLowerGoalButton) {

            manualGoalMode = true;

            upperGoalSelected = false;
            goalSelectionInitialized = true;

            applySelectedGoal();

            status =
                    allianceLabel()
                            + " LOWER target manually selected";
        }

        if (upperButton
                && !previousUpperGoalButton) {

            manualGoalMode = true;

            upperGoalSelected = true;
            goalSelectionInitialized = true;

            applySelectedGoal();

            status =
                    allianceLabel()
                            + " UPPER target manually selected";
        }

        previousLowerGoalButton =
                lowerButton;

        previousUpperGoalButton =
                upperButton;

        previousAutomaticGoalButton =
                automaticButton;
    }

    // ---------------------------------------------------------------------
    // Apply current target selection
    // ---------------------------------------------------------------------

    private void applySelectedGoal() {

        targetValid = false;

        goalX = Double.NaN;
        goalY = Double.NaN;

        if (activeAlliance
                == PoseStorage.Alliance.RED) {

            goalX =
                    upperGoalSelected
                            ? RED_UPPER_GOAL_X
                            : RED_LOWER_GOAL_X;

            goalY =
                    upperGoalSelected
                            ? RED_UPPER_GOAL_Y
                            : RED_LOWER_GOAL_Y;

        } else if (activeAlliance
                == PoseStorage.Alliance.BLUE) {

            goalX =
                    upperGoalSelected
                            ? BLUE_UPPER_GOAL_X
                            : BLUE_LOWER_GOAL_X;

            goalY =
                    upperGoalSelected
                            ? BLUE_UPPER_GOAL_Y
                            : BLUE_LOWER_GOAL_Y;

        } else {

            return;
        }

        targetValid =
                validFieldPoint(
                        goalX,
                        goalY
                );

        if (!targetValid) {
            return;
        }

        Pose existingTarget =
                turret.getTargetPose();

        boolean targetChanged =
                turret.getSelectedGoal()
                        != Turret.Goal.CUSTOM
                        || existingTarget == null
                        || Double.compare(
                        existingTarget.x(),
                        goalX
                ) != 0
                        || Double.compare(
                        existingTarget.y(),
                        goalY
                ) != 0;

        if (targetChanged) {

            turret.setTargetPose(
                    new Pose(
                            goalX,
                            goalY,
                            0.0
                    )
            );

            turret.realignToSelectedGoal();

            stopFeederOnly();
            resetFeedTimer();
        }
    }

    // ---------------------------------------------------------------------
    // Automatic lower/upper target selection
    // ---------------------------------------------------------------------

    private void updateGoalFromPose(
            Pose currentPose
    ) {

        targetValid = false;

        goalX = Double.NaN;
        goalY = Double.NaN;

        if (!allianceKnown()
                || !poseEstablished
                || !validPose(currentPose)) {

            return;
        }

        if (!Double.isFinite(GOAL_SWITCH_Y_IN)
                || !Double.isFinite(
                GOAL_HYSTERESIS_IN
        )) {

            return;
        }

        double hysteresis =
                Math.max(
                        0.0,
                        GOAL_HYSTERESIS_IN
                );

        double robotY =
                currentPose.y();

        boolean nextUpper =
                upperGoalSelected;

        if (!goalSelectionInitialized) {

            nextUpper =
                    robotY >= GOAL_SWITCH_Y_IN;

        } else if (hysteresis == 0.0) {

            nextUpper =
                    robotY >= GOAL_SWITCH_Y_IN;

        } else if (upperGoalSelected) {

            if (robotY
                    < GOAL_SWITCH_Y_IN
                    - hysteresis) {

                nextUpper = false;
            }

        } else if (robotY
                >= GOAL_SWITCH_Y_IN
                + hysteresis) {

            nextUpper = true;
        }

        upperGoalSelected = nextUpper;
        goalSelectionInitialized = true;

        applySelectedGoal();
    }

    // ---------------------------------------------------------------------
    // Flower
    // ---------------------------------------------------------------------

    private void handleFlowerControls() {

        boolean pressed =
                gamepad1.dpad_right;

        if (pressed
                && !previousFlowerButton) {

            flowerActive =
                    !flowerActive;
        }

        previousFlowerButton = pressed;

        flower.flowerActivate(
                flowerActive
                        ? RobotConstants.flowerDropPos
                        : RobotConstants.flowerLiftPos
        );
    }

    // ---------------------------------------------------------------------
    // Turret mode controls
    // ---------------------------------------------------------------------

    private void handleTurretModeControls() {

        boolean aimButton =
                gamepad1.square;

        if (aimButton
                && !previousAimButton) {

            if (allianceKnown()
                    && poseEstablished
                    && targetValid) {

                autoAimMode = true;

                turret.realignToSelectedGoal();

                status =
                        "Automatic fused aim enabled";

            } else {

                status =
                        "Aim blocked: alliance, pose or target invalid";
            }
        }

        previousAimButton = aimButton;

        boolean centreButton =
                gamepad2.square;

        if (centreButton
                && !previousCentreButton) {

            autoAimMode = false;

            turret.setCentre();

            stopFeederOnly();
            resetFeedTimer();

            status =
                    "Turret centred";
        }

        previousCentreButton = centreButton;
    }

    // ---------------------------------------------------------------------
    // Fused turret aim
    // ---------------------------------------------------------------------

    private void updateTurretAim(
            Pose currentPose
    ) {

        distanceToGoalIn =
                Double.NaN;

        if (!targetValid
                || !poseEstablished
                || !validPose(currentPose)) {

            if (autoAimMode) {
                turret.setCentre();
            }

            return;
        }

        distanceToGoalIn =
                calculateDistanceToGoal(
                        currentPose
                );

        if (!autoAimMode) {
            return;
        }

        ArduCam.Alliance cameraAlliance =
                getCameraAlliance();

        if (camera == null
                || cameraAlliance == null
                || !camera.isEnabled()) {

            turret.aimTurret(
                    currentPose,
                    0.0,
                    0.0
            );

            return;
        }

        /*
         * Pedro/Pinpoint supplies the absolute coarse angle.
         *
         * The turret-mounted Arducam supplies a residual visual correction
         * when a valid HIVE cluster is visible.
         *
         * Camera dropout automatically returns to PINPOINT.
         */
        turret.aimTurretFused(
                currentPose,
                0.0,
                0.0,
                camera,
                cameraAlliance
        );
    }

    private ArduCam.Alliance getCameraAlliance() {

        if (activeAlliance
                == PoseStorage.Alliance.RED) {

            return ArduCam.Alliance.RED;
        }

        if (activeAlliance
                == PoseStorage.Alliance.BLUE) {

            return ArduCam.Alliance.BLUE;
        }

        return null;
    }

    private double calculateDistanceToGoal(
            Pose currentPose
    ) {

        double heading =
                currentPose.heading();

        double pivotX =
                currentPose.x()
                        + Turret.TURRET_OFFSET_X_IN
                        * Math.cos(heading)
                        - Turret.TURRET_OFFSET_Y_IN
                        * Math.sin(heading);

        double pivotY =
                currentPose.y()
                        + Turret.TURRET_OFFSET_X_IN
                        * Math.sin(heading)
                        + Turret.TURRET_OFFSET_Y_IN
                        * Math.cos(heading);

        double distance =
                Math.hypot(
                        goalX - pivotX,
                        goalY - pivotY
                );

        return Double.isFinite(distance)
                ? distance
                : Double.NaN;
    }

    // ---------------------------------------------------------------------
    // Corner resets
    // ---------------------------------------------------------------------

    private void handleCornerResets() {

        boolean lowerButton =
                gamepad2.dpad_down;

        boolean upperButton =
                gamepad2.dpad_up;

        if (lowerButton
                && !previousLowerResetButton) {

            resetAtAllianceCorner(false);

        } else if (upperButton
                && !previousUpperResetButton) {

            resetAtAllianceCorner(true);
        }

        previousLowerResetButton = lowerButton;
        previousUpperResetButton = upperButton;
    }

    private void resetAtAllianceCorner(
            boolean upperCorner
    ) {

        if (!allianceKnown()) {

            status =
                    "Corner reset blocked: alliance unknown";

            return;
        }

        if (!Double.isFinite(FIELD_SIZE_IN)
                || FIELD_SIZE_IN <= 0.0
                || !Double.isFinite(ROBOT_LENGTH_MM)
                || !Double.isFinite(ROBOT_WIDTH_MM)
                || ROBOT_LENGTH_MM <= 0.0
                || ROBOT_WIDTH_MM <= 0.0) {

            status =
                    "Corner reset blocked: invalid geometry";

            return;
        }

        double halfLengthIn =
                ROBOT_LENGTH_MM
                        / 25.4
                        / 2.0;

        double halfWidthIn =
                ROBOT_WIDTH_MM
                        / 25.4
                        / 2.0;

        boolean red =
                activeAlliance
                        == PoseStorage.Alliance.RED;

        double headingDegrees =
                red
                        ? (upperCorner
                           ? RED_UPPER_RESET_HEADING_DEG
                           : RED_LOWER_RESET_HEADING_DEG)
                        : (upperCorner
                           ? BLUE_UPPER_RESET_HEADING_DEG
                           : BLUE_LOWER_RESET_HEADING_DEG);

        if (!Double.isFinite(headingDegrees)) {

            status =
                    "Corner reset blocked: invalid heading";

            return;
        }

        double x =
                red
                        ? halfWidthIn
                        : FIELD_SIZE_IN
                          - halfWidthIn;

        double y =
                upperCorner
                        ? FIELD_SIZE_IN
                          - halfLengthIn
                        : halfLengthIn;

        if (!validFieldPoint(x, y)) {

            status =
                    "Corner reset blocked: computed pose invalid";

            return;
        }

        Pose resetPose =
                new Pose(
                        x,
                        y,
                        Math.toRadians(
                                headingDegrees
                        )
                );

        shooterActive = false;

        outtake.setTargetTPS(0.0);

        stopFeederOnly();

        intake.stopIntake();

        robotSettings.block();

        resetFeedTimer();

        follower.manual(
                new DrivePowers(
                        0.0,
                        0.0,
                        0.0
                )
        );

        follower.setPose(resetPose);

        PoseStorage.setPose(resetPose);

        poseEstablished = true;
        poseResetThisLoop = true;

        manualGoalMode = false;
        goalSelectionInitialized = false;

        updateGoalFromPose(resetPose);

        turret.realignToSelectedGoal();

        if (camera != null) {
            camera.resetHiveState();
        }

        status =
                (red ? "RED " : "BLUE ")
                        + (upperCorner
                        ? "UPPER"
                        : "LOWER")
                        + " corner reset";
    }

    // ---------------------------------------------------------------------
    // Alliance
    // ---------------------------------------------------------------------

    private PoseStorage.Alliance resolveAlliance() {

        if (ALLIANCE_MODE
                == AllianceMode.RED) {

            return PoseStorage.Alliance.RED;
        }

        if (ALLIANCE_MODE
                == AllianceMode.BLUE) {

            return PoseStorage.Alliance.BLUE;
        }

        return PoseStorage.lastAlliance == null
                ? PoseStorage.Alliance.UNKNOWN
                : PoseStorage.lastAlliance;
    }

    private boolean allianceKnown() {

        return activeAlliance
                == PoseStorage.Alliance.RED
                || activeAlliance
                == PoseStorage.Alliance.BLUE;
    }

    private String allianceLabel() {

        if (activeAlliance
                == PoseStorage.Alliance.RED) {

            return "RED";
        }

        if (activeAlliance
                == PoseStorage.Alliance.BLUE) {

            return "BLUE";
        }

        return "UNKNOWN";
    }

    // ---------------------------------------------------------------------
    // Drive
    // ---------------------------------------------------------------------

    private void driveRobot() {

        double strafeInput =
                gamepad1.left_stick_x;

        double turnInput =
                gamepad1.right_stick_x;

        if (CUBIC_STRAFE_TURN) {

            strafeInput =
                    Math.pow(
                            strafeInput,
                            3
                    );

            turnInput =
                    Math.pow(
                            turnInput,
                            3
                    );
        }

        double forward =
                FORWARD_SIGN
                        * gamepad1.left_stick_y
                        * RobotConstants.DRIVE_SPEED;

        double strafe =
                STRAFE_SIGN
                        * strafeInput
                        * RobotConstants.DRIVE_SPEED;

        double turn =
                TURN_SIGN
                        * turnInput
                        * RobotConstants.DRIVE_SPEED;

        if (gamepad2.dpad_down
                || gamepad2.dpad_up) {

            forward = 0.0;
            strafe = 0.0;
            turn = 0.0;
        }

        follower.manual(
                new DrivePowers(
                        forward,
                        strafe,
                        turn
                )
        );

        follower.update();
    }

    // ---------------------------------------------------------------------
    // Bulk cache
    // ---------------------------------------------------------------------

    private void clearBulkCache() {

        if (hubs == null) {
            return;
        }

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    // ---------------------------------------------------------------------
    // Validation
    // ---------------------------------------------------------------------

    private boolean validPose(
            Pose pose
    ) {

        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(
                pose.heading()
        );
    }

    private boolean validFieldPoint(
            double x,
            double y
    ) {

        return Double.isFinite(FIELD_SIZE_IN)
                && FIELD_SIZE_IN > 0.0
                && Double.isFinite(x)
                && Double.isFinite(y)
                && x >= 0.0
                && y >= 0.0
                && x <= FIELD_SIZE_IN
                && y <= FIELD_SIZE_IN;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void displayTelemetry() {

        Pose pose =
                follower == null
                        ? null
                        : follower.pose();

        telemetry.addLine("=== STATUS ===");

        telemetry.addData(
                "Status",
                status
        );

        telemetry.addData(
                "LOCKED Alliance",
                activeAlliance
        );

        telemetry.addData(
                "Stored Alliance",
                PoseStorage.lastAlliance
        );

        telemetry.addData(
                "Pose Established",
                poseEstablished
        );

        if (validPose(pose)) {

            telemetry.addData(
                    "Pose",
                    "(%.2f, %.2f, %.1f deg)",
                    pose.x(),
                    pose.y(),
                    normalizeDegrees0To360(
                            Math.toDegrees(
                                    pose.heading()
                            )
                    )
            );
        }

        // -------------------------------------------------------------
        // Camera/HIVE
        // -------------------------------------------------------------

        telemetry.addLine("=== CAMERA / HIVE ===");

        telemetry.addData(
                "Vision Enabled",
                VISION_ENABLED
        );

        telemetry.addData(
                "Camera Available",
                camera != null
        );

        telemetry.addData(
                "Camera Enabled",
                camera != null
                        && camera.isEnabled()
        );

        telemetry.addData(
                "Camera Streaming",
                camera != null
                        && camera.isStreaming()
        );

        ArduCam.Alliance cameraAlliance =
                getCameraAlliance();

        if (camera != null
                && cameraAlliance != null) {

            telemetry.addData(
                    "Hive Visible",
                    camera.isHiveVisible(
                            cameraAlliance
                    )
            );

            telemetry.addData(
                    "Hive State",
                    camera.getHiveState()
            );

            telemetry.addData(
                    "Hive Tipped",
                    camera.isHiveTipped(
                            cameraAlliance
                    )
            );

            telemetry.addData(
                    "Cluster Details",
                    camera.getVisibleClusterDetails()
            );

            telemetry.addData(
                    "Visible HIVE Members",
                    formatVisibleHiveMembers()
            );
        }

        // -------------------------------------------------------------
        // Intake
        // -------------------------------------------------------------

        telemetry.addLine("=== INTAKE ===");

        telemetry.addData(
                "Front Command",
                intake.getFrontMotorPower()
        );

        telemetry.addData(
                "Back Command",
                intake.getBackMotorPower()
        );

        telemetry.addData(
                "Ball Triggered",
                Intake.sensorTriggered
        );

        telemetry.addData(
                "Ball Confirmed",
                Intake.ballConfirmed
        );

        // -------------------------------------------------------------
        // Shooter/feeder
        // -------------------------------------------------------------

        telemetry.addLine("=== SHOOTER / FEEDER ===");

        telemetry.addData(
                "Shooter Mode",
                shooterActive
                        ? "SHOOT"
                        : "STOPPED"
        );

        telemetry.addData(
                "Shooter Target TPS",
                outtake.getTargetTPS()
        );

        telemetry.addData(
                "Shooter Current TPS",
                outtake.getCurrentTPS()
        );

        telemetry.addData(
                "Shooter Raw TPS",
                outtake.getRawTPS()
        );

        telemetry.addData(
                "Shooter Error TPS",
                outtake.getErrorTPS()
        );

        telemetry.addData(
                "Shooter Power",
                outtake.getCommandedPower()
        );

        telemetry.addData(
                "Feeder Requested",
                feederRequested
        );

        telemetry.addData(
                "Feeder Permitted",
                feederPermitted
        );

        telemetry.addData(
                "Feeder Target TPS",
                outtake.getFeederTargetTPS()
        );

        telemetry.addData(
                "Feeder Current TPS",
                outtake.getFeederCurrentTPS()
        );

        telemetry.addData(
                "Feeder Power",
                outtake.getFeederCommandedPower()
        );

        telemetry.addData(
                "Feed Timer",
                "%.2f",
                RobotConstants.timer
        );

        // -------------------------------------------------------------
        // Regression
        // -------------------------------------------------------------

        telemetry.addLine("=== SHOT MODEL ===");

        telemetry.addData(
                "Regression Enabled",
                Outtake.DISTANCE_REGRESSION_ENABLED
        );

        telemetry.addData(
                "Regression Active",
                outtake.isUsingDistanceRegression()
        );

        telemetry.addData(
                "Regression Status",
                outtake.getRegressionStatus()
        );

        telemetry.addData(
                "Distance Supplied",
                "%.2f in",
                outtake.getGoalDistance()
        );

        telemetry.addData(
                "Fixed TPS",
                outtake.getFixedTargetTPS()
        );

        telemetry.addData(
                "Active Target TPS",
                outtake.getTargetTPS()
        );

        telemetry.addData(
                "Hood Base",
                outtake.getHoodBase()
        );

        telemetry.addData(
                "Hood Command",
                outtake.getHoodPosition()
        );

        telemetry.addData(
                "Voltage",
                "%.2f V",
                outtake.getVoltage()
        );

        telemetry.addData(
                "Voltage Scale",
                "%.3f",
                outtake.getVoltageScale()
        );

        // -------------------------------------------------------------
        // Turret/fusion
        // -------------------------------------------------------------

        telemetry.addLine("=== TARGET / TURRET ===");

        telemetry.addData(
                "Auto Aim",
                autoAimMode
        );

        telemetry.addData(
                "Aim Source",
                turret.getAimSource()
        );

        telemetry.addData(
                "Goal Selection Mode",
                manualGoalMode
                        ? "MANUAL"
                        : "AUTOMATIC Y"
        );

        telemetry.addData(
                "Selected Side",
                goalSelectionInitialized
                        ? (upperGoalSelected
                           ? "UPPER Y"
                           : "LOWER Y")
                        : "NOT SELECTED"
        );

        telemetry.addData(
                "Target Valid",
                targetValid
        );

        telemetry.addData(
                "Target",
                "(%.2f, %.2f)",
                goalX,
                goalY
        );

        telemetry.addData(
                "Distance",
                "%.2f in",
                distanceToGoalIn
        );

        telemetry.addData(
                "Field Target Angle",
                "%.1f deg",
                Turret.lastFieldTargetAngleDeg
        );

        telemetry.addData(
                "Pinpoint Angle",
                "%.1f deg",
                Turret.lastPinpointAngleDeg
        );

        telemetry.addData(
                "Camera Target Visible",
                Turret.lastCameraTargetVisible
        );

        telemetry.addData(
                "Camera Target Valid",
                Turret.lastCameraTargetValid
        );

        telemetry.addData(
                "Camera Bearing",
                "%.1f deg",
                Turret.lastCameraBearingDeg
        );

        telemetry.addData(
                "Filtered Bearing",
                "%.1f deg",
                Turret.lastFilteredCameraBearingDeg
        );

        telemetry.addData(
                "Camera Range",
                "%.1f in",
                Turret.lastCameraRangeIn
        );

        telemetry.addData(
                "Camera Correction",
                "%.1f deg",
                Turret.lastCameraCorrectionDeg
        );

        telemetry.addData(
                "Camera Weight",
                "%.2f",
                Turret.lastCameraWeight
        );

        telemetry.addData(
                "Camera Estimate",
                "%.1f deg",
                Turret.lastCameraEstimateAngleDeg
        );

        telemetry.addData(
                "Fused Angle",
                "%.1f deg",
                Turret.lastFusedAngleDeg
        );

        telemetry.addData(
                "Commanded Angle",
                "%.1f deg",
                Turret.lastFinalAngleDeg
        );

        telemetry.addData(
                "Servo Position",
                "%.3f",
                turret.getServoPosition()
        );

        telemetry.addData(
                "Target Reachable",
                turret.isTargetReachable()
        );

        telemetry.addData(
                "Camera Aligned",
                turret.isCameraAligned()
        );

        telemetry.addData(
                "Aim Ready",
                turret.isAimReady()
        );

        telemetry.addData(
                "Aim Clamped",
                turret.isHoldingBehind()
        );

        // -------------------------------------------------------------
        // Flower and controls
        // -------------------------------------------------------------

        telemetry.addLine("=== FLOWER ===");

        telemetry.addData(
                "Flower",
                flowerActive
                        ? "DROPPED"
                        : "LIFTED"
        );

        telemetry.addLine("=== CONTROLS ===");

        telemetry.addLine(
                "G1 Right bumper: collect"
        );

        telemetry.addLine(
                "G1 Left bumper: reverse intake"
        );

        telemetry.addLine(
                "G1 Right trigger: feed"
        );

        telemetry.addLine(
                "G1 Cross: shooter on"
        );

        telemetry.addLine(
                "G1 Triangle: shooter off"
        );

        telemetry.addLine(
                "G1 Circle: toggle regression"
        );

        telemetry.addLine(
                "G1 D-pad-right: flower toggle"
        );

        telemetry.addLine(
                "G1 Square: enable fused aim"
        );

        telemetry.addLine(
                "G2 Square: centre turret"
        );

        telemetry.addLine(
                "G2 Cross: manual lower target"
        );

        telemetry.addLine(
                "G2 Circle: manual upper target"
        );

        telemetry.addLine(
                "G2 Triangle: automatic targets"
        );

        telemetry.addLine(
                "G2 D-pad-down: lower corner reset"
        );

        telemetry.addLine(
                "G2 D-pad-up: upper corner reset"
        );

        telemetry.update();
    }

    private String formatVisibleHiveMembers() {

        if (camera == null) {
            return "NONE";
        }

        StringBuilder result =
                new StringBuilder();

        for (org.firstinspires.ftc.vision.apriltag
                .AprilTagSingleDetection detection
                : camera.getVisibleHiveMemberTags()) {

            ArduCam.HiveCell cell =
                    ArduCam.getHiveCellFromTagId(
                            detection.id
                    );

            if (result.length() > 0) {
                result.append(", ");
            }

            result.append(detection.id)
                    .append("=")
                    .append(
                            cell == null
                                    ? "UNKNOWN"
                                    : cell.toString()
                    );
        }

        return result.length() == 0
                ? "NONE"
                : result.toString();
    }

    private double normalizeDegrees0To360(
            double angleDegrees
    ) {

        double normalized =
                angleDegrees % 360.0;

        if (normalized < 0.0) {
            normalized += 360.0;
        }

        return normalized;
    }

    // ---------------------------------------------------------------------
    // Stop
    // ---------------------------------------------------------------------

    @Override
    public void stop() {

        if (outtake != null) {
            outtake.stopOuttake();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (robotSettings != null) {
            robotSettings.block();
        }

        if (turret != null) {
            turret.setCentre();
        }

        if (camera != null) {

            camera.close();
            camera = null;
        }

        if (follower != null) {

            follower.manual(
                    new DrivePowers(
                            0.0,
                            0.0,
                            0.0
                    )
            );

            follower.update();
        }

        if (hubs != null) {

            for (LynxModule hub : hubs) {

                hub.setBulkCachingMode(
                        LynxModule.BulkCachingMode.AUTO
                );
            }
        }
    }
}