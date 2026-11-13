package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * Combined TeleOp, Pinpoint-only turret aiming (no camera).
 *
 * Goal naming: the "upper" and "lower" goals here are the two goals at
 * different field Y positions. In Turret they map to the per-goal angle
 * trims:
 *
 *   UPPER goal -> Turret.HIGH_GOAL_ANGLE_TRIM_DEG
 *   LOWER goal -> Turret.LOW_GOAL_ANGLE_TRIM_DEG
 *
 * The goal coordinates themselves are the RED_/BLUE_ UPPER/LOWER fields
 * in this class. The Turret's own RED_LOW_GOAL_X/Y style coordinate
 * fields are not used by this TeleOp.
 */
@Configurable
@TeleOp(name = "0. Combined TeleOp", group = "Main")
public class CombinedTeleOp extends OpMode {

    public static double FIELD_SIZE_IN = 141.5;
    public static double FIELD_CENTER_IN = 70.75;
    public static double ROBOT_LENGTH_MM = 380.0;
    public static double ROBOT_WIDTH_MM = 308.0;

    public static double RED_LOWER_GOAL_X = 55.00;
    public static double RED_LOWER_GOAL_Y = 70.00;
    public static double RED_UPPER_GOAL_X = 59.06;
    public static double RED_UPPER_GOAL_Y = 94.81;
    public static double BLUE_LOWER_GOAL_X = 82.44;
    public static double BLUE_LOWER_GOAL_Y = 46.69;
    public static double BLUE_UPPER_GOAL_X = 82.44;
    public static double BLUE_UPPER_GOAL_Y = 94.81;

    public static double GOAL_SWITCH_Y_IN = 70.75;
    public static double GOAL_HYSTERESIS_IN = 1.0;

    public static double RED_LOWER_RESET_HEADING_DEG = 270.0;
    public static double RED_UPPER_RESET_HEADING_DEG = 90.0;
    public static double BLUE_LOWER_RESET_HEADING_DEG = 270.0;
    public static double BLUE_UPPER_RESET_HEADING_DEG = 90.0;

    public static double FORWARD_SIGN = -1.0;
    public static double STRAFE_SIGN = -1.0;
    public static double TURN_SIGN = -1.0;
    public static boolean CUBIC_STRAFE_TURN = false;

    public static boolean ALLOW_FIXED_TURRET_FEED = true;
    public static boolean FIXED_TURRET_BYPASSES_REGRESSION = true;

    public enum AllianceMode { STORED, RED, BLUE }
    public static AllianceMode ALLIANCE_MODE = AllianceMode.STORED;

    private PoseStorage.Alliance activeAlliance = PoseStorage.Alliance.UNKNOWN;
    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;
    private List<LynxModule> hubs;

    private boolean shooterActive;
    private boolean autoAimMode = true;
    private boolean flowerActive;
    private boolean poseEstablished;
    private boolean poseResetThisLoop;
    private boolean upperGoalSelected;
    private boolean goalSelectionInitialized;
    private boolean manualGoalMode;
    private boolean targetValid;
    private double goalX = Double.NaN;
    private double goalY = Double.NaN;
    private double distanceToGoalIn = Double.NaN;

    private boolean feederRequested;
    private boolean feederPermitted;
    private boolean fixedTurretFeedOverride;
    private String feederBlockReason = "NOT REQUESTED";

    private boolean previousFlowerButton;
    private boolean previousRegressionButton;
    private boolean previousAimButton;
    private boolean previousCentreButton;
    private boolean previousLowerResetButton;
    private boolean previousUpperResetButton;
    private boolean previousLowerGoalButton;
    private boolean previousUpperGoalButton;
    private boolean previousAutomaticGoalButton;
    private String status = "Initialising";

    @Override
    public void init() {
        ALLIANCE_MODE = AllianceMode.STORED;
        follower = Constants.create(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        intake = new Intake();
        outtake = new Outtake();
        turret = new Turret();
        flower = new Flower();
        robotSettings = new RobotSettings();
        intake.init(hardwareMap);
        outtake.init(hardwareMap);
        turret.init(hardwareMap);
        flower.init(hardwareMap);
        robotSettings.init(hardwareMap);

        Outtake.DISTANCE_REGRESSION_ENABLED = false;
        Outtake.HOOD_COMP_ENABLED = false;
        Turret.velocityCompensationActive = false;
        turret.setCentre();
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        robotSettings.block();

        activeAlliance = resolveAlliance();
        Pose savedPose = PoseStorage.getPose();
        if (validPose(savedPose)) {
            follower.setPose(savedPose);
            poseEstablished = true;
            status = "Stored pose restored; verify robot has not moved";
        } else {
            poseEstablished = false;
            status = "No stored pose; perform a corner reset";
        }
        displayTelemetry();
    }

    @Override
    public void init_loop() {
        clearBulkCache();
        if (gamepad1.dpad_left) ALLIANCE_MODE = AllianceMode.RED;
        else if (gamepad1.dpad_right) ALLIANCE_MODE = AllianceMode.BLUE;
        else if (gamepad1.dpad_up) ALLIANCE_MODE = AllianceMode.STORED;
        activeAlliance = resolveAlliance();
        updateGoalFromPose(poseEstablished ? follower.pose() : null);
        turret.setCentre();
        flowerActive = false;
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        displayTelemetry();
    }

    @Override
    public void start() {
        activeAlliance = resolveAlliance();
        if (activeAlliance == PoseStorage.Alliance.RED) PoseStorage.setRed();
        else if (activeAlliance == PoseStorage.Alliance.BLUE) PoseStorage.setBlue();

        shooterActive = false;
        autoAimMode = true;
        flowerActive = false;
        manualGoalMode = false;
        upperGoalSelected = false;
        goalSelectionInitialized = false;
        targetValid = false;
        goalX = goalY = distanceToGoalIn = Double.NaN;
        fixedTurretFeedOverride = false;
        feederBlockReason = "NOT REQUESTED";
        flower.flowerActivate(RobotConstants.flowerLiftPos);
        Outtake.DISTANCE_REGRESSION_ENABLED = false;
        Outtake.HOOD_COMP_ENABLED = false;

        previousFlowerButton = gamepad1.dpad_right;
        previousRegressionButton = gamepad1.circle;
        previousAimButton = gamepad1.square;
        previousCentreButton = gamepad2.square;
        previousLowerResetButton = gamepad2.dpad_down;
        previousUpperResetButton = gamepad2.dpad_up;
        previousLowerGoalButton = gamepad2.cross;
        previousUpperGoalButton = gamepad2.circle;
        previousAutomaticGoalButton = gamepad2.triangle;

        if (activeAlliance == PoseStorage.Alliance.RED) status = "LOCKED ALLIANCE: RED";
        else if (activeAlliance == PoseStorage.Alliance.BLUE) status = "LOCKED ALLIANCE: BLUE";
        else { autoAimMode = false; turret.setCentre(); status = "UNKNOWN ALLIANCE - AUTO AIM DISABLED"; }
    }

    @Override
    public void loop() {
        clearBulkCache();
        poseResetThisLoop = false;

        handleCornerResets();
        handleShooterControls();
        handleFlowerControls();
        handleRegressionToggle();

        if (!poseResetThisLoop) driveRobot();
        else follower.manual(new DrivePowers(0.0, 0.0, 0.0));

        Pose currentPose = follower.pose();
        if (poseEstablished && validPose(currentPose)) PoseStorage.setPose(currentPose);

        handleManualGoalSelection();
        if (manualGoalMode) applySelectedGoal();
        else updateGoalFromPose(currentPose);

        handleTurretModeControls();
        updateTurretAim(currentPose);

        boolean fixedShootingMode = ALLOW_FIXED_TURRET_FEED
                && !autoAimMode
                && turret.isFixedPositionMode();

        outtake.setGoalDistance(!fixedShootingMode
                && targetValid
                && poseEstablished
                && Double.isFinite(distanceToGoalIn)
                ? distanceToGoalIn : Double.NaN);

        // One update only. Calling Outtake.update() twice may distort its PID/TPS timing.
        outtake.update();
        handleIntakeAndFeeder();
        intake.update();
        displayTelemetry();
    }

    private void handleShooterControls() {
        if (gamepad1.right_bumper) shooterActive = false;
        if (gamepad1.cross) shooterActive = true;
        if (gamepad1.triangle) shooterActive = false;
        outtake.setTargetTPS(shooterActive ? Math.max(0.0, RobotConstants.OUTTAKE_TPS) : 0.0);
        outtake.setHoodPosition(RobotConstants.HOOD_POSITION);
    }

    private void handleRegressionToggle() {
        boolean pressed = gamepad1.circle;
        if (pressed && !previousRegressionButton) {
            Outtake.DISTANCE_REGRESSION_ENABLED = !Outtake.DISTANCE_REGRESSION_ENABLED;
            stopFeederOnly();
            intake.stopIntake();
            robotSettings.block();
            status = Outtake.DISTANCE_REGRESSION_ENABLED ? "REGRESSION ENABLED" : "FIXED TPS MODE";
        }
        previousRegressionButton = pressed;
    }

    private void handleIntakeAndFeeder() {
        feederRequested = false;
        feederPermitted = false;
        fixedTurretFeedOverride = false;
        feederBlockReason = "NOT REQUESTED";

        if (gamepad1.right_bumper) {
            intake.intake(RobotConstants.FRONT_INTAKE_SPEED, RobotConstants.BACK_INTAKE_SPEED);
            robotSettings.block();
            stopFeederOnly();
            feederBlockReason = "COLLECTING";
            return;
        }

        if (gamepad1.right_trigger > 0.3) {
            if (!feederAllowed()) {
                robotSettings.block();
                intake.stopIntake();
                stopFeederOnly();
                return;
            }
            robotSettings.release();
            intake.intake(RobotConstants.FRONT_FEED_SPEED, RobotConstants.BACK_FEED_SPEED);
            outtake.setFeederTPS(RobotConstants.FEEDER_TPS);
            feederRequested = true;
            feederPermitted = true;
            feederBlockReason = fixedTurretFeedOverride
                    ? "FEEDING: FIXED TURRET" : "FEEDING: AUTO AIM";
            return;
        }

        if (gamepad1.left_bumper) {
            intake.intake(-RobotConstants.FRONT_INTAKE_SPEED, -RobotConstants.BACK_INTAKE_SPEED);
            robotSettings.block();
            stopFeederOnly();
            feederBlockReason = "REVERSING INTAKE";
            return;
        }

        intake.stopIntake();
        robotSettings.block();
        stopFeederOnly();
    }

    private boolean feederAllowed() {
        fixedTurretFeedOverride = ALLOW_FIXED_TURRET_FEED
                && !autoAimMode
                && turret.isFixedPositionMode();

        if (!shooterActive) return blockFeed("SHOOTER OFF");

        if (!fixedTurretFeedOverride) {
            if (!autoAimMode) return blockFeed("AUTO AIM DISABLED");
            if (!poseEstablished) return blockFeed("POSE NOT ESTABLISHED");
            if (!targetValid) return blockFeed("TARGET INVALID");
            if (!turret.isTargetReachable()) return blockFeed("TURRET UNREACHABLE");
            if (!turret.isAimReady()) return blockFeed("AIM NOT READY");
        }

        boolean regressionRequired = Outtake.DISTANCE_REGRESSION_ENABLED
                && (!fixedTurretFeedOverride || !FIXED_TURRET_BYPASSES_REGRESSION);
        if (regressionRequired && !outtake.isUsingDistanceRegression())
            return blockFeed("REGRESSION INVALID");

        if (RobotConstants.FEED_REQUIRES_SPEED
                && !outtake.isAtSpeed(RobotConstants.FEED_SPEED_TOLERANCE_TPS))
            return blockFeed("SHOOTER NOT AT SPEED");

        feederBlockReason = fixedTurretFeedOverride
                ? "PERMITTED: FIXED TURRET" : "PERMITTED: AUTO AIM";
        return true;
    }

    private boolean blockFeed(String reason) {
        feederBlockReason = reason;
        return false;
    }

    private void stopFeederOnly() {
        feederRequested = false;
        feederPermitted = false;
        outtake.setFeederTPS(0.0);
        outtake.stopFeeder();
    }

    private void handleManualGoalSelection() {
        boolean lower = gamepad2.cross;
        boolean upper = gamepad2.circle;
        boolean automatic = gamepad2.triangle;
        if (automatic && !previousAutomaticGoalButton) {
            manualGoalMode = false;
            goalSelectionInitialized = false;
            stopFeederOnly();
            status = "Automatic Y target selection enabled";
        }
        if (lower && !previousLowerGoalButton) {
            manualGoalMode = true;
            upperGoalSelected = false;
            goalSelectionInitialized = true;
            applySelectedGoal();
            status = allianceLabel() + " LOWER target manually selected";
        }
        if (upper && !previousUpperGoalButton) {
            manualGoalMode = true;
            upperGoalSelected = true;
            goalSelectionInitialized = true;
            applySelectedGoal();
            status = allianceLabel() + " UPPER target manually selected";
        }
        previousLowerGoalButton = lower;
        previousUpperGoalButton = upper;
        previousAutomaticGoalButton = automatic;
    }

    private void applySelectedGoal() {
        /*
         * Tell the Turret which goal is selected so it applies that goal's
         * angle trim (UPPER = HIGH trim, LOWER = LOW trim). Done every time,
         * before anything can return early.
         */
        turret.setGoalHeight(upperGoalSelected
                ? Turret.GoalHeight.HIGH
                : Turret.GoalHeight.LOW);

        targetValid = false;
        goalX = goalY = Double.NaN;
        if (activeAlliance == PoseStorage.Alliance.RED) {
            goalX = upperGoalSelected ? RED_UPPER_GOAL_X : RED_LOWER_GOAL_X;
            goalY = upperGoalSelected ? RED_UPPER_GOAL_Y : RED_LOWER_GOAL_Y;
        } else if (activeAlliance == PoseStorage.Alliance.BLUE) {
            goalX = upperGoalSelected ? BLUE_UPPER_GOAL_X : BLUE_LOWER_GOAL_X;
            goalY = upperGoalSelected ? BLUE_UPPER_GOAL_Y : BLUE_LOWER_GOAL_Y;
        } else return;
        targetValid = validFieldPoint(goalX, goalY);
        if (!targetValid) return;
        Pose existing = turret.getTargetPose();
        boolean changed = turret.getSelectedGoal() != Turret.Goal.CUSTOM
                || existing == null
                || Double.compare(existing.x(), goalX) != 0
                || Double.compare(existing.y(), goalY) != 0;
        if (changed) {
            turret.setTargetPose(new Pose(goalX, goalY, 0.0));
            turret.realignToSelectedGoal();
            stopFeederOnly();
        }
    }

    private void updateGoalFromPose(Pose pose) {
        targetValid = false;
        goalX = goalY = Double.NaN;
        if (!allianceKnown() || !poseEstablished || !validPose(pose)) return;
        double hysteresis = Math.max(0.0, GOAL_HYSTERESIS_IN);
        double y = pose.y();
        boolean next = upperGoalSelected;
        if (!goalSelectionInitialized || hysteresis == 0.0) next = y >= GOAL_SWITCH_Y_IN;
        else if (upperGoalSelected && y < GOAL_SWITCH_Y_IN - hysteresis) next = false;
        else if (!upperGoalSelected && y >= GOAL_SWITCH_Y_IN + hysteresis) next = true;
        upperGoalSelected = next;
        goalSelectionInitialized = true;
        applySelectedGoal();
    }

    private void handleFlowerControls() {
        boolean pressed = gamepad1.dpad_right;
        if (pressed && !previousFlowerButton) flowerActive = !flowerActive;
        previousFlowerButton = pressed;
        flower.flowerActivate(flowerActive ? RobotConstants.flowerDropPos : RobotConstants.flowerLiftPos);
    }

    private void handleTurretModeControls() {
        boolean aim = gamepad1.square;
        if (aim && !previousAimButton) {
            if (allianceKnown() && poseEstablished && targetValid) {
                autoAimMode = true;
                fixedTurretFeedOverride = false;
                turret.realignToSelectedGoal();
                stopFeederOnly();
                status = "Automatic aim enabled";
            } else status = "Aim blocked: alliance, pose or target invalid";
        }
        previousAimButton = aim;

        boolean centre = gamepad2.square;
        if (centre && !previousCentreButton) {
            autoAimMode = false;
            turret.setCentre();
            stopFeederOnly();
            fixedTurretFeedOverride = ALLOW_FIXED_TURRET_FEED;
            feederBlockReason = ALLOW_FIXED_TURRET_FEED
                    ? "FIXED TURRET SELECTED" : "FIXED FEED DISABLED";
            status = ALLOW_FIXED_TURRET_FEED
                    ? "Turret centred; fixed shooting available" : "Turret centred; feeding disabled";
        }
        previousCentreButton = centre;
    }

    private void updateTurretAim(Pose pose) {
        distanceToGoalIn = Double.NaN;
        if (!targetValid || !poseEstablished || !validPose(pose)) {
            if (autoAimMode) turret.setCentre();
            return;
        }
        distanceToGoalIn = calculateDistanceToGoal(pose);
        if (!autoAimMode) return;
        turret.aimTurret(pose, 0.0, 0.0);
    }

    private double calculateDistanceToGoal(Pose pose) {
        double h = pose.heading();
        double pivotX = pose.x() + Turret.TURRET_OFFSET_X_IN * Math.cos(h)
                - Turret.TURRET_OFFSET_Y_IN * Math.sin(h);
        double pivotY = pose.y() + Turret.TURRET_OFFSET_X_IN * Math.sin(h)
                + Turret.TURRET_OFFSET_Y_IN * Math.cos(h);
        return Math.hypot(goalX - pivotX, goalY - pivotY);
    }

    private void handleCornerResets() {
        boolean lower = gamepad2.dpad_down;
        boolean upper = gamepad2.dpad_up;
        if (lower && !previousLowerResetButton) resetAtAllianceCorner(false);
        else if (upper && !previousUpperResetButton) resetAtAllianceCorner(true);
        previousLowerResetButton = lower;
        previousUpperResetButton = upper;
    }

    private void resetAtAllianceCorner(boolean upper) {
        if (!allianceKnown()) { status = "Corner reset blocked: alliance unknown"; return; }
        double halfLength = ROBOT_LENGTH_MM / 25.4 / 2.0;
        double halfWidth = ROBOT_WIDTH_MM / 25.4 / 2.0;
        boolean red = activeAlliance == PoseStorage.Alliance.RED;
        double heading = red
                ? (upper ? RED_UPPER_RESET_HEADING_DEG : RED_LOWER_RESET_HEADING_DEG)
                : (upper ? BLUE_UPPER_RESET_HEADING_DEG : BLUE_LOWER_RESET_HEADING_DEG);
        double x = red ? halfWidth : FIELD_SIZE_IN - halfWidth;
        double y = upper ? FIELD_SIZE_IN - halfLength : halfLength;
        if (!validFieldPoint(x, y)) { status = "Corner reset blocked: invalid geometry"; return; }
        Pose reset = new Pose(x, y, Math.toRadians(heading));
        shooterActive = false;
        outtake.setTargetTPS(0.0);
        stopFeederOnly();
        intake.stopIntake();
        robotSettings.block();
        follower.manual(new DrivePowers(0.0, 0.0, 0.0));
        follower.setPose(reset);
        PoseStorage.setPose(reset);
        poseEstablished = true;
        poseResetThisLoop = true;
        manualGoalMode = false;
        goalSelectionInitialized = false;
        updateGoalFromPose(reset);
        turret.realignToSelectedGoal();
        status = (red ? "RED " : "BLUE ") + (upper ? "UPPER" : "LOWER") + " corner reset";
    }

    private PoseStorage.Alliance resolveAlliance() {
        if (ALLIANCE_MODE == AllianceMode.RED) return PoseStorage.Alliance.RED;
        if (ALLIANCE_MODE == AllianceMode.BLUE) return PoseStorage.Alliance.BLUE;
        return PoseStorage.lastAlliance == null ? PoseStorage.Alliance.UNKNOWN : PoseStorage.lastAlliance;
    }

    private boolean allianceKnown() {
        return activeAlliance == PoseStorage.Alliance.RED || activeAlliance == PoseStorage.Alliance.BLUE;
    }

    private String allianceLabel() {
        return activeAlliance == PoseStorage.Alliance.RED ? "RED"
                : activeAlliance == PoseStorage.Alliance.BLUE ? "BLUE" : "UNKNOWN";
    }

    private void driveRobot() {
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;
        if (CUBIC_STRAFE_TURN) {
            strafeInput = Math.pow(strafeInput, 3);
            turnInput = Math.pow(turnInput, 3);
        }
        double forward = FORWARD_SIGN * gamepad1.left_stick_y * RobotConstants.DRIVE_SPEED;
        double strafe = STRAFE_SIGN * strafeInput * RobotConstants.DRIVE_SPEED;
        double turn = TURN_SIGN * turnInput * RobotConstants.DRIVE_SPEED;
        if (gamepad2.dpad_down || gamepad2.dpad_up) forward = strafe = turn = 0.0;
        follower.manual(new DrivePowers(forward, strafe, turn));
        follower.update();
    }

    private void clearBulkCache() {
        if (hubs != null) for (LynxModule hub : hubs) hub.clearBulkCache();
    }

    private boolean validPose(Pose pose) {
        return pose != null && Double.isFinite(pose.x())
                && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    private boolean validFieldPoint(double x, double y) {
        return Double.isFinite(x) && Double.isFinite(y)
                && x >= 0.0 && y >= 0.0 && x <= FIELD_SIZE_IN && y <= FIELD_SIZE_IN;
    }

    private void displayTelemetry() {
        Pose pose = follower == null ? null : follower.pose();
        telemetry.addLine("=== STATUS ===");
        telemetry.addData("Status", status);
        telemetry.addData("Alliance", activeAlliance);
        telemetry.addData("Pose Established", poseEstablished);
        if (validPose(pose)) telemetry.addData("Pose", "(%.2f, %.2f, %.1f deg)",
                pose.x(), pose.y(), normalizeDegrees0To360(Math.toDegrees(pose.heading())));

        telemetry.addLine("=== SHOOTER / FEEDER ===");
        telemetry.addData("Shooter", shooterActive ? "ON" : "OFF");
        telemetry.addData("TPS", "%.0f / %.0f", outtake.getCurrentTPS(), outtake.getTargetTPS());
        telemetry.addData("At Speed", outtake.isAtSpeed(RobotConstants.FEED_SPEED_TOLERANCE_TPS));
        telemetry.addData("Raw / Filtered TPS", "%.0f / %.0f", outtake.getRawTPS(), outtake.getCurrentTPS());
        //telemetry.addData("Shooter Power", "%.3f (integral %.3f, voltage scale %.2f)",
        //telemetry.addData("Regression", "%s | fixed target %.0f",
        telemetry.addData("Feeder Requested", feederRequested);
        telemetry.addData("Feeder Permitted", feederPermitted);
        telemetry.addData("Feeder Status", feederBlockReason);
        telemetry.addData("Fixed Override", fixedTurretFeedOverride);

        telemetry.addLine("=== TURRET ===");
        String turretState;
        if (!autoAimMode) {
            turretState = "FIXED (auto aim off - press G1 Square)";
        } else if (!targetValid || !poseEstablished) {
            turretState = "NO TARGET/POSE - turret centred";
        } else if (turret.isTargetReachable()) {
            turretState = "TRACKING";
        } else {
            turretState = "AT LIMIT " + (Turret.lastHoldSide > 0 ? "(+)" : "(-)")
                    + " - goal outside arc, check heading or turn robot";
        }
        telemetry.addData("Turret State", turretState);
        telemetry.addData("Auto Aim", autoAimMode);
        telemetry.addData("Aim Source", turret.getAimSource());
        telemetry.addData("Aim Ready", turret.isAimReady());
        telemetry.addData("Goal", "%s %s (%s)",
                allianceLabel(),
                upperGoalSelected ? "UPPER" : "LOWER",
                manualGoalMode ? "manual" : "auto");
        telemetry.addData("Target", "(%.2f, %.2f)", goalX, goalY);
        telemetry.addData("Distance", "%.2f in", distanceToGoalIn);
        telemetry.addData("Angle Trim", "%.2f deg", Turret.lastAngleTrimDeg);
        telemetry.addData("Requested / Commanded", "%.1f / %.1f deg",
                Turret.lastRelativeAngleDeg, Turret.lastFinalAngleDeg);
        telemetry.addData("Servo", "%.3f", turret.getServoPosition());
        telemetry.addData("Reachable", turret.isTargetReachable());
        telemetry.addData("Turret used pose", "(%.2f, %.2f, %.0f deg)",
                Turret.lastRobotX, Turret.lastRobotY, Turret.lastRobotHeadingDeg);

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("G1 Cross: shooter on | Triangle: off");
        telemetry.addLine("G1 RT: feed immediately");
        telemetry.addLine("G1 RB: collect / shooter off");
        telemetry.addLine("G1 Square: auto aim");
        telemetry.addLine("G2 Square: centre / fixed shooting");
        telemetry.addLine("G2 Cross/Circle/Triangle: lower/upper/auto goal");
        telemetry.addLine("G2 D-pad up/down: corner reset");
        telemetry.update();
    }

    private double normalizeDegrees0To360(double degrees) {
        double result = degrees % 360.0;
        return result < 0.0 ? result + 360.0 : result;
    }

    @Override
    public void stop() {
        if (outtake != null) outtake.stopOuttake();
        if (intake != null) intake.stopIntake();
        if (robotSettings != null) robotSettings.block();
        if (turret != null) turret.setCentre();
        if (follower != null) {
            follower.manual(new DrivePowers(0.0, 0.0, 0.0));
            follower.update();
        }
        if (hubs != null) for (LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
    }
}