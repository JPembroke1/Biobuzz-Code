package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@TeleOp(name = "0. Combined TeleOp", group = "Main")
public class CombinedTeleOp extends OpMode {

    // ---------------------------------------------------------------------
    // Field and robot geometry
    //
    // Origin: lower-left.
    // +X: right.
    // +Y: up.
    // Field coordinates are in inches.
    //
    // Corner resets assume Pedro's pose reference is at robot centre.
    // ---------------------------------------------------------------------

    public static double FIELD_SIZE_IN = 141.5;

    public static double ROBOT_LENGTH_MM = 340.0;
    public static double ROBOT_WIDTH_MM = 340.0;

    // ---------------------------------------------------------------------
    // Goal coordinates
    //
    // Provisional horizontal opening-centre coordinates.
    // Verify against the physical field before shooting.
    //
    // LOWER and UPPER refer to field Y, not raised/lowered CELL state.
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
    //
    // Initial selection:
    //   Lower when Y < 70.75.
    //   Upper when Y >= 70.75.
    //
    // Hysteresis:
    //   Lower -> upper at Y >= 71.75.
    //   Upper -> lower at Y < 69.75.
    // ---------------------------------------------------------------------

    public static double GOAL_SWITCH_Y_IN = 70.75;
    public static double GOAL_HYSTERESIS_IN = 1.0;

    // ---------------------------------------------------------------------
    // Corner reset headings
    //
    // Red resets: X = 0 side.
    // Blue resets: X = FIELD_SIZE_IN side.
    //
    // Robot front:
    //   Lower corners: negative Y, heading 270 degrees.
    //   Upper corners: positive Y, heading 90 degrees.
    // ---------------------------------------------------------------------

    public static double RED_LOWER_RESET_HEADING_DEG = 270.0;
    public static double RED_UPPER_RESET_HEADING_DEG = 90.0;

    public static double BLUE_LOWER_RESET_HEADING_DEG = 270.0;
    public static double BLUE_UPPER_RESET_HEADING_DEG = 90.0;

    // ---------------------------------------------------------------------
    // Alliance selection
    //
    // Testing override is available during INIT.
    // Alliance locks at START.
    // ---------------------------------------------------------------------

    public enum AllianceMode {
        STORED,
        RED,
        BLUE
    }

    public static AllianceMode ALLIANCE_MODE = AllianceMode.STORED;

    private PoseStorage.Alliance activeAlliance =
            PoseStorage.Alliance.UNKNOWN;

    // ---------------------------------------------------------------------
    // Subsystems
    // ---------------------------------------------------------------------

    private Follower follower;
    private Intake intake;
    private Outtake outtake;
    private Turret turret;

    // ---------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------

    private DcMotor frontRight;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor backLeft;

    private Servo blockServo;

    // ---------------------------------------------------------------------
    // Localization and target state
    // ---------------------------------------------------------------------

    private boolean poseEstablished = false;

    private boolean upperGoalSelected = false;
    private boolean goalSelectionInitialized = false;
    private boolean targetValid = false;

    private double goalX = Double.NaN;
    private double goalY = Double.NaN;
    private double distanceToGoalIn = Double.NaN;

    // ---------------------------------------------------------------------
    // Mechanism state
    // ---------------------------------------------------------------------

    private boolean shooterEnabled = false;
    private boolean autoAimMode = true;

    private boolean feedReleaseRequired = true;
    private boolean interruptFeedThisLoop = false;

    private long feedStartNanos = -1L;

    // ---------------------------------------------------------------------
    // Button edge detection
    // ---------------------------------------------------------------------

    private boolean previousCross = false;
    private boolean previousAutoAim = false;
    private boolean previousCentre = false;
    private boolean previousLowerReset = false;
    private boolean previousUpperReset = false;

    private String status = "Verify alliance, pose and target";

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    @Override
    public void init() {

        // Avoid retaining an override from an earlier TeleOp run.
        ALLIANCE_MODE = AllianceMode.STORED;

        follower = Constants.create(hardwareMap);

        intake = new Intake();
        intake.init(hardwareMap);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap);
        turret.setCentre();

        frontRight = hardwareMap.get(
                DcMotor.class, "frontRight"
        );

        frontLeft = hardwareMap.get(
                DcMotor.class, "frontLeft"
        );

        backRight = hardwareMap.get(
                DcMotor.class, "backRight"
        );

        backLeft = hardwareMap.get(
                DcMotor.class, "backLeft"
        );

        frontRight.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        backRight.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        frontLeft.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        backLeft.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        blockServo = hardwareMap.get(
                Servo.class, "blockServo"
        );

        blockServo.setDirection(Servo.Direction.FORWARD);
        blockServo.setPosition(RobotConstants.blockClosePos);

        outtake.setHoodPosition(
                RobotConstants.HOOD_POSITION
        );

        // Compatible with your original PoseStorage.
        Pose savedPose = PoseStorage.currentPose;

        if (validPose(savedPose)) {

            follower.setPose(copyPose(savedPose));
            poseEstablished = true;

            status = "Stored pose restored: verify robot has not moved";

        } else {

            poseEstablished = false;

            status = "No stored pose: perform a corner reset";
        }

        activeAlliance = resolveAlliance();

        displayTelemetry(follower.pose(), true);
    }

    @Override
    public void init_loop() {

        if (gamepad1.dpad_left) {
            ALLIANCE_MODE = AllianceMode.RED;

        } else if (gamepad1.dpad_right) {
            ALLIANCE_MODE = AllianceMode.BLUE;

        } else if (gamepad1.dpad_up) {
            ALLIANCE_MODE = AllianceMode.STORED;
        }

        activeAlliance = resolveAlliance();

        // Preview the target without automatically aiming during INIT.
        updateGoalSelection(follower.pose());

        turret.setCentre();

        displayTelemetry(follower.pose(), true);
    }

    @Override
    public void start() {

        activeAlliance = resolveAlliance();

        if (activeAlliance == PoseStorage.Alliance.RED) {
            PoseStorage.setRed();

        } else if (activeAlliance == PoseStorage.Alliance.BLUE) {
            PoseStorage.setBlue();
        }

        goalSelectionInitialized = false;
        targetValid = false;

        shooterEnabled = false;
        autoAimMode = true;
        feedReleaseRequired = true;

        resetFeedTimer();

        // Require fresh button presses after START.
        previousCross = gamepad1.cross;
        previousAutoAim = gamepad1.left_bumper;
        previousCentre = gamepad1.square;

        previousLowerReset =
                gamepad1.options && gamepad1.dpad_down;

        previousUpperReset =
                gamepad1.options && gamepad1.dpad_up;

        status = allianceKnown()
                ? "Alliance locked: verify selected target"
                : "Unknown alliance: restart and select during INIT";
    }

    // ---------------------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------------------

    @Override
    public void loop() {

        interruptFeedThisLoop = false;

        follower.update();

        handleCornerReset();

        // Read pose after any reset.
        Pose currentPose = follower.pose();

        updateGoalSelection(currentPose);
        handleTurretControls();

        boolean canAim =
                poseReady(currentPose) && targetValid;

        distanceToGoalIn = canAim
                ? calculateShotDistance(currentPose)
                : Double.NaN;

        if (!Double.isFinite(distanceToGoalIn)
                || distanceToGoalIn < Math.max(
                0.0, Turret.MIN_AIM_DISTANCE_IN
        )) {
            canAim = false;
        }

        if (canAim && autoAimMode) {

            // No velocity lead until field-frame velocities are supplied.
            turret.aimTurret(currentPose, 0.0, 0.0);

        } else {
            turret.setCentre();
        }

        handleShooterControls(canAim);

        outtake.setHoodPosition(
                RobotConstants.HOOD_POSITION
        );

        outtake.setGoalDistance(
                canAim ? distanceToGoalIn : Double.NaN
        );

        // Resolve the active TPS before checking feed readiness.
        outtake.update();

        handleIntakeControls(canAim);

        intake.update();
        driveRobot();

        if (poseReady(currentPose)) {
            PoseStorage.setPose(copyPose(currentPose));
        }

        displayTelemetry(currentPose, false);
    }

    // ---------------------------------------------------------------------
    // Alliance
    // ---------------------------------------------------------------------

    private PoseStorage.Alliance resolveAlliance() {

        if (ALLIANCE_MODE == AllianceMode.RED) {
            return PoseStorage.Alliance.RED;
        }

        if (ALLIANCE_MODE == AllianceMode.BLUE) {
            return PoseStorage.Alliance.BLUE;
        }

        return PoseStorage.lastAlliance == null
                ? PoseStorage.Alliance.UNKNOWN
                : PoseStorage.lastAlliance;
    }

    private boolean allianceKnown() {

        return activeAlliance == PoseStorage.Alliance.RED
                || activeAlliance == PoseStorage.Alliance.BLUE;
    }

    // ---------------------------------------------------------------------
    // Automatic goal selection
    // ---------------------------------------------------------------------

    private void updateGoalSelection(Pose pose) {

        targetValid = false;

        goalX = Double.NaN;
        goalY = Double.NaN;

        if (!allianceKnown() || !poseReady(pose)) {
            return;
        }

        if (!Double.isFinite(GOAL_SWITCH_Y_IN)
                || !Double.isFinite(GOAL_HYSTERESIS_IN)) {
            return;
        }

        double hysteresis =
                Math.max(0.0, GOAL_HYSTERESIS_IN);

        double y = pose.y();

        boolean nextUpper = upperGoalSelected;

        if (!goalSelectionInitialized) {

            nextUpper = y >= GOAL_SWITCH_Y_IN;

        } else if (hysteresis == 0.0) {

            nextUpper = y >= GOAL_SWITCH_Y_IN;

        } else if (upperGoalSelected) {

            if (y < GOAL_SWITCH_Y_IN - hysteresis) {
                nextUpper = false;
            }

        } else if (y >= GOAL_SWITCH_Y_IN + hysteresis) {

            nextUpper = true;
        }

        boolean openingChanged =
                goalSelectionInitialized
                        && nextUpper != upperGoalSelected;

        upperGoalSelected = nextUpper;
        goalSelectionInitialized = true;

        if (activeAlliance == PoseStorage.Alliance.RED) {

            goalX = upperGoalSelected
                    ? RED_UPPER_GOAL_X
                    : RED_LOWER_GOAL_X;

            goalY = upperGoalSelected
                    ? RED_UPPER_GOAL_Y
                    : RED_LOWER_GOAL_Y;

        } else {

            goalX = upperGoalSelected
                    ? BLUE_UPPER_GOAL_X
                    : BLUE_LOWER_GOAL_X;

            goalY = upperGoalSelected
                    ? BLUE_UPPER_GOAL_Y
                    : BLUE_LOWER_GOAL_Y;
        }

        if (!validFieldPoint(goalX, goalY)) {
            return;
        }

        Pose previousTarget = turret.getTargetPose();

        boolean targetChanged =
                turret.getSelectedGoal() != Turret.Goal.CUSTOM
                        || previousTarget == null
                        || Double.compare(
                        previousTarget.x(), goalX
                ) != 0
                        || Double.compare(
                        previousTarget.y(), goalY
                ) != 0;

        // Only reselect when necessary.
        // Repeated selection would reset turret hold hysteresis.
        if (targetChanged) {

            turret.setTargetPose(
                    new Pose(goalX, goalY, 0.0)
            );
        }

        targetValid = true;

        if (openingChanged || targetChanged) {

            // Keep the flywheel running but interrupt feeding.
            feedReleaseRequired = true;
            interruptFeedThisLoop = true;

            resetFeedTimer();
        }
    }

    private double calculateShotDistance(Pose pose) {

        double heading = pose.heading();

        double pivotX =
                pose.x()
                        + Turret.TURRET_OFFSET_X_IN
                        * Math.cos(heading)
                        - Turret.TURRET_OFFSET_Y_IN
                        * Math.sin(heading);

        double pivotY =
                pose.y()
                        + Turret.TURRET_OFFSET_X_IN
                        * Math.sin(heading)
                        + Turret.TURRET_OFFSET_Y_IN
                        * Math.cos(heading);

        double distance = Math.hypot(
                goalX - pivotX,
                goalY - pivotY
        );

        return Double.isFinite(distance)
                ? distance
                : Double.NaN;
    }

    // ---------------------------------------------------------------------
    // Turret controls
    // ---------------------------------------------------------------------

    private void handleTurretControls() {

        boolean toggle = gamepad1.left_bumper;

        if (toggle && !previousAutoAim) {

            autoAimMode = !autoAimMode;

            if (autoAimMode) {
                turret.realignToSelectedGoal();
            }

            feedReleaseRequired = true;
            interruptFeedThisLoop = true;
        }

        previousAutoAim = toggle;

        boolean centre = gamepad1.square;

        if (centre && !previousCentre) {

            autoAimMode = false;
            turret.setCentre();

            feedReleaseRequired = true;
            interruptFeedThisLoop = true;
        }

        previousCentre = centre;
    }

    // ---------------------------------------------------------------------
    // Shooter controls
    // ---------------------------------------------------------------------

    private void handleShooterControls(boolean canAim) {

        boolean cross = gamepad1.cross;

        if (cross
                && !previousCross
                && canAim
                && !interruptFeedThisLoop) {

            shooterEnabled = true;
        }

        previousCross = cross;

        // Stop takes priority.
        if (gamepad1.triangle || !canAim) {

            shooterEnabled = false;

            feedReleaseRequired = true;
            interruptFeedThisLoop = true;
        }

        if (shooterEnabled) {

            // Fixed fallback and shooter run request.
            // Outtake.update() selects regression TPS when available.
            outtake.setTargetTPS(
                    RobotConstants.OUTTAKE_TPS
            );

        } else {
            outtake.stopOuttake();
        }
    }

    // ---------------------------------------------------------------------
    // Intake and feeding
    //
    // Right bumper: collect.
    // Right trigger: feed.
    // Left trigger: reverse.
    //
    // The separate Outtake feeder motor is not automatically commanded.
    // ---------------------------------------------------------------------

    private void handleIntakeControls(boolean canAim) {

        boolean feedRequested =
                gamepad1.right_trigger > 0.3;

        if (!feedRequested) {
            feedReleaseRequired = false;
        }

        if (interruptFeedThisLoop) {
            stopIntakeAndCloseBlock();
            return;
        }

        if (gamepad1.right_bumper) {

            intake.intake(
                    clip(
                            RobotConstants.FRONT_INTAKE_SPEED,
                            -1.0,
                            1.0
                    ),
                    clip(
                            RobotConstants.BACK_INTAKE_SPEED,
                            -1.0,
                            1.0
                    )
            );

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );

            resetFeedTimer();

        } else if (feedRequested) {

            boolean speedReady =
                    !RobotConstants.FEED_REQUIRES_SPEED
                            || outtake.isAtSpeed(
                            RobotConstants.FEED_SPEED_TOLERANCE_TPS
                    );

            // This is an angular reach check, not measured alignment.
            boolean targetWithinTravel =
                    Double.isFinite(
                            Turret.lastRelativeAngleDeg
                    )
                            && Math.abs(
                            Turret.lastRelativeAngleDeg
                    ) <= 90.0
                            && !turret.isHoldingBehind();

            boolean feedAllowed =
                    canAim
                            && shooterEnabled
                            && outtake.getTargetTPS() > 0.0
                            && autoAimMode
                            && targetWithinTravel
                            && !feedReleaseRequired
                            && speedReady;

            if (!feedAllowed) {
                stopIntakeAndCloseBlock();
                return;
            }

            blockServo.setPosition(
                    RobotConstants.blockOpenPos
            );

            if (feedStartNanos < 0L) {
                feedStartNanos = System.nanoTime();
            }

            double elapsed =
                    (System.nanoTime() - feedStartNanos)
                            / 1_000_000_000.0;

            RobotConstants.timer = elapsed;

            if (elapsed >= Math.max(
                    0.0, RobotConstants.delayAmount
            )) {

                intake.intake(
                        clip(
                                RobotConstants.FRONT_FEED_SPEED,
                                -1.0,
                                1.0
                        ),
                        clip(
                                RobotConstants.BACK_FEED_SPEED,
                                -1.0,
                                1.0
                        )
                );

            } else {
                intake.stopIntake();
            }

        } else if (gamepad1.left_trigger > 0.3) {

            intake.intake(
                    -clip(
                            RobotConstants.FRONT_INTAKE_SPEED,
                            -1.0,
                            1.0
                    ),
                    -clip(
                            RobotConstants.BACK_INTAKE_SPEED,
                            -1.0,
                            1.0
                    )
            );

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );

            resetFeedTimer();

        } else {
            stopIntakeAndCloseBlock();
        }
    }

    private void stopIntakeAndCloseBlock() {

        intake.stopIntake();

        blockServo.setPosition(
                RobotConstants.blockClosePos
        );

        resetFeedTimer();
    }

    private void resetFeedTimer() {

        feedStartNanos = -1L;
        RobotConstants.timer = 0.0;
    }

    // ---------------------------------------------------------------------
    // Corner reset
    // ---------------------------------------------------------------------

    private void handleCornerReset() {

        boolean lower =
                gamepad1.options && gamepad1.dpad_down;

        boolean upper =
                gamepad1.options && gamepad1.dpad_up;

        if (lower && !previousLowerReset) {
            resetAtCorner(false);

        } else if (upper && !previousUpperReset) {
            resetAtCorner(true);
        }

        previousLowerReset = lower;
        previousUpperReset = upper;
    }

    private void resetAtCorner(boolean upper) {

        shooterEnabled = false;
        outtake.stopOuttake();

        stopIntakeAndCloseBlock();

        feedReleaseRequired = true;
        interruptFeedThisLoop = true;

        if (!allianceKnown()) {

            status = "Reset blocked: unknown alliance";
            return;
        }

        boolean red =
                activeAlliance == PoseStorage.Alliance.RED;

        double headingDeg = red
                ? (upper
                   ? RED_UPPER_RESET_HEADING_DEG
                   : RED_LOWER_RESET_HEADING_DEG)
                : (upper
                   ? BLUE_UPPER_RESET_HEADING_DEG
                   : BLUE_LOWER_RESET_HEADING_DEG);

        if (!Double.isFinite(headingDeg)
                || !Double.isFinite(FIELD_SIZE_IN)
                || FIELD_SIZE_IN <= 0.0
                || !Double.isFinite(ROBOT_LENGTH_MM)
                || !Double.isFinite(ROBOT_WIDTH_MM)
                || ROBOT_LENGTH_MM <= 0.0
                || ROBOT_WIDTH_MM <= 0.0) {

            status = "Reset blocked: invalid dimensions or heading";
            return;
        }

        double heading = Math.toRadians(headingDeg);

        double halfLength =
                ROBOT_LENGTH_MM / 25.4 / 2.0;

        double halfWidth =
                ROBOT_WIDTH_MM / 25.4 / 2.0;

        // Project the rectangular footprint onto field X and Y.
        // Assumes the localization reference is at robot centre.
        double extentX =
                Math.abs(Math.cos(heading)) * halfLength
                        + Math.abs(Math.sin(heading)) * halfWidth;

        double extentY =
                Math.abs(Math.sin(heading)) * halfLength
                        + Math.abs(Math.cos(heading)) * halfWidth;

        if (2.0 * extentX >= FIELD_SIZE_IN
                || 2.0 * extentY >= FIELD_SIZE_IN) {

            status = "Reset blocked: invalid robot footprint";
            return;
        }

        double x = red
                ? extentX
                : FIELD_SIZE_IN - extentX;

        double y = upper
                ? FIELD_SIZE_IN - extentY
                : extentY;

        if (!validFieldPoint(x, y)) {

            status = "Reset blocked: invalid corner position";
            return;
        }

        Pose resetPose = new Pose(
                x,
                y,
                heading
        );

        follower.setPose(resetPose);
        PoseStorage.setPose(copyPose(resetPose));

        poseEstablished = true;
        goalSelectionInitialized = false;

        autoAimMode = true;
        turret.realignToSelectedGoal();

        status = upper
                ? "Upper corner reset applied"
                : "Lower corner reset applied";
    }

    // ---------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------

    private void driveRobot() {

        // Prevent driving while holding a reset combination.
        if (gamepad1.options
                && (gamepad1.dpad_down || gamepad1.dpad_up)) {

            stopDrive();
            return;
        }

        // Original drive signs retained.
        double forward =
                gamepad1.left_stick_y
                        * RobotConstants.DRIVE_SPEED;

        double strafe =
                -gamepad1.left_stick_x
                        * RobotConstants.DRIVE_SPEED;

        double turn =
                -gamepad1.right_stick_x
                        * RobotConstants.DRIVE_SPEED;

        double denominator = Math.max(
                Math.abs(forward)
                        + Math.abs(strafe)
                        + Math.abs(turn),
                1.0
        );

        frontLeft.setPower(
                (forward + strafe + turn) / denominator
        );

        backLeft.setPower(
                (forward - strafe + turn) / denominator
        );

        frontRight.setPower(
                (forward - strafe - turn) / denominator
        );

        backRight.setPower(
                (forward + strafe - turn) / denominator
        );
    }

    private void stopDrive() {

        if (frontLeft != null) {
            frontLeft.setPower(0.0);
        }

        if (frontRight != null) {
            frontRight.setPower(0.0);
        }

        if (backLeft != null) {
            backLeft.setPower(0.0);
        }

        if (backRight != null) {
            backRight.setPower(0.0);
        }
    }

    // ---------------------------------------------------------------------
    // Validation and utilities
    // ---------------------------------------------------------------------

    private boolean validPose(Pose pose) {

        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private boolean poseReady(Pose pose) {

        return poseEstablished && validPose(pose);
    }

    private boolean validFieldPoint(double x, double y) {

        return Double.isFinite(FIELD_SIZE_IN)
                && FIELD_SIZE_IN > 0.0
                && Double.isFinite(x)
                && Double.isFinite(y)
                && x >= 0.0
                && y >= 0.0
                && x <= FIELD_SIZE_IN
                && y <= FIELD_SIZE_IN;
    }

    private Pose copyPose(Pose pose) {

        return new Pose(
                pose.x(),
                pose.y(),
                pose.heading()
        );
    }

    private double clip(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void displayTelemetry(
            Pose pose,
            boolean initializing
    ) {

        telemetry.addLine("=== ALLIANCE ===");

        telemetry.addData(
                "Active Alliance",
                activeAlliance
        );

        telemetry.addData(
                "Stored Alliance",
                PoseStorage.lastAlliance
        );

        if (initializing) {

            telemetry.addData(
                    "INIT Selection",
                    ALLIANCE_MODE
            );

            telemetry.addLine("D-pad LEFT: Red");
            telemetry.addLine("D-pad RIGHT: Blue");
            telemetry.addLine("D-pad UP: Stored alliance");
            telemetry.addLine("Alliance locks at START.");
        }

        telemetry.addLine("=== LOCALIZATION ===");

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
                    Math.toDegrees(pose.heading())
            );
        }

        telemetry.addData(
                "Status",
                status
        );

        telemetry.addLine("=== TARGET ===");

        telemetry.addData(
                "Opening",
                goalSelectionInitialized
                        ? (upperGoalSelected ? "UPPER Y" : "LOWER Y")
                        : "NOT SELECTED"
        );

        telemetry.addData(
                "Target Valid",
                targetValid
        );

        telemetry.addData(
                "Goal Coordinates",
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
                "Switch Band",
                "%.2f to %.2f in",
                GOAL_SWITCH_Y_IN - GOAL_HYSTERESIS_IN,
                GOAL_SWITCH_Y_IN + GOAL_HYSTERESIS_IN
        );

        telemetry.addData(
                "Auto Aim",
                autoAimMode
        );

        telemetry.addData(
                "Turret Servo",
                turret.getServoPosition()
        );

        telemetry.addData(
                "Relative Target Angle",
                Turret.lastRelativeAngleDeg
        );

        telemetry.addData(
                "Turret Wrap Hold",
                turret.isHoldingBehind()
        );

        telemetry.addLine("=== SHOOTER ===");

        telemetry.addData(
                "Shooter Enabled",
                shooterEnabled
        );

        telemetry.addData(
                "Shot Mode",
                outtake.isUsingDistanceRegression()
                        ? "DISTANCE"
                        : "FIXED / FALLBACK"
        );

        telemetry.addData(
                "Target TPS",
                outtake.getTargetTPS()
        );

        telemetry.addData(
                "Current TPS",
                outtake.getCurrentTPS()
        );

        telemetry.addData(
                "Power",
                outtake.getCommandedPower()
        );

        telemetry.addData(
                "At Speed",
                outtake.isAtSpeed(
                        RobotConstants.FEED_SPEED_TOLERANCE_TPS
                )
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
                "Release Feed Trigger Required",
                feedReleaseRequired
        );

        telemetry.addData(
                "Feed Timer",
                RobotConstants.timer
        );

        telemetry.addLine("=== CONTROLS ===");

        telemetry.addLine("Cross: shooter on | Triangle: off");
        telemetry.addLine("Left bumper: auto aim | Square: centre");
        telemetry.addLine("Right bumper: collect");
        telemetry.addLine("Right trigger: feed | Left trigger: reverse");
        telemetry.addLine("Options + DOWN: lower corner reset");
        telemetry.addLine("Options + UP: upper corner reset");

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // Stop
    // ---------------------------------------------------------------------

    @Override
    public void stop() {

        shooterEnabled = false;

        if (outtake != null) {
            outtake.stopOuttake();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (blockServo != null) {

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );
        }

        stopDrive();
    }
}