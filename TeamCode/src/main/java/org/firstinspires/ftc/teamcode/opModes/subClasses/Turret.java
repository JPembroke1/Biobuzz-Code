package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret {

    // ---------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------

    public static String TURRET_SERVO_NAME = "turretServo";

    private Servo turretServo;

    // ---------------------------------------------------------------------
    // Servo calibration
    //
    // Original calibration:
    // 0.0 = robot-right
    // 0.5 = robot-back
    // 1.0 = robot-left
    //
    // Verify these physical directions before changing the mapping.
    // ---------------------------------------------------------------------

    public static double SERVO_CENTER = 0.5;

    public static double RIGHT_SERVO_LIMIT = 0.0;
    public static double LEFT_SERVO_LIMIT = 1.0;

    /**
     * false preserves the original angle-to-servo mapping.
     *
     * Set true only if testing confirms automatic aiming moves
     * in the opposite direction.
     */
    public static boolean REVERSE_AIM_MAPPING = false;

    // ---------------------------------------------------------------------
    // Aim tuning
    // ---------------------------------------------------------------------

    public static double AIM_OFFSET = 0.0;
    public static double AIM_GAIN = 1.0;

    // Servo centre points out of the robot's back.
    public static double TURRET_ZERO_OFFSET_DEG = 180.0;

    // ---------------------------------------------------------------------
    // Turret pivot offset from localization reference
    //
    // Robot frame:
    // +X = forward.
    // +Y = left.
    // Units: inches.
    // ---------------------------------------------------------------------

    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    private static final double MIN_TURRET_ANGLE_RAD =
            -Math.PI / 2.0;

    private static final double MAX_TURRET_ANGLE_RAD =
            Math.PI / 2.0;

    // ---------------------------------------------------------------------
    // Legacy alliance targets
    //
    // Retained for compatibility with existing autonomous code.
    // CombinedTeleOp uses setTargetPose() instead.
    // ---------------------------------------------------------------------

    public enum Goal {
        RED,
        BLUE,
        CUSTOM
    }

    public static double RED_GOAL_X = 0.0;
    public static double RED_GOAL_Y = 0.0;

    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 0.0;

    private Goal selectedGoal = Goal.CUSTOM;
    private Pose targetPose = null;

    // ---------------------------------------------------------------------
    // Optional velocity lead
    //
    // vx/vy must be field-frame inches per second.
    // VELOCITY_LEAD_GAIN acts as a lead time in seconds.
    // ---------------------------------------------------------------------

    public static boolean velocityCompensationActive = false;
    public static double VELOCITY_LEAD_GAIN = 0.3;

    // ---------------------------------------------------------------------
    // Wrap protection near the unreachable robot-front region
    // ---------------------------------------------------------------------

    public static double BEHIND_HOLD_ENTER_DEG = 25.0;
    public static double BEHIND_HOLD_EXIT_DEG = 35.0;

    private boolean holdingBehind = false;
    private int holdSide = 1;

    public static double MIN_AIM_DISTANCE_IN = 1.0;

    // ---------------------------------------------------------------------
    // Diagnostics
    // ---------------------------------------------------------------------

    public static double lastRobotX = 0.0;
    public static double lastRobotY = 0.0;
    public static double lastRobotHeadingDeg = 0.0;

    public static double lastPivotX = 0.0;
    public static double lastPivotY = 0.0;

    public static double lastTargetX = Double.NaN;
    public static double lastTargetY = Double.NaN;

    public static double lastDistanceToTargetIn = Double.NaN;

    public static double lastFieldTargetAngleDeg = Double.NaN;
    public static double lastRelativeAngleDeg = Double.NaN;

    public static double lastVelocityLeadX = 0.0;
    public static double lastVelocityLeadY = 0.0;

    public static double lastFinalAngleDeg = 0.0;
    public static double lastServoPosition = 0.5;

    public static boolean lastHoldingBehind = false;
    public static int lastHoldSide = 1;

    public static String lastSelectedGoal = "CUSTOM";
    public static boolean lastFixedPositionMode = true;

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        turretServo = hardwareMap.get(
                Servo.class,
                TURRET_SERVO_NAME
        );

        turretServo.setDirection(Servo.Direction.FORWARD);

        targetPose = null;
        selectedGoal = Goal.CUSTOM;

        resetAimState();

        // Compatibility with existing autonomous routines.
        // Unknown alliance does not silently select blue.
        selectAllianceGoal();

        setCentre();
    }

    // ---------------------------------------------------------------------
    // Target selection
    // ---------------------------------------------------------------------

    public void selectAllianceGoal() {

        if (PoseStorage.isRed()) {
            selectRedGoal();

        } else if (PoseStorage.isBlue()) {
            selectBlueGoal();
        }
    }

    public void selectGoal(Goal goal) {

        if (goal == null || goal == Goal.CUSTOM) {
            return;
        }

        double x = goal == Goal.RED
                ? RED_GOAL_X
                : BLUE_GOAL_X;

        double y = goal == Goal.RED
                ? RED_GOAL_Y
                : BLUE_GOAL_Y;

        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return;
        }

        applyTarget(goal, x, y);
    }

    public void selectRedGoal() {
        selectGoal(Goal.RED);
    }

    public void selectBlueGoal() {
        selectGoal(Goal.BLUE);
    }

    public void setTargetPose(Pose pose) {

        if (pose == null
                || !Double.isFinite(pose.x())
                || !Double.isFinite(pose.y())) {
            return;
        }

        applyTarget(Goal.CUSTOM, pose.x(), pose.y());
    }

    private void applyTarget(Goal goal, double x, double y) {

        boolean changed =
                selectedGoal != goal
                        || targetPose == null
                        || Double.compare(targetPose.x(), x) != 0
                        || Double.compare(targetPose.y(), y) != 0;

        if (changed) {
            selectedGoal = goal;
            targetPose = new Pose(x, y, 0.0);
            resetAimState();
        }

        lastSelectedGoal = selectedGoal.name();
        lastTargetX = x;
        lastTargetY = y;
    }

    public Goal getSelectedGoal() {
        return selectedGoal;
    }

    public Pose getTargetPose() {

        if (targetPose == null) {
            return null;
        }

        return new Pose(
                targetPose.x(),
                targetPose.y(),
                targetPose.heading()
        );
    }

    public void realignToAllianceGoal() {
        selectAllianceGoal();
        resetAimState();
    }

    public void realignToSelectedGoal() {
        resetAimState();
    }

    private void resetAimState() {
        holdingBehind = false;
        holdSide = 1;

        lastHoldingBehind = false;
        lastHoldSide = holdSide;

        // Prevent stale aiming diagnostics being treated as current.
        lastRelativeAngleDeg = Double.NaN;
        lastFieldTargetAngleDeg = Double.NaN;
        lastDistanceToTargetIn = Double.NaN;
    }

    // ---------------------------------------------------------------------
    // Field-coordinate aiming
    // ---------------------------------------------------------------------

    public void aimTurret(Pose robotPose) {
        aimTurret(robotPose, 0.0, 0.0);
    }

    public void aimTurret(
            Pose robotPose,
            double vx,
            double vy
    ) {

        if (turretServo == null) {
            return;
        }

        if (!validPose(robotPose)
                || targetPose == null
                || !Double.isFinite(TURRET_OFFSET_X_IN)
                || !Double.isFinite(TURRET_OFFSET_Y_IN)
                || !Double.isFinite(TURRET_ZERO_OFFSET_DEG)
                || !Double.isFinite(AIM_GAIN)
                || !Double.isFinite(AIM_OFFSET)
                || !Double.isFinite(MIN_AIM_DISTANCE_IN)) {

            setCentre();
            return;
        }

        double heading = robotPose.heading();

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double pivotX =
                robotPose.x()
                        + TURRET_OFFSET_X_IN * cos
                        - TURRET_OFFSET_Y_IN * sin;

        double pivotY =
                robotPose.y()
                        + TURRET_OFFSET_X_IN * sin
                        + TURRET_OFFSET_Y_IN * cos;

        double dx = targetPose.x() - pivotX;
        double dy = targetPose.y() - pivotY;

        double leadX = 0.0;
        double leadY = 0.0;

        if (velocityCompensationActive
                && Double.isFinite(vx)
                && Double.isFinite(vy)
                && Double.isFinite(VELOCITY_LEAD_GAIN)) {

            leadX = -vx * VELOCITY_LEAD_GAIN;
            leadY = -vy * VELOCITY_LEAD_GAIN;
        }

        double aimDx = dx + leadX;
        double aimDy = dy + leadY;

        double physicalDistance = Math.hypot(dx, dy);
        double compensatedDistance = Math.hypot(aimDx, aimDy);

        if (!Double.isFinite(physicalDistance)
                || !Double.isFinite(compensatedDistance)
                || compensatedDistance <= Math.max(
                0.0, MIN_AIM_DISTANCE_IN
        )) {

            setCentre();
            return;
        }

        lastFixedPositionMode = false;

        lastRobotX = robotPose.x();
        lastRobotY = robotPose.y();
        lastRobotHeadingDeg = Math.toDegrees(heading);

        lastPivotX = pivotX;
        lastPivotY = pivotY;

        lastTargetX = targetPose.x();
        lastTargetY = targetPose.y();

        lastDistanceToTargetIn = physicalDistance;

        lastVelocityLeadX = leadX;
        lastVelocityLeadY = leadY;

        double fieldAngle = Math.atan2(aimDy, aimDx);

        double relativeAngle = normalizeRadians(
                fieldAngle
                        - heading
                        - Math.toRadians(TURRET_ZERO_OFFSET_DEG)
        );

        lastFieldTargetAngleDeg = Math.toDegrees(fieldAngle);
        lastRelativeAngleDeg = Math.toDegrees(relativeAngle);

        double commandedAngle = applyBehindHold(relativeAngle);

        if (!holdingBehind) {
            commandedAngle *= AIM_GAIN;
        }

        commandedAngle = clamp(
                commandedAngle,
                MIN_TURRET_ANGLE_RAD,
                MAX_TURRET_ANGLE_RAD
        );

        double servoPosition = clampToServoLimits(
                angleToServoPosition(commandedAngle) + AIM_OFFSET
        );

        turretServo.setPosition(servoPosition);

        lastFinalAngleDeg = Math.toDegrees(commandedAngle);
        lastServoPosition = servoPosition;

        lastHoldingBehind = holdingBehind;
        lastHoldSide = holdSide;
        lastSelectedGoal = selectedGoal.name();
    }

    // ---------------------------------------------------------------------
    // Fixed servo commands
    // ---------------------------------------------------------------------

    public void setFixedServoPosition(double position) {

        if (turretServo == null || !Double.isFinite(position)) {
            return;
        }

        double clipped = clampToServoLimits(position);

        turretServo.setPosition(clipped);

        lastFixedPositionMode = true;
        lastServoPosition = clipped;

        double mappingSign = REVERSE_AIM_MAPPING ? -1.0 : 1.0;

        lastFinalAngleDeg =
                mappingSign * (clipped - centrePosition()) * 180.0;

        holdingBehind = false;

        if (lastFinalAngleDeg > 0.0) {
            holdSide = 1;

        } else if (lastFinalAngleDeg < 0.0) {
            holdSide = -1;
        }

        lastHoldingBehind = false;
        lastHoldSide = holdSide;

        lastRelativeAngleDeg = Double.NaN;
        lastFieldTargetAngleDeg = Double.NaN;
    }

    public void setCentre() {
        setFixedServoPosition(centrePosition());
        resetAimState();
    }

    public void setForward() {
        setCentre();
    }

    public void setFullRight() {
        setFixedServoPosition(RIGHT_SERVO_LIMIT);
    }

    public void setFullLeft() {
        setFixedServoPosition(LEFT_SERVO_LIMIT);
    }

    // ---------------------------------------------------------------------
    // Wrap-region hysteresis
    // ---------------------------------------------------------------------

    private double applyBehindHold(double relativeAngle) {

        double absoluteAngle = Math.abs(relativeAngle);

        double enterDegrees = Double.isFinite(BEHIND_HOLD_ENTER_DEG)
                ? clamp(BEHIND_HOLD_ENTER_DEG, 0.0, 89.0)
                : 25.0;

        double exitDegrees = Double.isFinite(BEHIND_HOLD_EXIT_DEG)
                ? clamp(BEHIND_HOLD_EXIT_DEG, enterDegrees, 89.0)
                : Math.max(35.0, enterDegrees);

        double enterThreshold =
                Math.PI - Math.toRadians(enterDegrees);

        double exitThreshold =
                Math.PI - Math.toRadians(exitDegrees);

        if (!holdingBehind && absoluteAngle >= enterThreshold) {

            if (relativeAngle > 0.0) {
                holdSide = 1;

            } else if (relativeAngle < 0.0) {
                holdSide = -1;
            }

            holdingBehind = true;
        }

        if (holdingBehind && absoluteAngle <= exitThreshold) {
            holdingBehind = false;
        }

        if (holdingBehind) {
            return holdSide > 0
                    ? MAX_TURRET_ANGLE_RAD
                    : MIN_TURRET_ANGLE_RAD;
        }

        if (absoluteAngle > Math.toRadians(2.0)) {
            holdSide = relativeAngle >= 0.0 ? 1 : -1;
        }

        return relativeAngle;
    }

    // ---------------------------------------------------------------------
    // Servo mapping
    // ---------------------------------------------------------------------

    private double angleToServoPosition(double angleRad) {

        double mappingSign = REVERSE_AIM_MAPPING ? -1.0 : 1.0;

        return centrePosition()
                + mappingSign * Math.toDegrees(angleRad) / 180.0;
    }

    private double clampToServoLimits(double position) {

        double right = Double.isFinite(RIGHT_SERVO_LIMIT)
                ? clamp(RIGHT_SERVO_LIMIT, 0.0, 1.0)
                : 0.0;

        double left = Double.isFinite(LEFT_SERVO_LIMIT)
                ? clamp(LEFT_SERVO_LIMIT, 0.0, 1.0)
                : 1.0;

        return clamp(
                position,
                Math.min(right, left),
                Math.max(right, left)
        );
    }

    private static double centrePosition() {
        return Double.isFinite(SERVO_CENTER)
                ? clamp(SERVO_CENTER, 0.0, 1.0)
                : 0.5;
    }

    // ---------------------------------------------------------------------
    // Status and compatibility
    // ---------------------------------------------------------------------

    public double getDistanceToTarget() {
        return lastDistanceToTargetIn;
    }

    public double getServoPosition() {
        return turretServo == null
                ? lastServoPosition
                : turretServo.getPosition();
    }

    public boolean isHoldingBehind() {
        return holdingBehind;
    }

    public boolean isFixedPositionMode() {
        return lastFixedPositionMode;
    }

    public static double getCalibratedCentrePosition() {
        return centrePosition();
    }

    // ---------------------------------------------------------------------
    // Utilities
    // ---------------------------------------------------------------------

    private boolean validPose(Pose pose) {
        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static double normalizeRadians(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}