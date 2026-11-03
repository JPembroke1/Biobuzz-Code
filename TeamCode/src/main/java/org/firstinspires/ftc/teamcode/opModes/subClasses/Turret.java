package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.math.Pose;

@Configurable
public class Turret {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    private Servo turretServo;

    public static String TURRET_SERVO_NAME = "turretServo";

    // -------------------------------------------------------------------------
    // Mechanical servo calibration
    //
    // The turret physically points backwards relative to the robot's front
    // when the servo is at SERVO_CENTER.
    //
    // LEFT_SERVO_LIMIT and RIGHT_SERVO_LIMIT are the measured servo positions
    // at the turret's two physical 90-degree limits.
    //
    // Servo reversal is applied once through Servo.setDirection().
    // -------------------------------------------------------------------------

    public static double SERVO_CENTER = 0.3;

    public static double LEFT_SERVO_LIMIT = 0.0;
    public static double RIGHT_SERVO_LIMIT = 0.6;

    public static boolean SERVO_REVERSED = true;

    // -------------------------------------------------------------------------
    // Aim tuning
    //
    // AIM_OFFSET is measured in servo-position units, not degrees.
    // AIM_GAIN scales the calculated turret angle.
    // -------------------------------------------------------------------------

    public static double AIM_OFFSET = 0.0;

    // Retained for compatibility with other code.
    public static double AIM_OFFSET_BLUE = 0.0;
    public static double AIM_OFFSET_RED = 0.0;

    public static double AIM_GAIN = 1.0;

    // -------------------------------------------------------------------------
    // Turret pivot position
    //
    // Pedro's Pose normally represents the robot's localization point.
    // These values describe the turret pivot relative to that point.
    //
    // +X = robot forward
    // +Y = robot left
    //
    // Units are inches.
    // -------------------------------------------------------------------------

    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    // -------------------------------------------------------------------------
    // Turret mechanical zero
    //
    // The turret points backwards when the servo is at SERVO_CENTER.
    //
    // 0 degrees   = turret centre points out the robot's front
    // 180 degrees = turret centre points out the robot's back
    // -------------------------------------------------------------------------

    public static double TURRET_ZERO_OFFSET_DEG = 180.0;

    // -------------------------------------------------------------------------
    // Turret angle limits
    //
    // The turret can rotate 90 degrees to either side of its backwards-facing
    // centre position.
    // -------------------------------------------------------------------------

    private static final double MIN_TURRET_ANGLE_RAD = -Math.PI / 2.0;
    private static final double MAX_TURRET_ANGLE_RAD = Math.PI / 2.0;

    // -------------------------------------------------------------------------
    // Behind-target wrap protection
    //
    // Relative angles close to +180 or -180 are on the opposite side of the
    // turret's reachable arc. Small localization changes can make the angle
    // wrap between +180 and -180.
    //
    // Hysteresis prevents the turret from switching between its two limits.
    // -------------------------------------------------------------------------

    public static double BEHIND_HOLD_ENTER_DEG = 25.0;
    public static double BEHIND_HOLD_EXIT_DEG = 35.0;

    private boolean holdingBehind = false;
    private int holdSide = 1;

    // -------------------------------------------------------------------------
    // Minimum aiming distance
    //
    // atan2() does not provide a useful aiming direction when the turret pivot
    // and target are effectively at the same location.
    // -------------------------------------------------------------------------

    public static double MIN_AIM_DISTANCE_IN = 1.0;

    // -------------------------------------------------------------------------
    // Target
    //
    // This must be the target's real Pedro field coordinate.
    // It is not the robot reset position.
    // -------------------------------------------------------------------------

    public static Pose targetPose = new Pose(0, 0, 0);

    // -------------------------------------------------------------------------
    // Velocity compensation
    //
    // vx and vy must be field-frame velocities in inches per second.
    // VELOCITY_LEAD_GAIN represents approximate projectile flight time.
    // -------------------------------------------------------------------------

    public static boolean velocityCompensationActive = false;
    public static double VELOCITY_LEAD_GAIN = 0.3;

    // -------------------------------------------------------------------------
    // Limelight compatibility fields
    //
    // These are retained so existing code that references them still compiles.
    // This class does not currently apply Limelight corrections.
    // -------------------------------------------------------------------------

    public static boolean USE_LIMELIGHT_ASSIST = false;

    public static boolean UPDATE_LIMELIGHT_INSIDE_TURRET = true;
    public static boolean REQUIRE_LIMELIGHT_TARGET = true;
    public static boolean DISABLE_LIMELIGHT_WHEN_LOCKED = false;

    public static double LIMELIGHT_CORRECTION_GAIN = 1.0;
    public static double MAX_LIMELIGHT_CORRECTION_RAD =
            Math.toRadians(5.0);

    public static double VISION_SMOOTHING_ALPHA = 0.35;
    public static double MAX_VISION_STEP_RAD =
            Math.toRadians(1.5);

    public static boolean limelightAssistActive = false;
    public static boolean limelightTargetVisible = false;
    public static boolean limelightLocked = false;

    public static double lastLimelightTx = 0.0;
    public static double lastRawLimelightCorrectionDeg = 0.0;
    public static double lastAppliedLimelightCorrectionDeg = 0.0;

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    public static double lastRobotX = 0.0;
    public static double lastRobotY = 0.0;
    public static double lastRobotHeadingDeg = 0.0;

    public static double lastPivotX = 0.0;
    public static double lastPivotY = 0.0;

    public static double lastTargetX = 0.0;
    public static double lastTargetY = 0.0;
    public static double lastDistanceToTargetIn = 0.0;

    public static double lastFieldTargetAngleDeg = 0.0;

    // Raw relative angle before gain and limit clamping.
    public static double lastRelativeAngleDeg = 0.0;

    public static double lastVelocityLeadX = 0.0;
    public static double lastVelocityLeadY = 0.0;

    // Final commanded mechanical angle after gain and clamping.
    public static double lastFinalAngleDeg = 0.0;

    public static double lastServoPosition = 0.0;

    public static boolean lastHoldingBehind = false;
    public static int lastHoldSide = 1;

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(
                Servo.class,
                TURRET_SERVO_NAME
        );

        turretServo.setDirection(
                SERVO_REVERSED
                        ? Servo.Direction.REVERSE
                        : Servo.Direction.FORWARD
        );

        holdingBehind = false;
        holdSide = 1;

        setForward();
    }

    // -------------------------------------------------------------------------
    // Target control
    // -------------------------------------------------------------------------

    public void setTargetPose(Pose pose) {
        if (pose != null) {
            targetPose = pose;
        }
    }

    public Pose getTargetPose() {
        return targetPose;
    }

    // -------------------------------------------------------------------------
    // Aim turret
    //
    // 1. Calculate the turret pivot's field position.
    // 2. Calculate the field-frame vector from pivot to target.
    // 3. Apply optional field-frame velocity compensation.
    // 4. Convert the field target angle into a turret-relative angle.
    // 5. Handle the unreachable wrap region using hysteresis.
    // 6. Apply aim gain and clamp to the physical turret range.
    // 7. Convert the resulting angle into a servo position.
    // -------------------------------------------------------------------------

    public void aimTurret(Pose robotPose, double vx, double vy) {
        if (robotPose == null || turretServo == null || targetPose == null) {
            return;
        }

        double headingRad = robotPose.heading();

        double headingCos = Math.cos(headingRad);
        double headingSin = Math.sin(headingRad);

        // Rotate the robot-relative turret offset into field coordinates.
        double pivotX =
                robotPose.x()
                        + TURRET_OFFSET_X_IN * headingCos
                        - TURRET_OFFSET_Y_IN * headingSin;

        double pivotY =
                robotPose.y()
                        + TURRET_OFFSET_X_IN * headingSin
                        + TURRET_OFFSET_Y_IN * headingCos;

        // Vector from the turret pivot to the target.
        double dx = targetPose.x() - pivotX;
        double dy = targetPose.y() - pivotY;

        // The projectile inherits robot velocity, so subtract the expected
        // robot displacement during projectile flight.
        double leadX = velocityCompensationActive
                ? -vx * VELOCITY_LEAD_GAIN
                : 0.0;

        double leadY = velocityCompensationActive
                ? -vy * VELOCITY_LEAD_GAIN
                : 0.0;

        double aimDx = dx + leadX;
        double aimDy = dy + leadY;

        double physicalDistance = Math.hypot(dx, dy);
        double compensatedDistance = Math.hypot(aimDx, aimDy);

        updateBasicDiagnostics(
                robotPose,
                pivotX,
                pivotY,
                physicalDistance,
                leadX,
                leadY
        );

        // Do not calculate a new angle if there is no meaningful target vector.
        // The turret remains at its previous commanded position.
        if (compensatedDistance < MIN_AIM_DISTANCE_IN) {
            return;
        }

        // Field angle from the turret pivot to the target.
        double fieldTargetAngleRad = Math.atan2(aimDy, aimDx);

        // Convert the field angle into an angle relative to the turret's
        // backwards-facing mechanical centre.
        double rawRelativeAngleRad = normaliseRadians(
                fieldTargetAngleRad
                        - headingRad
                        - Math.toRadians(TURRET_ZERO_OFFSET_DEG)
        );

        lastFieldTargetAngleDeg =
                Math.toDegrees(fieldTargetAngleRad);

        lastRelativeAngleDeg =
                Math.toDegrees(rawRelativeAngleRad);

        double commandedAngleRad =
                applyBehindHold(rawRelativeAngleRad);

        if (!holdingBehind) {
            commandedAngleRad *= AIM_GAIN;
        }

        commandedAngleRad = clamp(
                commandedAngleRad,
                MIN_TURRET_ANGLE_RAD,
                MAX_TURRET_ANGLE_RAD
        );

        double servoPosition =
                angleToServoPosition(commandedAngleRad);

        servoPosition += AIM_OFFSET;

        servoPosition =
                clampToServoLimits(servoPosition);

        turretServo.setPosition(servoPosition);

        lastFinalAngleDeg =
                Math.toDegrees(commandedAngleRad);

        lastServoPosition = servoPosition;
        lastHoldingBehind = holdingBehind;
        lastHoldSide = holdSide;
    }

    /**
     * Convenience overload for aiming while velocity compensation is not used.
     */
    public void aimTurret(Pose robotPose) {
        aimTurret(robotPose, 0.0, 0.0);
    }

    // -------------------------------------------------------------------------
    // Behind-region hysteresis
    // -------------------------------------------------------------------------

    private double applyBehindHold(double relativeAngleRad) {
        double absoluteAngleRad =
                Math.abs(relativeAngleRad);

        double enterHoldThresholdRad =
                Math.PI - Math.toRadians(BEHIND_HOLD_ENTER_DEG);

        double exitHoldThresholdRad =
                Math.PI - Math.toRadians(BEHIND_HOLD_EXIT_DEG);

        // Enter the hold region when the target approaches the direction
        // opposite the turret's reachable arc.
        if (!holdingBehind
                && absoluteAngleRad >= enterHoldThresholdRad) {

            // Select the side indicated by the angle before locking.
            //
            // At exactly +/-180 degrees the side is fundamentally ambiguous,
            // so the previous holdSide is retained.
            if (relativeAngleRad > 0.0) {
                holdSide = 1;
            } else if (relativeAngleRad < 0.0) {
                holdSide = -1;
            }

            holdingBehind = true;
        }

        // Leave the hold region only after moving sufficiently far away.
        // This different threshold creates hysteresis.
        if (holdingBehind
                && absoluteAngleRad <= exitHoldThresholdRad) {

            holdingBehind = false;
        }

        if (holdingBehind) {
            return holdSide > 0
                    ? MAX_TURRET_ANGLE_RAD
                    : MIN_TURRET_ANGLE_RAD;
        }

        // Remember the most recently requested side.
        if (absoluteAngleRad > Math.toRadians(2.0)) {
            holdSide = relativeAngleRad >= 0.0 ? 1 : -1;
        }

        return relativeAngleRad;
    }

    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    private void updateBasicDiagnostics(
            Pose robotPose,
            double pivotX,
            double pivotY,
            double distance,
            double leadX,
            double leadY
    ) {
        lastRobotX = robotPose.x();
        lastRobotY = robotPose.y();
        lastRobotHeadingDeg =
                Math.toDegrees(robotPose.heading());

        lastPivotX = pivotX;
        lastPivotY = pivotY;

        lastTargetX = targetPose.x();
        lastTargetY = targetPose.y();

        lastDistanceToTargetIn = distance;

        lastVelocityLeadX = leadX;
        lastVelocityLeadY = leadY;

        lastHoldingBehind = holdingBehind;
        lastHoldSide = holdSide;
    }

    /**
     * Returns the distance from the turret pivot to the target in inches,
     * based on the most recent aimTurret() call.
     */
    public double getDistanceToTarget() {
        return lastDistanceToTargetIn;
    }

    // -------------------------------------------------------------------------
    // Centre position
    //
    // In this robot configuration, "forward" means the turret's calibrated
    // centre position. Physically, that centre points backwards relative to
    // the robot chassis.
    // -------------------------------------------------------------------------

    public void setForward() {
        setCentre();
    }

    public void setCentre() {
        if (turretServo == null) {
            return;
        }

        double centrePosition =
                clampToServoLimits(getCalibratedCentrePosition());

        turretServo.setPosition(centrePosition);

        holdingBehind = false;

        lastFinalAngleDeg = 0.0;
        lastServoPosition = centrePosition;
        lastHoldingBehind = false;
    }

    // -------------------------------------------------------------------------
    // Calibration helpers
    // -------------------------------------------------------------------------

    public static double getCalibratedCentrePosition() {
        return SERVO_CENTER;
    }

    /**
     * Converts a mechanical turret angle into a servo position.
     *
     * Negative angle:
     * SERVO_CENTER toward LEFT_SERVO_LIMIT.
     *
     * Positive angle:
     * SERVO_CENTER toward RIGHT_SERVO_LIMIT.
     */
    private double angleToServoPosition(double angleRad) {
        if (angleRad < 0.0) {
            double proportion =
                    angleRad / MIN_TURRET_ANGLE_RAD;

            return SERVO_CENTER
                    + (LEFT_SERVO_LIMIT - SERVO_CENTER)
                    * proportion;
        }

        double proportion =
                angleRad / MAX_TURRET_ANGLE_RAD;

        return SERVO_CENTER
                + (RIGHT_SERVO_LIMIT - SERVO_CENTER)
                * proportion;
    }

    private double clampToServoLimits(double value) {
        double minimumServoPosition =
                Math.min(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);

        double maximumServoPosition =
                Math.max(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);

        return clamp(
                value,
                minimumServoPosition,
                maximumServoPosition
        );
    }

    // -------------------------------------------------------------------------
    // Angle utilities
    // -------------------------------------------------------------------------

    /**
     * Normalizes an angle to the range -PI through +PI.
     */
    private double normaliseRadians(double angleRad) {
        while (angleRad > Math.PI) {
            angleRad -= 2.0 * Math.PI;
        }

        while (angleRad < -Math.PI) {
            angleRad += 2.0 * Math.PI;
        }

        return angleRad;
    }

    // -------------------------------------------------------------------------
    // General utilities
    // -------------------------------------------------------------------------

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(
                minimum,
                Math.min(maximum, value)
        );
    }
}