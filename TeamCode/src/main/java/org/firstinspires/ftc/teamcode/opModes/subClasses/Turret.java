package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    private Servo turretServo;

    public static String TURRET_SERVO_NAME = "turretServo";


    // -------------------------------------------------------------------------
    // Servo calibration
    //
    // Total mechanical travel is 180 degrees.
    //
    // Servo 0.0 = full robot-right
    // Servo 0.5 = centre, pointing out of the robot's back
    // Servo 1.0 = full robot-left
    // -------------------------------------------------------------------------

    public static double SERVO_CENTER = 0.5;

    public static double RIGHT_SERVO_LIMIT = 0.0;
    public static double LEFT_SERVO_LIMIT = 1.0;


    // -------------------------------------------------------------------------
    // Aim tuning
    //
    // AIM_OFFSET is measured in servo-position units.
    // -------------------------------------------------------------------------

    public static double AIM_OFFSET = 0.0;
    public static double AIM_GAIN = 1.0;


    // -------------------------------------------------------------------------
    // Turret pivot offset from Pedro's localization point
    //
    // +X = robot forward
    // +Y = robot left
    //
    // Units are inches.
    // -------------------------------------------------------------------------

    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;


    // -------------------------------------------------------------------------
    // Mechanical zero
    //
    // Servo centre points directly out of the robot's back.
    // -------------------------------------------------------------------------

    public static double TURRET_ZERO_OFFSET_DEG = 180.0;


    // -------------------------------------------------------------------------
    // Mechanical angle limits
    //
    // -90 degrees = full robot-right = servo 0.0
    //   0 degrees = robot-back       = servo 0.5
    // +90 degrees = full robot-left  = servo 1.0
    // -------------------------------------------------------------------------

    private static final double MIN_TURRET_ANGLE_RAD =
            -Math.PI / 2.0;

    private static final double MAX_TURRET_ANGLE_RAD =
            Math.PI / 2.0;


    // -------------------------------------------------------------------------
    // Goal selection
    // -------------------------------------------------------------------------

    public enum Goal {
        RED,
        BLUE,
        CUSTOM
    }

    /*
     * These values must be set to the actual Pedro field coordinates before
     * calling init().
     */
    public static double RED_GOAL_X = 0.0;
    public static double RED_GOAL_Y = 0.0;

    public static double BLUE_GOAL_X = 0.0;
    public static double BLUE_GOAL_Y = 0.0;

    private Goal selectedGoal = Goal.RED;

    private Pose targetPose =
            new Pose(
                    RED_GOAL_X,
                    RED_GOAL_Y,
                    0.0
            );


    // -------------------------------------------------------------------------
    // Optional velocity compensation
    //
    // vx and vy must be field-frame velocities in inches per second.
    // -------------------------------------------------------------------------

    public static boolean velocityCompensationActive = false;

    public static double VELOCITY_LEAD_GAIN = 0.3;


    // -------------------------------------------------------------------------
    // Behind-target wrap protection
    //
    // This prevents the turret switching rapidly between full-left and
    // full-right when the target is near the unreachable robot-front region.
    // -------------------------------------------------------------------------

    public static double BEHIND_HOLD_ENTER_DEG = 25.0;
    public static double BEHIND_HOLD_EXIT_DEG = 35.0;

    private boolean holdingBehind = false;
    private int holdSide = 1;


    // -------------------------------------------------------------------------
    // Minimum aiming distance
    // -------------------------------------------------------------------------

    public static double MIN_AIM_DISTANCE_IN = 1.0;


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
    public static double lastRelativeAngleDeg = 0.0;

    public static double lastVelocityLeadX = 0.0;
    public static double lastVelocityLeadY = 0.0;

    public static double lastFinalAngleDeg = 0.0;
    public static double lastServoPosition = 0.5;

    public static boolean lastHoldingBehind = false;
    public static int lastHoldSide = 1;

    public static String lastSelectedGoal = "RED";

    /*
     * true when a direct servo-position command controls the turret.
     *
     * false when aimTurret() is controlling the turret from field coordinates.
     */
    public static boolean lastFixedPositionMode = false;


    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        turretServo = hardwareMap.get(
                Servo.class,
                TURRET_SERVO_NAME
        );

        /*
         * Servo direction is not reversed.
         *
         * Direction is handled explicitly by the angle-to-position mapping.
         */
        turretServo.setDirection(
                Servo.Direction.FORWARD
        );

        holdingBehind = false;
        holdSide = 1;

        selectAllianceGoal();
        setCentre();
    }


    // -------------------------------------------------------------------------
    // Goal selection
    // -------------------------------------------------------------------------

    /**
     * Selects the goal belonging to the alliance stored in PoseStorage.
     *
     * This does not cycle between goals.
     */
    public void selectAllianceGoal() {

        if (PoseStorage.isRed()) {

            selectGoal(Goal.RED);

        } else {

            selectGoal(Goal.BLUE);
        }
    }


    /**
     * Selects one specific alliance goal.
     */
    public void selectGoal(Goal goal) {

        if (goal == null || goal == Goal.CUSTOM) {
            return;
        }

        selectedGoal = goal;

        if (selectedGoal == Goal.RED) {

            targetPose = new Pose(
                    RED_GOAL_X,
                    RED_GOAL_Y,
                    0.0
            );

        } else {

            targetPose = new Pose(
                    BLUE_GOAL_X,
                    BLUE_GOAL_Y,
                    0.0
            );
        }

        resetAimState();

        lastSelectedGoal =
                selectedGoal.name();
    }


    public void selectRedGoal() {
        selectGoal(Goal.RED);
    }


    public void selectBlueGoal() {
        selectGoal(Goal.BLUE);
    }


    public Goal getSelectedGoal() {
        return selectedGoal;
    }


    public Pose getTargetPose() {
        return targetPose;
    }


    /**
     * Allows an OpMode to provide a deliberate custom target.
     */
    public void setTargetPose(Pose pose) {

        if (pose == null) {
            return;
        }

        targetPose = pose;
        selectedGoal = Goal.CUSTOM;

        resetAimState();

        lastSelectedGoal =
                selectedGoal.name();
    }


    /**
     * Re-selects the alliance goal stored in PoseStorage.
     *
     * This never advances to another goal.
     */
    public void realignToAllianceGoal() {
        selectAllianceGoal();
    }


    /**
     * Clears the hold state without changing the selected target.
     */
    public void realignToSelectedGoal() {
        resetAimState();
    }


    private void resetAimState() {

        holdingBehind = false;
        holdSide = 1;

        lastHoldingBehind = false;
        lastHoldSide = holdSide;
    }


    // -------------------------------------------------------------------------
    // Field-coordinate aiming
    // -------------------------------------------------------------------------

    /**
     * Aims the turret at the selected field target.
     *
     * vx and vy must be field-frame velocities in inches per second.
     */
    public void aimTurret(
            Pose robotPose,
            double vx,
            double vy
    ) {

        if (robotPose == null
                || turretServo == null
                || targetPose == null) {

            return;
        }

        /*
         * Field-coordinate aiming now controls the turret.
         */
        lastFixedPositionMode = false;

        double headingRad =
                robotPose.heading();

        double headingCos =
                Math.cos(headingRad);

        double headingSin =
                Math.sin(headingRad);


        // ---------------------------------------------------------------------
        // Calculate turret pivot field position
        // ---------------------------------------------------------------------

        double pivotX =
                robotPose.x()
                        + TURRET_OFFSET_X_IN * headingCos
                        - TURRET_OFFSET_Y_IN * headingSin;

        double pivotY =
                robotPose.y()
                        + TURRET_OFFSET_X_IN * headingSin
                        + TURRET_OFFSET_Y_IN * headingCos;


        // ---------------------------------------------------------------------
        // Vector from turret pivot to selected target
        // ---------------------------------------------------------------------

        double dx =
                targetPose.x() - pivotX;

        double dy =
                targetPose.y() - pivotY;


        // ---------------------------------------------------------------------
        // Optional velocity compensation
        // ---------------------------------------------------------------------

        double leadX =
                velocityCompensationActive
                        ? -vx * VELOCITY_LEAD_GAIN
                        : 0.0;

        double leadY =
                velocityCompensationActive
                        ? -vy * VELOCITY_LEAD_GAIN
                        : 0.0;

        double aimDx =
                dx + leadX;

        double aimDy =
                dy + leadY;

        double physicalDistance =
                Math.hypot(dx, dy);

        double compensatedDistance =
                Math.hypot(aimDx, aimDy);


        updateDiagnostics(
                robotPose,
                pivotX,
                pivotY,
                physicalDistance,
                leadX,
                leadY
        );


        /*
         * Do not calculate an angle when the pivot and target are effectively
         * in the same location.
         */
        if (compensatedDistance < MIN_AIM_DISTANCE_IN) {
            return;
        }


        // ---------------------------------------------------------------------
        // Field angle from turret pivot to target
        // ---------------------------------------------------------------------

        double fieldTargetAngleRad =
                Math.atan2(
                        aimDy,
                        aimDx
                );


        // ---------------------------------------------------------------------
        // Convert field angle into turret-relative angle
        //
        // Turret centre points backwards, so the 180-degree mechanical zero
        // offset is subtracted along with the robot heading.
        // ---------------------------------------------------------------------

        double rawRelativeAngleRad =
                normaliseRadians(
                        fieldTargetAngleRad
                                - headingRad
                                - Math.toRadians(
                                TURRET_ZERO_OFFSET_DEG
                        )
                );

        lastFieldTargetAngleDeg =
                Math.toDegrees(
                        fieldTargetAngleRad
                );

        lastRelativeAngleDeg =
                Math.toDegrees(
                        rawRelativeAngleRad
                );


        // ---------------------------------------------------------------------
        // Protect against left/right jumping near robot-front
        // ---------------------------------------------------------------------

        double commandedAngleRad =
                applyBehindHold(
                        rawRelativeAngleRad
                );

        if (!holdingBehind) {

            commandedAngleRad *=
                    AIM_GAIN;
        }


        // ---------------------------------------------------------------------
        // Clamp to physical travel
        // ---------------------------------------------------------------------

        commandedAngleRad = clamp(
                commandedAngleRad,
                MIN_TURRET_ANGLE_RAD,
                MAX_TURRET_ANGLE_RAD
        );


        // ---------------------------------------------------------------------
        // Convert angle to servo position
        // ---------------------------------------------------------------------

        double servoPosition =
                angleToServoPosition(
                        commandedAngleRad
                );

        servoPosition +=
                AIM_OFFSET;

        servoPosition =
                clampToServoLimits(
                        servoPosition
                );

        turretServo.setPosition(
                servoPosition
        );


        // ---------------------------------------------------------------------
        // Diagnostics
        // ---------------------------------------------------------------------

        lastFinalAngleDeg =
                Math.toDegrees(
                        commandedAngleRad
                );

        lastServoPosition =
                servoPosition;

        lastHoldingBehind =
                holdingBehind;

        lastHoldSide =
                holdSide;
    }


    /**
     * Convenience overload when velocity compensation is not required.
     */
    public void aimTurret(Pose robotPose) {

        aimTurret(
                robotPose,
                0.0,
                0.0
        );
    }


    // -------------------------------------------------------------------------
    // Fixed autonomous position control
    // -------------------------------------------------------------------------

    /**
     * Commands an exact turret servo position.
     *
     * Calibration:
     *
     * 0.0 = full robot-right
     * 0.5 = centre, pointing robot-back
     * 1.0 = full robot-left
     *
     * This method is intended for autonomous routines that use pre-tuned
     * shooting positions rather than field-coordinate aiming.
     */
    public void setFixedServoPosition(double position) {

        if (turretServo == null) {
            return;
        }

        double clippedPosition =
                clampToServoLimits(
                        position
                );

        turretServo.setPosition(
                clippedPosition
        );

        lastFixedPositionMode = true;

        /*
         * Convert the servo command back into its equivalent mechanical angle.
         *
         * Servo 0.0 = -90 degrees
         * Servo 0.5 =   0 degrees
         * Servo 1.0 = +90 degrees
         */
        lastFinalAngleDeg =
                (clippedPosition - SERVO_CENTER)
                        * 180.0;

        lastServoPosition =
                clippedPosition;

        /*
         * A fixed command must not inherit the automatic aiming hold state.
         */
        holdingBehind = false;

        if (lastFinalAngleDeg > 0.0) {

            holdSide = 1;

        } else if (lastFinalAngleDeg < 0.0) {

            holdSide = -1;
        }

        lastHoldingBehind = false;
        lastHoldSide = holdSide;
    }


    /**
     * Centres the turret at servo position 0.5.
     *
     * Physically, this points directly out of the robot's back.
     */
    public void setCentre() {

        setFixedServoPosition(
                SERVO_CENTER
        );

        resetAimState();

        lastFinalAngleDeg = 0.0;
        lastServoPosition = SERVO_CENTER;
        lastFixedPositionMode = true;
    }


    /**
     * Compatibility method for older OpModes.
     *
     * The calibrated centre physically points out of the robot's back.
     */
    public void setForward() {
        setCentre();
    }


    /**
     * Moves the turret to full robot-right.
     */
    public void setFullRight() {

        setFixedServoPosition(
                RIGHT_SERVO_LIMIT
        );

        holdingBehind = false;
        holdSide = -1;

        lastFinalAngleDeg = -90.0;
        lastHoldingBehind = false;
        lastHoldSide = holdSide;
        lastFixedPositionMode = true;
    }


    /**
     * Moves the turret to full robot-left.
     */
    public void setFullLeft() {

        setFixedServoPosition(
                LEFT_SERVO_LIMIT
        );

        holdingBehind = false;
        holdSide = 1;

        lastFinalAngleDeg = 90.0;
        lastHoldingBehind = false;
        lastHoldSide = holdSide;
        lastFixedPositionMode = true;
    }


    public boolean isFixedPositionMode() {
        return lastFixedPositionMode;
    }


    // -------------------------------------------------------------------------
    // Behind-region hysteresis
    // -------------------------------------------------------------------------

    private double applyBehindHold(
            double relativeAngleRad
    ) {

        double absoluteAngleRad =
                Math.abs(
                        relativeAngleRad
                );

        double enterHoldThresholdRad =
                Math.PI
                        - Math.toRadians(
                        BEHIND_HOLD_ENTER_DEG
                );

        double exitHoldThresholdRad =
                Math.PI
                        - Math.toRadians(
                        BEHIND_HOLD_EXIT_DEG
                );


        /*
         * Enter the hold region as the target approaches robot-front.
         */
        if (!holdingBehind
                && absoluteAngleRad >= enterHoldThresholdRad) {

            if (relativeAngleRad > 0.0) {

                holdSide = 1;

            } else if (relativeAngleRad < 0.0) {

                holdSide = -1;
            }

            holdingBehind = true;
        }


        /*
         * Exit only when the target has moved far enough from the wrap region.
         */
        if (holdingBehind
                && absoluteAngleRad <= exitHoldThresholdRad) {

            holdingBehind = false;
        }


        if (holdingBehind) {

            return holdSide > 0
                    ? MAX_TURRET_ANGLE_RAD
                    : MIN_TURRET_ANGLE_RAD;
        }


        /*
         * Remember the last meaningful requested side.
         */
        if (absoluteAngleRad > Math.toRadians(2.0)) {

            holdSide =
                    relativeAngleRad >= 0.0
                            ? 1
                            : -1;
        }

        return relativeAngleRad;
    }


    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    private void updateDiagnostics(
            Pose robotPose,
            double pivotX,
            double pivotY,
            double distance,
            double leadX,
            double leadY
    ) {

        lastRobotX =
                robotPose.x();

        lastRobotY =
                robotPose.y();

        lastRobotHeadingDeg =
                Math.toDegrees(
                        robotPose.heading()
                );

        lastPivotX =
                pivotX;

        lastPivotY =
                pivotY;

        lastTargetX =
                targetPose.x();

        lastTargetY =
                targetPose.y();

        lastDistanceToTargetIn =
                distance;

        lastVelocityLeadX =
                leadX;

        lastVelocityLeadY =
                leadY;

        lastHoldingBehind =
                holdingBehind;

        lastHoldSide =
                holdSide;

        lastSelectedGoal =
                selectedGoal.name();
    }


    public double getDistanceToTarget() {
        return lastDistanceToTargetIn;
    }


    public double getServoPosition() {

        if (turretServo == null) {
            return lastServoPosition;
        }

        return turretServo.getPosition();
    }


    public boolean isHoldingBehind() {
        return holdingBehind;
    }


    public static double getCalibratedCentrePosition() {
        return SERVO_CENTER;
    }


    // -------------------------------------------------------------------------
    // Angle-to-servo conversion
    // -------------------------------------------------------------------------

    /**
     * Converts a mechanical turret angle into a servo position.
     *
     * -90 degrees = servo 0.0 = full robot-right
     *   0 degrees = servo 0.5 = centre, pointing robot-back
     * +90 degrees = servo 1.0 = full robot-left
     */
    private double angleToServoPosition(
            double angleRad
    ) {

        double angleDeg =
                Math.toDegrees(
                        angleRad
                );

        double servoPosition =
                SERVO_CENTER
                        + angleDeg / 180.0;

        return clampToServoLimits(
                servoPosition
        );
    }


    private double clampToServoLimits(
            double value
    ) {

        double minimumServoPosition =
                Math.min(
                        RIGHT_SERVO_LIMIT,
                        LEFT_SERVO_LIMIT
                );

        double maximumServoPosition =
                Math.max(
                        RIGHT_SERVO_LIMIT,
                        LEFT_SERVO_LIMIT
                );

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
     * Normalises an angle to the range -PI through +PI.
     */
    private double normaliseRadians(
            double angleRad
    ) {

        while (angleRad > Math.PI) {

            angleRad -=
                    2.0 * Math.PI;
        }

        while (angleRad < -Math.PI) {

            angleRad +=
                    2.0 * Math.PI;
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
                Math.min(
                        maximum,
                        value
                )
        );
    }
}