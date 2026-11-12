package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

@Configurable
public class Turret {

    // ---------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------

    public static String TURRET_SERVO_NAME = "turretServo";

    private Servo turretServo;

    // ---------------------------------------------------------------------
    // Confirmed physical calibration
    //
    // With robot heading = 180 degrees:
    //
    // Servo 0.0 -> shooter field heading  90 degrees
    // Servo 0.5 -> shooter field heading   0 degrees
    // Servo 1.0 -> shooter field heading 270 degrees
    //
    // Therefore:
    //
    // - servo centre is 0.5;
    // - centre points 180 degrees behind the robot;
    // - increasing turret angle DECREASES servo position.
    // ---------------------------------------------------------------------

    public static double SERVO_CENTER = 0.5;

    public static double RIGHT_SERVO_LIMIT = 0.0;
    public static double LEFT_SERVO_LIMIT = 1.0;

    /*
     * Confirmed true from physical endpoint testing.
     */
    public static boolean REVERSE_AIM_MAPPING = true;

    // ---------------------------------------------------------------------
    // Automatic aiming limit
    //
    // Fixed autonomous servo commands retain the full 0.0 to 1.0 range.
    // ---------------------------------------------------------------------

    public static double AUTO_AIM_MAX_ANGLE_DEG = 75.0;

    // ---------------------------------------------------------------------
    // Base aim tuning
    //
    // AIM_OFFSET is a servo-position offset, not degrees.
    // ---------------------------------------------------------------------

    public static double AIM_OFFSET = 0.0;
    public static double AIM_GAIN = 1.0;

    /*
     * At servo centre, the turret points behind the robot.
     */
    public static double TURRET_ZERO_OFFSET_DEG = 180.0;

    // ---------------------------------------------------------------------
    // Turret pivot offset from Pedro/Pinpoint reference point
    //
    // Robot frame:
    //
    // +X = robot forward
    // +Y = robot left
    //
    // Units: inches.
    // ---------------------------------------------------------------------

    public static double TURRET_OFFSET_X_IN = 0.0;
    public static double TURRET_OFFSET_Y_IN = 0.0;

    public static double MIN_AIM_DISTANCE_IN = 1.0;

    // ---------------------------------------------------------------------
    // Optional movement compensation
    //
    // velocityX and velocityY must be field-frame velocities in inches/s.
    // ---------------------------------------------------------------------

    public static boolean velocityCompensationActive = false;
    public static double VELOCITY_LEAD_GAIN = 0.3;

    // ---------------------------------------------------------------------
    // Turret-mounted Arducam fusion
    //
    // Pinpoint/Pedro supplies the coarse absolute turret angle.
    //
    // Because the camera rotates with the turret:
    //
    // camera turret estimate =
    // current commanded turret angle + visual bearing correction
    // ---------------------------------------------------------------------

    public static boolean CAMERA_FUSION_ENABLED = true;

    /*
     * If positive bearing moves farther away from zero during testing,
     * change this to -1.0.
     */
    public static double CAMERA_BEARING_SIGN = 1.0;

    public static double CAMERA_CORRECTION_GAIN = 0.60;

    /*
     * Camera optical-axis to shooter alignment correction in degrees.
     */
    public static double CAMERA_AIM_OFFSET_DEG = 0.0;

    public static double CAMERA_DEADBAND_DEG = 0.75;
    public static double CAMERA_MAX_CORRECTION_DEG = 8.0;

    public static double CAMERA_MIN_RANGE_IN = 6.0;
    public static double CAMERA_MAX_RANGE_IN = 120.0;

    public static double CAMERA_MAX_ABS_BEARING_DEG = 45.0;

    public static double CAMERA_FILTER_ALPHA = 0.35;

    /*
     * 0.0 = Pinpoint only
     * 1.0 = camera estimate only
     */
    public static double CAMERA_FUSION_WEIGHT = 0.75;

    public static double CAMERA_SHOOT_TOLERANCE_DEG = 2.0;

    /*
     * Camera range-based confidence.
     */
    public static double CAMERA_FULL_WEIGHT_RANGE_IN = 30.0;
    public static double CAMERA_ZERO_WEIGHT_RANGE_IN = 100.0;

    private double filteredCameraBearingDeg = 0.0;
    private boolean cameraBearingPrimed = false;

    // ---------------------------------------------------------------------
    // Goal compatibility
    // ---------------------------------------------------------------------

    public enum Goal {
        RED,
        BLUE,
        CUSTOM
    }

    public static double RED_GOAL_X = Double.NaN;
    public static double RED_GOAL_Y = Double.NaN;

    public static double BLUE_GOAL_X = Double.NaN;
    public static double BLUE_GOAL_Y = Double.NaN;

    private Goal selectedGoal = Goal.CUSTOM;
    private Pose targetPose = null;

    // ---------------------------------------------------------------------
    // Aim source
    // ---------------------------------------------------------------------

    public enum AimSource {
        NONE,
        PINPOINT,
        FUSED,
        FIXED
    }

    private AimSource currentAimSource = AimSource.NONE;

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

    public static boolean lastTargetReachable = false;
    public static boolean lastHoldingBehind = false;

    public static int lastHoldSide = 1;

    public static String lastSelectedGoal = "CUSTOM";
    public static boolean lastFixedPositionMode = true;

    public static String lastAimSource = "NONE";

    public static boolean lastCameraTargetVisible = false;
    public static boolean lastCameraTargetValid = false;
    public static boolean lastCameraAligned = false;

    public static double lastCameraBearingDeg = Double.NaN;
    public static double lastFilteredCameraBearingDeg = Double.NaN;
    public static double lastCameraRangeIn = Double.NaN;
    public static double lastCameraCorrectionDeg = 0.0;
    public static double lastCameraWeight = 0.0;

    public static double lastPinpointAngleDeg = Double.NaN;
    public static double lastCameraEstimateAngleDeg = Double.NaN;
    public static double lastFusedAngleDeg = Double.NaN;

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        turretServo =
                hardwareMap.get(
                        Servo.class,
                        TURRET_SERVO_NAME
                );

        turretServo.setDirection(
                Servo.Direction.FORWARD
        );

        selectedGoal = Goal.CUSTOM;
        targetPose = null;

        clearAimDiagnostics();

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

        if (goal == null
                || goal == Goal.CUSTOM) {

            return;
        }

        double targetX;
        double targetY;

        if (goal == Goal.RED) {

            targetX = RED_GOAL_X;
            targetY = RED_GOAL_Y;

        } else {

            targetX = BLUE_GOAL_X;
            targetY = BLUE_GOAL_Y;
        }

        if (!Double.isFinite(targetX)
                || !Double.isFinite(targetY)) {

            return;
        }

        applyTarget(
                goal,
                targetX,
                targetY
        );
    }

    public void selectRedGoal() {

        selectGoal(
                Goal.RED
        );
    }

    public void selectBlueGoal() {

        selectGoal(
                Goal.BLUE
        );
    }

    /**
     * Supplies an explicit field-coordinate target.
     *
     * Pose heading is ignored.
     */
    public void setTargetPose(Pose pose) {

        if (pose == null
                || !Double.isFinite(pose.x())
                || !Double.isFinite(pose.y())) {

            return;
        }

        applyTarget(
                Goal.CUSTOM,
                pose.x(),
                pose.y()
        );
    }

    private void applyTarget(
            Goal goal,
            double targetX,
            double targetY
    ) {

        boolean targetChanged =
                selectedGoal != goal
                        || targetPose == null
                        || Double.compare(
                        targetPose.x(),
                        targetX
                ) != 0
                        || Double.compare(
                        targetPose.y(),
                        targetY
                ) != 0;

        if (targetChanged) {

            selectedGoal = goal;

            targetPose =
                    new Pose(
                            targetX,
                            targetY,
                            0.0
                    );

            clearAimDiagnostics();
        }

        lastSelectedGoal =
                selectedGoal.name();

        lastTargetX = targetX;
        lastTargetY = targetY;
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
        clearAimDiagnostics();
    }

    public void realignToSelectedGoal() {

        clearAimDiagnostics();
    }

    private void clearAimDiagnostics() {

        lastDistanceToTargetIn = Double.NaN;

        lastFieldTargetAngleDeg = Double.NaN;
        lastRelativeAngleDeg = Double.NaN;

        lastVelocityLeadX = 0.0;
        lastVelocityLeadY = 0.0;

        lastTargetReachable = false;
        lastHoldingBehind = false;
        lastHoldSide = 1;

        lastPinpointAngleDeg = Double.NaN;
        lastFusedAngleDeg = Double.NaN;

        currentAimSource = AimSource.NONE;
        lastAimSource = currentAimSource.name();

        resetCameraBearingFilter();
        clearCameraDiagnostics();
    }

    // ---------------------------------------------------------------------
    // Fused aiming
    // ---------------------------------------------------------------------

    public void aimTurretFused(
            Pose robotPose,
            ArduCam camera,
            ArduCam.Alliance alliance
    ) {

        aimTurretFused(
                robotPose,
                0.0,
                0.0,
                camera,
                alliance
        );
    }

    public void aimTurretFused(
            Pose robotPose,
            double velocityX,
            double velocityY,
            ArduCam camera,
            ArduCam.Alliance alliance
    ) {

        clearCameraDiagnostics();

        AimCalculation pinpointCalculation =
                calculatePinpointAim(
                        robotPose,
                        velocityX,
                        velocityY
                );

        if (pinpointCalculation == null) {

            resetCameraBearingFilter();
            setCentre();

            return;
        }

        lastPinpointAngleDeg =
                Math.toDegrees(
                        pinpointCalculation.requestedAngleRad
                );

        double finalRequestedAngleRad =
                pinpointCalculation.requestedAngleRad;

        boolean fusionApplied = false;

        if (CAMERA_FUSION_ENABLED
                && camera != null
                && alliance != null
                && camera.isEnabled()) {

            AprilTagClusterDetection detection =
                    camera.getActiveCellDetection(
                            alliance
                    );

            if (detection == null) {

                detection =
                        camera.getAnyHiveDetection(
                                alliance
                        );
            }

            if (isValidCameraDetection(detection)) {

                CameraCalculation cameraCalculation =
                        calculateCameraEstimate(
                                detection
                        );

                /*
                 * Weight zero means the camera is not contributing.
                 *
                 * Keep the source as PINPOINT so isAimReady() does not
                 * unnecessarily require camera alignment.
                 */
                if (cameraCalculation != null
                        && cameraCalculation.fusionWeight > 0.0) {

                    double differenceRad =
                            normalizeRadians(
                                    cameraCalculation
                                            .cameraEstimateAngleRad
                                            - pinpointCalculation
                                            .requestedAngleRad
                            );

                    finalRequestedAngleRad =
                            normalizeRadians(
                                    pinpointCalculation
                                            .requestedAngleRad
                                            + differenceRad
                                            * cameraCalculation
                                            .fusionWeight
                            );

                    fusionApplied = true;

                    lastCameraEstimateAngleDeg =
                            Math.toDegrees(
                                    cameraCalculation
                                            .cameraEstimateAngleRad
                            );

                    lastCameraWeight =
                            cameraCalculation.fusionWeight;

                    lastFusedAngleDeg =
                            Math.toDegrees(
                                    finalRequestedAngleRad
                            );
                }
            }
        }

        if (!fusionApplied) {

            resetCameraBearingFilter();

            lastCameraTargetValid = false;
            lastCameraAligned = false;
            lastCameraWeight = 0.0;

            lastCameraEstimateAngleDeg =
                    Double.NaN;

            lastFusedAngleDeg =
                    lastPinpointAngleDeg;
        }

        applyAutomaticAngle(
                finalRequestedAngleRad,
                fusionApplied
                        ? AimSource.FUSED
                        : AimSource.PINPOINT
        );

        updatePinpointDiagnostics(
                robotPose,
                pinpointCalculation
        );
    }

    // ---------------------------------------------------------------------
    // Pinpoint-only aiming
    // ---------------------------------------------------------------------

    public void aimTurret(Pose robotPose) {

        aimTurret(
                robotPose,
                0.0,
                0.0
        );
    }

    public void aimTurret(
            Pose robotPose,
            double velocityX,
            double velocityY
    ) {

        resetCameraBearingFilter();
        clearCameraDiagnostics();

        AimCalculation calculation =
                calculatePinpointAim(
                        robotPose,
                        velocityX,
                        velocityY
                );

        if (calculation == null) {

            setCentre();

            return;
        }

        lastPinpointAngleDeg =
                Math.toDegrees(
                        calculation.requestedAngleRad
                );

        lastFusedAngleDeg =
                lastPinpointAngleDeg;

        applyAutomaticAngle(
                calculation.requestedAngleRad,
                AimSource.PINPOINT
        );

        updatePinpointDiagnostics(
                robotPose,
                calculation
        );
    }

    // ---------------------------------------------------------------------
    // Pinpoint/Pedro calculation
    // ---------------------------------------------------------------------

    private AimCalculation calculatePinpointAim(
            Pose robotPose,
            double velocityX,
            double velocityY
    ) {

        if (!validPose(robotPose)
                || targetPose == null
                || !Double.isFinite(TURRET_OFFSET_X_IN)
                || !Double.isFinite(TURRET_OFFSET_Y_IN)
                || !Double.isFinite(TURRET_ZERO_OFFSET_DEG)
                || !Double.isFinite(MIN_AIM_DISTANCE_IN)) {

            return null;
        }

        double robotHeading =
                robotPose.heading();

        double headingCos =
                Math.cos(robotHeading);

        double headingSin =
                Math.sin(robotHeading);

        double pivotX =
                robotPose.x()
                        + TURRET_OFFSET_X_IN
                        * headingCos
                        - TURRET_OFFSET_Y_IN
                        * headingSin;

        double pivotY =
                robotPose.y()
                        + TURRET_OFFSET_X_IN
                        * headingSin
                        + TURRET_OFFSET_Y_IN
                        * headingCos;

        double targetDx =
                targetPose.x()
                        - pivotX;

        double targetDy =
                targetPose.y()
                        - pivotY;

        double physicalDistance =
                Math.hypot(
                        targetDx,
                        targetDy
                );

        double minimumDistance =
                Math.max(
                        0.0,
                        MIN_AIM_DISTANCE_IN
                );

        if (!Double.isFinite(physicalDistance)
                || physicalDistance <= minimumDistance) {

            return null;
        }

        double leadX = 0.0;
        double leadY = 0.0;

        if (velocityCompensationActive
                && Double.isFinite(velocityX)
                && Double.isFinite(velocityY)
                && Double.isFinite(VELOCITY_LEAD_GAIN)) {

            leadX =
                    -velocityX
                            * VELOCITY_LEAD_GAIN;

            leadY =
                    -velocityY
                            * VELOCITY_LEAD_GAIN;
        }

        double aimDx =
                targetDx
                        + leadX;

        double aimDy =
                targetDy
                        + leadY;

        double compensatedDistance =
                Math.hypot(
                        aimDx,
                        aimDy
                );

        if (!Double.isFinite(compensatedDistance)
                || compensatedDistance <= minimumDistance) {

            return null;
        }

        double fieldTargetAngle =
                Math.atan2(
                        aimDy,
                        aimDx
                );

        double requestedAngle =
                normalizeRadians(
                        fieldTargetAngle
                                - robotHeading
                                - Math.toRadians(
                                TURRET_ZERO_OFFSET_DEG
                        )
                );

        if (!Double.isFinite(requestedAngle)) {
            return null;
        }

        return new AimCalculation(
                requestedAngle,
                fieldTargetAngle,
                pivotX,
                pivotY,
                physicalDistance,
                leadX,
                leadY
        );
    }

    private void updatePinpointDiagnostics(
            Pose robotPose,
            AimCalculation calculation
    ) {

        lastRobotX =
                robotPose.x();

        lastRobotY =
                robotPose.y();

        lastRobotHeadingDeg =
                normalizeDegrees0To360(
                        Math.toDegrees(
                                robotPose.heading()
                        )
                );

        lastPivotX =
                calculation.pivotX;

        lastPivotY =
                calculation.pivotY;

        lastTargetX =
                targetPose.x();

        lastTargetY =
                targetPose.y();

        if (currentAimSource != AimSource.FUSED
                || !Double.isFinite(lastCameraRangeIn)) {

            lastDistanceToTargetIn =
                    calculation.physicalDistance;
        }

        lastFieldTargetAngleDeg =
                normalizeDegrees0To360(
                        Math.toDegrees(
                                calculation.fieldTargetAngleRad
                        )
                );

        lastVelocityLeadX =
                calculation.leadX;

        lastVelocityLeadY =
                calculation.leadY;

        lastSelectedGoal =
                selectedGoal.name();
    }

    // ---------------------------------------------------------------------
    // Camera calculation
    // ---------------------------------------------------------------------

    private CameraCalculation calculateCameraEstimate(
            AprilTagClusterDetection detection
    ) {

        if (!isValidCameraDetection(detection)) {
            return null;
        }

        double rawBearingDeg =
                detection.ftcPose.bearing;

        double rangeIn =
                detection.ftcPose.range;

        double filteredBearingDeg =
                filterCameraBearing(
                        rawBearingDeg
                );

        double errorDeg =
                CAMERA_BEARING_SIGN
                        * filteredBearingDeg
                        + CAMERA_AIM_OFFSET_DEG;

        if (!Double.isFinite(errorDeg)
                || !Double.isFinite(CAMERA_CORRECTION_GAIN)) {

            return null;
        }

        double shootTolerance =
                safeAbsolute(
                        CAMERA_SHOOT_TOLERANCE_DEG,
                        2.0
                );

        lastCameraAligned =
                Math.abs(errorDeg)
                        <= shootTolerance;

        double deadband =
                safeAbsolute(
                        CAMERA_DEADBAND_DEG,
                        0.75
                );

        if (Math.abs(errorDeg) <= deadband) {
            errorDeg = 0.0;
        }

        double correctionDeg =
                errorDeg
                        * CAMERA_CORRECTION_GAIN;

        if (!Double.isFinite(correctionDeg)) {
            return null;
        }

        double correctionLimit =
                safeAbsolute(
                        CAMERA_MAX_CORRECTION_DEG,
                        8.0
                );

        correctionDeg =
                clamp(
                        correctionDeg,
                        -correctionLimit,
                        correctionLimit
                );

        double currentServoPosition =
                getServoPosition();

        if (!Double.isFinite(currentServoPosition)
                || !Double.isFinite(AIM_OFFSET)) {

            return null;
        }

        /*
         * AIM_OFFSET was added after angle-to-servo conversion.
         *
         * Remove it before reconstructing the commanded turret angle.
         */
        double currentAngleRad =
                Math.toRadians(
                        servoPositionToAngleDeg(
                                currentServoPosition
                                        - AIM_OFFSET
                        )
                );

        double cameraEstimateAngleRad =
                normalizeRadians(
                        currentAngleRad
                                + Math.toRadians(
                                correctionDeg
                        )
                );

        double fusionWeight =
                calculateCameraWeight(
                        rangeIn
                );

        if (!Double.isFinite(cameraEstimateAngleRad)
                || !Double.isFinite(fusionWeight)) {

            return null;
        }

        lastCameraTargetVisible = true;
        lastCameraTargetValid = true;

        lastCameraBearingDeg =
                rawBearingDeg;

        lastFilteredCameraBearingDeg =
                filteredBearingDeg;

        lastCameraRangeIn =
                rangeIn;

        lastCameraCorrectionDeg =
                correctionDeg;

        lastDistanceToTargetIn =
                rangeIn;

        return new CameraCalculation(
                cameraEstimateAngleRad,
                fusionWeight
        );
    }

    private boolean isValidCameraDetection(
            AprilTagClusterDetection detection
    ) {

        if (detection == null) {
            return false;
        }

        lastCameraTargetVisible = true;

        if (detection.ftcPose == null) {
            return false;
        }

        double bearing =
                detection.ftcPose.bearing;

        double range =
                detection.ftcPose.range;

        lastCameraBearingDeg =
                bearing;

        lastCameraRangeIn =
                range;

        if (!Double.isFinite(bearing)
                || !Double.isFinite(range)) {

            return false;
        }

        double minimumRange =
                Double.isFinite(CAMERA_MIN_RANGE_IN)
                        ? Math.max(
                        0.0,
                        CAMERA_MIN_RANGE_IN
                )
                        : 0.0;

        double maximumRange =
                Double.isFinite(CAMERA_MAX_RANGE_IN)
                        ? Math.max(
                        minimumRange,
                        CAMERA_MAX_RANGE_IN
                )
                        : 120.0;

        double maximumBearing =
                safeAbsolute(
                        CAMERA_MAX_ABS_BEARING_DEG,
                        45.0
                );

        return range >= minimumRange
                && range <= maximumRange
                && Math.abs(bearing)
                <= maximumBearing;
    }

    private double filterCameraBearing(
            double bearingDeg
    ) {

        if (!cameraBearingPrimed) {

            filteredCameraBearingDeg =
                    bearingDeg;

            cameraBearingPrimed = true;

            return filteredCameraBearingDeg;
        }

        double alpha =
                Double.isFinite(CAMERA_FILTER_ALPHA)
                        ? clamp(
                        CAMERA_FILTER_ALPHA,
                        0.0,
                        1.0
                )
                        : 0.35;

        filteredCameraBearingDeg +=
                alpha
                        * (bearingDeg
                        - filteredCameraBearingDeg);

        return filteredCameraBearingDeg;
    }

    private double calculateCameraWeight(
            double rangeIn
    ) {

        double configuredWeight =
                Double.isFinite(CAMERA_FUSION_WEIGHT)
                        ? clamp(
                        CAMERA_FUSION_WEIGHT,
                        0.0,
                        1.0
                )
                        : 0.75;

        double fullWeightRange =
                Double.isFinite(
                        CAMERA_FULL_WEIGHT_RANGE_IN
                )
                        ? Math.max(
                        0.0,
                        CAMERA_FULL_WEIGHT_RANGE_IN
                )
                        : 30.0;

        double zeroWeightRange =
                Double.isFinite(
                        CAMERA_ZERO_WEIGHT_RANGE_IN
                )
                        ? Math.max(
                        fullWeightRange + 1.0,
                        CAMERA_ZERO_WEIGHT_RANGE_IN
                )
                        : 100.0;

        if (rangeIn <= fullWeightRange) {
            return configuredWeight;
        }

        if (rangeIn >= zeroWeightRange) {
            return 0.0;
        }

        double rangeConfidence =
                1.0
                        - (rangeIn - fullWeightRange)
                        / (zeroWeightRange
                        - fullWeightRange);

        return configuredWeight
                * clamp(
                rangeConfidence,
                0.0,
                1.0
        );
    }

    private void resetCameraBearingFilter() {

        filteredCameraBearingDeg = 0.0;
        cameraBearingPrimed = false;
    }

    private void clearCameraDiagnostics() {

        lastCameraTargetVisible = false;
        lastCameraTargetValid = false;
        lastCameraAligned = false;

        lastCameraBearingDeg = Double.NaN;
        lastFilteredCameraBearingDeg = Double.NaN;
        lastCameraRangeIn = Double.NaN;

        lastCameraCorrectionDeg = 0.0;
        lastCameraWeight = 0.0;

        lastCameraEstimateAngleDeg = Double.NaN;
    }

    // ---------------------------------------------------------------------
    // Automatic command application
    // ---------------------------------------------------------------------

    private void applyAutomaticAngle(
            double requestedAngleRad,
            AimSource aimSource
    ) {

        if (turretServo == null
                || !Double.isFinite(requestedAngleRad)
                || !Double.isFinite(AIM_GAIN)
                || !Double.isFinite(AIM_OFFSET)) {

            setCentre();

            return;
        }

        double gainedAngle =
                requestedAngleRad
                        * AIM_GAIN;

        if (!Double.isFinite(gainedAngle)) {

            setCentre();

            return;
        }

        double maximumAngle =
                getAutoAimMaxAngleRad();

        if (!Double.isFinite(maximumAngle)
                || maximumAngle <= 0.0) {

            setCentre();

            return;
        }

        lastFixedPositionMode = false;

        lastRelativeAngleDeg =
                Math.toDegrees(
                        requestedAngleRad
                );

        lastTargetReachable =
                Math.abs(gainedAngle)
                        <= maximumAngle;

        lastHoldingBehind =
                !lastTargetReachable;

        if (gainedAngle > 0.0) {

            lastHoldSide = 1;

        } else if (gainedAngle < 0.0) {

            lastHoldSide = -1;
        }

        double commandedAngle =
                clamp(
                        gainedAngle,
                        -maximumAngle,
                        maximumAngle
                );

        double servoPosition =
                angleToServoPosition(
                        commandedAngle
                )
                        + AIM_OFFSET;

        servoPosition =
                clampToServoLimits(
                        servoPosition
                );

        turretServo.setPosition(
                servoPosition
        );

        lastFinalAngleDeg =
                Math.toDegrees(
                        commandedAngle
                );

        lastServoPosition =
                servoPosition;

        currentAimSource =
                aimSource;

        lastAimSource =
                currentAimSource.name();
    }

    // ---------------------------------------------------------------------
    // Reachability and shooting readiness
    // ---------------------------------------------------------------------

    public boolean isTargetReachable() {

        return lastTargetReachable
                && !lastFixedPositionMode
                && Double.isFinite(
                lastRelativeAngleDeg
        )
                && Double.isFinite(
                lastFinalAngleDeg
        );
    }

    public boolean isCameraAligned() {

        return currentAimSource == AimSource.FUSED
                && lastCameraTargetValid
                && lastCameraAligned;
    }

    /**
     * Pinpoint-only aim is ready when reachable.
     *
     * Fused aim additionally requires the camera bearing to be inside the
     * shooting tolerance.
     */
    public boolean isAimReady() {

        if (!isTargetReachable()) {
            return false;
        }

        if (currentAimSource == AimSource.FUSED) {
            return isCameraAligned();
        }

        return currentAimSource == AimSource.PINPOINT;
    }

    public AimSource getAimSource() {

        return currentAimSource;
    }

    public boolean isUsingCameraFusion() {

        return currentAimSource == AimSource.FUSED;
    }

    public boolean isUsingPinpointAim() {

        return currentAimSource == AimSource.PINPOINT;
    }

    // ---------------------------------------------------------------------
    // Automatic limit
    // ---------------------------------------------------------------------

    private double getAutoAimMaxAngleDeg() {

        if (!Double.isFinite(
                AUTO_AIM_MAX_ANGLE_DEG
        )) {

            return 75.0;
        }

        return clamp(
                Math.abs(
                        AUTO_AIM_MAX_ANGLE_DEG
                ),
                0.0,
                90.0
        );
    }

    private double getAutoAimMaxAngleRad() {

        return Math.toRadians(
                getAutoAimMaxAngleDeg()
        );
    }

    // ---------------------------------------------------------------------
    // Fixed servo commands
    //
    // These bypass the automatic +/-75-degree limit.
    // ---------------------------------------------------------------------

    public void setFixedServoPosition(
            double position
    ) {

        if (turretServo == null
                || !Double.isFinite(position)) {

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
        lastServoPosition = clippedPosition;

        lastFinalAngleDeg =
                servoPositionToAngleDeg(
                        clippedPosition
                );

        lastTargetReachable = false;
        lastHoldingBehind = false;

        if (lastFinalAngleDeg > 0.0) {

            lastHoldSide = 1;

        } else if (lastFinalAngleDeg < 0.0) {

            lastHoldSide = -1;
        }

        lastRelativeAngleDeg = Double.NaN;
        lastFieldTargetAngleDeg = Double.NaN;

        currentAimSource = AimSource.FIXED;
        lastAimSource = currentAimSource.name();

        resetCameraBearingFilter();
        clearCameraDiagnostics();
    }

    public void setCentre() {

        setFixedServoPosition(
                centrePosition()
        );

        lastFinalAngleDeg = 0.0;
        lastServoPosition = centrePosition();
        lastFixedPositionMode = true;
    }

    /**
     * Compatibility method.
     *
     * On this robot, servo centre points behind the robot.
     */
    public void setForward() {

        setCentre();
    }

    public void setFullRight() {

        setFixedServoPosition(
                RIGHT_SERVO_LIMIT
        );
    }

    public void setFullLeft() {

        setFixedServoPosition(
                LEFT_SERVO_LIMIT
        );
    }

    // ---------------------------------------------------------------------
    // Servo mapping
    // ---------------------------------------------------------------------

    private double angleToServoPosition(
            double angleRadians
    ) {

        double mappingSign =
                REVERSE_AIM_MAPPING
                        ? -1.0
                        : 1.0;

        return centrePosition()
                + mappingSign
                * Math.toDegrees(
                angleRadians
        )
                / 180.0;
    }

    private double servoPositionToAngleDeg(
            double servoPosition
    ) {

        double mappingSign =
                REVERSE_AIM_MAPPING
                        ? -1.0
                        : 1.0;

        return mappingSign
                * (servoPosition
                - centrePosition())
                * 180.0;
    }

    private double clampToServoLimits(
            double position
    ) {

        double rightLimit =
                Double.isFinite(
                        RIGHT_SERVO_LIMIT
                )
                        ? clamp(
                        RIGHT_SERVO_LIMIT,
                        0.0,
                        1.0
                )
                        : 0.0;

        double leftLimit =
                Double.isFinite(
                        LEFT_SERVO_LIMIT
                )
                        ? clamp(
                        LEFT_SERVO_LIMIT,
                        0.0,
                        1.0
                )
                        : 1.0;

        return clamp(
                position,
                Math.min(
                        rightLimit,
                        leftLimit
                ),
                Math.max(
                        rightLimit,
                        leftLimit
                )
        );
    }

    private static double centrePosition() {

        return Double.isFinite(
                SERVO_CENTER
        )
                ? clamp(
                SERVO_CENTER,
                0.0,
                1.0
        )
                : 0.5;
    }

    // ---------------------------------------------------------------------
    // Status
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

        return lastHoldingBehind;
    }

    public boolean isFixedPositionMode() {

        return lastFixedPositionMode;
    }

    public static double getCalibratedCentrePosition() {

        return centrePosition();
    }

    // ---------------------------------------------------------------------
    // Internal calculations
    // ---------------------------------------------------------------------

    private static class AimCalculation {

        final double requestedAngleRad;
        final double fieldTargetAngleRad;

        final double pivotX;
        final double pivotY;

        final double physicalDistance;

        final double leadX;
        final double leadY;

        AimCalculation(
                double requestedAngleRad,
                double fieldTargetAngleRad,
                double pivotX,
                double pivotY,
                double physicalDistance,
                double leadX,
                double leadY
        ) {

            this.requestedAngleRad =
                    requestedAngleRad;

            this.fieldTargetAngleRad =
                    fieldTargetAngleRad;

            this.pivotX = pivotX;
            this.pivotY = pivotY;

            this.physicalDistance =
                    physicalDistance;

            this.leadX = leadX;
            this.leadY = leadY;
        }
    }

    private static class CameraCalculation {

        final double cameraEstimateAngleRad;
        final double fusionWeight;

        CameraCalculation(
                double cameraEstimateAngleRad,
                double fusionWeight
        ) {

            this.cameraEstimateAngleRad =
                    cameraEstimateAngleRad;

            this.fusionWeight =
                    fusionWeight;
        }
    }

    // ---------------------------------------------------------------------
    // Utilities
    // ---------------------------------------------------------------------

    private boolean validPose(Pose pose) {

        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(
                pose.heading()
        );
    }

    private static double normalizeRadians(
            double angle
    ) {

        return Math.atan2(
                Math.sin(angle),
                Math.cos(angle)
        );
    }

    private static double normalizeDegrees0To360(
            double angleDegrees
    ) {

        double normalized =
                angleDegrees % 360.0;

        if (normalized < 0.0) {
            normalized += 360.0;
        }

        return normalized;
    }

    private static double safeAbsolute(
            double value,
            double fallback
    ) {

        return Double.isFinite(value)
                ? Math.abs(value)
                : Math.abs(fallback);
    }

    private static double clamp(
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