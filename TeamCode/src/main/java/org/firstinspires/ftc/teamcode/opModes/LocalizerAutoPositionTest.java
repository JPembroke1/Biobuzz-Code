package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@TeleOp(
        name = "Localizer Auto Position Test",
        group = "Test"
)
public class LocalizerAutoPositionTest extends OpMode {

    // ---------------------------------------------------------------------
    // Field and starting pose
    //
    // Coordinate model used by this robot:
    //
    // Field size: 141.5 x 141.5 inches
    // Exact centre: 70.75, 70.75
    // Starting heading: 180 degrees
    //
    // Physically place the robot's LOCALIZATION REFERENCE POINT at the
    // centre before pressing START.
    // ---------------------------------------------------------------------

    public static double FIELD_SIZE_IN = 141.5;

    public static double FIELD_CENTER_IN =
            FIELD_SIZE_IN / 2.0;

    public static double START_X =
            FIELD_CENTER_IN;

    public static double START_Y =
            FIELD_CENTER_IN;

    public static double START_HEADING_DEG = 180.0;

    // ---------------------------------------------------------------------
    // Autonomous positions under test
    //
    // These match DelayedFlowerRedPath5Auto.
    //
    // Do not scale these coordinates when changing FIELD_SIZE_IN.
    // The purpose of this OpMode is to physically validate these values.
    // ---------------------------------------------------------------------

    public static double AUTO_START_X = 23.7064;
    public static double AUTO_START_Y = 130.1432;
    public static double AUTO_START_HEADING_DEG = 90.0;

    public static double POINT_6_X = 42.0;
    public static double POINT_6_Y = 104.1657;
    public static double POINT_6_HEADING_DEG = 90.0;

    public static double POINT_7_X = 42.0;
    public static double POINT_7_Y = 127.0;
    public static double POINT_7_HEADING_DEG = 75.0;

    public static double POINT_8_X = 5.0;
    public static double POINT_8_Y = 116.1719;
    public static double POINT_8_HEADING_DEG = 0.0;

    // ---------------------------------------------------------------------
    // HIVE targets currently used in CombinedTeleOp
    //
    // Included here so those coordinates can be physically checked as
    // well as the autonomous waypoints.
    // ---------------------------------------------------------------------

    public static double RED_LOWER_GOAL_X = 58.00;
    public static double RED_LOWER_GOAL_Y = 46.69;
    public static double RED_LOWER_GOAL_HEADING_DEG = 270.0;

    public static double RED_UPPER_GOAL_X = 58.00;
    public static double RED_UPPER_GOAL_Y = 94.81;
    public static double RED_UPPER_GOAL_HEADING_DEG = 90.0;

    public static double BLUE_LOWER_GOAL_X = 83.50;
    public static double BLUE_LOWER_GOAL_Y = 46.69;
    public static double BLUE_LOWER_GOAL_HEADING_DEG = 270.0;

    public static double BLUE_UPPER_GOAL_X = 83.50;
    public static double BLUE_UPPER_GOAL_Y = 94.81;
    public static double BLUE_UPPER_GOAL_HEADING_DEG = 90.0;

    // ---------------------------------------------------------------------
    // Test drive configuration
    //
    // Keep speed conservative for the first validation run.
    // ---------------------------------------------------------------------

    public static double DRIVE_SPEED = 0.30;

    public static double PATH_TIMEOUT_SECONDS = 8.0;

    public static double POSITION_TOLERANCE_IN = 1.0;

    public static double HEADING_TOLERANCE_DEG = 5.0;

    /*
     * Prevent a stale atParametricEnd result from immediately completing
     * a newly started test path.
     */
    public static double MIN_PATH_RUNTIME_SECONDS = 0.10;

    // ---------------------------------------------------------------------
    // Available test locations
    // ---------------------------------------------------------------------

    private enum TestLocation {

        AUTO_START(
                "Auto Start / Path 5 End"
        ),

        POINT_6(
                "Point 6"
        ),

        POINT_7(
                "Point 7 / Flower Pickup"
        ),

        POINT_8(
                "Point 8 / Final Park"
        ),

        RED_LOWER_HIVE(
                "Red Lower HIVE"
        ),

        RED_UPPER_HIVE(
                "Red Upper HIVE"
        ),

        BLUE_LOWER_HIVE(
                "Blue Lower HIVE"
        ),

        BLUE_UPPER_HIVE(
                "Blue Upper HIVE"
        ),

        FIELD_CENTRE(
                "Field Centre"
        );

        final String label;

        TestLocation(
                String label
        ) {

            this.label = label;
        }
    }

    // ---------------------------------------------------------------------
    // Pedro
    // ---------------------------------------------------------------------

    private final PoseFactory poseFactory =
            PoseFactory.degrees();

    private Follower follower;

    private List<LynxModule> hubs;

    // ---------------------------------------------------------------------
    // Test state
    // ---------------------------------------------------------------------

    private TestLocation selectedLocation =
            TestLocation.AUTO_START;

    private Pose activeTarget = null;

    private boolean pathActive = false;

    private boolean pathTimedOut = false;

    private boolean targetReached = false;

    private double pathStartTime = 0.0;

    private double positionError = Double.NaN;

    private double headingErrorDeg = Double.NaN;

    private String status =
            "Select a target";

    // ---------------------------------------------------------------------
    // Button edge detection
    // ---------------------------------------------------------------------

    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    private boolean previousCross = false;
    private boolean previousCircle = false;
    private boolean previousTriangle = false;

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    @Override
    public void init() {

        configureBulkCaching();

        clearBulkCache();

        follower =
                Constants.create(
                        hardwareMap
                );

        Pose centrePose =
                getFieldCentrePose();

        /*
         * We are deliberately telling the localizer exactly where the
         * physically positioned robot is.
         */
        follower.setPose(
                centrePose
        );

        follower.update();

        PoseStorage.setPose(
                centrePose
        );

        activeTarget =
                getSelectedTarget();

        pathActive = false;

        pathTimedOut = false;

        targetReached = false;

        status =
                "Pose set to field centre";

        displayTelemetry();
    }

    // ---------------------------------------------------------------------
    // INIT loop
    //
    // Robot must remain physically at the field centre during INIT.
    // ---------------------------------------------------------------------

    @Override
    public void init_loop() {

        clearBulkCache();

        handleTargetSelection();

        activeTarget =
                getSelectedTarget();

        displayTelemetry();
    }

    // ---------------------------------------------------------------------
    // START
    // ---------------------------------------------------------------------

    @Override
    public void start() {

        Pose centrePose =
                getFieldCentrePose();

        /*
         * Re-establish the known pose at START so any INIT movement from
         * encoder/localizer noise does not contaminate the test.
         */
        follower.setPose(
                centrePose
        );

        PoseStorage.setPose(
                centrePose
        );

        activeTarget =
                getSelectedTarget();

        pathActive = false;

        pathTimedOut = false;

        targetReached = false;

        positionError = Double.NaN;

        headingErrorDeg = Double.NaN;

        status =
                "Ready to drive";

        previousDpadLeft =
                gamepad1.dpad_left;

        previousDpadRight =
                gamepad1.dpad_right;

        previousCross =
                gamepad1.cross;

        previousCircle =
                gamepad1.circle;

        previousTriangle =
                gamepad1.triangle;
    }

    // ---------------------------------------------------------------------
    // Main loop
    // ---------------------------------------------------------------------

    @Override
    public void loop() {

        clearBulkCache();

        handleTargetSelection();

        handleDriveControls();

        follower.update();

        Pose currentPose =
                follower.pose();

        if (validPose(currentPose)) {

            PoseStorage.setPose(
                    currentPose
            );
        }

        if (pathActive) {

            updatePathState();
        }

        updateErrors(
                currentPose
        );

        displayTelemetry();
    }

    // ---------------------------------------------------------------------
    // Target selection
    //
    // G1 D-pad Left  = previous target
    // G1 D-pad Right = next target
    // ---------------------------------------------------------------------

    private void handleTargetSelection() {

        boolean leftPressed =
                gamepad1.dpad_left;

        boolean rightPressed =
                gamepad1.dpad_right;

        if (!pathActive) {

            if (leftPressed
                    && !previousDpadLeft) {

                selectPreviousLocation();
            }

            if (rightPressed
                    && !previousDpadRight) {

                selectNextLocation();
            }
        }

        previousDpadLeft =
                leftPressed;

        previousDpadRight =
                rightPressed;
    }

    private void selectNextLocation() {

        TestLocation[] locations =
                TestLocation.values();

        int nextIndex =
                selectedLocation.ordinal()
                        + 1;

        if (nextIndex >= locations.length) {
            nextIndex = 0;
        }

        selectedLocation =
                locations[nextIndex];

        activeTarget =
                getSelectedTarget();

        resetResultState();

        status =
                "Selected "
                        + selectedLocation.label;
    }

    private void selectPreviousLocation() {

        TestLocation[] locations =
                TestLocation.values();

        int previousIndex =
                selectedLocation.ordinal()
                        - 1;

        if (previousIndex < 0) {
            previousIndex =
                    locations.length
                            - 1;
        }

        selectedLocation =
                locations[previousIndex];

        activeTarget =
                getSelectedTarget();

        resetResultState();

        status =
                "Selected "
                        + selectedLocation.label;
    }

    // ---------------------------------------------------------------------
    // Driver controls
    //
    // G1 Cross    = drive to selected target
    // G1 Circle   = abort path
    // G1 Triangle = reset LOCALIZER to field-centre pose
    //
    // Important:
    //
    // Triangle must only be pressed when the robot has physically been
    // returned to the centre reference position.
    // ---------------------------------------------------------------------

    private void handleDriveControls() {

        boolean crossPressed =
                gamepad1.cross;

        boolean circlePressed =
                gamepad1.circle;

        boolean trianglePressed =
                gamepad1.triangle;

        if (crossPressed
                && !previousCross
                && !pathActive) {

            beginSelectedPath();
        }

        if (circlePressed
                && !previousCircle) {

            abortPath(
                    "Path aborted by driver"
            );
        }

        if (trianglePressed
                && !previousTriangle) {

            resetToFieldCentre();
        }

        previousCross =
                crossPressed;

        previousCircle =
                circlePressed;

        previousTriangle =
                trianglePressed;
    }

    // ---------------------------------------------------------------------
    // Begin selected test path
    // ---------------------------------------------------------------------

    private void beginSelectedPath() {

        Pose currentPose =
                follower.pose();

        activeTarget =
                getSelectedTarget();

        if (!validPose(currentPose)) {

            status =
                    "Cannot start: current pose invalid";

            return;
        }

        if (!validPose(activeTarget)) {

            status =
                    "Cannot start: target pose invalid";

            return;
        }

        Path path =
                line(
                        currentPose,
                        activeTarget
                )
                        .linear(
                                currentPose,
                                activeTarget
                        )
                        .with(
                                Constants.foresightConfig
                                        .maxPathSpeed
                                        .at(
                                                clamp(
                                                        DRIVE_SPEED,
                                                        0.05,
                                                        1.0
                                                )
                                        )
                        );

        pathStartTime =
                getRuntime();

        pathActive = true;

        pathTimedOut = false;

        targetReached = false;

        positionError = Double.NaN;

        headingErrorDeg = Double.NaN;

        follower.follow(
                path
        );

        status =
                "Driving to "
                        + selectedLocation.label;
    }

    // ---------------------------------------------------------------------
    // Path completion
    // ---------------------------------------------------------------------

    private void updatePathState() {

        double elapsed =
                getRuntime()
                        - pathStartTime;

        if (elapsed >= Math.max(
                0.0,
                PATH_TIMEOUT_SECONDS
        )) {

            pathTimedOut = true;

            finishPath(
                    "Path timed out"
            );

            return;
        }

        /*
         * Ignore stale completion state from the previous path.
         */
        if (elapsed < Math.max(
                0.0,
                MIN_PATH_RUNTIME_SECONDS
        )) {

            return;
        }

        Pose currentPose =
                follower.pose();

        if (!validPose(currentPose)
                || !validPose(activeTarget)) {

            return;
        }

        double currentPositionError =
                Math.hypot(
                        activeTarget.x()
                                - currentPose.x(),
                        activeTarget.y()
                                - currentPose.y()
                );

        double currentHeadingError =
                Math.abs(
                        angleDifferenceDegrees(
                                Math.toDegrees(
                                        activeTarget.heading()
                                ),
                                Math.toDegrees(
                                        currentPose.heading()
                                )
                        )
                );

        boolean withinTolerance =
                currentPositionError
                        <= Math.max(
                        0.0,
                        POSITION_TOLERANCE_IN
                )
                        && currentHeadingError
                        <= Math.max(
                        0.0,
                        HEADING_TOLERANCE_DEG
                );

        if (withinTolerance) {

            targetReached = true;

            finishPath(
                    "Target reached"
            );

            return;
        }

        if (follower.atParametricEnd()) {

            finishPath(
                    "Follower reached path end"
            );
        }
    }

    private void finishPath(
            String finishStatus
    ) {

        if (follower != null) {
            follower.stop();
        }

        pathActive = false;

        updateErrors(
                follower == null
                        ? null
                        : follower.pose()
        );

        status =
                finishStatus;
    }

    private void abortPath(
            String abortStatus
    ) {

        if (follower != null) {
            follower.stop();
        }

        pathActive = false;

        updateErrors(
                follower == null
                        ? null
                        : follower.pose()
        );

        status =
                abortStatus;
    }

    // ---------------------------------------------------------------------
    // Field-centre localizer reset
    // ---------------------------------------------------------------------

    private void resetToFieldCentre() {

        /*
         * This changes the software pose only.
         *
         * The driver is responsible for physically placing the robot's
         * localization reference point at field centre first.
         */
        abortPath(
                "Resetting localizer"
        );

        Pose centrePose =
                getFieldCentrePose();

        follower.setPose(
                centrePose
        );

        PoseStorage.setPose(
                centrePose
        );

        activeTarget =
                getSelectedTarget();

        resetResultState();

        status =
                "Localizer reset to (70.75, 70.75, 180 deg)";
    }

    private Pose getFieldCentrePose() {

        return poseFactory.of(
                START_X,
                START_Y,
                START_HEADING_DEG
        );
    }

    // ---------------------------------------------------------------------
    // Target generation
    // ---------------------------------------------------------------------

    private Pose getSelectedTarget() {

        switch (selectedLocation) {

            case AUTO_START:

                return poseFactory.of(
                        AUTO_START_X,
                        AUTO_START_Y,
                        AUTO_START_HEADING_DEG
                );

            case POINT_6:

                return poseFactory.of(
                        POINT_6_X,
                        POINT_6_Y,
                        POINT_6_HEADING_DEG
                );

            case POINT_7:

                return poseFactory.of(
                        POINT_7_X,
                        POINT_7_Y,
                        POINT_7_HEADING_DEG
                );

            case POINT_8:

                return poseFactory.of(
                        POINT_8_X,
                        POINT_8_Y,
                        POINT_8_HEADING_DEG
                );

            case RED_LOWER_HIVE:

                return poseFactory.of(
                        RED_LOWER_GOAL_X,
                        RED_LOWER_GOAL_Y,
                        RED_LOWER_GOAL_HEADING_DEG
                );

            case RED_UPPER_HIVE:

                return poseFactory.of(
                        RED_UPPER_GOAL_X,
                        RED_UPPER_GOAL_Y,
                        RED_UPPER_GOAL_HEADING_DEG
                );

            case BLUE_LOWER_HIVE:

                return poseFactory.of(
                        BLUE_LOWER_GOAL_X,
                        BLUE_LOWER_GOAL_Y,
                        BLUE_LOWER_GOAL_HEADING_DEG
                );

            case BLUE_UPPER_HIVE:

                return poseFactory.of(
                        BLUE_UPPER_GOAL_X,
                        BLUE_UPPER_GOAL_Y,
                        BLUE_UPPER_GOAL_HEADING_DEG
                );

            case FIELD_CENTRE:
            default:

                return getFieldCentrePose();
        }
    }

    // ---------------------------------------------------------------------
    // Position and heading errors
    // ---------------------------------------------------------------------

    private void updateErrors(
            Pose currentPose
    ) {

        if (!validPose(currentPose)
                || !validPose(activeTarget)) {

            positionError =
                    Double.NaN;

            headingErrorDeg =
                    Double.NaN;

            return;
        }

        positionError =
                Math.hypot(
                        activeTarget.x()
                                - currentPose.x(),
                        activeTarget.y()
                                - currentPose.y()
                );

        headingErrorDeg =
                Math.abs(
                        angleDifferenceDegrees(
                                Math.toDegrees(
                                        activeTarget.heading()
                                ),
                                Math.toDegrees(
                                        currentPose.heading()
                                )
                        )
                );
    }

    private void resetResultState() {

        pathTimedOut = false;

        targetReached = false;

        positionError =
                Double.NaN;

        headingErrorDeg =
                Double.NaN;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void displayTelemetry() {

        Pose currentPose =
                follower == null
                        ? null
                        : follower.pose();

        telemetry.addLine(
                "===== LOCALIZER AUTO POSITION TEST ====="
        );

        telemetry.addData(
                "Status",
                status
        );

        telemetry.addData(
                "Selected",
                selectedLocation.label
        );

        telemetry.addData(
                "Path Active",
                pathActive
        );

        telemetry.addData(
                "Target Reached",
                targetReached
        );

        telemetry.addData(
                "Path Timed Out",
                pathTimedOut
        );

        // -------------------------------------------------------------
        // Field reference
        // -------------------------------------------------------------

        telemetry.addLine(
                "===== FIELD REFERENCE ====="
        );

        telemetry.addData(
                "Field Size",
                "%.2f x %.2f in",
                FIELD_SIZE_IN,
                FIELD_SIZE_IN
        );

        telemetry.addData(
                "Field Centre",
                "(%.2f, %.2f)",
                FIELD_CENTER_IN,
                FIELD_CENTER_IN
        );

        telemetry.addData(
                "Declared Start",
                "(%.2f, %.2f, %.1f deg)",
                START_X,
                START_Y,
                START_HEADING_DEG
        );

        // -------------------------------------------------------------
        // Current pose
        // -------------------------------------------------------------

        telemetry.addLine(
                "===== CURRENT LOCALIZER POSE ====="
        );

        if (validPose(currentPose)) {

            telemetry.addData(
                    "Current",
                    "(%.2f, %.2f, %.1f deg)",
                    currentPose.x(),
                    currentPose.y(),
                    normalizeDegrees0To360(
                            Math.toDegrees(
                                    currentPose.heading()
                            )
                    )
            );

        } else {

            telemetry.addData(
                    "Current",
                    "INVALID"
            );
        }

        // -------------------------------------------------------------
        // Selected target
        // -------------------------------------------------------------

        telemetry.addLine(
                "===== SELECTED TARGET ====="
        );

        if (validPose(activeTarget)) {

            telemetry.addData(
                    "Target",
                    "(%.2f, %.2f, %.1f deg)",
                    activeTarget.x(),
                    activeTarget.y(),
                    normalizeDegrees0To360(
                            Math.toDegrees(
                                    activeTarget.heading()
                            )
                    )
            );

        } else {

            telemetry.addData(
                    "Target",
                    "INVALID"
            );
        }

        telemetry.addData(
                "Position Error",
                "%.2f in",
                positionError
        );

        telemetry.addData(
                "Heading Error",
                "%.2f deg",
                headingErrorDeg
        );

        telemetry.addData(
                "Within Position Tolerance",
                Double.isFinite(positionError)
                        && positionError
                        <= Math.max(
                        0.0,
                        POSITION_TOLERANCE_IN
                )
        );

        telemetry.addData(
                "Within Heading Tolerance",
                Double.isFinite(headingErrorDeg)
                        && headingErrorDeg
                        <= Math.max(
                        0.0,
                        HEADING_TOLERANCE_DEG
                )
        );

        // -------------------------------------------------------------
        // Controls
        // -------------------------------------------------------------

        telemetry.addLine(
                "===== CONTROLS ====="
        );

        telemetry.addLine(
                "G1 D-pad Left/Right: select test location"
        );

        telemetry.addLine(
                "G1 Cross: drive to selected location"
        );

        telemetry.addLine(
                "G1 Circle: abort current path"
        );

        telemetry.addLine(
                "G1 Triangle: reset LOCALIZER to centre"
        );

        telemetry.addLine(
                "Before START: place localization reference at"
        );

        telemetry.addLine(
                "(70.75, 70.75), facing 180 deg"
        );

        telemetry.addLine(
                "Triangle is safe only after physically returning robot to centre"
        );

        telemetry.update();
    }

    // ---------------------------------------------------------------------
    // Bulk caching
    // ---------------------------------------------------------------------

    private void configureBulkCaching() {

        hubs =
                hardwareMap.getAll(
                        LynxModule.class
                );

        for (LynxModule hub : hubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }
    }

    private void clearBulkCache() {

        if (hubs == null) {
            return;
        }

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    private void restoreBulkCaching() {

        if (hubs == null) {
            return;
        }

        for (LynxModule hub : hubs) {

            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }
    }

    // ---------------------------------------------------------------------
    // Utilities
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

    private double angleDifferenceDegrees(
            double targetDegrees,
            double currentDegrees
    ) {

        double difference =
                (targetDegrees - currentDegrees)
                        % 360.0;

        if (difference > 180.0) {
            difference -= 360.0;
        }

        if (difference < -180.0) {
            difference += 360.0;
        }

        return difference;
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

    // ---------------------------------------------------------------------
    // Stop
    // ---------------------------------------------------------------------

    @Override
    public void stop() {

        if (follower != null) {

            Pose finalPose =
                    follower.pose();

            if (validPose(finalPose)) {

                PoseStorage.setPose(
                        finalPose
                );
            }

            follower.stop();
        }

        restoreBulkCaching();
    }
}