package org.firstinspires.ftc.teamcode.opModes.subClasses;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Arducam / VisionPortal subsystem for FTC SDK 12 and BIOBUZZ.
 *
 * Features:
 *
 * - Official BIOBUZZ AprilTag library
 * - SDK 12 single-tag and cluster detections
 * - Official HIVE member-tag ranges
 * - HIVE alliance and CELL-side identification
 * - Immediate HIVE visibility
 * - Active/upward CELL selection
 * - Debounced HIVE state
 * - Latched HIVE-tip detection
 * - Turret-fusion detection access
 * - Configurable AprilTag decimation
 * - Configurable manual exposure and gain
 * - Panels dashboard streaming
 *
 * SDK 12 detection structure:
 *
 * AprilTagDetection
 *   |- AprilTagSingleDetection
 *   |    |- id
 *   |    |- metadata
 *   |
 *   |- AprilTagClusterDetection
 *        |- cluster metadata
 *        |- cluster name
 *
 * ftcPose is available through the common AprilTagDetection base class.
 */
public class ArduCam {

    // ---------------------------------------------------------------------
    // Panels camera stream
    // ---------------------------------------------------------------------

    private static class PanelsStreamProcessor
            implements VisionProcessor, CameraStreamSource {

        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(
                        Bitmap.createBitmap(
                                1,
                                1,
                                Bitmap.Config.RGB_565
                        )
                );

        @Override
        public void init(
                int width,
                int height,
                CameraCalibration calibration
        ) {

            lastFrame.set(
                    Bitmap.createBitmap(
                            width,
                            height,
                            Bitmap.Config.RGB_565
                    )
            );
        }

        @Override
        public Object processFrame(
                Mat frame,
                long captureTimeNanos
        ) {

            Bitmap bitmap =
                    Bitmap.createBitmap(
                            frame.width(),
                            frame.height(),
                            Bitmap.Config.RGB_565
                    );

            Utils.matToBitmap(
                    frame,
                    bitmap
            );

            lastFrame.set(bitmap);

            return null;
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext
        ) {
            // No custom overlay.
        }

        @Override
        public void getFrameBitmap(
                Continuation<? extends Consumer<Bitmap>>
                        continuation
        ) {

            continuation.dispatch(
                    bitmapConsumer ->
                            bitmapConsumer.accept(
                                    lastFrame.get()
                            )
            );
        }
    }

    // ---------------------------------------------------------------------
    // HIVE definitions
    // ---------------------------------------------------------------------

    public enum Alliance {
        RED,
        BLUE
    }

    public enum HiveSide {
        AUDIENCE,
        FAR
    }

    public enum HiveState {
        UP,
        DOWN,
        UNKNOWN
    }

    public static class HiveCell {

        public final Alliance alliance;
        public final HiveSide side;

        public HiveCell(
                Alliance alliance,
                HiveSide side
        ) {

            this.alliance = alliance;
            this.side = side;
        }

        @Override
        public String toString() {

            return alliance
                    + "_"
                    + side;
        }
    }

    // ---------------------------------------------------------------------
    // Official BIOBUZZ HIVE member-tag ranges
    //
    // Red FAR:      30, 31, 32, 33
    // Red Audience: 34, 35, 36, 37
    // Blue Audience:38, 39, 40, 41
    // Blue FAR:     42, 43, 44, 45
    // ---------------------------------------------------------------------

    public static final int RED_FAR_TAG_MIN = 30;
    public static final int RED_FAR_TAG_MAX = 33;

    public static final int RED_AUDIENCE_TAG_MIN = 34;
    public static final int RED_AUDIENCE_TAG_MAX = 37;

    public static final int BLUE_AUDIENCE_TAG_MIN = 38;
    public static final int BLUE_AUDIENCE_TAG_MAX = 41;

    public static final int BLUE_FAR_TAG_MIN = 42;
    public static final int BLUE_FAR_TAG_MAX = 45;

    public static HiveCell getHiveCellFromTagId(
            int tagId
    ) {

        if (tagId >= RED_FAR_TAG_MIN
                && tagId <= RED_FAR_TAG_MAX) {

            return new HiveCell(
                    Alliance.RED,
                    HiveSide.FAR
            );
        }

        if (tagId >= RED_AUDIENCE_TAG_MIN
                && tagId <= RED_AUDIENCE_TAG_MAX) {

            return new HiveCell(
                    Alliance.RED,
                    HiveSide.AUDIENCE
            );
        }

        if (tagId >= BLUE_AUDIENCE_TAG_MIN
                && tagId <= BLUE_AUDIENCE_TAG_MAX) {

            return new HiveCell(
                    Alliance.BLUE,
                    HiveSide.AUDIENCE
            );
        }

        if (tagId >= BLUE_FAR_TAG_MIN
                && tagId <= BLUE_FAR_TAG_MAX) {

            return new HiveCell(
                    Alliance.BLUE,
                    HiveSide.FAR
            );
        }

        return null;
    }

    public static boolean isHiveMemberTag(
            int tagId
    ) {

        return getHiveCellFromTagId(
                tagId
        ) != null;
    }

    // ---------------------------------------------------------------------
    // HIVE state configuration
    // ---------------------------------------------------------------------

    public static int HIVE_STATE_FRAMES_TO_CONFIRM = 5;

    public static int TIP_FRAMES_TO_CONFIRM = 3;

    /*
     * Team-calibrated values.
     *
     * Record pitch and roll with the HIVE in both states and adjust from
     * measured data.
     */
    public static double UP_CELL_PITCH_THRESHOLD_DEG = 35.0;

    public static double TIPPED_PITCH_THRESHOLD_DEG = 35.0;

    // ---------------------------------------------------------------------
    // AprilTag detection tuning
    // ---------------------------------------------------------------------

    /*
     * Lower decimation retains more image detail at the cost of CPU.
     *
     * Suggested values:
     *
     * 2.0 = lighter processing
     * 1.5 = recommended starting point
     * 1.0 = maximum detail and CPU load
     */
    public static float APRILTAG_DECIMATION = 1.5f;

    // ---------------------------------------------------------------------
    // Exposure and gain tuning
    //
    // These are starting values, not universal Arducam values.
    //
    // Tune under representative competition lighting.
    // ---------------------------------------------------------------------

    public static boolean MANUAL_EXPOSURE_ENABLED = true;

    public static int EXPOSURE_MS = 6;

    public static int CAMERA_GAIN = 80;

    /*
     * Public diagnostic fields for telemetry.
     */
    public static boolean cameraSettingsApplied = false;

    public static int appliedExposureMs = -1;
    public static int appliedGain = -1;

    public static String cameraSettingsStatus =
            "WAITING FOR CAMERA";

    /*
     * Prevent unsupported camera controls from being retried every frame.
     */
    private boolean cameraSettingsAttempted = false;

    // ---------------------------------------------------------------------
    // HIVE state tracking
    // ---------------------------------------------------------------------

    private HiveState confirmedState =
            HiveState.UNKNOWN;

    private HiveState candidateState =
            HiveState.UNKNOWN;

    private int candidateStateFrames = 0;

    private boolean hiveTipDetected = false;

    private int hiveTipCandidateFrames = 0;

    private Alliance hiveTipAlliance = null;

    // ---------------------------------------------------------------------
    // Hardware and VisionPortal
    // ---------------------------------------------------------------------

    private final HardwareMap hardwareMap;

    private final String webcamName;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private PanelsStreamProcessor streamProcessor;

    /*
     * Null means use the official BIOBUZZ library bundled with SDK 12.
     */
    private AprilTagLibrary tagLibrary = null;

    private boolean enabled = true;

    // ---------------------------------------------------------------------
    // Camera calibration and stream configuration
    // ---------------------------------------------------------------------

    private Double fx = null;
    private Double fy = null;
    private Double cx = null;
    private Double cy = null;

    private int resolutionWidth = 640;
    private int resolutionHeight = 480;

    private VisionPortal.StreamFormat streamFormat =
            VisionPortal.StreamFormat.MJPEG;

    // ---------------------------------------------------------------------
    // Construction
    // ---------------------------------------------------------------------

    public ArduCam(
            HardwareMap hardwareMap,
            String webcamName
    ) {

        if (hardwareMap == null) {

            throw new IllegalArgumentException(
                    "HardwareMap cannot be null"
            );
        }

        if (webcamName == null
                || webcamName.trim().isEmpty()) {

            throw new IllegalArgumentException(
                    "Webcam name cannot be empty"
            );
        }

        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    // ---------------------------------------------------------------------
    // Configuration
    // ---------------------------------------------------------------------

    public ArduCam setLensIntrinsics(
            double fx,
            double fy,
            double cx,
            double cy
    ) {

        requireNotInitialized();

        if (!Double.isFinite(fx)
                || !Double.isFinite(fy)
                || !Double.isFinite(cx)
                || !Double.isFinite(cy)
                || fx <= 0.0
                || fy <= 0.0) {

            throw new IllegalArgumentException(
                    "Invalid camera lens intrinsics"
            );
        }

        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;

        return this;
    }

    public ArduCam setResolution(
            int width,
            int height
    ) {

        requireNotInitialized();

        if (width <= 0
                || height <= 0) {

            throw new IllegalArgumentException(
                    "Camera resolution must be positive"
            );
        }

        resolutionWidth = width;
        resolutionHeight = height;

        return this;
    }

    public ArduCam setStreamFormat(
            VisionPortal.StreamFormat format
    ) {

        requireNotInitialized();

        if (format == null) {

            throw new IllegalArgumentException(
                    "Stream format cannot be null"
            );
        }

        streamFormat = format;

        return this;
    }

    public ArduCam setTagLibrary(
            AprilTagLibrary library
    ) {

        requireNotInitialized();

        tagLibrary = library;

        return this;
    }

    private void requireNotInitialized() {

        if (visionPortal != null) {

            throw new IllegalStateException(
                    "ArduCam configuration must occur before init()"
            );
        }
    }

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    public void init() {

        if (visionPortal != null) {
            return;
        }

        cameraSettingsApplied = false;
        cameraSettingsAttempted = false;

        appliedExposureMs = -1;
        appliedGain = -1;

        cameraSettingsStatus =
                "INITIALIZING";

        AprilTagLibrary selectedLibrary =
                tagLibrary != null
                        ? tagLibrary
                        : AprilTagGameDatabase
                        .getBioBuzzTagLibrary();

        AprilTagProcessor.Builder tagBuilder =
                new AprilTagProcessor.Builder()
                        .setTagLibrary(
                                selectedLibrary
                        );

        /*
         * Lens intrinsics must be supplied as a complete set.
         */
        if (fx != null
                && fy != null
                && cx != null
                && cy != null) {

            tagBuilder.setLensIntrinsics(
                    fx,
                    fy,
                    cx,
                    cy
            );
        }

        aprilTag =
                tagBuilder.build();

        /*
         * Lower decimation preserves more detail for distant or small
         * AprilTags.
         */
        if (Float.isFinite(APRILTAG_DECIMATION)
                && APRILTAG_DECIMATION > 0.0f) {

            aprilTag.setDecimation(
                    APRILTAG_DECIMATION
            );
        }

        /*
         * Panels stream remains enabled for camera tuning.
         */
        streamProcessor =
                new PanelsStreamProcessor();

        visionPortal =
                new VisionPortal.Builder()
                        .setCamera(
                                hardwareMap.get(
                                        WebcamName.class,
                                        webcamName
                                )
                        )
                        .addProcessor(
                                aprilTag
                        )
                        .addProcessor(
                                streamProcessor
                        )
                        .setCameraResolution(
                                new Size(
                                        resolutionWidth,
                                        resolutionHeight
                                )
                        )
                        .setStreamFormat(
                                streamFormat
                        )
                        .build();

        visionPortal.setProcessorEnabled(
                aprilTag,
                enabled
        );

        cameraSettingsStatus =
                "WAITING FOR STREAM";

        resetHiveState();
    }

    // ---------------------------------------------------------------------
    // Exposure and gain
    // ---------------------------------------------------------------------

    /**
     * Applies manual exposure and gain once VisionPortal reaches
     * STREAMING.
     *
     * This is non-blocking and can safely be called every OpMode loop.
     * Work is performed only once.
     */
    public void updateCameraSettings() {

        if (visionPortal == null) {

            cameraSettingsStatus =
                    "VISION PORTAL NOT INITIALIZED";

            return;
        }

        if (!MANUAL_EXPOSURE_ENABLED) {

            cameraSettingsStatus =
                    "AUTO EXPOSURE ENABLED";

            return;
        }

        if (cameraSettingsApplied
                || cameraSettingsAttempted) {

            return;
        }

        VisionPortal.CameraState cameraState =
                visionPortal.getCameraState();

        if (cameraState
                != VisionPortal.CameraState.STREAMING) {

            cameraSettingsStatus =
                    "WAITING: "
                            + cameraState;

            return;
        }

        cameraSettingsAttempted = true;

        try {

            ExposureControl exposureControl =
                    visionPortal.getCameraControl(
                            ExposureControl.class
                    );

            GainControl gainControl =
                    visionPortal.getCameraControl(
                            GainControl.class
                    );

            if (exposureControl == null) {

                cameraSettingsStatus =
                        "EXPOSURE CONTROL UNSUPPORTED";

                return;
            }

            if (gainControl == null) {

                cameraSettingsStatus =
                        "GAIN CONTROL UNSUPPORTED";

                return;
            }

            long minimumExposureMs =
                    exposureControl.getMinExposure(
                            TimeUnit.MILLISECONDS
                    );

            long maximumExposureMs =
                    exposureControl.getMaxExposure(
                            TimeUnit.MILLISECONDS
                    );

            /*
             * Defensive handling for cameras that report unusual ranges.
             */
            if (minimumExposureMs < 0L) {
                minimumExposureMs = 0L;
            }

            if (maximumExposureMs
                    < minimumExposureMs) {

                maximumExposureMs =
                        minimumExposureMs;
            }

            long requestedExposureMs =
                    Math.max(
                            minimumExposureMs,
                            Math.min(
                                    maximumExposureMs,
                                    Math.max(
                                            0,
                                            EXPOSURE_MS
                                    )
                            )
                    );

            int minimumGain =
                    gainControl.getMinGain();

            int maximumGain =
                    gainControl.getMaxGain();

            if (maximumGain < minimumGain) {

                maximumGain =
                        minimumGain;
            }

            int requestedGain =
                    Math.max(
                            minimumGain,
                            Math.min(
                                    maximumGain,
                                    CAMERA_GAIN
                            )
                    );

            boolean manualModeAccepted =
                    exposureControl.setMode(
                            ExposureControl.Mode.Manual
                    );

            if (!manualModeAccepted) {

                cameraSettingsStatus =
                        "MANUAL EXPOSURE REJECTED";

                return;
            }

            boolean exposureAccepted =
                    exposureControl.setExposure(
                            requestedExposureMs,
                            TimeUnit.MILLISECONDS
                    );

            boolean gainAccepted =
                    gainControl.setGain(
                            requestedGain
                    );

            appliedExposureMs =
                    (int) requestedExposureMs;

            appliedGain =
                    requestedGain;

            cameraSettingsApplied =
                    exposureAccepted
                            && gainAccepted;

            if (cameraSettingsApplied) {

                cameraSettingsStatus =
                        "MANUAL SETTINGS APPLIED";

            } else {

                cameraSettingsStatus =
                        "CAMERA REJECTED SETTINGS";
            }

        } catch (RuntimeException exception) {

            cameraSettingsApplied = false;

            cameraSettingsStatus =
                    "SETTINGS ERROR: "
                            + exception
                            .getClass()
                            .getSimpleName();
        }
    }

    /**
     * Allows dashboard values to be reapplied during testing.
     *
     * Call after changing EXPOSURE_MS or CAMERA_GAIN.
     */
    public void requestCameraSettingsReapply() {

        cameraSettingsApplied = false;
        cameraSettingsAttempted = false;

        appliedExposureMs = -1;
        appliedGain = -1;

        cameraSettingsStatus =
                "REAPPLY REQUESTED";
    }

    // ---------------------------------------------------------------------
    // General detection access
    // ---------------------------------------------------------------------

    public List<AprilTagDetection> getDetections() {

        /*
         * Applies exposure and gain as soon as the camera reaches the
         * STREAMING state.
         */
        updateCameraSettings();

        if (!enabled
                || aprilTag == null) {

            return Collections.emptyList();
        }

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        return detections == null
                ? Collections.emptyList()
                : detections;
    }

    public List<AprilTagSingleDetection>
    getSingleDetections() {

        List<AprilTagSingleDetection> result =
                new ArrayList<>();

        for (AprilTagDetection detection
                : getDetections()) {

            if (detection
                    instanceof AprilTagSingleDetection) {

                result.add(
                        (AprilTagSingleDetection) detection
                );
            }
        }

        return result;
    }

    public List<AprilTagClusterDetection>
    getClusterDetections() {

        List<AprilTagClusterDetection> result =
                new ArrayList<>();

        for (AprilTagDetection detection
                : getDetections()) {

            if (detection
                    instanceof AprilTagClusterDetection) {

                result.add(
                        (AprilTagClusterDetection) detection
                );
            }
        }

        return result;
    }

    /**
     * Returns an ordinary single AprilTag by ID.
     *
     * Cluster detections are ignored because clusters do not have one
     * cluster-level ID.
     */
    public AprilTagSingleDetection getDetection(
            int tagId
    ) {

        for (AprilTagSingleDetection detection
                : getSingleDetections()) {

            if (detection.id == tagId) {
                return detection;
            }
        }

        return null;
    }

    public AprilTagClusterDetection getClusterDetection(
            String clusterName
    ) {

        if (clusterName == null) {
            return null;
        }

        for (AprilTagClusterDetection detection
                : getClusterDetections()) {

            String detectedName =
                    getClusterName(
                            detection
                    );

            if (clusterName.equals(
                    detectedName
            )) {

                return detection;
            }
        }

        return null;
    }

    private String getClusterName(
            AprilTagClusterDetection detection
    ) {

        if (detection == null
                || detection.metadata == null) {

            return null;
        }

        return detection.metadata.name;
    }

    private String getClusterShortName(
            AprilTagClusterDetection detection
    ) {

        if (detection == null
                || detection.metadata == null) {

            return null;
        }

        return detection.metadata.shortName;
    }

    public List<String> getVisibleClusterNames() {

        List<String> names =
                new ArrayList<>();

        for (AprilTagClusterDetection detection
                : getClusterDetections()) {

            String name =
                    getClusterName(
                            detection
                    );

            if (name != null
                    && !names.contains(name)) {

                names.add(name);
            }
        }

        return names;
    }

    public List<String> getVisibleClusterDetails() {

        List<String> details =
                new ArrayList<>();

        for (AprilTagClusterDetection detection
                : getClusterDetections()) {

            String name =
                    getClusterName(
                            detection
                    );

            String shortName =
                    getClusterShortName(
                            detection
                    );

            String poseDetails;

            if (detection.ftcPose == null) {

                poseDetails =
                        "pose=NONE";

            } else {

                poseDetails =
                        String.format(
                                Locale.US,
                                "bearing=%.1f range=%.1f pitch=%.1f roll=%.1f",
                                detection.ftcPose.bearing,
                                detection.ftcPose.range,
                                detection.ftcPose.pitch,
                                detection.ftcPose.roll
                        );
            }

            details.add(
                    "name="
                            + String.valueOf(name)
                            + " | shortName="
                            + String.valueOf(shortName)
                            + " | "
                            + poseDetails
            );
        }

        return details;
    }

    // ---------------------------------------------------------------------
    // Visible HIVE member tags
    // ---------------------------------------------------------------------

    public List<AprilTagSingleDetection>
    getVisibleHiveMemberTags() {

        List<AprilTagSingleDetection> result =
                new ArrayList<>();

        for (AprilTagSingleDetection detection
                : getSingleDetections()) {

            if (isHiveMemberTag(
                    detection.id
            )) {

                result.add(detection);
            }
        }

        return result;
    }

    public HiveCell getVisibleHiveCellFromMemberTags() {

        for (AprilTagSingleDetection detection
                : getVisibleHiveMemberTags()) {

            HiveCell cell =
                    getHiveCellFromTagId(
                            detection.id
                    );

            if (cell != null) {
                return cell;
            }
        }

        return null;
    }

    // ---------------------------------------------------------------------
    // HIVE cluster identity
    //
    // Cluster metadata is preferred.
    //
    // Single member-tag IDs provide a conservative identity fallback
    // when exactly one cluster is visible.
    // ---------------------------------------------------------------------

    private HiveCell getHiveCell(
            AprilTagClusterDetection detection
    ) {

        if (detection == null) {
            return null;
        }

        HiveCell metadataCell =
                getHiveCellFromClusterMetadata(
                        detection
                );

        if (metadataCell != null) {
            return metadataCell;
        }

        List<AprilTagClusterDetection> clusters =
                getClusterDetections();

        HiveCell memberCell =
                getVisibleHiveCellFromMemberTags();

        /*
         * Only associate a member-tag identity with a cluster when there
         * is exactly one visible cluster. This avoids assigning one
         * member-tag group to the wrong cluster.
         */
        if (clusters.size() == 1
                && memberCell != null) {

            return memberCell;
        }

        return null;
    }

    private HiveCell getHiveCellFromClusterMetadata(
            AprilTagClusterDetection detection
    ) {

        String name =
                normalizeMetadataText(
                        getClusterName(
                                detection
                        )
                );

        String shortName =
                normalizeMetadataText(
                        getClusterShortName(
                                detection
                        )
                );

        String combined =
                name
                        + " "
                        + shortName;

        Alliance alliance =
                null;

        HiveSide side =
                null;

        if (containsWord(
                combined,
                "red"
        )) {

            alliance =
                    Alliance.RED;

        } else if (containsWord(
                combined,
                "blue"
        )) {

            alliance =
                    Alliance.BLUE;
        }

        if (containsWord(
                combined,
                "audience"
        )) {

            side =
                    HiveSide.AUDIENCE;

        } else if (containsWord(
                combined,
                "far"
        )
                || combined.contains(
                "opposite audience"
        )
                || combined.contains(
                "oppositeaudience"
        )
                || containsWord(
                combined,
                "rear"
        )) {

            side =
                    HiveSide.FAR;
        }

        if (alliance == null
                || side == null) {

            return null;
        }

        return new HiveCell(
                alliance,
                side
        );
    }

    private static String normalizeMetadataText(
            String value
    ) {

        if (value == null) {
            return "";
        }

        return value
                .trim()
                .toLowerCase(Locale.US)
                .replace('-', ' ')
                .replace('_', ' ')
                .replaceAll("\\s+", " ");
    }

    private static boolean containsWord(
            String text,
            String word
    ) {

        if (text == null
                || word == null) {

            return false;
        }

        String paddedText =
                " "
                        + text
                        + " ";

        String paddedWord =
                " "
                        + word
                        .toLowerCase(
                                Locale.US
                        )
                        + " ";

        return paddedText.contains(
                paddedWord
        );
    }

    // ---------------------------------------------------------------------
    // HIVE cluster filtering
    // ---------------------------------------------------------------------

    public List<AprilTagClusterDetection>
    getHiveDetections() {

        List<AprilTagClusterDetection> result =
                new ArrayList<>();

        for (AprilTagClusterDetection detection
                : getClusterDetections()) {

            if (getHiveCell(
                    detection
            ) != null) {

                result.add(detection);
            }
        }

        return result;
    }

    // ---------------------------------------------------------------------
    // Immediate HIVE visibility
    // ---------------------------------------------------------------------

    public boolean isHiveVisible(
            Alliance alliance
    ) {

        if (alliance == null
                || !enabled
                || aprilTag == null) {

            return false;
        }

        /*
         * Prefer a full SDK 12 cluster detection with valid pose.
         */
        for (AprilTagClusterDetection detection
                : getHiveDetections()) {

            HiveCell cell =
                    getHiveCell(
                            detection
                    );

            if (cell != null
                    && cell.alliance == alliance
                    && detection.ftcPose != null) {

                return true;
            }
        }

        /*
         * Identity-only fallback using one visible member tag.
         *
         * This can prove HIVE visibility, but it is not used for turret
         * cluster-pose fusion.
         */
        for (AprilTagSingleDetection detection
                : getVisibleHiveMemberTags()) {

            HiveCell cell =
                    getHiveCellFromTagId(
                            detection.id
                    );

            if (cell != null
                    && cell.alliance == alliance) {

                return true;
            }
        }

        return false;
    }

    // ---------------------------------------------------------------------
    // Active/upward HIVE CELL
    // ---------------------------------------------------------------------

    public HiveCell getActiveCell(
            Alliance alliance
    ) {

        return getHiveCell(
                getActiveCellDetection(
                        alliance
                )
        );
    }

    /**
     * Returns the upward alliance CELL with the smallest absolute pitch
     * inside UP_CELL_PITCH_THRESHOLD_DEG.
     */
    public AprilTagClusterDetection
    getActiveCellDetection(
            Alliance alliance
    ) {

        if (alliance == null) {
            return null;
        }

        AprilTagClusterDetection best =
                null;

        double bestPitch =
                Double.MAX_VALUE;

        double threshold =
                validPositiveThreshold(
                        UP_CELL_PITCH_THRESHOLD_DEG,
                        35.0
                );

        for (AprilTagClusterDetection detection
                : getHiveDetections()) {

            HiveCell cell =
                    getHiveCell(
                            detection
                    );

            if (cell == null
                    || cell.alliance != alliance
                    || detection.ftcPose == null) {

                continue;
            }

            double pitch =
                    Math.abs(
                            detection.ftcPose.pitch
                    );

            if (Double.isFinite(pitch)
                    && pitch < threshold
                    && pitch < bestPitch) {

                bestPitch = pitch;
                best = detection;
            }
        }

        return best;
    }

    /**
     * Returns the nearest visible HIVE cluster for the alliance,
     * regardless of whether the CELL is considered up or down.
     */
    public AprilTagClusterDetection
    getAnyHiveDetection(
            Alliance alliance
    ) {

        if (alliance == null) {
            return null;
        }

        AprilTagClusterDetection best =
                null;

        double bestRange =
                Double.MAX_VALUE;

        for (AprilTagClusterDetection detection
                : getHiveDetections()) {

            HiveCell cell =
                    getHiveCell(
                            detection
                    );

            if (cell == null
                    || cell.alliance != alliance
                    || detection.ftcPose == null) {

                continue;
            }

            double range =
                    detection.ftcPose.range;

            if (Double.isFinite(range)
                    && range >= 0.0
                    && range < bestRange) {

                bestRange = range;
                best = detection;
            }
        }

        return best;
    }

    // ---------------------------------------------------------------------
    // HIVE up/down state
    // ---------------------------------------------------------------------

    private HiveState readRawState(
            Alliance alliance
    ) {

        if (alliance == null) {
            return HiveState.UNKNOWN;
        }

        boolean sawAny =
                false;

        double threshold =
                validPositiveThreshold(
                        UP_CELL_PITCH_THRESHOLD_DEG,
                        35.0
                );

        for (AprilTagClusterDetection detection
                : getHiveDetections()) {

            HiveCell cell =
                    getHiveCell(
                            detection
                    );

            if (cell == null
                    || cell.alliance != alliance
                    || detection.ftcPose == null) {

                continue;
            }

            sawAny = true;

            double pitch =
                    Math.abs(
                            detection.ftcPose.pitch
                    );

            if (Double.isFinite(pitch)
                    && pitch < threshold) {

                return HiveState.UP;
            }
        }

        return sawAny
                ? HiveState.DOWN
                : HiveState.UNKNOWN;
    }

    public HiveState updateHiveState(
            Alliance alliance
    ) {

        HiveState raw =
                readRawState(
                        alliance
                );

        if (raw == candidateState) {

            candidateStateFrames++;

        } else {

            candidateState = raw;
            candidateStateFrames = 1;
        }

        int requiredFrames =
                Math.max(
                        1,
                        HIVE_STATE_FRAMES_TO_CONFIRM
                );

        if (candidateStateFrames
                >= requiredFrames
                && candidateState
                != HiveState.UNKNOWN) {

            confirmedState =
                    candidateState;
        }

        return confirmedState;
    }

    public HiveState getHiveState() {

        return confirmedState;
    }

    // ---------------------------------------------------------------------
    // Latched generic HIVE-tip detection
    //
    // For autonomous first-tip detection, comparing the active CELL side
    // before/after the expected tip is safer than this generic method.
    // ---------------------------------------------------------------------

    public boolean updateHiveTipDetected(
            Alliance alliance
    ) {

        if (alliance == null
                || !enabled
                || aprilTag == null) {

            hiveTipCandidateFrames = 0;

            return hiveTipDetected;
        }

        boolean tippedThisFrame =
                false;

        double threshold =
                validPositiveThreshold(
                        TIPPED_PITCH_THRESHOLD_DEG,
                        35.0
                );

        for (AprilTagClusterDetection detection
                : getHiveDetections()) {

            HiveCell cell =
                    getHiveCell(
                            detection
                    );

            if (cell == null
                    || cell.alliance != alliance
                    || detection.ftcPose == null) {

                continue;
            }

            double pitch =
                    Math.abs(
                            detection.ftcPose.pitch
                    );

            if (Double.isFinite(pitch)
                    && pitch < threshold) {

                tippedThisFrame = true;

                break;
            }
        }

        if (tippedThisFrame) {

            hiveTipCandidateFrames++;

        } else {

            hiveTipCandidateFrames = 0;
        }

        int requiredFrames =
                Math.max(
                        1,
                        TIP_FRAMES_TO_CONFIRM
                );

        if (hiveTipCandidateFrames
                >= requiredFrames) {

            hiveTipDetected = true;
            hiveTipAlliance = alliance;
        }

        return hiveTipDetected;
    }

    public boolean isHiveTipped() {

        return hiveTipDetected;
    }

    public boolean isHiveTipped(
            Alliance alliance
    ) {

        return hiveTipDetected
                && alliance != null
                && hiveTipAlliance == alliance;
    }

    public void resetHiveTipDetection() {

        hiveTipDetected = false;

        hiveTipCandidateFrames = 0;

        hiveTipAlliance = null;
    }

    /**
     * Updates both the debounced state and generic tip state.
     */
    public void updateHive(
            Alliance alliance
    ) {

        updateHiveState(
                alliance
        );

        updateHiveTipDetected(
                alliance
        );
    }

    public void resetHiveState() {

        confirmedState =
                HiveState.UNKNOWN;

        candidateState =
                HiveState.UNKNOWN;

        candidateStateFrames = 0;

        resetHiveTipDetection();
    }

    // ---------------------------------------------------------------------
    // Processor enable/disable
    // ---------------------------------------------------------------------

    public void setEnabled(
            boolean enabled
    ) {

        this.enabled = enabled;

        resetHiveState();

        if (visionPortal != null
                && aprilTag != null) {

            visionPortal.setProcessorEnabled(
                    aprilTag,
                    enabled
            );
        }
    }

    public boolean isEnabled() {

        return enabled;
    }

    // ---------------------------------------------------------------------
    // Panels streaming
    // ---------------------------------------------------------------------

    public void streamToPanels() {

        if (streamProcessor == null) {

            throw new IllegalStateException(
                    "Call ArduCam.init() before streamToPanels()"
            );
        }

        PanelsCameraStream.INSTANCE.startStream(
                streamProcessor,
                100
        );
    }

    public void stopPanelsStream() {

        PanelsCameraStream.INSTANCE.stopStream();
    }

    // ---------------------------------------------------------------------
    // Camera status and control
    // ---------------------------------------------------------------------

    public boolean isStreaming() {

        return visionPortal != null
                && visionPortal.getCameraState()
                == VisionPortal.CameraState.STREAMING;
    }

    public VisionPortal.CameraState getCameraState() {

        return visionPortal == null
                ? null
                : visionPortal.getCameraState();
    }

    public void stopStreaming() {

        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public void resumeStreaming() {

        if (visionPortal != null) {

            visionPortal.resumeStreaming();

            /*
             * Some cameras restore automatic settings after streaming is
             * restarted. Allow manual settings to be reapplied.
             */
            requestCameraSettingsReapply();
        }
    }

    public void close() {

        stopPanelsStream();

        if (visionPortal != null) {

            visionPortal.close();

            visionPortal = null;
        }

        aprilTag = null;
        streamProcessor = null;

        cameraSettingsApplied = false;
        cameraSettingsAttempted = false;

        appliedExposureMs = -1;
        appliedGain = -1;

        cameraSettingsStatus =
                "CAMERA CLOSED";

        resetHiveState();
    }

    // ---------------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------------

    private static double validPositiveThreshold(
            double configuredValue,
            double fallbackValue
    ) {

        return Double.isFinite(configuredValue)
                ? Math.abs(configuredValue)
                : Math.abs(fallbackValue);
    }
}