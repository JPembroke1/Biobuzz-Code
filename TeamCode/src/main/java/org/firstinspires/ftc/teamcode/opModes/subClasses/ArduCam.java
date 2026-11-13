package org.firstinspires.ftc.teamcode.opModes.subClasses;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.configurables.annotations.Configurable;
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
 * Optimisation notes (vs. previous version):
 *
 * - Panels stream reuses two bitmaps and only copies every Nth frame
 *   (STREAM_ENABLED / STREAM_EVERY_N_FRAMES), so it no longer competes
 *   with AprilTag processing.
 * - Detections are snapshotted once per camera frame. Every getter reads
 *   the same cached snapshot, so a single OpMode loop always sees one
 *   consistent frame and nothing is re-allocated per call.
 * - getFrameId() lets consumers (Turret) act only on NEW frames.
 * - Cached snapshots expire after DETECTION_MAX_AGE_MS if vision stalls.
 * - Exposure/gain are applied with delay, retry and live re-apply when the
 *   Panels values change.
 * - Decimation can be changed live from Panels.
 * - FPS, pose-solve time, detection age and lens-intrinsics diagnostics.
 * - HIVE debounce counters advance per camera frame, not per OpMode loop.
 * - getTurretDetection() gives the turret one sticky target selection.
 *
 * Usage:
 *
 * Optionally call camera.update() once at the top of every loop. If you
 * never call it, the class refreshes itself automatically on each getter.
 */
@Configurable
public class ArduCam {

    // ---------------------------------------------------------------------
    // Panels camera stream
    // ---------------------------------------------------------------------

    /*
     * Turn the stream off completely during competition if every bit of
     * vision CPU matters. Streaming every 3rd frame is plenty for tuning.
     */
    public static boolean STREAM_ENABLED = true;
    public static int STREAM_EVERY_N_FRAMES = 3;

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

        /*
         * Two reusable buffers: one being displayed, one being written.
         */
        private final Bitmap[] buffers = new Bitmap[2];
        private int bufferIndex = 0;
        private int frameCounter = 0;

        @Override
        public void init(
                int width,
                int height,
                CameraCalibration calibration
        ) {

            buffers[0] =
                    Bitmap.createBitmap(
                            width,
                            height,
                            Bitmap.Config.RGB_565
                    );

            buffers[1] =
                    Bitmap.createBitmap(
                            width,
                            height,
                            Bitmap.Config.RGB_565
                    );

            lastFrame.set(buffers[0]);
        }

        @Override
        public Object processFrame(
                Mat frame,
                long captureTimeNanos
        ) {

            if (!STREAM_ENABLED) {
                return null;
            }

            frameCounter++;

            int every =
                    Math.max(
                            1,
                            STREAM_EVERY_N_FRAMES
                    );

            if (frameCounter % every != 0) {
                return null;
            }

            Bitmap target =
                    buffers[bufferIndex];

            if (target == null
                    || target.getWidth() != frame.width()
                    || target.getHeight() != frame.height()) {

                target =
                        Bitmap.createBitmap(
                                frame.width(),
                                frame.height(),
                                Bitmap.Config.RGB_565
                        );

                buffers[bufferIndex] = target;
            }

            Utils.matToBitmap(
                    frame,
                    target
            );

            lastFrame.set(target);

            bufferIndex ^= 1;

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

    /**
     * A cluster detection paired with its resolved HIVE cell.
     * Built once per camera frame.
     */
    private static class HiveEntry {

        final AprilTagClusterDetection detection;
        final HiveCell cell;

        HiveEntry(
                AprilTagClusterDetection detection,
                HiveCell cell
        ) {

            this.detection = detection;
            this.cell = cell;
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

    /*
     * These now count CAMERA FRAMES, not OpMode loops.
     */
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
     * Changes made from Panels are applied live.
     *
     * 3.0 = fastest, short range only
     * 2.0 = balanced starting point (default)
     * 1.5 = more range, more CPU
     * 1.0 = maximum detail and CPU load
     *
     * Watch fps and poseSolveMs while tuning.
     */
    public static float APRILTAG_DECIMATION = 2.0f;

    /*
     * Cached detections older than this are discarded (vision stalled).
     */
    public static long DETECTION_MAX_AGE_MS = 300;

    // ---------------------------------------------------------------------
    // Exposure and gain tuning
    //
    // These are starting values, not universal Arducam values.
    //
    // Tune under representative competition lighting. A moving turret
    // needs a SHORT exposure to avoid motion blur (try 3-6 ms with more
    // gain).
    //
    // Changes made from Panels are re-applied automatically.
    // ---------------------------------------------------------------------

    public static boolean MANUAL_EXPOSURE_ENABLED = true;

    public static int EXPOSURE_MS = 6;

    public static int CAMERA_GAIN = 80;

    public static int MAX_SETTINGS_ATTEMPTS = 5;
    public static long SETTINGS_START_DELAY_MS = 500;
    public static long SETTINGS_RETRY_DELAY_MS = 500;

    /*
     * Public diagnostic fields for telemetry.
     */
    public static boolean cameraSettingsApplied = false;

    public static int appliedExposureMs = -1;
    public static int appliedGain = -1;

    public static String cameraSettingsStatus =
            "WAITING FOR CAMERA";

    public static String lensIntrinsicsStatus =
            "NOT INITIALIZED";

    private int settingsAttempts = 0;
    private long streamingSinceNs = 0L;
    private long lastSettingsAttemptNs = 0L;

    private int configuredExposureMs = Integer.MIN_VALUE;
    private int configuredGain = Integer.MIN_VALUE;

    private float appliedDecimation = Float.NaN;

    // ---------------------------------------------------------------------
    // Per-frame detection snapshot
    // ---------------------------------------------------------------------

    private List<AprilTagDetection> cachedDetections =
            Collections.emptyList();

    private List<AprilTagSingleDetection> cachedSingles =
            Collections.emptyList();

    private List<AprilTagClusterDetection> cachedClusters =
            Collections.emptyList();

    private List<AprilTagSingleDetection> cachedMemberTags =
            Collections.emptyList();

    private List<HiveEntry> cachedHiveEntries =
            Collections.emptyList();

    private List<AprilTagClusterDetection> cachedHiveDetections =
            Collections.emptyList();

    private HiveCell cachedMemberCell = null;

    private long cachedAtNs = 0L;

    private long frameId = 0L;

    /*
     * Once update() has been called by the OpMode, getters stop
     * refreshing on their own so a whole loop sees one frame.
     */
    private boolean manualUpdateUsed = false;

    // ---------------------------------------------------------------------
    // HIVE state tracking
    // ---------------------------------------------------------------------

    private HiveState confirmedState =
            HiveState.UNKNOWN;

    private HiveState candidateState =
            HiveState.UNKNOWN;

    private int candidateStateFrames = 0;

    private long lastStateFrameId = -1L;

    private boolean hiveTipDetected = false;

    private int hiveTipCandidateFrames = 0;

    private long lastTipFrameId = -1L;

    private Alliance hiveTipAlliance = null;

    // ---------------------------------------------------------------------
    // Turret target selection (sticky)
    // ---------------------------------------------------------------------

    private String turretTargetKey = null;

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

        resetCameraSettingsState();

        clearCaches();

        cachedAtNs = 0L;

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
         *
         * Without them the SDK uses a built-in calibration, if it has one
         * for this camera and resolution. A wrong calibration gives wrong
         * bearing and range, which makes the turret gates reject tags.
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

            lensIntrinsicsStatus =
                    String.format(
                            Locale.US,
                            "CUSTOM fx=%.1f fy=%.1f cx=%.1f cy=%.1f @%dx%d",
                            fx,
                            fy,
                            cx,
                            cy,
                            resolutionWidth,
                            resolutionHeight
                    );

        } else {

            lensIntrinsicsStatus =
                    "NOT SET - SDK default used. Calibrate the Arducam at "
                            + resolutionWidth
                            + "x"
                            + resolutionHeight
                            + " and call setLensIntrinsics()";
        }

        aprilTag =
                tagBuilder.build();

        applyDecimationIfChanged();

        /*
         * Panels stream remains available for camera tuning, but is
         * throttled and can be disabled with STREAM_ENABLED.
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
    // Per-loop update and snapshot handling
    // ---------------------------------------------------------------------

    /**
     * Optional. Call once at the top of every OpMode loop.
     *
     * Applies camera settings and live decimation changes, and pulls the
     * newest frame result. After this has been called once, getters read
     * the cached snapshot without refreshing, so one loop always sees one
     * consistent frame.
     */
    public void update() {

        manualUpdateUsed = true;

        updateCameraSettings();

        refreshFromProcessor();

        expireStaleCache();
    }

    private void prepare() {

        if (!manualUpdateUsed) {

            updateCameraSettings();

            refreshFromProcessor();
        }

        expireStaleCache();
    }

    private void refreshFromProcessor() {

        if (!enabled
                || aprilTag == null) {

            return;
        }

        /*
         * Returns null when no new frame has been processed since the last
         * call, so this is cheap to call every loop.
         */
        List<AprilTagDetection> fresh =
                aprilTag.getFreshDetections();

        if (fresh == null) {
            return;
        }

        cachedDetections = fresh;
        cachedAtNs = System.nanoTime();
        frameId++;

        rebuildCaches();
    }

    private void expireStaleCache() {

        if (cachedAtNs == 0L
                || cachedDetections.isEmpty()) {

            return;
        }

        if (getDetectionAgeMs()
                > Math.max(
                1L,
                DETECTION_MAX_AGE_MS
        )) {

            clearCaches();
        }
    }

    private void clearCaches() {

        cachedDetections = Collections.emptyList();
        cachedSingles = Collections.emptyList();
        cachedClusters = Collections.emptyList();
        cachedMemberTags = Collections.emptyList();
        cachedHiveEntries = Collections.emptyList();
        cachedHiveDetections = Collections.emptyList();
        cachedMemberCell = null;
    }

    private void rebuildCaches() {

        List<AprilTagSingleDetection> singles =
                new ArrayList<>();

        List<AprilTagClusterDetection> clusters =
                new ArrayList<>();

        List<AprilTagSingleDetection> members =
                new ArrayList<>();

        HiveCell memberCell =
                null;

        for (AprilTagDetection detection
                : cachedDetections) {

            if (detection
                    instanceof AprilTagSingleDetection) {

                AprilTagSingleDetection single =
                        (AprilTagSingleDetection) detection;

                singles.add(single);

                HiveCell cell =
                        getHiveCellFromTagId(
                                single.id
                        );

                if (cell != null) {

                    members.add(single);

                    if (memberCell == null) {
                        memberCell = cell;
                    }
                }

            } else if (detection
                    instanceof AprilTagClusterDetection) {

                clusters.add(
                        (AprilTagClusterDetection) detection
                );
            }
        }

        List<HiveEntry> entries =
                new ArrayList<>();

        List<AprilTagClusterDetection> hiveDetections =
                new ArrayList<>();

        for (AprilTagClusterDetection cluster
                : clusters) {

            HiveCell cell =
                    resolveHiveCell(
                            cluster,
                            clusters.size(),
                            memberCell
                    );

            if (cell != null) {

                entries.add(
                        new HiveEntry(
                                cluster,
                                cell
                        )
                );

                hiveDetections.add(cluster);
            }
        }

        cachedSingles =
                Collections.unmodifiableList(singles);

        cachedClusters =
                Collections.unmodifiableList(clusters);

        cachedMemberTags =
                Collections.unmodifiableList(members);

        cachedHiveEntries =
                Collections.unmodifiableList(entries);

        cachedHiveDetections =
                Collections.unmodifiableList(
                        hiveDetections
                );

        cachedMemberCell = memberCell;
    }

    /**
     * Monotonically increasing ID of the newest processed camera frame.
     * Consumers can compare it with the last value they used to act only
     * on new frames.
     */
    public long getFrameId() {

        prepare();

        return frameId;
    }

    public double getDetectionAgeMs() {

        if (cachedAtNs == 0L) {
            return Double.POSITIVE_INFINITY;
        }

        return (System.nanoTime() - cachedAtNs)
                / 1_000_000.0;
    }

    // ---------------------------------------------------------------------
    // Camera diagnostics
    // ---------------------------------------------------------------------

    public double getFps() {

        return visionPortal == null
                ? 0.0
                : visionPortal.getFps();
    }

    public double getPoseSolveMs() {

        return aprilTag == null
                ? 0.0
                : aprilTag.getPerTagAvgPoseSolveTime();
    }

    // ---------------------------------------------------------------------
    // Decimation, exposure and gain
    // ---------------------------------------------------------------------

    private void applyDecimationIfChanged() {

        if (aprilTag == null
                || !Float.isFinite(APRILTAG_DECIMATION)
                || APRILTAG_DECIMATION <= 0.0f) {

            return;
        }

        if (Float.compare(
                appliedDecimation,
                APRILTAG_DECIMATION
        ) == 0) {

            return;
        }

        aprilTag.setDecimation(
                APRILTAG_DECIMATION
        );

        appliedDecimation =
                APRILTAG_DECIMATION;
    }

    private void resetCameraSettingsState() {

        cameraSettingsApplied = false;

        settingsAttempts = 0;
        streamingSinceNs = 0L;
        lastSettingsAttemptNs = 0L;

        configuredExposureMs = Integer.MIN_VALUE;
        configuredGain = Integer.MIN_VALUE;

        appliedExposureMs = -1;
        appliedGain = -1;
    }

    /**
     * Applies manual exposure and gain once VisionPortal has been
     * STREAMING for a short settling time.
     *
     * Non-blocking and safe to call every OpMode loop. If the camera
     * rejects the settings it is retried up to MAX_SETTINGS_ATTEMPTS
     * times. Changing EXPOSURE_MS or CAMERA_GAIN from Panels re-applies
     * them automatically.
     */
    public void updateCameraSettings() {

        if (visionPortal == null) {

            cameraSettingsStatus =
                    "VISION PORTAL NOT INITIALIZED";

            return;
        }

        applyDecimationIfChanged();

        if (!MANUAL_EXPOSURE_ENABLED) {

            cameraSettingsStatus =
                    "AUTO EXPOSURE ENABLED";

            return;
        }

        /*
         * Live tuning from Panels.
         */
        if (cameraSettingsApplied
                && (configuredExposureMs != EXPOSURE_MS
                || configuredGain != CAMERA_GAIN)) {

            requestCameraSettingsReapply();
        }

        if (cameraSettingsApplied) {
            return;
        }

        VisionPortal.CameraState cameraState =
                visionPortal.getCameraState();

        if (cameraState
                != VisionPortal.CameraState.STREAMING) {

            streamingSinceNs = 0L;

            cameraSettingsStatus =
                    "WAITING: "
                            + cameraState;

            return;
        }

        long nowNs =
                System.nanoTime();

        if (streamingSinceNs == 0L) {
            streamingSinceNs = nowNs;
        }

        if (nowNs - streamingSinceNs
                < SETTINGS_START_DELAY_MS * 1_000_000L) {

            cameraSettingsStatus =
                    "STREAM STABILISING";

            return;
        }

        if (settingsAttempts
                >= Math.max(
                1,
                MAX_SETTINGS_ATTEMPTS
        )) {

            return;
        }

        if (settingsAttempts > 0
                && nowNs - lastSettingsAttemptNs
                < SETTINGS_RETRY_DELAY_MS * 1_000_000L) {

            return;
        }

        settingsAttempts++;
        lastSettingsAttemptNs = nowNs;

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

                settingsAttempts =
                        MAX_SETTINGS_ATTEMPTS;

                return;
            }

            if (gainControl == null) {

                cameraSettingsStatus =
                        "GAIN CONTROL UNSUPPORTED";

                settingsAttempts =
                        MAX_SETTINGS_ATTEMPTS;

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
                        "MANUAL EXPOSURE REJECTED (attempt "
                                + settingsAttempts
                                + ")";

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

            if (exposureAccepted
                    && gainAccepted) {

                appliedExposureMs =
                        (int) requestedExposureMs;

                appliedGain =
                        requestedGain;

                configuredExposureMs =
                        EXPOSURE_MS;

                configuredGain =
                        CAMERA_GAIN;

                cameraSettingsApplied = true;

                cameraSettingsStatus =
                        "MANUAL SETTINGS APPLIED";

            } else {

                cameraSettingsStatus =
                        "CAMERA REJECTED SETTINGS (attempt "
                                + settingsAttempts
                                + ")";
            }

        } catch (RuntimeException exception) {

            cameraSettingsApplied = false;

            cameraSettingsStatus =
                    "SETTINGS ERROR: "
                            + exception
                            .getClass()
                            .getSimpleName()
                            + " (attempt "
                            + settingsAttempts
                            + ")";
        }
    }

    /**
     * Forces exposure and gain to be applied again.
     */
    public void requestCameraSettingsReapply() {

        cameraSettingsApplied = false;

        settingsAttempts = 0;
        streamingSinceNs = 0L;
        lastSettingsAttemptNs = 0L;

        configuredExposureMs = Integer.MIN_VALUE;
        configuredGain = Integer.MIN_VALUE;

        cameraSettingsStatus =
                "REAPPLY REQUESTED";
    }

    // ---------------------------------------------------------------------
    // General detection access (cached snapshot)
    // ---------------------------------------------------------------------

    public List<AprilTagDetection> getDetections() {

        prepare();

        return enabled
                ? cachedDetections
                : Collections.<AprilTagDetection>emptyList();
    }

    public List<AprilTagSingleDetection>
    getSingleDetections() {

        prepare();

        return cachedSingles;
    }

    public List<AprilTagClusterDetection>
    getClusterDetections() {

        prepare();

        return cachedClusters;
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

            if (clusterName.equals(
                    getClusterName(detection)
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
                    getClusterName(detection);

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
                            + getClusterName(detection)
                            + " | shortName="
                            + getClusterShortName(detection)
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

        prepare();

        return cachedMemberTags;
    }

    public HiveCell getVisibleHiveCellFromMemberTags() {

        prepare();

        return cachedMemberCell;
    }

    // ---------------------------------------------------------------------
    // HIVE cluster identity
    //
    // Cluster metadata is preferred.
    //
    // Single member-tag IDs provide a conservative identity fallback
    // when exactly one cluster is visible.
    //
    // Resolved once per camera frame in rebuildCaches().
    // ---------------------------------------------------------------------

    private HiveCell resolveHiveCell(
            AprilTagClusterDetection detection,
            int visibleClusterCount,
            HiveCell memberCell
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

        /*
         * Only associate a member-tag identity with a cluster when there
         * is exactly one visible cluster. This avoids assigning one
         * member-tag group to the wrong cluster.
         */
        if (visibleClusterCount == 1
                && memberCell != null) {

            return memberCell;
        }

        return null;
    }

    private HiveCell getHiveCell(
            AprilTagClusterDetection detection
    ) {

        if (detection == null) {
            return null;
        }

        for (HiveEntry entry
                : cachedHiveEntries) {

            if (entry.detection == detection) {
                return entry.cell;
            }
        }

        return getHiveCellFromClusterMetadata(
                detection
        );
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
                        + word.toLowerCase(Locale.US)
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

        prepare();

        return cachedHiveDetections;
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

        prepare();

        /*
         * Prefer a full SDK 12 cluster detection with valid pose.
         */
        for (HiveEntry entry
                : cachedHiveEntries) {

            if (entry.cell.alliance == alliance
                    && entry.detection.ftcPose != null) {

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
                : cachedMemberTags) {

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

        prepare();

        AprilTagClusterDetection best =
                null;

        double bestPitch =
                Double.MAX_VALUE;

        double threshold =
                validPositiveThreshold(
                        UP_CELL_PITCH_THRESHOLD_DEG,
                        35.0
                );

        for (HiveEntry entry
                : cachedHiveEntries) {

            AprilTagClusterDetection detection =
                    entry.detection;

            if (entry.cell.alliance != alliance
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

        prepare();

        AprilTagClusterDetection best =
                null;

        double bestRange =
                Double.MAX_VALUE;

        for (HiveEntry entry
                : cachedHiveEntries) {

            AprilTagClusterDetection detection =
                    entry.detection;

            if (entry.cell.alliance != alliance
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
    // Turret target selection
    // ---------------------------------------------------------------------

    /**
     * Returns the single cluster the turret should track.
     *
     * Selection is STICKY: once a cluster has been chosen, that same
     * cluster is returned for as long as it stays visible. This stops the
     * turret bearing from jumping between two clusters (for example an
     * "active" cell on one frame and the "nearest" cell on the next).
     *
     * When the previous cluster is gone, the active (upward) cell is
     * preferred, then the nearest HIVE cluster.
     *
     * getTurretTargetKey() returns the identity of the last selection so
     * the caller can tell when the target changed.
     */
    public AprilTagClusterDetection getTurretDetection(
            Alliance alliance
    ) {

        if (alliance == null) {
            return null;
        }

        prepare();

        double threshold =
                validPositiveThreshold(
                        UP_CELL_PITCH_THRESHOLD_DEG,
                        35.0
                );

        HiveEntry bestActive = null;
        double bestPitch = Double.MAX_VALUE;

        HiveEntry bestNearest = null;
        double bestRange = Double.MAX_VALUE;

        for (HiveEntry entry
                : cachedHiveEntries) {

            AprilTagClusterDetection detection =
                    entry.detection;

            if (entry.cell.alliance != alliance
                    || detection.ftcPose == null
                    || !Double.isFinite(
                    detection.ftcPose.bearing
            )
                    || !Double.isFinite(
                    detection.ftcPose.range
            )) {

                continue;
            }

            if (turretTargetKey != null
                    && turretTargetKey.equals(
                    makeClusterKey(entry)
            )) {

                return detection;
            }

            double pitch =
                    Math.abs(
                            detection.ftcPose.pitch
                    );

            if (Double.isFinite(pitch)
                    && pitch < threshold
                    && pitch < bestPitch) {

                bestPitch = pitch;
                bestActive = entry;
            }

            double range =
                    detection.ftcPose.range;

            if (range >= 0.0
                    && range < bestRange) {

                bestRange = range;
                bestNearest = entry;
            }
        }

        HiveEntry chosen =
                bestActive != null
                        ? bestActive
                        : bestNearest;

        if (chosen == null) {
            return null;
        }

        turretTargetKey =
                makeClusterKey(chosen);

        return chosen.detection;
    }

    /**
     * Identity of the cluster last returned by getTurretDetection().
     */
    public String getTurretTargetKey() {

        return turretTargetKey;
    }

    public void resetTurretTarget() {

        turretTargetKey = null;
    }

    private String makeClusterKey(
            HiveEntry entry
    ) {

        String name =
                getClusterName(
                        entry.detection
                );

        return name != null
                ? name
                : String.valueOf(entry.cell);
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

        for (HiveEntry entry
                : cachedHiveEntries) {

            if (entry.cell.alliance != alliance
                    || entry.detection.ftcPose == null) {

                continue;
            }

            sawAny = true;

            double pitch =
                    Math.abs(
                            entry.detection.ftcPose.pitch
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

    /**
     * Debounced HIVE state. Advances once per CAMERA FRAME, so it is
     * independent of how fast the OpMode loop runs.
     */
    public HiveState updateHiveState(
            Alliance alliance
    ) {

        prepare();

        if (frameId == lastStateFrameId) {
            return confirmedState;
        }

        lastStateFrameId = frameId;

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

        prepare();

        if (frameId == lastTipFrameId) {
            return hiveTipDetected;
        }

        lastTipFrameId = frameId;

        boolean tippedThisFrame =
                false;

        double threshold =
                validPositiveThreshold(
                        TIPPED_PITCH_THRESHOLD_DEG,
                        35.0
                );

        for (HiveEntry entry
                : cachedHiveEntries) {

            if (entry.cell.alliance != alliance
                    || entry.detection.ftcPose == null) {

                continue;
            }

            double pitch =
                    Math.abs(
                            entry.detection.ftcPose.pitch
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

        lastTipFrameId = -1L;

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

        lastStateFrameId = -1L;

        resetHiveTipDetection();
    }

    // ---------------------------------------------------------------------
    // Processor enable/disable
    // ---------------------------------------------------------------------

    public void setEnabled(
            boolean enabled
    ) {

        this.enabled = enabled;

        clearCaches();

        resetHiveState();

        resetTurretTarget();

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

        appliedDecimation = Float.NaN;

        resetCameraSettingsState();

        clearCaches();

        cachedAtNs = 0L;

        cameraSettingsStatus =
                "CAMERA CLOSED";

        resetHiveState();

        resetTurretTarget();
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