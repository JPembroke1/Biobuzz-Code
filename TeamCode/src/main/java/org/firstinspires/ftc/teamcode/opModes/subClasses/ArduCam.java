package org.firstinspires.ftc.teamcode.opModes.subClasses;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import java.util.concurrent.atomic.AtomicReference;

/**
 * SDK 12 Arducam / VisionPortal subsystem for BIOBUZZ.
 *
 * Features:
 *
 * - Official BIOBUZZ AprilTag library
 * - SDK 12 cluster and single-tag handling
 * - Official HIVE member-tag ranges 30 through 45
 * - HIVE alliance and side identification
 * - Immediate HIVE visibility
 * - Debounced HIVE state
 * - Latched tip detection
 * - Turret fusion detection access
 * - Optional Panels streaming
 *
 * SDK 12 notes:
 *
 * AprilTagDetection is the shared base type.
 *
 * AprilTagSingleDetection contains:
 * - id
 * - single-tag metadata
 *
 * AprilTagClusterDetection contains:
 * - cluster metadata
 * - cluster name
 *
 * ftcPose is available through the base detection type.
 */
public class ArduCam {

    // ---------------------------------------------------------------------
    // Panels camera streaming
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
    // Each CELL contains one cluster of four 3.25-inch 36h11 tags.
    // ---------------------------------------------------------------------

    public static final int RED_FAR_TAG_MIN = 30;
    public static final int RED_FAR_TAG_MAX = 33;

    public static final int RED_AUDIENCE_TAG_MIN = 34;
    public static final int RED_AUDIENCE_TAG_MAX = 37;

    public static final int BLUE_AUDIENCE_TAG_MIN = 38;
    public static final int BLUE_AUDIENCE_TAG_MAX = 41;

    public static final int BLUE_FAR_TAG_MIN = 42;
    public static final int BLUE_FAR_TAG_MAX = 45;

    /**
     * Converts a BIOBUZZ HIVE member-tag ID into its CELL identity.
     */
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

        return getHiveCellFromTagId(tagId) != null;
    }

    // ---------------------------------------------------------------------
    // HIVE configuration
    // ---------------------------------------------------------------------

    public static int HIVE_STATE_FRAMES_TO_CONFIRM = 5;
    public static int TIP_FRAMES_TO_CONFIRM = 3;

    /*
     * Team-calibrated thresholds.
     *
     * Verify actual pitch/roll telemetry with your turret-mounted camera.
     */
    public static double UP_CELL_PITCH_THRESHOLD_DEG = 35.0;
    public static double TIPPED_PITCH_THRESHOLD_DEG = 35.0;

    // ---------------------------------------------------------------------
    // HIVE state
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
     * Null means use the official SDK 12 BIOBUZZ library.
     */
    private AprilTagLibrary tagLibrary = null;

    private boolean enabled = true;

    // ---------------------------------------------------------------------
    // Camera configuration
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

        if (width <= 0 || height <= 0) {

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

        resetHiveState();
    }

    // ---------------------------------------------------------------------
    // General detection access
    // ---------------------------------------------------------------------

    public List<AprilTagDetection> getDetections() {

        if (!enabled || aprilTag == null) {

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
                    getClusterName(detection);

            if (clusterName.equals(detectedName)) {
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

            String name =
                    getClusterName(detection);

            String shortName =
                    getClusterShortName(detection);

            details.add(
                    "name="
                            + String.valueOf(name)
                            + " | shortName="
                            + String.valueOf(shortName)
            );
        }

        return details;
    }

    // ---------------------------------------------------------------------
    // Visible HIVE member tags
    //
    // Useful for diagnostics and identity fallback.
    // ---------------------------------------------------------------------

    public List<AprilTagSingleDetection>
    getVisibleHiveMemberTags() {

        List<AprilTagSingleDetection> result =
                new ArrayList<>();

        for (AprilTagSingleDetection detection
                : getSingleDetections()) {

            if (isHiveMemberTag(detection.id)) {
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
    // Cluster identity
    //
    // SDK 12 cluster pose is preferred for turret aiming.
    //
    // The official cluster metadata name/short name is inspected for
    // alliance and side. Single member IDs remain an identity fallback.
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

        /*
         * If one cluster and one visible member-tag group are present,
         * use the member ID range to identify that cluster.
         *
         * This fallback is intentionally conservative.
         */
        List<AprilTagClusterDetection> clusters =
                getClusterDetections();

        HiveCell memberCell =
                getVisibleHiveCellFromMemberTags();

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
                        getClusterName(detection)
                );

        String shortName =
                normalizeMetadataText(
                        getClusterShortName(detection)
                );

        String combined =
                name + " " + shortName;

        Alliance alliance =
                null;

        HiveSide side =
                null;

        if (containsWord(combined, "red")) {

            alliance =
                    Alliance.RED;

        } else if (containsWord(combined, "blue")) {

            alliance =
                    Alliance.BLUE;
        }

        if (containsWord(combined, "audience")) {

            side =
                    HiveSide.AUDIENCE;

        } else if (containsWord(combined, "far")
                || combined.contains("opposite audience")
                || combined.contains("opposite_audience")
                || combined.contains("rear")) {

            side =
                    HiveSide.FAR;
        }

        if (alliance == null || side == null) {
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
                .replace('_', ' ');
    }

    private static boolean containsWord(
            String text,
            String word
    ) {

        if (text == null || word == null) {
            return false;
        }

        String paddedText =
                " " + text + " ";

        String paddedWord =
                " " + word.toLowerCase(Locale.US) + " ";

        return paddedText.contains(paddedWord);
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

            if (getHiveCell(detection) != null) {
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

        for (AprilTagClusterDetection detection
                : getHiveDetections()) {

            HiveCell cell =
                    getHiveCell(detection);

            if (cell != null
                    && cell.alliance == alliance
                    && detection.ftcPose != null) {

                return true;
            }
        }

        /*
         * Identity-only fallback.
         *
         * This confirms member visibility but does not provide a cluster
         * pose for turret fusion.
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
    // Active HIVE cluster
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
                    getHiveCell(detection);

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
                    getHiveCell(detection);

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
                    getHiveCell(detection);

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
                readRawState(alliance);

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

        if (candidateStateFrames >= requiredFrames
                && candidateState != HiveState.UNKNOWN) {

            confirmedState =
                    candidateState;
        }

        return confirmedState;
    }

    public HiveState getHiveState() {

        return confirmedState;
    }

    // ---------------------------------------------------------------------
    // Latched HIVE tip detection
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
                    getHiveCell(detection);

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

        if (hiveTipCandidateFrames >= requiredFrames) {

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

    public void updateHive(
            Alliance alliance
    ) {

        updateHiveState(alliance);
        updateHiveTipDetected(alliance);
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
    // Enable/disable
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
    // Camera status/control
    // ---------------------------------------------------------------------

    public boolean isStreaming() {

        return visionPortal != null
                && visionPortal.getCameraState()
                == VisionPortal.CameraState.STREAMING;
    }

    public void stopStreaming() {

        if (visionPortal != null) {
            visionPortal.stopStreaming();
        }
    }

    public void resumeStreaming() {

        if (visionPortal != null) {
            visionPortal.resumeStreaming();
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