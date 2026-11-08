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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import java.util.HashMap;
import java.util.Map;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Wraps an Arducam (or any UVC-compatible webcam) + AprilTagProcessor + VisionPortal,
 * with optional live streaming to the Panels dashboard, into one reusable subsystem class.
 *
 * Usage:
 *
 *   camera = new ArduCam(hardwareMap, "Webcam 1")
 *           .setResolution(640, 480);
 *   camera.init();
 *   camera.streamToPanels(); // optional — call after init() to preview in Panels
 *
 *   waitForStart();
 *   while (opModeIsActive()) {
 *       AprilTagDetection tag = camera.getDetection(DESIRED_TAG_ID);
 *       ...
 *   }
 *
 *   camera.close();
 */
public class ArduCam {

    /**
     * Bridges VisionPortal frames to the Panels dashboard (and to any other consumer of
     * CameraStreamSource, like FTC Dashboard). Keeps the latest frame as a Bitmap.
     */
    private static class PanelsStreamProcessor implements VisionProcessor, CameraStreamSource {
        private final AtomicReference<Bitmap> lastFrame =
                new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
            Utils.matToBitmap(frame, b);
            lastFrame.set(b);
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // No overlay drawing needed for a plain stream preview.
        }

        @Override
        public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
            continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
        }
    }

    public enum Alliance { RED, BLUE }
    public enum HiveSide { AUDIENCE, FAR }

    public enum HiveState { UP, DOWN, UNKNOWN }

    private static final int FRAMES_TO_CONFIRM = 5;
    private HiveState confirmedState = HiveState.UNKNOWN;
    private HiveState candidateState = HiveState.UNKNOWN;
    private int candidateCount = 0;

    public static class HiveCell {
        public final Alliance alliance;
        public final HiveSide side;
        public HiveCell(Alliance alliance, HiveSide side) {
            this.alliance = alliance;
            this.side = side;
        }
        @Override
        public String toString() {
            return alliance + "_" + side;
        }
    }

    // Fill these in from Figure 9-17 / AprilTagGameDatabase — each CELL's
    // cluster is 4 tag IDs, but any one of the 4 is enough to identify the cell.
    private static final Map<Integer, HiveCell> HIVE_CELL_TAGS = new HashMap<>();
    static {
        HIVE_CELL_TAGS.put(/* TAG_ID */ 30, new HiveCell(Alliance.RED,  HiveSide.FAR));
        HIVE_CELL_TAGS.put(/* TAG_ID */ 34, new HiveCell(Alliance.RED,  HiveSide.AUDIENCE));
        HIVE_CELL_TAGS.put(/* TAG_ID */ 38, new HiveCell(Alliance.BLUE, HiveSide.AUDIENCE));
        HIVE_CELL_TAGS.put(/* TAG_ID */ 42, new HiveCell(Alliance.BLUE, HiveSide.FAR));
        // repeat for the other 3 tag IDs in each cluster if you want redundancy
    }

    public List<AprilTagDetection> getHiveDetections() {
        List<AprilTagDetection> hiveDetections = new java.util.ArrayList<>();
        for (AprilTagDetection d : getDetections()) {
            if (d.metadata != null && HIVE_CELL_TAGS.containsKey(d.id)) {
                hiveDetections.add(d);
            }
        }
        return hiveDetections;
    }

    public HiveCell getActiveCell(Alliance alliance) {
        HiveCell best = null;
        double bestPitch = Double.MAX_VALUE;

        for (AprilTagDetection d : getHiveDetections()) {
            HiveCell cell = HIVE_CELL_TAGS.get(d.id);
            if (cell.alliance != alliance) continue;

            double pitch = Math.abs(d.ftcPose.pitch);
            if (pitch < UP_CELL_PITCH_THRESHOLD_DEG && pitch < bestPitch) {
                bestPitch = pitch;
                best = cell;
            }
        }
        return best;
    }

    public AprilTagDetection getActiveCellDetection(Alliance alliance) {
        for (AprilTagDetection d : getHiveDetections()) {
            HiveCell cell = HIVE_CELL_TAGS.get(d.id);
            if (cell.alliance == alliance
                    && Math.abs(d.ftcPose.pitch) < UP_CELL_PITCH_THRESHOLD_DEG) {
                return d;
            }
        }
        return null;
    }

    /** Raw single-frame read: UP, DOWN (hive tags seen but tilted), or UNKNOWN (no hive tags seen). */
    private HiveState readRawState(Alliance alliance) {
        boolean sawAny = false;
        for (AprilTagDetection d : getHiveDetections()) {
            HiveCell cell = HIVE_CELL_TAGS.get(d.id);
            if (cell.alliance != alliance) continue;
            sawAny = true;
            if (Math.abs(d.ftcPose.pitch) < UP_CELL_PITCH_THRESHOLD_DEG) {
                return HiveState.UP;
            }
        }
        return sawAny ? HiveState.DOWN : HiveState.UNKNOWN;
    }

    /** Call once per loop iteration. Returns a debounced state. */
    public HiveState updateHiveState(Alliance alliance) {
        HiveState raw = readRawState(alliance);
        if (raw == candidateState) {
            candidateCount++;
        } else {
            candidateState = raw;
            candidateCount = 1;
        }
        // UNKNOWN never overwrites a confirmed UP/DOWN, so a blocked view doesn't wipe your last good reading.
        if (candidateCount >= FRAMES_TO_CONFIRM && candidateState != HiveState.UNKNOWN) {
            confirmedState = candidateState;
        }
        return confirmedState;
    }

    public HiveState getHiveState() { return confirmedState; }

    private boolean enabled = true;

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, enabled);
        }
    }

    public boolean isEnabled() { return enabled; }

    /** Any visible hive tag for this alliance, for turret tracking regardless of up/down. */
    public AprilTagDetection getAnyHiveDetection(Alliance alliance) {
        AprilTagDetection best = null;
        for (AprilTagDetection d : getHiveDetections()) {
            if (HIVE_CELL_TAGS.get(d.id).alliance != alliance) continue;
            if (best == null || d.ftcPose.range < best.ftcPose.range) best = d;
        }
        return best;
    }

    private static final double UP_CELL_PITCH_THRESHOLD_DEG = 35.0;
    private final HardwareMap hardwareMap;
    private final String webcamName;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private PanelsStreamProcessor streamProcessor;

    // Populate via setLensIntrinsics() if your Arducam/resolution isn't in the
    // SDK's built-in calibration list and you want accurate ftcPose data.
    private Double fx, fy, cx, cy;

    private int resolutionWidth = 640;
    private int resolutionHeight = 480;

    // Many Arducam boards (especially global-shutter ones) don't support the SDK's
    // default YUY2 format at every resolution. MJPEG is more broadly supported.
    private VisionPortal.StreamFormat streamFormat = VisionPortal.StreamFormat.MJPEG;

    public ArduCam(HardwareMap hardwareMap, String webcamName) {
        this.hardwareMap = hardwareMap;
        this.webcamName = webcamName;
    }

    /** Supply your own calibration (from an OpenCV checkerboard calibration) for accurate pose data. */
    public ArduCam setLensIntrinsics(double fx, double fy, double cx, double cy) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        return this;
    }

    /** Sets the capture resolution. 640x480 is the safest default (guaranteed built-in calibration). */
    public ArduCam setResolution(int width, int height) {
        this.resolutionWidth = width;
        this.resolutionHeight = height;
        return this;
    }

    /**
     * Sets the USB stream format. Defaults to MJPEG here (broader Arducam support, lower
     * bandwidth). Switch to VisionPortal.StreamFormat.YUY2 only if your specific camera/resolution
     * combo works better uncompressed.
     */
    public ArduCam setStreamFormat(VisionPortal.StreamFormat format) {
        this.streamFormat = format;
        return this;
    }

    /** Builds and starts the AprilTag pipeline. Call once during OpMode init. */
    public void init() {
        AprilTagProcessor.Builder tagBuilder = new AprilTagProcessor.Builder()
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        if (fx != null) {
            tagBuilder.setLensIntrinsics(fx, fy, cx, cy);
        }
        aprilTag = tagBuilder.build();

        streamProcessor = new PanelsStreamProcessor();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .addProcessor(aprilTag)
                .addProcessor(streamProcessor)
                .setCameraResolution(new Size(resolutionWidth, resolutionHeight))
                .setStreamFormat(streamFormat)
                .build();
    }

    /** Starts pushing live frames to the Panels dashboard. Call after init(). */
    public void streamToPanels() {
        PanelsCameraStream.INSTANCE.startStream(streamProcessor, 100);
    }

    /** Stops pushing frames to Panels without tearing down the camera. */
    public void stopPanelsStream() {
        PanelsCameraStream.INSTANCE.stopStream();
    }

    /** All tags visible in the current frame. */
    public List<AprilTagDetection> getDetections() {
        if (!enabled) return new java.util.ArrayList<>();
        return aprilTag.getDetections();
    }

    /** First detection matching a specific tag ID, or null if that tag isn't currently visible. */
    public AprilTagDetection getDetection(int tagId) {
        for (AprilTagDetection d : getDetections()) {
            if (d.id == tagId && d.metadata != null) {
                return d;
            }
        }
        return null;
    }

    /** True once the camera stream is actually up and running (useful for a "camera ready" check). */
    public boolean isStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Pauses the camera stream to free up USB bandwidth/CPU without tearing everything down. */
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

    /** Call from your OpMode's stop() (or after runOpMode ends) to release the camera cleanly. */
    public void close() {
        stopPanelsStream();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

