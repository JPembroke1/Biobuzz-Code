package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/**
 * Localizer check for Pinpoint / Pedro.
 *
 * This OpMode does NOT drive to targets. It only reports where the
 * localizer thinks the robot is, so you can compare that with tape-measure
 * readings taken on the real field. A localizer cannot be validated by
 * driving to coordinates using itself, because the robot will always agree
 * with its own pose.
 *
 * What it does:
 *
 *  - Corner resets use the same maths as CombinedTeleOp, so the pose set
 *    here is exactly what the match code sets.
 *  - After every reset (or MARK press) it reports the distance, X/Y change
 *    and heading change since that point.
 *  - It tracks total rotation (unwrapped), so a number of full spins can
 *    be checked against 360 degrees each.
 *  - It shows the pose-implied distance from the robot's reference point
 *    to each field wall, for comparing with a tape measure.
 *
 * Controls (gamepad 1):
 *
 *   Cross     reset to RED   LOWER corner
 *   Circle    reset to RED   UPPER corner
 *   Square    reset to BLUE  LOWER corner
 *   Triangle  reset to BLUE  UPPER corner
 *   Left bumper   MARK: zero the "since mark" readings here
 *   Left stick / right stick   drive (slow), same signs as the TeleOp
 */
@Configurable
@TeleOp(name = "Localizer Check", group = "Test")
public class LocalizerCheck extends OpMode {

    // Keep these identical to CombinedTeleOp.
    public static double FIELD_SIZE_IN = 141.5;
    public static double ROBOT_LENGTH_MM = 380.0;
    public static double ROBOT_WIDTH_MM = 308.0;

    public static double RED_LOWER_RESET_HEADING_DEG = 270.0;
    public static double RED_UPPER_RESET_HEADING_DEG = 90.0;
    public static double BLUE_LOWER_RESET_HEADING_DEG = 270.0;
    public static double BLUE_UPPER_RESET_HEADING_DEG = 90.0;

    public static double FORWARD_SIGN = -1.0;
    public static double STRAFE_SIGN = -1.0;
    public static double TURN_SIGN = -1.0;

    /*
     * Slow on purpose, so distance and rotation tests are controllable.
     */
    public static double DRIVE_SCALE = 0.4;

    private Follower follower;
    private List<LynxModule> hubs;

    private Pose markPose = null;

    private double lastHeadingDeg = Double.NaN;
    private double rotationSinceMarkDeg = 0.0;

    private String lastResetLabel = "none";
    private Pose lastResetPose = null;

    private String status = "Square the robot in a corner, then press a corner button";

    private boolean prevCross, prevCircle, prevSquare, prevTriangle, prevLb;

    @Override
    public void init() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        follower = Constants.create(hardwareMap);

        displayTelemetry();
    }

    @Override
    public void start() {
        prevCross = gamepad1.cross;
        prevCircle = gamepad1.circle;
        prevSquare = gamepad1.square;
        prevTriangle = gamepad1.triangle;
        prevLb = gamepad1.left_bumper;
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        handleButtons();
        drive();

        Pose pose = follower.pose();

        if (validPose(pose)) {
            trackRotation(Math.toDegrees(pose.heading()));
        }

        displayTelemetry();
    }

    // ---------------------------------------------------------------------
    // Buttons
    // ---------------------------------------------------------------------

    private void handleButtons() {
        boolean cross = gamepad1.cross;
        boolean circle = gamepad1.circle;
        boolean square = gamepad1.square;
        boolean triangle = gamepad1.triangle;
        boolean lb = gamepad1.left_bumper;

        if (cross && !prevCross) resetAtCorner(true, false);
        if (circle && !prevCircle) resetAtCorner(true, true);
        if (square && !prevSquare) resetAtCorner(false, false);
        if (triangle && !prevTriangle) resetAtCorner(false, true);
        if (lb && !prevLb) mark("Marked current pose");

        prevCross = cross;
        prevCircle = circle;
        prevSquare = square;
        prevTriangle = triangle;
        prevLb = lb;
    }

    /**
     * Same maths as CombinedTeleOp.resetAtAllianceCorner.
     */
    private void resetAtCorner(boolean red, boolean upper) {
        double halfLength = ROBOT_LENGTH_MM / 25.4 / 2.0;
        double halfWidth = ROBOT_WIDTH_MM / 25.4 / 2.0;

        double heading = red
                ? (upper ? RED_UPPER_RESET_HEADING_DEG : RED_LOWER_RESET_HEADING_DEG)
                : (upper ? BLUE_UPPER_RESET_HEADING_DEG : BLUE_LOWER_RESET_HEADING_DEG);

        double x = red ? halfWidth : FIELD_SIZE_IN - halfWidth;
        double y = upper ? FIELD_SIZE_IN - halfLength : halfLength;

        Pose reset = new Pose(x, y, Math.toRadians(heading));

        follower.manual(new DrivePowers(0.0, 0.0, 0.0));
        follower.setPose(reset);

        lastResetPose = reset;
        lastResetLabel = (red ? "RED " : "BLUE ") + (upper ? "UPPER" : "LOWER");

        lastHeadingDeg = Double.NaN;

        mark("Reset at " + lastResetLabel + " corner");
    }

    private void mark(String message) {
        Pose pose = follower.pose();

        markPose = validPose(pose)
                ? new Pose(pose.x(), pose.y(), pose.heading())
                : null;

        rotationSinceMarkDeg = 0.0;
        lastHeadingDeg = validPose(pose)
                ? Math.toDegrees(pose.heading())
                : Double.NaN;

        status = message;
    }

    // ---------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------

    private void drive() {
        double forward = FORWARD_SIGN * gamepad1.left_stick_y * DRIVE_SCALE;
        double strafe = STRAFE_SIGN * gamepad1.left_stick_x * DRIVE_SCALE;
        double turn = TURN_SIGN * gamepad1.right_stick_x * DRIVE_SCALE;

        follower.manual(new DrivePowers(forward, strafe, turn));
        follower.update();
    }

    // ---------------------------------------------------------------------
    // Rotation tracking (unwrapped)
    // ---------------------------------------------------------------------

    private void trackRotation(double headingDeg) {
        if (Double.isFinite(lastHeadingDeg)) {
            rotationSinceMarkDeg += wrapDegrees(headingDeg - lastHeadingDeg);
        }

        lastHeadingDeg = headingDeg;
    }

    // ---------------------------------------------------------------------
    // Telemetry
    // ---------------------------------------------------------------------

    private void displayTelemetry() {
        Pose pose = follower == null ? null : follower.pose();

        telemetry.addLine("=== LOCALIZER CHECK ===");
        telemetry.addData("Status", status);

        telemetry.addLine("--- Pose now ---");

        if (validPose(pose)) {
            telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
            telemetry.addData("Heading (deg)", "%.2f",
                    normalize0To360(Math.toDegrees(pose.heading())));

            telemetry.addLine("--- Pose-implied distance to walls (in) ---");
            telemetry.addData("From x=0 wall", "%.2f", pose.x());
            telemetry.addData("From x=max wall", "%.2f", FIELD_SIZE_IN - pose.x());
            telemetry.addData("From y=0 wall", "%.2f", pose.y());
            telemetry.addData("From y=max wall", "%.2f", FIELD_SIZE_IN - pose.y());
        } else {
            telemetry.addData("Pose", "INVALID");
        }

        telemetry.addLine("--- Since mark / reset ---");

        if (validPose(pose) && markPose != null) {
            double dx = pose.x() - markPose.x();
            double dy = pose.y() - markPose.y();

            telemetry.addData("dX / dY (in)", "%.2f / %.2f", dx, dy);
            telemetry.addData("Distance (in)", "%.2f", Math.hypot(dx, dy));
            telemetry.addData("Heading change (deg)", "%.2f",
                    wrapDegrees(Math.toDegrees(pose.heading() - markPose.heading())));
            telemetry.addData("Total rotation (deg)", "%.1f", rotationSinceMarkDeg);
        } else {
            telemetry.addLine("No mark yet");
        }

        telemetry.addLine("--- Last corner reset ---");
        telemetry.addData("Corner", lastResetLabel);

        if (lastResetPose != null) {
            telemetry.addData("Set to", "(%.2f, %.2f, %.0f deg)",
                    lastResetPose.x(),
                    lastResetPose.y(),
                    normalize0To360(Math.toDegrees(lastResetPose.heading())));
        }

        telemetry.addLine("Cross RL | Circle RU | Square BL | Triangle BU | LB mark");
        telemetry.update();
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

    private static double wrapDegrees(double degrees) {
        double result = degrees % 360.0;

        if (result > 180.0) result -= 360.0;
        if (result < -180.0) result += 360.0;

        return result;
    }

    private static double normalize0To360(double degrees) {
        double result = degrees % 360.0;

        return result < 0.0 ? result + 360.0 : result;
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.manual(new DrivePowers(0.0, 0.0, 0.0));
            follower.update();
        }

        if (hubs != null) {
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        }
    }
}