package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.pedropathing.math.Pose;

/**
 * Shares alliance and pose between autonomous and TeleOp.
 *
 * Static storage is in-memory only.
 * TeleOp should always display and validate the stored values.
 */
public final class PoseStorage {

    private PoseStorage() {
        // Utility class: no instances required.
    }

    public enum Alliance {
        BLUE,
        RED,
        UNKNOWN
    }

    public static Pose currentPose = null;

    public static Alliance lastAlliance = Alliance.UNKNOWN;

    // ---------------------------------------------------------------------
    // Alliance
    // ---------------------------------------------------------------------

    public static void setBlue() {
        lastAlliance = Alliance.BLUE;
    }

    public static void setRed() {
        lastAlliance = Alliance.RED;
    }

    public static void setUnknown() {
        lastAlliance = Alliance.UNKNOWN;
    }

    public static boolean isBlue() {
        return lastAlliance == Alliance.BLUE;
    }

    public static boolean isRed() {
        return lastAlliance == Alliance.RED;
    }

    public static boolean isUnknown() {
        return lastAlliance == null
                || lastAlliance == Alliance.UNKNOWN;
    }

    // ---------------------------------------------------------------------
    // Pose
    // ---------------------------------------------------------------------

    /**
     * Stores a snapshot of the supplied pose.
     * Pose headings are retained in radians.
     *
     * Invalid input clears the saved pose.
     */
    public static void setPose(Pose pose) {

        if (!isValidPose(pose)) {
            currentPose = null;
            return;
        }

        currentPose = new Pose(
                pose.x(),
                pose.y(),
                pose.heading()
        );
    }

    /**
     * Returns a copy so callers do not modify the stored pose directly.
     */
    public static Pose getPose() {

        if (!isValidPose(currentPose)) {
            return null;
        }

        return new Pose(
                currentPose.x(),
                currentPose.y(),
                currentPose.heading()
        );
    }

    public static boolean hasPose() {
        return isValidPose(currentPose);
    }

    public static void clearPose() {
        currentPose = null;
    }

    private static boolean isValidPose(Pose pose) {

        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    // ---------------------------------------------------------------------
    // Clear all
    // ---------------------------------------------------------------------

    public static void clearAll() {
        currentPose = null;
        lastAlliance = Alliance.UNKNOWN;
    }
}