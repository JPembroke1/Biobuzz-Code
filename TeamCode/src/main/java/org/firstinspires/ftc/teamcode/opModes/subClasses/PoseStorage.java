package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.pedropathing.math.Pose;

/**
 * In-memory handover between autonomous and TeleOp.
 *
 * Autonomous should:
 * 1. Set the alliance.
 * 2. Save the current pose during execution and before finishing.
 *
 * TeleOp should:
 * 1. Restore the saved pose during init().
 * 2. Lock the saved/selected alliance during start().
 */
public final class PoseStorage {

    private PoseStorage() {
        // Static utility class.
    }

    public enum Alliance {
        BLUE,
        RED,
        UNKNOWN
    }

    public static Pose currentPose = null;

    public static Alliance lastAlliance =
            Alliance.UNKNOWN;

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
     * Stores a snapshot of the supplied Pedro pose.
     *
     * Pedro heading remains in radians.
     */
    public static void setPose(Pose pose) {

        if (!isValidPose(pose)) {
            currentPose = null;
            return;
        }

        currentPose = copyPose(pose);
    }

    /**
     * Returns a copy of the stored pose.
     *
     * This prevents a caller from modifying the stored Pose object.
     */
    public static Pose getPose() {

        if (!isValidPose(currentPose)) {
            return null;
        }

        return copyPose(currentPose);
    }

    public static boolean hasPose() {
        return isValidPose(currentPose);
    }

    public static void clearPose() {
        currentPose = null;
    }

    // ---------------------------------------------------------------------
    // Clear all
    // ---------------------------------------------------------------------

    public static void clearAll() {

        currentPose = null;
        lastAlliance = Alliance.UNKNOWN;
    }

    // ---------------------------------------------------------------------
    // Validation
    // ---------------------------------------------------------------------

    private static boolean isValidPose(Pose pose) {

        return pose != null
                && Double.isFinite(pose.x())
                && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    private static Pose copyPose(Pose pose) {

        return new Pose(
                pose.x(),
                pose.y(),
                pose.heading()
        );
    }
}