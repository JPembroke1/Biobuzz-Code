package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Turret {
    public Servo turretServo;

    // Calibrate these: the raw servo positions that physically put the
    // turret at the right side and the left side of the robot.
    private double rightServoPos = 0.0;
    private double leftServoPos  = 1.0;

    private boolean swapped = false;   // true flips: 0 = left, 1 = right
    private double target = 0.5;       // 0 = right, 0.5 = back, 1 = left

    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
    }

    /** Set target as 0..1 (0 = right, 0.5 = back, 1 = left). */
    public void setTarget(double t) {
        target = Range.clip(t, 0.0, 1.0);
    }

    /** Set target as degrees: 0 = right, 90 = back, 180 = left. */
    public void setTargetDegrees(double deg) {
        setTarget(deg / 180.0);
    }

    public void nudge(double delta) {
        setTarget(target + delta);
    }

    public void setSwapped(boolean swapped) { this.swapped = swapped; }
    public void toggleSwapped()             { swapped = !swapped; }
    public boolean isSwapped()              { return swapped; }

    public double getTarget()        { return target; }
    public double getTargetDegrees() { return target * 180.0; }

    /** Call every loop. */
    public void update() {
        double t = swapped ? 1.0 - target : target;
        double pos = rightServoPos + t * (leftServoPos - rightServoPos);
        turretServo.setPosition(Range.clip(pos, 0.0, 1.0));
    }

    /**
     * Aim the turret at a field point.
     * Assumes standard math convention: heading and angles in radians,
     * counter-clockwise positive, 0 = facing +X.
     *
     * @param robotX, robotY   robot position on the field
     * @param robotHeading     robot heading (radians, CCW from +X)
     * @param targetX, targetY field point to aim at
     */
    public void aimAt(double robotX, double robotY, double robotHeading,
                      double targetX, double targetY) {
        // Angle from robot to target in field frame
        double fieldAngle = Math.atan2(targetY - robotY, targetX - robotX);

        // Angle relative to robot: 0 = forward, +90° = left, -90° = right, 180° = back
        double rel = Math.toDegrees(fieldAngle - robotHeading);

        // Turret frame: 0° = right (rel -90), 90° = back (rel 180), 180° = left (rel +90)
        double turretDeg = (-rel - 90.0) % 360.0;
        if (turretDeg < 0) turretDeg += 360.0;   // wrap into 0..360

        // 180..360 is the front half, which the turret can't reach.
        // Clamp to whichever side is closer (270 is straight forward).
        if (turretDeg > 180.0) {
            turretDeg = (turretDeg >= 270.0) ? 0.0 : 180.0;
        }

        setTargetDegrees(turretDeg);
    }

    /** Use to calibrate endpoints. */
    public void setRange(double rightPos, double leftPos) {
        rightServoPos = rightPos;
        leftServoPos = leftPos;
    }
}