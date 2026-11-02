package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class RobotHardware {

    public Servo blockServo;
    public Servo hoodServo;
    private Outtake outtake;

    public static double hoodSlope = 0.00365;
    public static double hoodIntercept = 0.8;
    public static double hoodVelocityFactor = 1.8;

    public static double hoodMin = 0.5;
    public static double hoodMax = 0.9;

    public static double hoodDeadband = 0.001;
    public static double hoodMaxStep = 0.04;

    public static boolean useDistanceSmoothing = false;
    public static double distanceAlpha = 0.4;

    public static double lastCommandedHood = 0.20;
    public static double filteredDistanceCM = 0.0;

    public static boolean hoodInitialised = false;
    public static boolean distanceFilterInitialised = false;

    public static double lastRawHoodTarget = 0.0;
    public static double lastClippedHoodTarget = 0.0;


    public RobotHardware(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");
        outtake = new Outtake();
        outtake.init(hardwareMap);
    }

    public void setHoodPosition(double targetPosition) {
        double clipped = clamp(targetPosition, hoodMin, hoodMax);

        lastRawHoodTarget = targetPosition;
        lastClippedHoodTarget = clipped;

        if (!hoodInitialised) {
            lastCommandedHood = clipped;
            hoodServo.setPosition(clipped);
            hoodInitialised = true;
            return;
        }

        if (Math.abs(clipped - lastCommandedHood) < hoodDeadband) {
            return;
        }

        double delta = clipped - lastCommandedHood;
        delta = clamp(delta, -hoodMaxStep, hoodMaxStep);

        double newPosition = lastCommandedHood + delta;
        newPosition = clamp(newPosition, hoodMin, hoodMax);

        hoodServo.setPosition(newPosition);
        lastCommandedHood = newPosition;
    }

    public void linearHoodRegression(double distanceCM) {
        double workingDistanceCM = filterDistance(distanceCM);
        double velocityComp = calculateVelocityCompensation(hoodVelocityFactor);

        double position =
                (hoodSlope * workingDistanceCM)
                        + hoodIntercept
                        + velocityComp;

        setHoodPosition(position);
    }

    private double calculateVelocityCompensation(double velocityFactor) {
        if (outtake.targetTPS <= 0.0) {
            return 0.0;
        }

        double tpsRatio = outtake.currentTPS / outtake.targetTPS;
        tpsRatio = clamp(tpsRatio, 0.0, 1.2);

        double velocityComp = (1.0 - tpsRatio) * velocityFactor;

        return Math.max(0.0, velocityComp);
    }

    private double filterDistance(double distanceCM) {
        if (!useDistanceSmoothing) {
            filteredDistanceCM = distanceCM;
            return distanceCM;
        }

        if (!distanceFilterInitialised) {
            filteredDistanceCM = distanceCM;
            distanceFilterInitialised = true;
        } else {
            filteredDistanceCM =
                    (distanceAlpha * distanceCM)
                            + ((1.0 - distanceAlpha) * filteredDistanceCM);
        }

        return filteredDistanceCM;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void updateHood(double position) {
        hoodServo.setPosition(position);
    }
    public void block() {
        blockServo.setPosition(RobotConstants.blockClosePos);
    }

    public void open() {
        blockServo.setPosition(RobotConstants.blockOpenPos);
    }


}
