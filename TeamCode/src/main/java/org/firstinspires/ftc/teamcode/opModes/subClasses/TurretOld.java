package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import com.pedropathing.math.Pose;

@Configurable
public class TurretOld {

    // ─────────────────────────────────────────────────────────────────────────
    // Hardware
    // ─────────────────────────────────────────────────────────────────────────

    private Servo turretServo;

    public static String TURRET_SERVO_NAME = "turretServo";

    // ─────────────────────────────────────────────────────────────────────────
    // Mechanical Servo Calibration
    // ─────────────────────────────────────────────────────────────────────────

    public static double LEFT_SERVO_LIMIT = 0.0;
    public static double RIGHT_SERVO_LIMIT = 1.0;

    public static boolean SERVO_REVERSED = false;

    // ─────────────────────────────────────────────────────────────────────────
    // Aim Tuning
    //
    // AIM_OFFSET is still a servo-position trim.
    // Do not treat AIM_OFFSET as degrees or radians.
    // ─────────────────────────────────────────────────────────────────────────

    public static double AIM_OFFSET = 0.0;
    public static double AIM_OFFSET_BLUE = 0.0; //-0.08
    public static double AIM_OFFSET_RED = 0.0; //0.04
    public static double AIM_GAIN = 1.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Turret Angle Limits
    // ─────────────────────────────────────────────────────────────────────────

    private static final double MIN_TURRET_ANGLE = -Math.PI / 2.0;
    private static final double MAX_TURRET_ANGLE = Math.PI / 2.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Target
    // ─────────────────────────────────────────────────────────────────────────

    public static Pose targetPose = new Pose(0, 0, 0);

    // ─────────────────────────────────────────────────────────────────────────
    // Velocity Compensation
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean velocityCompensationActive = true;
    public static double VELOCITY_LEAD_GAIN = 0.01;

    // ─────────────────────────────────────────────────────────────────────────
    // Limelight Assist
    //
    // Default is OFF.
    //
    // Enable both:
    // Turret.USE_LIMELIGHT_ASSIST = true;
    // LimelightAssist.LIMELIGHT_ENABLED = true;
    //
    // If correction moves the wrong way:
    // LimelightAssist.CORRECTION_DIRECTION = -1.0;
    // ─────────────────────────────────────────────────────────────────────────

    public static boolean USE_LIMELIGHT_ASSIST = false;

    public static boolean UPDATE_LIMELIGHT_INSIDE_TURRET = true;
    public static boolean REQUIRE_LIMELIGHT_TARGET = true;
    public static boolean DISABLE_LIMELIGHT_WHEN_LOCKED = false;

    public static double LIMELIGHT_CORRECTION_GAIN = 1.0;
    public static double MAX_LIMELIGHT_CORRECTION_RAD = Math.toRadians(5.0);

    public static double VISION_SMOOTHING_ALPHA = 0.35;
    public static double MAX_VISION_STEP_RAD = Math.toRadians(1.5);

    public static boolean limelightAssistActive = false;
    public static boolean limelightTargetVisible = false;
    public static boolean limelightLocked = false;

    public static double lastLimelightTx = 0.0;
    public static double lastRawLimelightCorrectionDeg = 0.0;
    public static double lastAppliedLimelightCorrectionDeg = 0.0;

    private double previousVisionCorrectionRad = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Diagnostics
    // ─────────────────────────────────────────────────────────────────────────

    public static double lastRobotX = 0.0;
    public static double lastRobotY = 0.0;
    public static double lastRobotHeadingDeg = 0.0;

    public static double lastTargetX = 0.0;
    public static double lastTargetY = 0.0;

    public static double lastFieldTargetAngleDeg = 0.0;
    public static double lastRelativeAngleDeg = 0.0;
    public static double lastVelocityLeadX = 0.0;
    public static double lastVelocityLeadY = 0.0;
    public static double lastFinalAngleDeg = 0.0;
    public static double lastServoPosition = 0.0;

    // ─────────────────────────────────────────────────────────────────────────
    // Init
    // ─────────────────────────────────────────────────────────────────────────

    public void init(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);
        turretServo.setDirection(Servo.Direction.REVERSE);

        setForward();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Target Control
    // ─────────────────────────────────────────────────────────────────────────

    public void setTargetPose(Pose pose) {
        if (pose != null) {
            targetPose = pose;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Aim Turret
    //
    // Process:
    // 1. Calculate field angle from robot to goal
    // 2. Add simple velocity lead if enabled
    // 3. Convert field angle to robot-relative angle
    // 4. Normalise to -π to +π
    // 5. Apply AIM_GAIN
    // 6. Add optional Limelight angular correction
    // 7. Clamp to -90° to +90°
    // 8. Convert angle to 0.0 to 1.0 normalised position
    // 9. Apply servo reversal if needed
    // 10. Map to calibrated servo limits
    // 11. Apply AIM_OFFSET as servo-position trim
    // 12. Clamp to calibrated servo limits
    // ─────────────────────────────────────────────────────────────────────────

    public void aimTurret(Pose robotPose, double vx, double vy) {

        if (robotPose == null || turretServo == null) {
            return;
        }

        double dx = targetPose.x() - robotPose.x();
        double dy = targetPose.y() - robotPose.y();

        double leadX = velocityCompensationActive ? vx * VELOCITY_LEAD_GAIN : 0.0;
        double leadY = velocityCompensationActive ? vy * VELOCITY_LEAD_GAIN : 0.0;

        double targetAngle = Math.atan2(dy + leadY, dx + leadX);

        double relativeAngle = targetAngle - robotPose.heading();

        relativeAngle = normaliseRadians(relativeAngle);

        relativeAngle *= AIM_GAIN;

        relativeAngle = clamp(
                relativeAngle,
                MIN_TURRET_ANGLE,
                MAX_TURRET_ANGLE
        );

        double normalisedPosition = angleToNormalisedPosition(relativeAngle);

        if (SERVO_REVERSED) {
            normalisedPosition = 1.0 - normalisedPosition;
        }

        double servoPosition = mapNormalisedToServoLimits(normalisedPosition);

        /*if (PoseStorage.lastAlliance == PoseStorage.Alliance.BLUE) {
            AIM_OFFSET = AIM_OFFSET_BLUE;
        }

        if (PoseStorage.lastAlliance == PoseStorage.Alliance.RED) {
            AIM_OFFSET = AIM_OFFSET_RED;
        } */

        servoPosition += AIM_OFFSET;

        servoPosition = clampToServoLimits(servoPosition);

        turretServo.setPosition(servoPosition);

        lastRobotX = robotPose.x();
        lastRobotY = robotPose.y();
        lastRobotHeadingDeg = Math.toDegrees(robotPose.heading());

        lastTargetX = targetPose.x();
        lastTargetY = targetPose.y();

        lastVelocityLeadX = leadX;
        lastVelocityLeadY = leadY;

        lastFieldTargetAngleDeg = Math.toDegrees(targetAngle);
        lastRelativeAngleDeg = Math.toDegrees(relativeAngle);
        lastFinalAngleDeg = Math.toDegrees(relativeAngle);
        lastServoPosition = servoPosition;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Forward Position
    // ─────────────────────────────────────────────────────────────────────────

    public void setForward() {
        if (turretServo == null) {
            return;
        }
        turretServo.setPosition(getCalibratedCentrePosition());
        lastServoPosition = getCalibratedCentrePosition();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Calibration Helpers
    // ─────────────────────────────────────────────────────────────────────────

    public static double getCalibratedCentrePosition() {
        return (LEFT_SERVO_LIMIT + RIGHT_SERVO_LIMIT) / 2.0;
    }

    private double angleToNormalisedPosition(double angleRadians) {
        return (angleRadians - MIN_TURRET_ANGLE) /
                (MAX_TURRET_ANGLE - MIN_TURRET_ANGLE);
    }

    private double mapNormalisedToServoLimits(double normalisedPosition) {
        double minServo = Math.min(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);
        double maxServo = Math.max(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);

        return minServo + normalisedPosition * (maxServo - minServo);
    }

    private double clampToServoLimits(double value) {
        double minServo = Math.min(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);
        double maxServo = Math.max(LEFT_SERVO_LIMIT, RIGHT_SERVO_LIMIT);

        return clamp(value, minServo, maxServo);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Angle Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double normaliseRadians(double angleRadians) {
        while (angleRadians > Math.PI) {
            angleRadians -= 2.0 * Math.PI;
        }

        while (angleRadians < -Math.PI) {
            angleRadians += 2.0 * Math.PI;
        }

        return angleRadians;
    }

    // ─────────────────────────────────────────────────────────────────────────
    // General Utility
    // ─────────────────────────────────────────────────────────────────────────

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}


/*package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Turret {

    public Servo TurretServo;

    public static double turretPos;

    public void init(HardwareMap hardwareMap) {
        TurretServo = hardwareMap.get(Servo.class, "turretServo");
    }

    public void aimTurret(double position) {
        TurretServo.setPosition(position);
    }
} */