package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {
    public static double FRONT_INTAKE_SPEED = 0.9;
    public static double BACK_INTAKE_SPEED = 1;

    public static double FRONT_FEED_SPEED = 0.35;
    public static double BACK_FEED_SPEED = 1;
    public static double TURRET_POSITION = 0.5;

    public static double DRIVE_SPEED = 1;

    // ---------- Shooter ----------
    public static double OUTTAKE_TPS = 1880;
    // Idle speed as a fraction of OUTTAKE_TPS (0 = no idle spin).
    public static double IDLE_FRACTION = 0.5;
    // "At Speed" telemetry tolerance for the shooter and feeder, in ticks per second.
    public static double AT_SPEED_TOLERANCE_TPS = 50;

    // ---------- Hood ----------
    // Base position (0..1). Outtake adds its flywheel-velocity compensation on top of this.
    public static double HOOD_POSITION = 0.65;

    // ---------- Feeder ----------
    // Feeder speed in ticks per second while feeding (6000 rpm motor = 2800 TPS max).
    public static double FEEDER_TPS = 2800;
    // If true, the feeder motor only runs in SHOOT mode within FEED_SPEED_TOLERANCE_TPS of target.
    public static boolean FEED_REQUIRES_SPEED = false;
    public static double FEED_SPEED_TOLERANCE_TPS = 100;

    // ---------- Field and HIVE geometry (Pedro inches, field centre = 72, 72) ----------
    // The two HIVES share an x and sit either side of the centre line in y. The up CELL opens
    // outward along x, so the aim point is the CELL mouth at one end of the HIVE.
    // HIVE_Y_OFFSET and CELL_X_OFFSET come from a third-party reading of the manual figures.
    // Check both with a tape measure.
    public static double FIELD_SIZE_IN = 144;
    public static double FIELD_CENTER = 72;
    public static double HIVE_Y_OFFSET = 12.75;  // half of the 25.5 in HIVE centre-to-centre
    public static double CELL_X_OFFSET = 24.0;   // aim point distance from the HIVE centre along x (estimated)
    // PLACEHOLDER: true = the blue HIVE is the one at the lower y. Flip this if the turret aims at the wrong HIVE.
    public static boolean BLUE_IS_LOW_Y = true;

    // ---------- Relocalising against a wall ----------
    // Robot centre ends up this far from the wall: ROBOT_LENGTH_MM / 2.
    public static double ROBOT_LENGTH_MM = 280;
    // true = the robot's back is against the wall and it faces the HIVE. false = front against the wall.
    public static boolean RESET_BACK_AGAINST_WALL = false;

    // ---------- Block servo ----------
    public static double blockClosePos = 0;
    public static double blockOpenPos = 0.35;

    // Feed delay: the block must be open this long before the rollers and feeder start.
    // Note: `timer` is state, not a tuning value. IntakeTest adds 0.1 per loop, so the real delay
    // depends on loop rate. Tune delayAmount on the robot.
    public static double timer = 0;
    public static double delayAmount = 1;

    // ---------- Brakes ----------
    public static double rightLiftBrakesPos = 0.1;
    public static double rightLowerBrakePos = 0.25;
    public static double leftLiftBrakesPos = 0.1;
    public static double leftLowerBrakePos = 0.53;


}
