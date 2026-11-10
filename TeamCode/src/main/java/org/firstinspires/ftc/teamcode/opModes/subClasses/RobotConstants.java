package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class RobotConstants {

    // -------------------------------------------------------------------------
    // Intake
    // -------------------------------------------------------------------------

    public static double FRONT_INTAKE_SPEED = 0.9;
    public static double BACK_INTAKE_SPEED = 0.5;

    public static double FRONT_FEED_SPEED = 1.0;
    public static double BACK_FEED_SPEED = 1.0;


    // -------------------------------------------------------------------------
    // Turret
    // -------------------------------------------------------------------------

    public static double TURRET_POSITION = 0.5;


    // -------------------------------------------------------------------------
    // Drivetrain
    // -------------------------------------------------------------------------

    public static double DRIVE_SPEED = 1.0;


    // -------------------------------------------------------------------------
    // Shooter
    // -------------------------------------------------------------------------

    public static double OUTTAKE_TPS = 1950.0;


    // -------------------------------------------------------------------------
    // Hood
    // -------------------------------------------------------------------------

    public static double HOOD_POSITION = 0.60;


    // -------------------------------------------------------------------------
    // Feeder
    // -------------------------------------------------------------------------

    public static double FEEDER_TPS = 2800.0;

    public static boolean FEED_REQUIRES_SPEED = false;

    public static double FEED_SPEED_TOLERANCE_TPS = 100.0;


    // -------------------------------------------------------------------------
    // Field and HIVE geometry
    //
    // Pedro coordinates are in inches.
    // Field centre = (72, 72).
    // -------------------------------------------------------------------------

    public static double FIELD_SIZE_IN = 144.0;
    public static double FIELD_CENTER = 72.0;

    public static double HIVE_Y_OFFSET = 12.75;

    public static double CELL_X_OFFSET = 24.0;

    // true = blue HIVE is on the lower-Y side.
    public static boolean BLUE_IS_LOW_Y = true;


    // -------------------------------------------------------------------------
    // Relocalising against a wall
    // -------------------------------------------------------------------------

    public static double ROBOT_LENGTH_MM = 320.0;

    // true = robot back against wall
    // false = robot front against wall
    public static boolean RESET_BACK_AGAINST_WALL = false;


    // -------------------------------------------------------------------------
    // Block servo
    // -------------------------------------------------------------------------

    public static double blockClosePos = 0.0;
    public static double blockOpenPos = 0.35;


    // -------------------------------------------------------------------------
    // Feed timing
    // -------------------------------------------------------------------------

    public static double timer = 0.0;
    public static double delayAmount = 0.3;


    // -------------------------------------------------------------------------
    // Flowers
    // -------------------------------------------------------------------------

    public static double flowerLiftPos = 0.0;
    public static double flowerDropPos = 0.35;


    // -------------------------------------------------------------------------
    // Toggle state
    // -------------------------------------------------------------------------

    public static boolean lastToggle = false;
}