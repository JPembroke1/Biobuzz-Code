package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Outtake: one shooter motor, a separate feeder motor, and a hood servo.
 *
 * Shooter and feeder each run their own PF velocity loop (ticks per second) with
 * their own kF/kP, so they tune independently. Both read the same filtered
 * battery voltage, and the feedforward term is scaled by it.
 *
 * The hood servo holds a base position you set. Optional velocity compensation
 * nudges it based on how far the flywheel is from its target speed.
 *
 * Bare 6000 rpm motor, 28 ticks/rev: 6000 rpm = 2800 TPS (1000 TPS = about 2143 rpm).
 */
@Configurable
public class Outtake {

    // FTC Robot Configuration names
    public static String SHOOTER_MOTOR_NAME = "shooterMotor1";
    public static String FEEDER_MOTOR_NAME = "shooterMotor";
    public static String HOOD_SERVO_NAME = "hoodServo";

    // Directions (shooter carried over from the old master motor; flip if wrong)
    public static boolean SHOOTER_REVERSED = false;
    public static boolean FEEDER_REVERSED = false;

    // Shooter PF, ticks per second. kF must be tuned at NOMINAL_VOLTAGE.
    public static double kF = 0.00045;
    public static double kP = 0.01;

    // Feeder PF, tuned separately from the shooter.
    public static double FEEDER_kF = 0.00045;
    public static double FEEDER_kP = 0.004;

    // Maximum motor power
    public static double MAX_POWER = 1.0;

    // Velocity filter: 1.0 = no filtering, smaller = smoother but laggier.
    public static double VELOCITY_FILTER_ALPHA = 0.15;

    // Voltage compensation
    public static boolean VOLTAGE_COMP_ENABLED = true;
    public static double NOMINAL_VOLTAGE = 12.0;   // battery voltage kF was tuned at
    public static double VOLTAGE_FILTER_ALPHA = 0.1;
    public static double MIN_VOLTAGE = 9.0;        // clamp to avoid huge scaling on bad reads
    public static double MAX_VOLTAGE = 14.5;

    // Hood servo
    public static double HOOD_HOME = 0.5;          // position at init
    public static double HOOD_MIN = 0.4;
    public static double HOOD_MAX = 0.8;

    // Hood velocity compensation:
    //   correction = HOOD_COMP_GAIN * (currentTPS - targetTPS), minus a deadband, clamped to +/- HOOD_COMP_MAX
    // Negative error (flywheel slow) gives a correction of the opposite sign to the gain.
    // Gain 0 = no effect. Tune the sign and size by shooting with the flywheel deliberately low.
    public static boolean HOOD_COMP_ENABLED = true;
    public static double HOOD_COMP_GAIN = 0.035;     // servo units per TPS of error
    public static double HOOD_COMP_DEADBAND_TPS = 30.0;
    public static double HOOD_COMP_MAX = 0.10;     // largest correction either way
    public static double HOOD_MIN_STEP = 0.003;    // skip tiny servo updates to avoid jitter

    /** One PF velocity loop on one motor. */
    private static class Loop {
        final DcMotorEx motor;
        double targetTPS = 0.0;   // signed; negative runs the motor backwards
        double rawTPS = 0.0;
        double currentTPS = 0.0;  // filtered, always >= 0
        double errorTPS = 0.0;
        double power = 0.0;
        boolean primed = false;

        Loop(DcMotorEx motor) {
            this.motor = motor;
        }

        void update(double kF, double kP, double voltageScale) {
            rawTPS = Math.abs(motor.getVelocity());
            if (!primed) {
                currentTPS = rawTPS;
                primed = true;
            } else {
                currentTPS += VELOCITY_FILTER_ALPHA * (rawTPS - currentTPS);
            }

            double wanted = Math.abs(targetTPS);
            if (wanted <= 0.0) {
                errorTPS = 0.0;
                power = 0.0;
            } else {
                errorTPS = wanted - currentTPS;
                double p = (kF * wanted * voltageScale) + (kP * errorTPS);
                p = Math.max(0.0, Math.min(MAX_POWER, p));
                power = Math.signum(targetTPS) * p;
            }
            motor.setPower(power);
        }

        void stop() {
            targetTPS = 0.0;
            rawTPS = 0.0;
            currentTPS = 0.0;
            errorTPS = 0.0;
            power = 0.0;
            primed = false;
            motor.setPower(0.0);
        }

        boolean atSpeed(double tolerance) {
            return targetTPS != 0.0 && Math.abs(errorTPS) <= Math.abs(tolerance);
        }
    }

    private Loop shooter;
    private Loop feeder;
    private Servo hood;
    private VoltageSensor voltageSensor;

    private double filteredVoltage = 0.0;
    private double voltageScale = 1.0;
    private boolean voltagePrimed = false;

    private double hoodBase = HOOD_HOME;
    private double hoodCompensation = 0.0;
    private double hoodCommanded = HOOD_HOME;

    public void init(HardwareMap hardwareMap) {
        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);
        DcMotorEx feederMotor = hardwareMap.get(DcMotorEx.class, FEEDER_MOTOR_NAME);
        hood = hardwareMap.get(Servo.class, HOOD_SERVO_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooterMotor.setDirection(SHOOTER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feederMotor.setDirection(FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // We calculate power ourselves; the encoders are still readable.
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feederMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter = new Loop(shooterMotor);
        feeder = new Loop(feederMotor);

        hoodBase = HOOD_HOME;
        hoodCommanded = clampHood(hoodBase);
        hood.setPosition(hoodCommanded);

        stopOuttake();
    }

    // ---------- Commands ----------

    /** Sets the shooter target in encoder ticks per second. */
    public void setTargetTPS(double targetTPS) {
        shooter.targetTPS = Math.max(0.0, targetTPS);
    }

    /** Sets the feeder target in ticks per second. Negative runs it backwards (unjam). */
    public void setFeederTPS(double tps) {
        feeder.targetTPS = tps;
    }

    public void stopFeeder() {
        feeder.stop();
    }

    /** Sets the hood base position (0..1). Velocity compensation is added on top of this. */
    public void setHoodPosition(double position) {
        hoodBase = clampHood(position);
    }

    /** Stops the shooter and feeder. The hood stays where it is. */
    public void stopOuttake() {
        if (shooter != null) shooter.stop();
        if (feeder != null) feeder.stop();
        voltagePrimed = false;
    }

    // ---------- Update ----------

    /** Runs the shooter, feeder and hood. Call once per OpMode loop. */
    public void update() {
        updateVoltageScale();
        shooter.update(kF, kP, voltageScale);
        feeder.update(FEEDER_kF, FEEDER_kP, voltageScale);
        updateHood();
    }

    /** Compatibility method for existing OpModes. */
    public void updatePF() {
        update();
    }

    private void updateHood() {
        hoodCompensation = 0.0;
        if (HOOD_COMP_ENABLED && shooter.targetTPS > 0.0) {
            double error = shooter.currentTPS - shooter.targetTPS; // negative = flywheel slow
            double beyondDeadband = Math.abs(error) - HOOD_COMP_DEADBAND_TPS;
            if (beyondDeadband > 0.0) {
                hoodCompensation = HOOD_COMP_GAIN * Math.signum(error) * beyondDeadband;
                hoodCompensation = Math.max(-HOOD_COMP_MAX, Math.min(HOOD_COMP_MAX, hoodCompensation));
            }
        }

        double position = clampHood(hoodBase + hoodCompensation);
        if (Math.abs(position - hoodCommanded) >= HOOD_MIN_STEP) {
            hood.setPosition(position);
            hoodCommanded = position;
        }
    }

    private double clampHood(double position) {
        return Math.max(HOOD_MIN, Math.min(HOOD_MAX, position));
    }

    private void updateVoltageScale() {
        if (!VOLTAGE_COMP_ENABLED) {
            voltageScale = 1.0;
            voltagePrimed = true;
            return;
        }

        double v = voltageSensor.getVoltage();
        v = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, v));

        if (!voltagePrimed) {
            filteredVoltage = v;
        } else {
            filteredVoltage += VOLTAGE_FILTER_ALPHA * (v - filteredVoltage);
        }

        voltageScale = NOMINAL_VOLTAGE / filteredVoltage;
        voltagePrimed = true;
    }

    // ---------- Status: shooter ----------

    public boolean isAtSpeed(double toleranceTPS) {
        return shooter.atSpeed(toleranceTPS);
    }

    public double getTargetTPS() {
        return shooter.targetTPS;
    }

    /** Filtered velocity used by the controller. */
    public double getCurrentTPS() {
        return shooter.currentTPS;
    }

    /** Unfiltered shooter velocity, for comparing against the filtered value. */
    public double getRawTPS() {
        return shooter.rawTPS;
    }

    public double getErrorTPS() {
        return shooter.errorTPS;
    }

    public double getCommandedPower() {
        return shooter.power;
    }

    // ---------- Status: feeder ----------

    public boolean isFeederAtSpeed(double toleranceTPS) {
        return feeder.atSpeed(toleranceTPS);
    }

    public double getFeederTargetTPS() {
        return feeder.targetTPS;
    }

    public double getFeederCurrentTPS() {
        return feeder.currentTPS;
    }

    public double getFeederCommandedPower() {
        return feeder.power;
    }

    // ---------- Status: hood and voltage ----------

    /** Position last sent to the servo (base plus compensation). */
    public double getHoodPosition() {
        return hoodCommanded;
    }

    public double getHoodBase() {
        return hoodBase;
    }

    public double getHoodCompensation() {
        return hoodCompensation;
    }

    public double getVoltage() {
        return filteredVoltage;
    }

    public double getVoltageScale() {
        return voltageScale;
    }
}