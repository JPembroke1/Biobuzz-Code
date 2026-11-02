package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Shooter with two mechanically linked motors.
 *
 * The master motor (front) is the only one used for velocity feedback.
 * The slave motor (back) simply receives the same power as the master.
 *
 * Velocity is low-pass filtered, and the feedforward term is scaled by
 * battery voltage so kF behaves the same on a full or a sagging battery.
 */
@Configurable
public class Outtake {

    // FTC Robot Configuration names
    public static String MASTER_MOTOR_NAME = "frontOuttake";
    public static String SLAVE_MOTOR_NAME = "backOuttake";

    // PF tuning values, using encoder ticks per second.
    // kF must be tuned at NOMINAL_VOLTAGE (see below).
    public static double kF = 0.00045;
    public static double kP = 0.0015;

    // Maximum motor power
    public static double MAX_POWER = 1.0;

    // Velocity filter: 1.0 = no filtering, smaller = smoother but laggier.
    public static double VELOCITY_FILTER_ALPHA = 0.3;

    // Voltage compensation
    public static boolean VOLTAGE_COMP_ENABLED = true;
    public static double NOMINAL_VOLTAGE = 12.0;   // battery voltage kF was tuned at
    public static double VOLTAGE_FILTER_ALPHA = 0.1;
    public static double MIN_VOLTAGE = 9.0;        // clamp to avoid huge scaling on bad reads
    public static double MAX_VOLTAGE = 14.5;

    private DcMotorEx master;
    private DcMotorEx slave;
    private VoltageSensor voltageSensor;

    public double targetTPS = 0.0;
    public double rawTPS = 0.0;
    public double currentTPS = 0.0;   // filtered
    public double errorTPS = 0.0;
    public double commandedPower = 0.0;

    private double filteredVoltage = 0.0;
    private double voltageScale = 1.0;

    private boolean filtersPrimed = false;

    public void init(HardwareMap hardwareMap) {
        master = hardwareMap.get(DcMotorEx.class, MASTER_MOTOR_NAME);
        slave = hardwareMap.get(DcMotorEx.class, SLAVE_MOTOR_NAME);
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        master.setDirection(DcMotorSimple.Direction.FORWARD);
        slave.setDirection(DcMotorSimple.Direction.FORWARD);

        master.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // We calculate power ourselves; the master encoder is still readable.
        master.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slave.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stopOuttake();
    }

    /** Sets the shooter target in encoder ticks per second. */
    public void setTargetTPS(double targetTPS) {
        this.targetTPS = Math.max(0.0, targetTPS);
    }

    /** Runs the PF controller. Call once per OpMode loop. */
    public void update() {
        updateVelocity();
        updateVoltageScale();

        if (targetTPS <= 0.0) {
            errorTPS = 0.0;
            commandedPower = 0.0;
        } else {
            errorTPS = targetTPS - currentTPS;
            commandedPower = (kF * targetTPS * voltageScale) + (kP * errorTPS);
            commandedPower = Math.max(0.0, Math.min(MAX_POWER, commandedPower));
        }

        // Slave always mirrors the master's commanded power.
        master.setPower(commandedPower);
        slave.setPower(commandedPower);
    }

    /** Compatibility method for existing OpModes. */
    public void updatePF() {
        update();
    }

    private void updateVelocity() {
        rawTPS = Math.abs(master.getVelocity());

        if (!filtersPrimed) {
            currentTPS = rawTPS;
        } else {
            currentTPS += VELOCITY_FILTER_ALPHA * (rawTPS - currentTPS);
        }
    }

    private void updateVoltageScale() {
        if (!VOLTAGE_COMP_ENABLED) {
            voltageScale = 1.0;
            filtersPrimed = true;
            return;
        }

        double v = voltageSensor.getVoltage();
        v = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, v));

        if (!filtersPrimed) {
            filteredVoltage = v;
        } else {
            filteredVoltage += VOLTAGE_FILTER_ALPHA * (v - filteredVoltage);
        }

        voltageScale = NOMINAL_VOLTAGE / filteredVoltage;
        filtersPrimed = true;
    }

    public void stopOuttake() {
        targetTPS = 0.0;
        rawTPS = 0.0;
        currentTPS = 0.0;
        errorTPS = 0.0;
        commandedPower = 0.0;
        filtersPrimed = false;

        if (master != null) master.setPower(0.0);
        if (slave != null) slave.setPower(0.0);
    }

    public boolean isAtSpeed(double toleranceTPS) {
        if (targetTPS <= 0.0) {
            return false;
        }
        return Math.abs(errorTPS) <= Math.abs(toleranceTPS);
    }

    public double getTargetTPS() {
        return targetTPS;
    }

    /** Filtered velocity used by the controller. */
    public double getCurrentTPS() {
        return currentTPS;
    }

    /** Unfiltered master velocity, for comparing against the filtered value. */
    public double getRawTPS() {
        return rawTPS;
    }

    public double getErrorTPS() {
        return errorTPS;
    }

    public double getCommandedPower() {
        return commandedPower;
    }

    public double getVoltage() {
        return filteredVoltage;
    }

    public double getVoltageScale() {
        return voltageScale;
    }

    public double getMasterPower() {
        return master.getPower();
    }

    public double getSlavePower() {
        return slave.getPower();
    }

    /** Diagnostic only: not used in control. Handy for spotting belt/gear slip. */
    public double getSlaveVelocityTPS() {
        return Math.abs(slave.getVelocity());
    }
}