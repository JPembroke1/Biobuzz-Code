package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Outtake subsystem containing:
 *
 * 1. One shooter motor
 * 2. One feeder motor
 * 3. One hood servo
 *
 * The shooter and feeder use independent PF velocity controllers.
 * Motor velocity is measured in encoder ticks per second.
 *
 * The hood holds a configurable base position and can optionally apply
 * a small correction based on normalized shooter velocity error.
 */
@Configurable
public class Outtake {

    // -------------------------------------------------------------------------
    // FTC Robot Configuration names
    // -------------------------------------------------------------------------

    public static String SHOOTER_MOTOR_NAME =
            "shooterMotor1";

    public static String FEEDER_MOTOR_NAME =
            "shooterMotor";

    public static String HOOD_SERVO_NAME =
            "hoodServo";


    // -------------------------------------------------------------------------
    // Motor directions
    // -------------------------------------------------------------------------

    public static boolean SHOOTER_REVERSED = true;
    public static boolean FEEDER_REVERSED = true;


    // -------------------------------------------------------------------------
    // Shooter PF tuning
    // -------------------------------------------------------------------------
    // Units are encoder ticks per second.
    // -------------------------------------------------------------------------

    public static double kF = 0.00045;
    public static double kP = 0.03;


    // -------------------------------------------------------------------------
    // Feeder PF tuning
    // -------------------------------------------------------------------------

    public static double FEEDER_kF = 0.00045;
    public static double FEEDER_kP = 0.01;


    // -------------------------------------------------------------------------
    // Motor limits
    // -------------------------------------------------------------------------

    public static double MAX_POWER = 1.0;


    // -------------------------------------------------------------------------
    // Velocity filtering
    //
    // 1.0 means no filtering.
    // Smaller values provide more smoothing but add more delay.
    // -------------------------------------------------------------------------

    public static double VELOCITY_FILTER_ALPHA = 0.15;


    // -------------------------------------------------------------------------
    // Voltage compensation
    //
    // kF should be tuned at NOMINAL_VOLTAGE.
    // -------------------------------------------------------------------------

    public static boolean VOLTAGE_COMP_ENABLED = true;

    public static double NOMINAL_VOLTAGE = 13.2;

    public static double VOLTAGE_FILTER_ALPHA = 0.1;

    public static double MIN_VOLTAGE = 9.0;
    public static double MAX_VOLTAGE = 14.5;


    // -------------------------------------------------------------------------
    // Hood servo calibration
    // -------------------------------------------------------------------------

    public static double HOOD_HOME = 0.5;

    public static double HOOD_MIN = 0.4;
    public static double HOOD_MAX = 0.8;


    // -------------------------------------------------------------------------
    // Hood velocity compensation
    //
    // Compensation is based on normalized shooter velocity error:
    //
    // normalizedError =
    //     errorBeyondDeadband / targetTPS
    //
    // hoodCorrection =
    //     HOOD_COMP_GAIN * normalizedError
    //
    // This prevents a raw TPS error from immediately saturating the servo.
    // -------------------------------------------------------------------------

    public static boolean HOOD_COMP_ENABLED = true;

    /*
     * Servo correction applied for a normalized speed error of 1.0.
     *
     * Positive values decrease the hood position when the shooter is below
     * speed. Change this to a negative value if the physical hood needs to
     * move in the opposite direction.
     */
    public static double HOOD_COMP_GAIN = 0.05;

    /*
     * Ignore shooter velocity error inside this range.
     */
    public static double HOOD_COMP_DEADBAND_TPS = 50.0;

    /*
     * Maximum correction from the configured hood base position.
     */
    public static double HOOD_COMP_MAX = 0.08;

    /*
     * Skip very small servo changes to reduce jitter.
     */
    public static double HOOD_MIN_STEP = 0.002;


    // -------------------------------------------------------------------------
    // Individual PF motor loop
    // -------------------------------------------------------------------------

    private static class Loop {

        private final DcMotorEx motor;

        /*
         * Target may be signed.
         * A negative feeder target can be used for unjamming.
         */
        private double targetTPS = 0.0;

        private double rawTPS = 0.0;
        private double currentTPS = 0.0;
        private double errorTPS = 0.0;
        private double power = 0.0;

        private boolean primed = false;


        Loop(DcMotorEx motor) {
            this.motor = motor;
        }


        void update(
                double loopKF,
                double loopKP,
                double voltageScale
        ) {

            rawTPS =
                    Math.abs(
                            motor.getVelocity()
                    );

            /*
             * Prime the filter with the first valid reading so it does not
             * slowly rise from zero at startup.
             */
            if (!primed) {

                currentTPS = rawTPS;
                primed = true;

            } else {

                double alpha =
                        clampStatic(
                                VELOCITY_FILTER_ALPHA,
                                0.0,
                                1.0
                        );

                currentTPS +=
                        alpha
                                * (rawTPS - currentTPS);
            }

            double wantedTPS =
                    Math.abs(targetTPS);

            if (wantedTPS <= 0.0) {

                errorTPS = 0.0;
                power = 0.0;

                motor.setPower(0.0);
                return;
            }

            errorTPS =
                    wantedTPS - currentTPS;

            double calculatedPower =
                    loopKF
                            * wantedTPS
                            * voltageScale
                            + loopKP
                            * errorTPS;

            /*
             * Shooter/feeder magnitude is kept positive, then the requested
             * target sign determines motor direction.
             */
            calculatedPower =
                    clampStatic(
                            calculatedPower,
                            0.0,
                            MAX_POWER
                    );

            power =
                    Math.signum(targetTPS)
                            * calculatedPower;

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


        boolean atSpeed(double toleranceTPS) {

            return targetTPS != 0.0
                    && Math.abs(errorTPS)
                    <= Math.abs(toleranceTPS);
        }


        double getMotorPower() {
            return motor.getPower();
        }
    }


    // -------------------------------------------------------------------------
    // Hardware and controllers
    // -------------------------------------------------------------------------

    private Loop shooter;
    private Loop feeder;

    private Servo hood;
    private VoltageSensor voltageSensor;


    // -------------------------------------------------------------------------
    // Voltage state
    // -------------------------------------------------------------------------

    private double filteredVoltage = 0.0;
    private double voltageScale = 1.0;

    private boolean voltagePrimed = false;


    // -------------------------------------------------------------------------
    // Hood state
    // -------------------------------------------------------------------------

    private double hoodBase = HOOD_HOME;
    private double hoodCompensation = 0.0;
    private double hoodCommanded = HOOD_HOME;


    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        DcMotorEx shooterMotor =
                hardwareMap.get(
                        DcMotorEx.class,
                        SHOOTER_MOTOR_NAME
                );

        DcMotorEx feederMotor =
                hardwareMap.get(
                        DcMotorEx.class,
                        FEEDER_MOTOR_NAME
                );

        hood =
                hardwareMap.get(
                        Servo.class,
                        HOOD_SERVO_NAME
                );

        shooterMotor.setDirection(
                SHOOTER_REVERSED
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD
        );

        feederMotor.setDirection(
                FEEDER_REVERSED
                        ? DcMotorSimple.Direction.REVERSE
                        : DcMotorSimple.Direction.FORWARD
        );

        shooterMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        feederMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        /*
         * Motor power is calculated by this subsystem.
         * Encoder velocity remains available through getVelocity().
         */
        shooterMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        feederMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooter =
                new Loop(shooterMotor);

        feeder =
                new Loop(feederMotor);

        /*
         * Use the first available Control Hub or Expansion Hub voltage sensor.
         */
        voltageSensor = null;

        for (VoltageSensor sensor
                : hardwareMap.voltageSensor) {

            if (sensor != null) {

                voltageSensor = sensor;
                break;
            }
        }

        hoodBase =
                clampHood(HOOD_HOME);

        hoodCompensation = 0.0;
        hoodCommanded = hoodBase;

        hood.setPosition(
                hoodCommanded
        );

        stopOuttake();
    }


    // -------------------------------------------------------------------------
    // Shooter commands
    // -------------------------------------------------------------------------

    /**
     * Sets the shooter target in encoder ticks per second.
     */
    public void setTargetTPS(double targetTPS) {

        if (shooter == null) {
            return;
        }

        shooter.targetTPS =
                Math.max(
                        0.0,
                        targetTPS
                );
    }


    // -------------------------------------------------------------------------
    // Feeder commands
    // -------------------------------------------------------------------------

    /**
     * Sets the feeder target in encoder ticks per second.
     *
     * Positive runs in the configured forward direction.
     * Negative may be used for unjamming.
     */
    public void setFeederTPS(double targetTPS) {

        if (feeder == null) {
            return;
        }

        feeder.targetTPS =
                targetTPS;
    }


    public void stopFeeder() {

        if (feeder != null) {
            feeder.stop();
        }
    }


    // -------------------------------------------------------------------------
    // Hood commands
    // -------------------------------------------------------------------------

    /**
     * Sets the hood base position.
     *
     * Velocity compensation is added to this position during update().
     */
    public void setHoodPosition(double position) {

        hoodBase =
                clampHood(position);

        /*
         * If the shooter is stopped, apply the new base immediately.
         */
        if (shooter == null
                || shooter.targetTPS <= 0.0) {

            setHoodServoPosition(
                    hoodBase
            );
        }
    }


    /**
     * Returns the hood to its configured home position.
     */
    public void setHoodHome() {

        setHoodPosition(
                HOOD_HOME
        );
    }


    // -------------------------------------------------------------------------
    // Stop
    // -------------------------------------------------------------------------

    /**
     * Stops the shooter and feeder.
     *
     * The hood remains at its last commanded position.
     */
    public void stopOuttake() {

        if (shooter != null) {
            shooter.stop();
        }

        if (feeder != null) {
            feeder.stop();
        }

        voltagePrimed = false;
        voltageScale = 1.0;

        hoodCompensation = 0.0;
    }


    // -------------------------------------------------------------------------
    // Periodic update
    // -------------------------------------------------------------------------

    /**
     * Updates the shooter, feeder, voltage compensation and hood.
     *
     * Call this once during every OpMode loop.
     */
    public void update() {

        if (shooter == null
                || feeder == null) {

            return;
        }

        updateVoltageScale();

        shooter.update(
                kF,
                kP,
                voltageScale
        );

        feeder.update(
                FEEDER_kF,
                FEEDER_kP,
                voltageScale
        );

        updateHood();
    }


    /**
     * Compatibility method for existing OpModes.
     */
    public void updatePF() {
        update();
    }


    // -------------------------------------------------------------------------
    // Corrected hood compensation
    // -------------------------------------------------------------------------

    private void updateHood() {

        if (hood == null) {
            return;
        }

        hoodCompensation = 0.0;

        if (HOOD_COMP_ENABLED
                && shooter != null
                && shooter.targetTPS > 0.0) {

            double targetTPS =
                    Math.abs(
                            shooter.targetTPS
                    );

            double currentTPS =
                    shooter.currentTPS;

            /*
             * Positive error means shooter is above target.
             * Negative error means shooter is below target.
             */
            double errorTPS =
                    currentTPS - targetTPS;

            if (Math.abs(errorTPS)
                    > HOOD_COMP_DEADBAND_TPS) {

                /*
                 * Remove the deadband so compensation begins smoothly at zero.
                 */
                double errorBeyondDeadband =
                        Math.signum(errorTPS)
                                * (
                                Math.abs(errorTPS)
                                        - HOOD_COMP_DEADBAND_TPS
                        );

                /*
                 * Normalize to a fraction of the target speed.
                 *
                 * This prevents raw TPS values from creating excessive servo
                 * corrections.
                 */
                double normalizedError =
                        errorBeyondDeadband
                                / Math.max(
                                1.0,
                                targetTPS
                        );

                hoodCompensation =
                        HOOD_COMP_GAIN
                                * normalizedError;

                hoodCompensation =
                        clamp(
                                hoodCompensation,
                                -Math.abs(HOOD_COMP_MAX),
                                Math.abs(HOOD_COMP_MAX)
                        );
            }
        }

        double requestedPosition =
                clampHood(
                        hoodBase
                                + hoodCompensation
                );

        setHoodServoPosition(
                requestedPosition
        );
    }


    private void setHoodServoPosition(
            double position
    ) {

        if (hood == null) {
            return;
        }

        double clippedPosition =
                clampHood(position);

        if (Math.abs(
                clippedPosition
                        - hoodCommanded
        ) >= Math.abs(HOOD_MIN_STEP)) {

            hood.setPosition(
                    clippedPosition
            );

            hoodCommanded =
                    clippedPosition;
        }
    }


    private double clampHood(
            double position
    ) {

        double minimum =
                Math.min(
                        HOOD_MIN,
                        HOOD_MAX
                );

        double maximum =
                Math.max(
                        HOOD_MIN,
                        HOOD_MAX
                );

        return clamp(
                position,
                minimum,
                maximum
        );
    }


    // -------------------------------------------------------------------------
    // Voltage compensation
    // -------------------------------------------------------------------------

    private void updateVoltageScale() {

        if (!VOLTAGE_COMP_ENABLED
                || voltageSensor == null) {

            voltageScale = 1.0;
            voltagePrimed = true;

            return;
        }

        double measuredVoltage =
                voltageSensor.getVoltage();

        /*
         * Reject invalid sensor readings.
         */
        if (!Double.isFinite(measuredVoltage)
                || measuredVoltage <= 0.0) {

            voltageScale = 1.0;
            return;
        }

        measuredVoltage =
                clamp(
                        measuredVoltage,
                        MIN_VOLTAGE,
                        MAX_VOLTAGE
                );

        if (!voltagePrimed) {

            filteredVoltage =
                    measuredVoltage;

            voltagePrimed = true;

        } else {

            double alpha =
                    clamp(
                            VOLTAGE_FILTER_ALPHA,
                            0.0,
                            1.0
                    );

            filteredVoltage +=
                    alpha
                            * (
                            measuredVoltage
                                    - filteredVoltage
                    );
        }

        if (filteredVoltage <= 0.0) {

            voltageScale = 1.0;

        } else {

            voltageScale =
                    NOMINAL_VOLTAGE
                            / filteredVoltage;
        }
    }


    // -------------------------------------------------------------------------
    // Shooter status
    // -------------------------------------------------------------------------

    public boolean isAtSpeed(
            double toleranceTPS
    ) {

        return shooter != null
                && shooter.atSpeed(
                toleranceTPS
        );
    }


    public double getTargetTPS() {

        if (shooter == null) {
            return 0.0;
        }

        return shooter.targetTPS;
    }


    /**
     * Returns the filtered shooter velocity used by the PF controller.
     */
    public double getCurrentTPS() {

        if (shooter == null) {
            return 0.0;
        }

        return shooter.currentTPS;
    }


    /**
     * Returns unfiltered shooter encoder velocity.
     */
    public double getRawTPS() {

        if (shooter == null) {
            return 0.0;
        }

        return shooter.rawTPS;
    }


    public double getErrorTPS() {

        if (shooter == null) {
            return 0.0;
        }

        return shooter.errorTPS;
    }


    public double getCommandedPower() {

        if (shooter == null) {
            return 0.0;
        }

        return shooter.power;
    }


    // -------------------------------------------------------------------------
    // Feeder status
    // -------------------------------------------------------------------------

    public boolean isFeederAtSpeed(
            double toleranceTPS
    ) {

        return feeder != null
                && feeder.atSpeed(
                toleranceTPS
        );
    }


    public double getFeederTargetTPS() {

        if (feeder == null) {
            return 0.0;
        }

        return feeder.targetTPS;
    }


    public double getFeederCurrentTPS() {

        if (feeder == null) {
            return 0.0;
        }

        return feeder.currentTPS;
    }


    public double getFeederRawTPS() {

        if (feeder == null) {
            return 0.0;
        }

        return feeder.rawTPS;
    }


    public double getFeederErrorTPS() {

        if (feeder == null) {
            return 0.0;
        }

        return feeder.errorTPS;
    }


    public double getFeederCommandedPower() {

        if (feeder == null) {
            return 0.0;
        }

        return feeder.power;
    }


    // -------------------------------------------------------------------------
    // Hood status
    // -------------------------------------------------------------------------

    /**
     * Returns the last position sent to the hood servo.
     */
    public double getHoodPosition() {
        return hoodCommanded;
    }


    public double getHoodBase() {
        return hoodBase;
    }


    public double getHoodCompensation() {
        return hoodCompensation;
    }


    // -------------------------------------------------------------------------
    // Voltage status
    // -------------------------------------------------------------------------

    public double getVoltage() {
        return filteredVoltage;
    }


    public double getVoltageScale() {
        return voltageScale;
    }


    // -------------------------------------------------------------------------
    // Compatibility getters for existing telemetry
    //
    // In the current hardware:
    //
    // "Front" refers to the shooter motor.
    // "Back" refers to the feeder motor.
    // -------------------------------------------------------------------------

    public double getFrontMotorPower() {

        if (shooter == null) {
            return 0.0;
        }

        return shooter.getMotorPower();
    }


    public double getBackMotorPower() {

        if (feeder == null) {
            return 0.0;
        }

        return feeder.getMotorPower();
    }


    public double getFrontVelocityTPS() {
        return getCurrentTPS();
    }


    public double getBackVelocityTPS() {
        return getFeederCurrentTPS();
    }


    // -------------------------------------------------------------------------
    // Utilities
    // -------------------------------------------------------------------------

    private double clamp(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(
                minimum,
                Math.min(
                        maximum,
                        value
                )
        );
    }


    private static double clampStatic(
            double value,
            double minimum,
            double maximum
    ) {

        return Math.max(
                minimum,
                Math.min(
                        maximum,
                        value
                )
        );
    }
}