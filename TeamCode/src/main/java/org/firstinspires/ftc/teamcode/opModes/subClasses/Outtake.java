package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Shooter, feeder and variable hood subsystem.
 *
 * Optional distance models:
 *
 * TPS = TPS_INTERCEPT + TPS_SLOPE * distance
 * hood = HOOD_INTERCEPT + HOOD_SLOPE * distance
 *
 * Distance units are inches.
 *
 * A positive setTargetTPS() command requests shooter operation.
 * Zero stops the shooter regardless of regression configuration.
 *
 * Call update() once per OpMode loop.
 */
@Configurable
public class Outtake {

    // ---------------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------------

    public static String SHOOTER_MOTOR_NAME = "shooterMotor1";
    public static String FEEDER_MOTOR_NAME = "shooterMotor";
    public static String HOOD_SERVO_NAME = "hoodServo";

    public static boolean SHOOTER_REVERSED = true;
    public static boolean FEEDER_REVERSED = true;

    // ---------------------------------------------------------------------
    // Shooter PF tuning
    // ---------------------------------------------------------------------

    public static double kF = 0.00045;
    public static double kP = 0.03;

    // ---------------------------------------------------------------------
    // Feeder PF tuning
    // ---------------------------------------------------------------------

    public static double FEEDER_kF = 0.00045;
    public static double FEEDER_kP = 0.01;

    // ---------------------------------------------------------------------
    // Motor limits and filtering
    // ---------------------------------------------------------------------

    public static double MAX_POWER = 1.0;

    // 1.0 means no filtering.
    public static double VELOCITY_FILTER_ALPHA = 0.15;

    // ---------------------------------------------------------------------
    // Voltage compensation
    // ---------------------------------------------------------------------

    public static boolean VOLTAGE_COMP_ENABLED = true;

    public static double NOMINAL_VOLTAGE = 13.2;
    public static double VOLTAGE_FILTER_ALPHA = 0.1;

    public static double MIN_VOLTAGE = 9.0;
    public static double MAX_VOLTAGE = 14.5;

    // ---------------------------------------------------------------------
    // Hood calibration
    // ---------------------------------------------------------------------

    public static double HOOD_HOME = 0.5;

    public static double HOOD_MIN = 0.4;
    public static double HOOD_MAX = 0.8;

    public static double HOOD_MIN_STEP = 0.002;

    // ---------------------------------------------------------------------
    // Hood velocity compensation
    //
    // Disable while calibrating the distance models.
    // ---------------------------------------------------------------------

    public static boolean HOOD_COMP_ENABLED = false;

    public static double HOOD_COMP_GAIN = 0.05;
    public static double HOOD_COMP_DEADBAND_TPS = 50.0;
    public static double HOOD_COMP_MAX = 0.08;

    // ---------------------------------------------------------------------
    // Distance regression
    //
    // Initial intercepts reproduce the existing fixed shot.
    // Slopes and tested range must be calibrated.
    // ---------------------------------------------------------------------

    public static boolean DISTANCE_REGRESSION_ENABLED = false;

    public static double TPS_SLOPE = 0.0;
    public static double TPS_INTERCEPT = 1950.0;

    public static double HOOD_SLOPE = 0.0;
    public static double HOOD_INTERCEPT = 0.60;

    // Equal defaults intentionally prevent regression activation.
    public static double SHOT_DISTANCE_MIN = 0.0;
    public static double SHOT_DISTANCE_MAX = 0.0;

    // Regression-only cap, not a verified hardware maximum.
    public static double REGRESSION_MAX_TPS = 2100.0;

    public static double DISTANCE_TIMEOUT_SECONDS = 0.5;

    // ---------------------------------------------------------------------
    // Individual motor controller
    // ---------------------------------------------------------------------

    private static class Loop {

        private final DcMotorEx motor;

        private double targetTPS = 0.0;

        private double rawTPS = 0.0;
        private double currentTPS = 0.0;
        private double errorTPS = 0.0;
        private double power = 0.0;

        private boolean primed = false;
        private boolean controllerValid = false;

        Loop(DcMotorEx motor) {
            this.motor = motor;
        }

        void update(
                double loopKF,
                double loopKP,
                double voltageScale
        ) {
            controllerValid = false;

            rawTPS = Math.abs(motor.getVelocity());

            if (!Double.isFinite(rawTPS)) {
                primed = false;
                currentTPS = 0.0;
                errorTPS = 0.0;
                commandZero();
                return;
            }

            if (!primed) {
                currentTPS = rawTPS;
                primed = true;
            } else {
                double alpha = Double.isFinite(
                        VELOCITY_FILTER_ALPHA
                )
                        ? clamp(
                        VELOCITY_FILTER_ALPHA,
                        0.0,
                        1.0
                )
                        : 1.0;

                currentTPS += alpha * (rawTPS - currentTPS);
            }

            double wantedTPS = Math.abs(targetTPS);

            if (!Double.isFinite(wantedTPS)
                    || wantedTPS <= 0.0) {
                errorTPS = 0.0;
                commandZero();
                return;
            }

            errorTPS = wantedTPS - currentTPS;

            double calculatedPower =
                    loopKF * wantedTPS * voltageScale
                            + loopKP * errorTPS;

            if (!Double.isFinite(calculatedPower)
                    || !Double.isFinite(MAX_POWER)) {
                commandZero();
                return;
            }

            double limit = clamp(MAX_POWER, 0.0, 1.0);

            calculatedPower = clamp(
                    calculatedPower,
                    0.0,
                    limit
            );

            power = Math.signum(targetTPS) * calculatedPower;

            motor.setPower(power);
            controllerValid = true;
        }

        private void commandZero() {
            power = 0.0;
            motor.setPower(0.0);
        }

        void stop() {
            targetTPS = 0.0;
            rawTPS = 0.0;
            currentTPS = 0.0;
            errorTPS = 0.0;
            power = 0.0;

            primed = false;
            controllerValid = false;

            motor.setPower(0.0);
        }

        boolean atSpeed(double toleranceTPS) {
            return primed
                    && controllerValid
                    && targetTPS != 0.0
                    && Double.isFinite(toleranceTPS)
                    && Math.abs(
                    Math.abs(targetTPS) - currentTPS
            ) <= Math.abs(toleranceTPS);
        }

        double getMotorPower() {
            return motor.getPower();
        }
    }

    // ---------------------------------------------------------------------
    // Hardware and controller state
    // ---------------------------------------------------------------------

    private Loop shooter;
    private Loop feeder;

    private Servo hood;
    private VoltageSensor voltageSensor;

    private double filteredVoltage = 0.0;
    private double voltageScale = 1.0;
    private boolean voltagePrimed = false;

    private double hoodBase = HOOD_HOME;
    private double fixedHoodBase = HOOD_HOME;
    private double hoodCommanded = HOOD_HOME;
    private double hoodCompensation = 0.0;

    private double fixedTargetTPS = 0.0;

    private double goalDistance = Double.NaN;
    private long distanceUpdatedNanos = 0L;
    private boolean distanceReceived = false;

    private boolean usingDistanceRegression = false;

    // ---------------------------------------------------------------------
    // Initialization
    // ---------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        DcMotorEx shooterMotor = hardwareMap.get(
                DcMotorEx.class,
                SHOOTER_MOTOR_NAME
        );

        DcMotorEx feederMotor = hardwareMap.get(
                DcMotorEx.class,
                FEEDER_MOTOR_NAME
        );

        hood = hardwareMap.get(
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

        shooterMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        feederMotor.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        shooter = new Loop(shooterMotor);
        feeder = new Loop(feederMotor);

        voltageSensor = null;

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            if (sensor != null) {
                voltageSensor = sensor;
                break;
            }
        }

        double home = Double.isFinite(HOOD_HOME)
                ? HOOD_HOME
                : 0.5;

        hoodBase = clampHood(home);
        fixedHoodBase = hoodBase;
        hoodCommanded = hoodBase;
        hoodCompensation = 0.0;

        hood.setPosition(hoodCommanded);

        stopOuttake();
    }

    // ---------------------------------------------------------------------
    // Shooter commands
    // ---------------------------------------------------------------------

    /**
     * Stores the fixed fallback TPS and requests shooter operation.
     *
     * A positive target enables shooting.
     * Zero disables shooting even when regression is enabled.
     */
    public void setTargetTPS(double targetTPS) {

        fixedTargetTPS = Double.isFinite(targetTPS)
                ? Math.max(0.0, targetTPS)
                : 0.0;

        usingDistanceRegression = false;

        if (shooter != null) {
            shooter.targetTPS = fixedTargetTPS;
        }
    }

    // ---------------------------------------------------------------------
    // Distance input and target selection
    // ---------------------------------------------------------------------

    /**
     * Refresh every loop with the current selected-goal distance.
     * Units: inches.
     *
     * Pass Double.NaN when unavailable.
     * This method does not start the shooter.
     */
    public void setGoalDistance(double distance) {
        goalDistance = distance;
        distanceUpdatedNanos = System.nanoTime();
        distanceReceived = true;
    }

    private void updateShotTargets() {

        usingDistanceRegression = false;

        shooter.targetTPS = fixedTargetTPS;

        if (fixedTargetTPS <= 0.0) {
            return;
        }

        hoodBase = clampHood(fixedHoodBase);

        if (!DISTANCE_REGRESSION_ENABLED || !distanceReceived) {
            return;
        }

        double ageSeconds =
                (System.nanoTime() - distanceUpdatedNanos)
                        / 1_000_000_000.0;

        if (!Double.isFinite(goalDistance)
                || goalDistance < 0.0
                || !Double.isFinite(DISTANCE_TIMEOUT_SECONDS)
                || DISTANCE_TIMEOUT_SECONDS <= 0.0
                || ageSeconds > DISTANCE_TIMEOUT_SECONDS
                || !Double.isFinite(SHOT_DISTANCE_MIN)
                || !Double.isFinite(SHOT_DISTANCE_MAX)
                || SHOT_DISTANCE_MIN < 0.0
                || SHOT_DISTANCE_MAX <= SHOT_DISTANCE_MIN
                || goalDistance < SHOT_DISTANCE_MIN
                || goalDistance > SHOT_DISTANCE_MAX) {
            return;
        }

        double predictedTPS =
                TPS_INTERCEPT + TPS_SLOPE * goalDistance;

        double predictedHood =
                HOOD_INTERCEPT + HOOD_SLOPE * goalDistance;

        if (!Double.isFinite(predictedTPS)
                || predictedTPS <= 0.0
                || !Double.isFinite(predictedHood)
                || !Double.isFinite(REGRESSION_MAX_TPS)
                || REGRESSION_MAX_TPS <= 0.0) {
            return;
        }

        shooter.targetTPS = Math.min(
                predictedTPS,
                REGRESSION_MAX_TPS
        );

        hoodBase = clampHood(predictedHood);
        usingDistanceRegression = true;
    }

    // ---------------------------------------------------------------------
    // Feeder commands
    // ---------------------------------------------------------------------

    /**
     * Positive runs forward.
     * Negative can be used for unjamming.
     */
    public void setFeederTPS(double targetTPS) {
        if (feeder != null) {
            feeder.targetTPS = Double.isFinite(targetTPS)
                    ? targetTPS
                    : 0.0;
        }
    }

    public void stopFeeder() {
        if (feeder != null) {
            feeder.stop();
        }
    }

    // ---------------------------------------------------------------------
    // Hood commands
    // ---------------------------------------------------------------------

    /**
     * Stores the fixed fallback hood position.
     * Valid distance regression overrides it during update().
     */
    public void setHoodPosition(double position) {

        if (!Double.isFinite(position)) {
            return;
        }

        fixedHoodBase = clampHood(position);
        hoodBase = fixedHoodBase;

        if (shooter == null || shooter.targetTPS <= 0.0) {
            setHoodServoPosition(hoodBase);
        }
    }

    public void setHoodHome() {
        setHoodPosition(HOOD_HOME);
    }

    // ---------------------------------------------------------------------
    // Stop and update
    // ---------------------------------------------------------------------

    public void stopOuttake() {

        fixedTargetTPS = 0.0;
        usingDistanceRegression = false;

        goalDistance = Double.NaN;
        distanceReceived = false;
        distanceUpdatedNanos = 0L;

        if (shooter != null) {
            shooter.stop();
        }

        if (feeder != null) {
            feeder.stop();
        }

        voltagePrimed = false;
        voltageScale = 1.0;

        hoodBase = hoodCommanded;
        hoodCompensation = 0.0;
    }

    public void update() {

        if (shooter == null || feeder == null) {
            return;
        }

        updateShotTargets();
        updateVoltageScale();

        shooter.update(kF, kP, voltageScale);

        feeder.update(
                FEEDER_kF,
                FEEDER_kP,
                voltageScale
        );

        updateHood();
    }

    /**
     * Compatibility alias.
     * Call update() OR updatePF(), not both in one loop.
     */
    public void updatePF() {
        update();
    }

    // ---------------------------------------------------------------------
    // Hood compensation
    // ---------------------------------------------------------------------

    private void updateHood() {

        if (hood == null) {
            return;
        }

        hoodCompensation = 0.0;

        if (HOOD_COMP_ENABLED
                && shooter != null
                && shooter.controllerValid
                && shooter.targetTPS > 0.0
                && Double.isFinite(HOOD_COMP_GAIN)
                && Double.isFinite(HOOD_COMP_DEADBAND_TPS)
                && Double.isFinite(HOOD_COMP_MAX)) {

            double targetTPS = shooter.targetTPS;

            double errorTPS =
                    shooter.currentTPS - targetTPS;

            double deadband =
                    Math.abs(HOOD_COMP_DEADBAND_TPS);

            if (Math.abs(errorTPS) > deadband) {

                double beyondDeadband =
                        Math.signum(errorTPS)
                                * (Math.abs(errorTPS) - deadband);

                double normalizedError =
                        beyondDeadband / Math.max(1.0, targetTPS);

                hoodCompensation = clamp(
                        HOOD_COMP_GAIN * normalizedError,
                        -Math.abs(HOOD_COMP_MAX),
                        Math.abs(HOOD_COMP_MAX)
                );
            }
        }

        setHoodServoPosition(
                hoodBase + hoodCompensation
        );
    }

    private void setHoodServoPosition(double position) {

        if (hood == null || !Double.isFinite(position)) {
            return;
        }

        double clipped = clampHood(position);

        double minimumStep = Double.isFinite(HOOD_MIN_STEP)
                ? Math.abs(HOOD_MIN_STEP)
                : 0.0;

        boolean outsideLimits =
                hoodCommanded != clampHood(hoodCommanded);

        if (outsideLimits
                || Math.abs(clipped - hoodCommanded) >= minimumStep) {

            hood.setPosition(clipped);
            hoodCommanded = clipped;
        }
    }

    private double clampHood(double position) {

        double first = Double.isFinite(HOOD_MIN)
                ? clamp(HOOD_MIN, 0.0, 1.0)
                : 0.4;

        double second = Double.isFinite(HOOD_MAX)
                ? clamp(HOOD_MAX, 0.0, 1.0)
                : 0.8;

        return clamp(
                position,
                Math.min(first, second),
                Math.max(first, second)
        );
    }

    // ---------------------------------------------------------------------
    // Voltage compensation
    // ---------------------------------------------------------------------

    private void updateVoltageScale() {

        if (!VOLTAGE_COMP_ENABLED || voltageSensor == null) {
            voltageScale = 1.0;
            voltagePrimed = false;
            return;
        }

        double measured = voltageSensor.getVoltage();

        if (!Double.isFinite(measured)
                || measured <= 0.0
                || !Double.isFinite(MIN_VOLTAGE)
                || !Double.isFinite(MAX_VOLTAGE)
                || MIN_VOLTAGE <= 0.0
                || MAX_VOLTAGE < MIN_VOLTAGE
                || !Double.isFinite(NOMINAL_VOLTAGE)
                || NOMINAL_VOLTAGE <= 0.0) {

            voltageScale = 1.0;
            voltagePrimed = false;
            return;
        }

        measured = clamp(
                measured,
                MIN_VOLTAGE,
                MAX_VOLTAGE
        );

        if (!voltagePrimed) {
            filteredVoltage = measured;
            voltagePrimed = true;

        } else {
            double alpha = Double.isFinite(VOLTAGE_FILTER_ALPHA)
                    ? clamp(VOLTAGE_FILTER_ALPHA, 0.0, 1.0)
                    : 1.0;

            filteredVoltage +=
                    alpha * (measured - filteredVoltage);
        }

        voltageScale = NOMINAL_VOLTAGE / filteredVoltage;
    }

    // ---------------------------------------------------------------------
    // Status
    // ---------------------------------------------------------------------

    public boolean isUsingDistanceRegression() {
        return usingDistanceRegression;
    }

    public double getGoalDistance() {
        return goalDistance;
    }

    public double getFixedTargetTPS() {
        return fixedTargetTPS;
    }

    public double getFixedHoodBase() {
        return fixedHoodBase;
    }

    public boolean isAtSpeed(double toleranceTPS) {
        return shooter != null && shooter.atSpeed(toleranceTPS);
    }

    public double getTargetTPS() {
        return shooter == null ? 0.0 : shooter.targetTPS;
    }

    public double getCurrentTPS() {
        return shooter == null ? 0.0 : shooter.currentTPS;
    }

    public double getRawTPS() {
        return shooter == null ? 0.0 : shooter.rawTPS;
    }

    public double getErrorTPS() {
        return shooter == null ? 0.0 : shooter.errorTPS;
    }

    public double getCommandedPower() {
        return shooter == null ? 0.0 : shooter.power;
    }

    public boolean isFeederAtSpeed(double toleranceTPS) {
        return feeder != null && feeder.atSpeed(toleranceTPS);
    }

    public double getFeederTargetTPS() {
        return feeder == null ? 0.0 : feeder.targetTPS;
    }

    public double getFeederCurrentTPS() {
        return feeder == null ? 0.0 : feeder.currentTPS;
    }

    public double getFeederRawTPS() {
        return feeder == null ? 0.0 : feeder.rawTPS;
    }

    public double getFeederErrorTPS() {
        return feeder == null ? 0.0 : feeder.errorTPS;
    }

    public double getFeederCommandedPower() {
        return feeder == null ? 0.0 : feeder.power;
    }

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

    // ---------------------------------------------------------------------
    // Compatibility getters
    // ---------------------------------------------------------------------

    public double getFrontMotorPower() {
        return shooter == null ? 0.0 : shooter.getMotorPower();
    }

    public double getBackMotorPower() {
        return feeder == null ? 0.0 : feeder.getMotorPower();
    }

    public double getFrontVelocityTPS() {
        return getCurrentTPS();
    }

    public double getBackVelocityTPS() {
        return getFeederCurrentTPS();
    }

    private static double clamp(
            double value,
            double minimum,
            double maximum
    ) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}