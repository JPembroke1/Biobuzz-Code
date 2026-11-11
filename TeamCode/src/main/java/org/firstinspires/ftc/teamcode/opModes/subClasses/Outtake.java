package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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
    // Shooter controller
    //
    // Starting estimates only.
    //
    // Recommended calibration:
    // shooter kF = 1 / measured full-power shooter TPS
    //
    // Example:
    // 2300 TPS at full power -> kF = 1 / 2300 = 0.000435
    //
    // kP = 0.00050 produces approximately:
    // 100 TPS error -> 0.05 power correction
    // ---------------------------------------------------------------------

    public static double kF = 0.00043;
    public static double kP = 0.00050;

    // ---------------------------------------------------------------------
    // Feeder controller
    //
    // Starting estimate assumes approximately 3200 TPS at full power:
    //
    // 1 / 3200 = 0.0003125
    //
    // Measure the actual feeder velocity and replace this estimate.
    // ---------------------------------------------------------------------

    public static double FEEDER_kF = 0.00031;
    public static double FEEDER_kP = 0.00025;

    // ---------------------------------------------------------------------
    // Motor limits and filtering
    // ---------------------------------------------------------------------

    public static double MAX_POWER = 1.0;

    /*
     * 0.15 provides significant smoothing.
     * Increase carefully if telemetry response is too slow.
     */
    public static double VELOCITY_FILTER_ALPHA = 0.15;

    // ---------------------------------------------------------------------
    // Voltage compensation
    // ---------------------------------------------------------------------

    public static boolean VOLTAGE_COMP_ENABLED = true;

    public static double NOMINAL_VOLTAGE = 13.2;
    public static double VOLTAGE_FILTER_ALPHA = 0.10;

    public static double MIN_VOLTAGE = 9.0;
    public static double MAX_VOLTAGE = 14.5;

    /*
     * Prevent unusually low voltage readings from applying excessive
     * feedforward scaling.
     */
    public static double MIN_VOLTAGE_SCALE = 0.80;
    public static double MAX_VOLTAGE_SCALE = 1.20;

    // ---------------------------------------------------------------------
    // Hood calibration
    // ---------------------------------------------------------------------

    public static double HOOD_HOME = 0.50;

    public static double HOOD_MIN = 0.40;
    public static double HOOD_MAX = 0.80;

    public static double HOOD_MIN_STEP = 0.002;

    // ---------------------------------------------------------------------
    // Hood velocity compensation
    //
    // Keep disabled until the TPS-distance regression is validated.
    // ---------------------------------------------------------------------

    public static boolean HOOD_COMP_ENABLED = false;

    public static double HOOD_COMP_GAIN = 0.05;
    public static double HOOD_COMP_DEADBAND_TPS = 50.0;
    public static double HOOD_COMP_MAX = 0.08;

    // ---------------------------------------------------------------------
    // Distance regression
    //
    // TPS = intercept + slope * distanceInches
    //
    // Provisional two-point model:
    //
    // 39.10 inches -> approximately 1950 TPS
    // 65.06 inches -> approximately 2100 TPS
    //
    // Regression starts disabled. CombinedTeleOp enables it with G1 Circle.
    // ---------------------------------------------------------------------

    public static boolean DISTANCE_REGRESSION_ENABLED = false;

    public static double TPS_SLOPE = 5.779158656;
    public static double TPS_INTERCEPT = 1724.035454492;

    /*
     * Hood remains fixed at 0.60 while TPS regression is calibrated.
     */
    public static double HOOD_SLOPE = 0.0;
    public static double HOOD_INTERCEPT = 0.60;

    /*
     * Do not extrapolate beyond the tested range.
     */
    public static double SHOT_DISTANCE_MIN = 39.09;
    public static double SHOT_DISTANCE_MAX = 65.06;

    /*
     * Predictions outside this TPS range are rejected rather than silently
     * clamped. This helps expose calibration errors.
     */
    public static double REGRESSION_MIN_TPS = 1900.0;
    public static double REGRESSION_MAX_TPS = 2100.0;

    public static double DISTANCE_TIMEOUT_SECONDS = 0.50;

    // ---------------------------------------------------------------------
    // Regression diagnostics
    // ---------------------------------------------------------------------

    public enum RegressionStatus {
        DISABLED,
        SHOOTER_STOPPED,
        NO_DISTANCE,
        INVALID_DISTANCE,
        STALE_DISTANCE,
        INVALID_RANGE,
        BELOW_RANGE,
        ABOVE_RANGE,
        INVALID_MODEL,
        TPS_OUT_OF_RANGE,
        ACTIVE
    }

    private RegressionStatus regressionStatus =
            RegressionStatus.DISABLED;

    // ---------------------------------------------------------------------
    // Individual motor controller
    // ---------------------------------------------------------------------

    private static class Loop {

        private final DcMotorEx motor;

        private double targetTPS = 0.0;

        private double signedRawTPS = 0.0;
        private double rawTPS = 0.0;
        private double currentTPS = 0.0;
        private double errorTPS = 0.0;
        private double power = 0.0;

        private boolean primed = false;
        private boolean controllerValid = false;

        Loop(DcMotorEx motor) {
            this.motor = motor;
        }

        void setTargetTPS(double target) {

            targetTPS =
                    Double.isFinite(target)
                            ? target
                            : 0.0;
        }

        void update(
                double loopKF,
                double loopKP,
                double voltageScale
        ) {

            controllerValid = false;

            signedRawTPS =
                    motor.getVelocity();

            rawTPS =
                    Math.abs(signedRawTPS);

            if (!Double.isFinite(rawTPS)
                    || !Double.isFinite(loopKF)
                    || !Double.isFinite(loopKP)
                    || !Double.isFinite(voltageScale)) {

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

                double alpha =
                        Double.isFinite(VELOCITY_FILTER_ALPHA)
                                ? clamp(
                                VELOCITY_FILTER_ALPHA,
                                0.0,
                                1.0
                        )
                                : 1.0;

                currentTPS +=
                        alpha
                                * (rawTPS - currentTPS);
            }

            double wantedTPS =
                    Math.abs(targetTPS);

            if (!Double.isFinite(wantedTPS)
                    || wantedTPS <= 0.0) {

                errorTPS = 0.0;

                commandZero();

                return;
            }

            errorTPS =
                    wantedTPS - currentTPS;

            /*
             * Feedforward estimates the steady-state power.
             * Proportional correction responds to velocity error.
             */
            double calculatedPower =
                    loopKF
                            * wantedTPS
                            * voltageScale
                            + loopKP
                            * errorTPS;

            if (!Double.isFinite(calculatedPower)
                    || !Double.isFinite(MAX_POWER)) {

                commandZero();

                return;
            }

            double powerLimit =
                    clamp(
                            MAX_POWER,
                            0.0,
                            1.0
                    );

            /*
             * A flywheel may coast when overspeeding, but it is never
             * actively driven backward by the speed controller.
             */
            calculatedPower =
                    clamp(
                            calculatedPower,
                            0.0,
                            powerLimit
                    );

            power =
                    Math.signum(targetTPS)
                            * calculatedPower;

            motor.setPower(power);

            controllerValid = true;
        }

        private void commandZero() {

            power = 0.0;

            motor.setPower(0.0);
        }

        void stop() {

            targetTPS = 0.0;

            signedRawTPS = 0.0;
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
                    Math.abs(targetTPS)
                            - currentTPS
            ) <= Math.abs(toleranceTPS);
        }

        double getMotorPower() {
            return motor.getPower();
        }
    }

    // ---------------------------------------------------------------------
    // Hardware and state
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

        voltageSensor = null;

        for (VoltageSensor sensor
                : hardwareMap.voltageSensor) {

            if (sensor != null) {

                voltageSensor = sensor;

                break;
            }
        }

        double initialHood =
                Double.isFinite(HOOD_HOME)
                        ? HOOD_HOME
                        : 0.50;

        hoodBase =
                clampHood(initialHood);

        fixedHoodBase =
                hoodBase;

        hoodCommanded =
                hoodBase;

        hoodCompensation = 0.0;

        hood.setPosition(
                hoodCommanded
        );

        stopOuttake();
    }

    // ---------------------------------------------------------------------
    // Shooter command
    // ---------------------------------------------------------------------

    public void setTargetTPS(double targetTPS) {

        fixedTargetTPS =
                Double.isFinite(targetTPS)
                        ? Math.max(
                        0.0,
                        targetTPS
                )
                        : 0.0;

        usingDistanceRegression = false;

        if (shooter != null) {

            shooter.setTargetTPS(
                    fixedTargetTPS
            );
        }
    }

    // ---------------------------------------------------------------------
    // Distance input
    // ---------------------------------------------------------------------

    public void setGoalDistance(double distance) {

        goalDistance = distance;

        distanceUpdatedNanos =
                System.nanoTime();

        distanceReceived =
                Double.isFinite(distance)
                        && distance >= 0.0;
    }

    // ---------------------------------------------------------------------
    // Shot-model selection
    // ---------------------------------------------------------------------

    private void updateShotTargets() {

        usingDistanceRegression = false;

        shooter.setTargetTPS(
                fixedTargetTPS
        );

        hoodBase =
                clampHood(
                        fixedHoodBase
                );

        if (!DISTANCE_REGRESSION_ENABLED) {

            regressionStatus =
                    RegressionStatus.DISABLED;

            return;
        }

        if (fixedTargetTPS <= 0.0) {

            regressionStatus =
                    RegressionStatus.SHOOTER_STOPPED;

            return;
        }

        if (!distanceReceived) {

            regressionStatus =
                    RegressionStatus.NO_DISTANCE;

            return;
        }

        if (!Double.isFinite(goalDistance)
                || goalDistance < 0.0) {

            regressionStatus =
                    RegressionStatus.INVALID_DISTANCE;

            return;
        }

        double ageSeconds =
                (System.nanoTime()
                        - distanceUpdatedNanos)
                        / 1_000_000_000.0;

        if (!Double.isFinite(ageSeconds)
                || !Double.isFinite(DISTANCE_TIMEOUT_SECONDS)
                || DISTANCE_TIMEOUT_SECONDS <= 0.0
                || ageSeconds > DISTANCE_TIMEOUT_SECONDS) {

            regressionStatus =
                    RegressionStatus.STALE_DISTANCE;

            return;
        }

        if (!Double.isFinite(SHOT_DISTANCE_MIN)
                || !Double.isFinite(SHOT_DISTANCE_MAX)
                || SHOT_DISTANCE_MIN < 0.0
                || SHOT_DISTANCE_MAX <= SHOT_DISTANCE_MIN) {

            regressionStatus =
                    RegressionStatus.INVALID_RANGE;

            return;
        }

        if (goalDistance < SHOT_DISTANCE_MIN) {

            regressionStatus =
                    RegressionStatus.BELOW_RANGE;

            return;
        }

        if (goalDistance > SHOT_DISTANCE_MAX) {

            regressionStatus =
                    RegressionStatus.ABOVE_RANGE;

            return;
        }

        if (!Double.isFinite(TPS_INTERCEPT)
                || !Double.isFinite(TPS_SLOPE)
                || !Double.isFinite(HOOD_INTERCEPT)
                || !Double.isFinite(HOOD_SLOPE)) {

            regressionStatus =
                    RegressionStatus.INVALID_MODEL;

            return;
        }

        double predictedTPS =
                TPS_INTERCEPT
                        + TPS_SLOPE
                        * goalDistance;

        double predictedHood =
                HOOD_INTERCEPT
                        + HOOD_SLOPE
                        * goalDistance;

        if (!Double.isFinite(predictedTPS)
                || predictedTPS <= 0.0
                || !Double.isFinite(predictedHood)
                || !Double.isFinite(REGRESSION_MIN_TPS)
                || !Double.isFinite(REGRESSION_MAX_TPS)
                || REGRESSION_MIN_TPS <= 0.0
                || REGRESSION_MAX_TPS < REGRESSION_MIN_TPS) {

            regressionStatus =
                    RegressionStatus.INVALID_MODEL;

            return;
        }

        /*
         * Reject unexpected predictions rather than hiding the mistake
         * through silent clamping.
         */
        if (predictedTPS < REGRESSION_MIN_TPS
                || predictedTPS > REGRESSION_MAX_TPS) {

            regressionStatus =
                    RegressionStatus.TPS_OUT_OF_RANGE;

            return;
        }

        shooter.setTargetTPS(
                predictedTPS
        );

        hoodBase =
                clampHood(
                        predictedHood
                );

        usingDistanceRegression = true;

        regressionStatus =
                RegressionStatus.ACTIVE;
    }

    // ---------------------------------------------------------------------
    // Feeder commands
    // ---------------------------------------------------------------------

    public void setFeederTPS(double targetTPS) {

        if (feeder == null) {
            return;
        }

        feeder.setTargetTPS(
                Double.isFinite(targetTPS)
                        ? targetTPS
                        : 0.0
        );
    }

    public void stopFeeder() {

        if (feeder != null) {
            feeder.stop();
        }
    }

    // ---------------------------------------------------------------------
    // Hood commands
    // ---------------------------------------------------------------------

    public void setHoodPosition(double position) {

        if (!Double.isFinite(position)) {
            return;
        }

        fixedHoodBase =
                clampHood(position);

        hoodBase =
                fixedHoodBase;

        if (shooter == null
                || shooter.targetTPS <= 0.0) {

            setHoodServoPosition(
                    hoodBase
            );
        }
    }

    public void setHoodHome() {

        setHoodPosition(
                HOOD_HOME
        );
    }

    // ---------------------------------------------------------------------
    // Update and stop
    // ---------------------------------------------------------------------

    public void update() {

        if (shooter == null
                || feeder == null) {

            return;
        }

        updateShotTargets();
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

    public void updatePF() {
        update();
    }

    public void stopOuttake() {

        fixedTargetTPS = 0.0;

        usingDistanceRegression = false;

        regressionStatus =
                DISTANCE_REGRESSION_ENABLED
                        ? RegressionStatus.SHOOTER_STOPPED
                        : RegressionStatus.DISABLED;

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
        filteredVoltage = 0.0;

        hoodBase =
                hoodCommanded;

        hoodCompensation = 0.0;
    }

    // ---------------------------------------------------------------------
    // Hood update
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

            double targetTPS =
                    shooter.targetTPS;

            double errorTPS =
                    shooter.currentTPS
                            - targetTPS;

            double deadband =
                    Math.abs(
                            HOOD_COMP_DEADBAND_TPS
                    );

            if (Math.abs(errorTPS) > deadband) {

                double beyondDeadband =
                        Math.signum(errorTPS)
                                * (Math.abs(errorTPS)
                                - deadband);

                double normalizedError =
                        beyondDeadband
                                / Math.max(
                                1.0,
                                targetTPS
                        );

                hoodCompensation =
                        clamp(
                                HOOD_COMP_GAIN
                                        * normalizedError,
                                -Math.abs(HOOD_COMP_MAX),
                                Math.abs(HOOD_COMP_MAX)
                        );
            }
        }

        setHoodServoPosition(
                hoodBase
                        + hoodCompensation
        );
    }

    private void setHoodServoPosition(double position) {

        if (hood == null
                || !Double.isFinite(position)) {

            return;
        }

        double clipped =
                clampHood(position);

        double minimumStep =
                Double.isFinite(HOOD_MIN_STEP)
                        ? Math.abs(HOOD_MIN_STEP)
                        : 0.0;

        if (Math.abs(
                clipped - hoodCommanded
        ) >= minimumStep) {

            hood.setPosition(clipped);
            hoodCommanded = clipped;
        }
    }

    private double clampHood(double position) {

        double minimum =
                Double.isFinite(HOOD_MIN)
                        ? clamp(
                        HOOD_MIN,
                        0.0,
                        1.0
                )
                        : 0.40;

        double maximum =
                Double.isFinite(HOOD_MAX)
                        ? clamp(
                        HOOD_MAX,
                        0.0,
                        1.0
                )
                        : 0.80;

        return clamp(
                position,
                Math.min(minimum, maximum),
                Math.max(minimum, maximum)
        );
    }

    // ---------------------------------------------------------------------
    // Voltage compensation
    // ---------------------------------------------------------------------

    private void updateVoltageScale() {

        if (!VOLTAGE_COMP_ENABLED
                || voltageSensor == null) {

            voltageScale = 1.0;
            voltagePrimed = false;

            return;
        }

        double measured =
                voltageSensor.getVoltage();

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

        measured =
                clamp(
                        measured,
                        MIN_VOLTAGE,
                        MAX_VOLTAGE
                );

        if (!voltagePrimed) {

            filteredVoltage = measured;
            voltagePrimed = true;

        } else {

            double alpha =
                    Double.isFinite(VOLTAGE_FILTER_ALPHA)
                            ? clamp(
                            VOLTAGE_FILTER_ALPHA,
                            0.0,
                            1.0
                    )
                            : 1.0;

            filteredVoltage +=
                    alpha
                            * (measured
                            - filteredVoltage);
        }

        double minimumScale =
                Double.isFinite(MIN_VOLTAGE_SCALE)
                        ? Math.max(
                        0.0,
                        MIN_VOLTAGE_SCALE
                )
                        : 0.80;

        double maximumScale =
                Double.isFinite(MAX_VOLTAGE_SCALE)
                        ? Math.max(
                        minimumScale,
                        MAX_VOLTAGE_SCALE
                )
                        : 1.20;

        voltageScale =
                clamp(
                        NOMINAL_VOLTAGE
                                / filteredVoltage,
                        minimumScale,
                        maximumScale
                );
    }

    // ---------------------------------------------------------------------
    // Regression status
    // ---------------------------------------------------------------------

    public boolean isUsingDistanceRegression() {
        return usingDistanceRegression;
    }

    public RegressionStatus getRegressionStatus() {
        return regressionStatus;
    }

    public double getGoalDistance() {
        return goalDistance;
    }

    public double getFixedTargetTPS() {
        return fixedTargetTPS;
    }

    // ---------------------------------------------------------------------
    // Shooter telemetry
    // ---------------------------------------------------------------------

    public boolean isAtSpeed(double toleranceTPS) {

        return shooter != null
                && shooter.atSpeed(toleranceTPS);
    }

    public double getTargetTPS() {
        return shooter == null
                ? 0.0
                : shooter.targetTPS;
    }

    public double getCurrentTPS() {
        return shooter == null
                ? 0.0
                : shooter.currentTPS;
    }

    public double getRawTPS() {
        return shooter == null
                ? 0.0
                : shooter.rawTPS;
    }

    public double getSignedRawTPS() {
        return shooter == null
                ? 0.0
                : shooter.signedRawTPS;
    }

    public double getErrorTPS() {
        return shooter == null
                ? 0.0
                : shooter.errorTPS;
    }

    public double getCommandedPower() {
        return shooter == null
                ? 0.0
                : shooter.power;
    }

    // ---------------------------------------------------------------------
    // Feeder telemetry
    // ---------------------------------------------------------------------

    public boolean isFeederAtSpeed(double toleranceTPS) {

        return feeder != null
                && feeder.atSpeed(toleranceTPS);
    }

    public double getFeederTargetTPS() {
        return feeder == null
                ? 0.0
                : feeder.targetTPS;
    }

    public double getFeederCurrentTPS() {
        return feeder == null
                ? 0.0
                : feeder.currentTPS;
    }

    public double getFeederRawTPS() {
        return feeder == null
                ? 0.0
                : feeder.rawTPS;
    }

    public double getFeederSignedRawTPS() {
        return feeder == null
                ? 0.0
                : feeder.signedRawTPS;
    }

    public double getFeederErrorTPS() {
        return feeder == null
                ? 0.0
                : feeder.errorTPS;
    }

    public double getFeederCommandedPower() {
        return feeder == null
                ? 0.0
                : feeder.power;
    }

    // ---------------------------------------------------------------------
    // Hood and voltage telemetry
    // ---------------------------------------------------------------------

    public double getFixedHoodBase() {
        return fixedHoodBase;
    }

    public double getHoodBase() {
        return hoodBase;
    }

    public double getHoodPosition() {
        return hoodCommanded;
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

        return shooter == null
                ? 0.0
                : shooter.getMotorPower();
    }

    public double getBackMotorPower() {

        return feeder == null
                ? 0.0
                : feeder.getMotorPower();
    }

    public double getFrontVelocityTPS() {
        return getCurrentTPS();
    }

    public double getBackVelocityTPS() {
        return getFeederCurrentTPS();
    }

    // ---------------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------------

    private static double clamp(
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