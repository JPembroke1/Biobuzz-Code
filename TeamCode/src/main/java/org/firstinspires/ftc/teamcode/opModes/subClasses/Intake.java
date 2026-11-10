package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
public class Intake {

    // -------------------------------------------------------------------------
    // Hardware
    // -------------------------------------------------------------------------

    private DcMotor intakeMotorLeft;
    private DcMotor intakeMotorRight;

    private DigitalChannel ballSensor;
    private ServoImplEx led;


    // -------------------------------------------------------------------------
    // LED calibration
    // -------------------------------------------------------------------------

    public static double red = 0.310;
    public static double green = 0.500;


    // -------------------------------------------------------------------------
    // Ball detection
    // -------------------------------------------------------------------------

    /*
     * The sensor must remain triggered for this long before a ball is
     * considered confirmed.
     */
    public static double ballConfirmDelaySeconds = 0.15;

    /*
     * true:
     * Digital input HIGH means triggered.
     *
     * false:
     * Digital input LOW means triggered.
     */
    public static boolean sensorTriggeredWhenHigh = true;


    // -------------------------------------------------------------------------
    // LED testing
    //
    // 0 = automatic
    // 1 = force green
    // 2 = force red
    // -------------------------------------------------------------------------

    public static int ledTestMode = 0;


    // -------------------------------------------------------------------------
    // Diagnostics
    // -------------------------------------------------------------------------

    public static boolean rawSensorState = false;
    public static boolean sensorTriggered = false;
    public static boolean ballConfirmed = false;

    public static double ballDetectedTimeSeconds = 0.0;
    public static double lastLedPosition = 0.0;

    public static int confirmedBallCount = 0;


    // -------------------------------------------------------------------------
    // Internal state
    // -------------------------------------------------------------------------

    private long ballDetectionStartTimeMs = -1;

    /*
     * Prevents one ball from being counted on every loop while it remains
     * in front of the sensor.
     */
    private boolean currentBallCounted = false;


    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    public void init(HardwareMap hardwareMap) {

        intakeMotorLeft = hardwareMap.get(
                DcMotor.class,
                "intakeMotorFront"
        );

        intakeMotorRight = hardwareMap.get(
                DcMotor.class,
                "intakeMotorBack"
        );

        intakeMotorLeft.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        intakeMotorRight.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        intakeMotorLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intakeMotorRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        ballSensor = hardwareMap.get(
                DigitalChannel.class,
                "ballSensor"
        );

        ballSensor.setMode(
                DigitalChannel.Mode.INPUT
        );

        led = hardwareMap.get(
                ServoImplEx.class,
                "ballLed"
        );

        led.setPwmRange(
                new PwmControl.PwmRange(
                        500,
                        2500
                )
        );

        led.setPwmEnable();

        rawSensorState =
                ballSensor.getState();

        sensorTriggered =
                getSensorTriggeredFromRaw(
                        rawSensorState
                );

        ballConfirmed = false;
        ballDetectedTimeSeconds = 0.0;
        confirmedBallCount = 0;

        ballDetectionStartTimeMs = -1;
        currentBallCounted = false;

        stopIntake();
        setLedRed();
    }


    // -------------------------------------------------------------------------
    // Motor control
    // -------------------------------------------------------------------------

    public void intake(
            double leftPower,
            double rightPower
    ) {

        intakeMotorLeft.setPower(
                clip(leftPower, -1.0, 1.0)
        );

        intakeMotorRight.setPower(
                clip(rightPower, -1.0, 1.0)
        );
    }


    public void stopIntake() {

        if (intakeMotorLeft != null) {
            intakeMotorLeft.setPower(0.0);
        }

        if (intakeMotorRight != null) {
            intakeMotorRight.setPower(0.0);
        }
    }


    // -------------------------------------------------------------------------
    // Periodic sensor update
    // -------------------------------------------------------------------------

    /**
     * Call once during every OpMode loop.
     */
    public void update() {

        if (ballSensor == null) {
            return;
        }

        rawSensorState =
                ballSensor.getState();

        sensorTriggered =
                getSensorTriggeredFromRaw(
                        rawSensorState
                );

        /*
         * Always update the ball state, even when the LED is being forced into
         * a test mode.
         */
        if (sensorTriggered) {

            updateBallConfirmationTimer();

        } else {

            clearCurrentBallState();
        }

        updateLed();
    }


    private void updateBallConfirmationTimer() {

        if (ballDetectionStartTimeMs < 0) {

            ballDetectionStartTimeMs =
                    System.currentTimeMillis();
        }

        ballDetectedTimeSeconds =
                (System.currentTimeMillis()
                        - ballDetectionStartTimeMs) / 1000.0;

        ballConfirmed =
                ballDetectedTimeSeconds
                        >= ballConfirmDelaySeconds;

        /*
         * Count the ball once when it first becomes confirmed.
         *
         * The count latch is not released until the sensor clears.
         */
        if (ballConfirmed && !currentBallCounted) {

            confirmedBallCount++;
            currentBallCounted = true;
        }
    }


    private void clearCurrentBallState() {

        ballDetectionStartTimeMs = -1;
        ballDetectedTimeSeconds = 0.0;
        ballConfirmed = false;

        /*
         * The sensor has cleared, so the next confirmed detection represents a
         * new ball.
         */
        currentBallCounted = false;
    }


    private void updateLed() {

        if (ledTestMode == 1) {

            setLedGreen();
            return;
        }

        if (ledTestMode == 2) {

            setLedRed();
            return;
        }

        if (ballConfirmed) {

            setLedGreen();

        } else {

            setLedRed();
        }
    }


    private boolean getSensorTriggeredFromRaw(
            boolean rawState
    ) {

        if (sensorTriggeredWhenHigh) {
            return rawState;
        }

        return !rawState;
    }


    // -------------------------------------------------------------------------
    // Ball state
    // -------------------------------------------------------------------------

    /**
     * Returns the most recently updated sensor state.
     *
     * The autonomous already calls update() every loop, so this method does not
     * restart or independently modify the confirmation timer.
     */
    public boolean isSensorTriggered() {

        if (ballSensor == null) {
            return false;
        }

        rawSensorState =
                ballSensor.getState();

        sensorTriggered =
                getSensorTriggeredFromRaw(
                        rawSensorState
                );

        return sensorTriggered;
    }


    public boolean isBallConfirmed() {
        return ballConfirmed;
    }


    /**
     * Updates the sensor state immediately and returns the confirmation result.
     */
    public boolean isBallReady() {

        update();
        return ballConfirmed;
    }


    /**
     * Clears the current detection timer without resetting the total count.
     */
    public void resetBallConfirmation() {

        clearCurrentBallState();
        setLedRed();
    }


    /**
     * Returns the number of separate confirmed ball detections.
     */
    public int getConfirmedBallCount() {
        return confirmedBallCount;
    }


    /**
     * Resets the autonomous ball counter.
     */
    public void resetConfirmedBallCount() {

        confirmedBallCount = 0;

        /*
         * Do not immediately recount a ball that is already sitting in front
         * of the sensor.
         */
        currentBallCounted =
                ballConfirmed;
    }


    // -------------------------------------------------------------------------
    // Motor telemetry
    // -------------------------------------------------------------------------

    public double getFrontMotorPower() {

        if (intakeMotorLeft == null) {
            return 0.0;
        }

        return intakeMotorLeft.getPower();
    }


    public double getBackMotorPower() {

        if (intakeMotorRight == null) {
            return 0.0;
        }

        return intakeMotorRight.getPower();
    }


    /**
     * Compatibility getter using the subsystem's original left motor name.
     */
    public double getLeftMotorPower() {
        return getFrontMotorPower();
    }


    /**
     * Compatibility getter using the subsystem's original right motor name.
     */
    public double getRightMotorPower() {
        return getBackMotorPower();
    }


    // -------------------------------------------------------------------------
    // LED control
    // -------------------------------------------------------------------------

    public void setLedRed() {
        setLedPosition(red);
    }


    public void setLedGreen() {
        setLedPosition(green);
    }


    public void setLedPosition(
            double position
    ) {

        double clippedPosition =
                clip(
                        position,
                        0.0,
                        1.0
                );

        if (led != null) {
            led.setPosition(clippedPosition);
        }

        lastLedPosition =
                clippedPosition;
    }


    // -------------------------------------------------------------------------
    // Utility
    // -------------------------------------------------------------------------

    private double clip(
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