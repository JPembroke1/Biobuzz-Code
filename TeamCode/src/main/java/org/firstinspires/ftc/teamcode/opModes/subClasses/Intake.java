package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@Configurable
public class Intake {

    public DcMotor IntakeMotorLeft;
    public DcMotor IntakeMotorRight;
    public DigitalChannel ballSensor;
    public ServoImplEx led;

    public static double red = 0.310;
    public static double green = 0.500;

    public static double ballConfirmDelaySeconds = 0.15;

    public static boolean sensorTriggeredWhenHigh = true;

    public static int ledTestMode = 0;

    public static boolean rawSensorState = false;
    public static boolean sensorTriggered = false;
    public static boolean ballConfirmed = false;

    public static double ballDetectedTimeSeconds = 0.0;
    public static double lastLedPosition = 0.0;

    private long ballDetectionStartTime = -1;


    public void init(HardwareMap hardwareMap) {

        IntakeMotorLeft = hardwareMap.get(DcMotor.class, "intakeMotorFront");
        IntakeMotorRight = hardwareMap.get(DcMotor.class, "intakeMotorBack");

        IntakeMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ballSensor = hardwareMap.get(DigitalChannel.class, "ballSensor");
        ballSensor.setMode(DigitalChannel.Mode.INPUT);

        led = hardwareMap.get(ServoImplEx.class, "ballLed");
        led.setPwmRange(new PwmControl.PwmRange(500, 2500));
        led.setPwmEnable();

        rawSensorState = ballSensor.getState();
        sensorTriggered = getSensorTriggeredFromRaw(rawSensorState);

        ballConfirmed = false;
        ballDetectedTimeSeconds = 0.0;
        ballDetectionStartTime = -1;

        lastLedPosition = red;
    }

    public void intake(double leftPower, double rightPower) {

        IntakeMotorLeft.setPower(leftPower);
        IntakeMotorRight.setPower(rightPower);

    }

    public void update() {
        rawSensorState = ballSensor.getState();
        sensorTriggered = getSensorTriggeredFromRaw(rawSensorState);

        if (ledTestMode == 1) {
            setLedGreen();
            return;
        }

        if (ledTestMode == 2) {
            setLedRed();
            return;
        }

        if (sensorTriggered) {
            updateBallConfirmationTimer();

            if (ballConfirmed) {
                setLedGreen();
            } else {
                setLedRed();
            }

            return;
        }

        resetBallConfirmation();
    }

    private void updateBallConfirmationTimer() {
        if (ballDetectionStartTime < 0) {
            ballDetectionStartTime = System.currentTimeMillis();
        }

        ballDetectedTimeSeconds =
                (System.currentTimeMillis() - ballDetectionStartTime) / 1000.0;

        ballConfirmed = ballDetectedTimeSeconds >= ballConfirmDelaySeconds;
    }

    private boolean getSensorTriggeredFromRaw(boolean rawState) {
        if (sensorTriggeredWhenHigh) {
            return rawState;
        }

        return !rawState;
    }

    public boolean isSensorTriggered() {
        rawSensorState = ballSensor.getState();
        sensorTriggered = getSensorTriggeredFromRaw(rawSensorState);

        return sensorTriggered;
    }

    public boolean isBallConfirmed() {
        return ballConfirmed;
    }

    public boolean isBallReady() {
        update();
        return isBallConfirmed();
    }

    public void resetBallConfirmation() {
        ballDetectionStartTime = -1;
        ballDetectedTimeSeconds = 0.0;
        ballConfirmed = false;

        setLedRed();
    }

    public void setLedRed() {
        setLedPosition(red);
    }

    public void setLedGreen() {
        setLedPosition(green);
    }

    public void setLedPosition(double position) {
        double clippedPosition = clamp(position, 0.0, 1.0);

        led.setPosition(clippedPosition);
        lastLedPosition = clippedPosition;
    }

    public void stopIntake() {
        IntakeMotorRight.setPower(0);
        IntakeMotorLeft.setPower(0);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
