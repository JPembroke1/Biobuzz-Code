package org.firstinspires.ftc.teamcode.opModes;

import android.hardware.HardwareBuffer;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Brakes;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.opencv.core.Mat;

import java.util.List;

@Configurable
@TeleOp(name = "Intake Test", group = "Main")
public class IntakeTest extends OpMode {

    private final PoseFactory p = PoseFactory.degrees();

    private Intake intake;
    private Outtake outtake;
    private RobotHardware robotHardware;
    private Brakes brakes;

    private Turret turret;

    // The follower owns the mecanum drivetrain (and its four drive motors).
    private Follower follower;

    private Servo blockServo;

    private List<LynxModule> hubs;

    private String allianceLabel = "UNKNOWN";

    public static double BLUE_GOAL_X = 84.0;
    public static double BLUE_GOAL_Y = 53.0;

    public static double RED_GOAL_X = 138.0;
    public static double RED_GOAL_Y = 138.0;

    public static double TELEOP_RESET_X = 0;
    public static double TELEOP_RESET_Y = 0;
    public static double TELEOP_RESET_HEADING_DEG = 145.0;

    public static double TELEOP_GOAL_X = 0;
    public static double TELEOP_GOAL_Y = 0;

    public static double BLUE_RESET_X = 24.0;
    public static double BLUE_RESET_Y = 138.0;
    public static double BLUE_RESET_HEADING_DEG = 145.0;

    public static double RED_RESET_X = 120.0;
    public static double RED_RESET_Y = 138.0;
    public static double RED_RESET_HEADING_DEG = 35.0;
    public static double distanceCM = 0;

    public static double HOOD_POSITION = 0.34;

    public static double right_servo_limit = 0.1;
    public static double left_servo_limit = 0.9;

    // Drive input signs. Pedro's convention is forward = -stickY,
    // strafe = -stickX (positive = left), turn = -stickX (positive = CCW).
    // Flip any of these from the dashboard if the robot drives the wrong way.
    public static double FORWARD_SIGN = -1.0;
    public static double STRAFE_SIGN = -1.0;
    public static double TURN_SIGN = -1.0;
    // Cubic shaping on strafe and turn gives finer control at low stick values.
    public static boolean CUBIC_STRAFE_TURN = false;

    // Idle speed as a fraction of RobotConstants.OUTTAKE_TPS (0 = no idle spin).
    public static double IDLE_FRACTION = 0.5;

    // Press gamepad2 dpad up to turn off auto-aim and set the turret to this position.
    public static double TURRET_OVERRIDE_POSITION = 0.5;

    //public Servo turretServo;

    private boolean previousG1DpadUp = false;
    private boolean previousG1DpadRight = false;
    private boolean previousG1DpadLeft = false;
    private boolean previousLeftBumper = false;

    private boolean autoAimMode = true;
    private boolean shooterActive = false;

    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        // Manual bulk caching: one hub read per loop instead of one per call.
        // clearBulkCache() MUST be called at the top of every loop.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        intake = new Intake();
        intake.init(hardwareMap);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        brakes = new Brakes();
        brakes.init(hardwareMap);

        robotHardware = new RobotHardware(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap);

        blockServo = hardwareMap.get(Servo.class, "blockServo");
        blockServo.setDirection(Servo.Direction.FORWARD);
        blockServo.setPosition(RobotConstants.blockClosePos);



        //turret.setTarget(0);

        applyAllianceFromPoseStorage();

        /*if (PoseStorage.currentPose != null) {
            follower.setPose(PoseStorage.currentPose);
        } else {
            follower.setPose(getTeleOpResetPose());
        }*/

        follower.setPose(new Pose(0, 0, 180));

        //turretServo = hardwareMap.get(Servo.class, "turretServo");
    }

    @Override
    public void loop() {

        clearBulkCache();

        handleControls();

        // Sends manual drive powers, then updates localizer and drivetrain.
        driveRobot();

        outtake.update();

        displayTelemetry();

        Pose currentPose = follower.pose();
        PoseStorage.setPose(currentPose);

        updateTurretAim(currentPose);

        //turretServo.setPosition(0);

        handleTurretModeControls();

        //DetermineGoal(currentPose);

        handlePoseReset();

        handleGoalPose(currentPose);

        getDistanceCM(currentPose);

        //robotHardware.linearHoodRegression(distanceCM);
        robotHardware.updateHood(HOOD_POSITION);

        turret.setRange(right_servo_limit, left_servo_limit);
    }

    private void handleControls() {

        // =====================================================
        // INTAKE
        // =====================================================

        if (gamepad1.right_bumper) {

            intake.intake(
                    RobotConstants.FRONT_INTAKE_SPEED,
                    RobotConstants.BACK_INTAKE_SPEED
            );
            robotHardware.block();
            RobotConstants.timer = 0;

        } else if (gamepad1.right_trigger > 0.3) {

            robotHardware.open();
            RobotConstants.timer += 0.1;

            if (RobotConstants.timer > RobotConstants.delayAmount) {
                intake.intake(
                        RobotConstants.FRONT_FEED_SPEED,
                        RobotConstants.BACK_FEED_SPEED
                );
            }

        } else if (gamepad1.leftBumperWasPressed()) {

            intake.intake(
                    -RobotConstants.FRONT_INTAKE_SPEED,
                    -RobotConstants.BACK_INTAKE_SPEED
            );

        } else {

            intake.stopIntake();
            robotHardware.block();
            RobotConstants.timer = 0;
        }

        // =====================================================
        // OUTTAKE
        // =====================================================

        // The shooter idles by default. Right bumper (intake) returns it to idle,
        // cross (or triangle for idle) switches modes. Cross is checked last so it
        // wins if pressed together with the intake button.

        if (gamepad1.triangle) {
            shooterActive = false;
        }

        if (gamepad1.cross) {
            shooterActive = true;
        }

        // The target follows RobotConstants.OUTTAKE_TPS every loop, so Panels edits
        // take effect live in both modes.
        double shootTPS = RobotConstants.OUTTAKE_TPS;

        outtake.setTargetTPS(shooterActive ? shootTPS : 0);

        // =====================================================
        // TURRET
        // =====================================================

        // gamepad2 dpad up: auto-aim OFF and turret to the override position.
        /*if (gamepad2.dpadUpWasPressed()) {
            autoAimMode = false;
            RobotConstants.TURRET_POSITION = TURRET_OVERRIDE_POSITION;
        }

        // gamepad1 square: auto-aim back ON.
        if (gamepad1.squareWasPressed()) {
            autoAimMode = true;
        } */

        /*if (gamepad1.dpadRightWasPressed() && RobotConstants.TURRET_POSITION < 1) {
            RobotConstants.TURRET_POSITION += 0.05;
        } else if (gamepad1.dpadLeftWasPressed() && RobotConstants.TURRET_POSITION > 0) {
            RobotConstants.TURRET_POSITION -= 0.05;
        } */

        // =====================================================
        // BRAKES
        // =====================================================

        if (gamepad1.dpad_up) {
            brakes.liftBrakes();
        }

        if (gamepad1.dpad_down) {
            brakes.lowerBrakes();
        }

        if (gamepad2.dpadDownWasPressed()) {
            CUBIC_STRAFE_TURN = !CUBIC_STRAFE_TURN;
        }
    }

    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    private void driveRobot() {

        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;

        if (CUBIC_STRAFE_TURN) {
            strafeInput = Math.pow(strafeInput, 3);
            turnInput = Math.pow(turnInput, 3);
        }

        double forward = FORWARD_SIGN * gamepad1.left_stick_y * RobotConstants.DRIVE_SPEED;
        double strafe = STRAFE_SIGN * strafeInput * RobotConstants.DRIVE_SPEED;
        double turn = TURN_SIGN * turnInput * RobotConstants.DRIVE_SPEED;

        // Pedro's Mecanum drivetrain normalizes and applies the wheel powers.
        follower.manual(new DrivePowers(forward, strafe, turn));
        follower.update();
    }

    private void displayTelemetry() {

        // =====================================================
        // OUTTAKE TARGETS
        // =====================================================

        telemetry.addData("Configured TPS", RobotConstants.OUTTAKE_TPS);
        telemetry.addData("Shooter Mode", shooterActive ? "SHOOT" : "IDLE");
        telemetry.addData("Active TPS", outtake.getTargetTPS());

        // =====================================================
        // VELOCITY
        // =====================================================

        telemetry.addData("Current TPS (filtered)", outtake.getCurrentTPS());
        telemetry.addData("Raw TPS", outtake.getRawTPS());
        telemetry.addData("Slave TPS (diagnostic)", outtake.getSlaveVelocityTPS());

        // =====================================================
        // CONTROLLER
        // =====================================================

        telemetry.addData("Error TPS", outtake.getErrorTPS());
        telemetry.addData("Commanded Power", outtake.getCommandedPower());
        telemetry.addData("At Speed", outtake.isAtSpeed(50.0));

        // =====================================================
        // MOTOR OUTPUTS
        // =====================================================

        telemetry.addData("Master Motor Power", outtake.getMasterPower());
        telemetry.addData("Slave Motor Power", outtake.getSlavePower());

        // =====================================================
        // VOLTAGE COMPENSATION
        // =====================================================

        telemetry.addData("Battery Voltage (filtered)", outtake.getVoltage());
        telemetry.addData("Voltage Scale", outtake.getVoltageScale());

        // =====================================================
        // CONTROLLER CONSTANTS
        // =====================================================

        telemetry.addData("kF", Outtake.kF);
        telemetry.addData("kP", Outtake.kP);
        telemetry.addData("Max Power", Outtake.MAX_POWER);

        // =====================================================
        // INTAKE
        // =====================================================

        telemetry.addData("Front Intake Speed", RobotConstants.FRONT_INTAKE_SPEED);
        telemetry.addData("Back Intake Speed", RobotConstants.BACK_INTAKE_SPEED);

        telemetry.addData("Timer", RobotConstants.timer);
        telemetry.addData("TurretPos", RobotConstants.TURRET_POSITION);
        telemetry.addData("Auto Aim", autoAimMode);

        telemetry.addData("Pose", follower.pose());

        telemetry.update();
    }

    private void handlePoseReset() {
        boolean currentDpadRight = gamepad1.dpad_right;

        if (currentDpadRight && !previousG1DpadRight) {
            Pose resetPose = getTeleOpResetPose();

            follower.setPose(resetPose);
            PoseStorage.setPose(resetPose);

            autoAimMode = true;
        }

        previousG1DpadRight = currentDpadRight;
    }

    /*private void handleGoalReset() {
        boolean currentDpadLeft = gamepad1.dpad_left;

        if (currentDpadLeft && !previousG1DpadLeft) {
            Pose goalPose = getTeleOpGoalPose();
        }
    }*/


    private Pose getTeleOpGoalPose() {
        return new Pose(TELEOP_GOAL_X, TELEOP_GOAL_Y, 0);
    }

    private double distanceToTeleOpGoalCM(Pose robotPose) {
        if (robotPose == null) {
            return 0.0;
        }

        Pose goalPose = getTeleOpGoalPose();

        double dx = goalPose.x() - robotPose.x();
        double dy = goalPose.y() - robotPose.y();

        return Math.hypot(dx, dy) * 2.54;
    }

    private void handleTurretModeControls() {
        boolean currentLeftBumper = gamepad1.left_bumper;
        boolean currentDpadUp = gamepad1.dpad_up;

        if (currentLeftBumper && !previousLeftBumper) {
            autoAimMode = !autoAimMode;

            if (!autoAimMode) {
                turret.setTarget(0);
            }
        }

        if (currentDpadUp && !previousG1DpadUp) {
            autoAimMode = true;
        }

        previousLeftBumper = currentLeftBumper;
        previousG1DpadUp = currentDpadUp;
    }

    private void updateTurretAim(Pose currentPose) {
        if (autoAimMode) {
            turret.aimAt(
                    currentPose.x(),
                    currentPose.y(),
                    currentPose.heading(),
                    TELEOP_GOAL_X,
                    TELEOP_GOAL_Y
            );
        } else {
            turret.setTarget(0);
        }
        turret.update();
    }

    /*private void DetermineGoal(Pose currentPose) {
        if (currentPose.x() > 72) {
            TELEOP_GOAL_X = 86.0;
        } else {
            TELEOP_GOAL_X = 34.0;
        }
    }*/

    private void handleGoalPose(Pose currentPose) {
        if (gamepad1.dpad_left) {
            TELEOP_GOAL_X = currentPose.x();
            TELEOP_GOAL_Y = currentPose.y();
        }
    }

    private void limitIntakeSpeeds() {

        RobotConstants.FRONT_INTAKE_SPEED = clip(
                RobotConstants.FRONT_INTAKE_SPEED,
                -1.0,
                1.0
        );

        RobotConstants.BACK_INTAKE_SPEED = clip(
                RobotConstants.BACK_INTAKE_SPEED,
                -1.0,
                1.0
        );
    }

    private Pose getTeleOpResetPose() {
        return new Pose(
                TELEOP_RESET_X,
                TELEOP_RESET_Y,
                Math.toRadians(TELEOP_RESET_HEADING_DEG)
        );
    }

    private void getDistanceCM(Pose currentPose) {
        distanceCM = Math.sqrt(Math.pow((currentPose.x() - TELEOP_GOAL_X), 2) + Math.pow((currentPose.y() - TELEOP_GOAL_Y), 2));
    }

    private void applyAllianceFromPoseStorage() {
        if (PoseStorage.isRed()) {
            allianceLabel = "RED";

            TELEOP_GOAL_X = RED_GOAL_X;
            TELEOP_GOAL_Y = RED_GOAL_Y;

            TELEOP_RESET_X = RED_RESET_X;
            TELEOP_RESET_Y = RED_RESET_Y;
            TELEOP_RESET_HEADING_DEG = RED_RESET_HEADING_DEG;

            return;
        }

        if (PoseStorage.isBlue()) {
            allianceLabel = "BLUE";

            TELEOP_GOAL_X = BLUE_GOAL_X;
            TELEOP_GOAL_Y = BLUE_GOAL_Y;

            TELEOP_RESET_X = BLUE_RESET_X;
            TELEOP_RESET_Y = BLUE_RESET_Y;
            TELEOP_RESET_HEADING_DEG = BLUE_RESET_HEADING_DEG;

            return;
        }

        allianceLabel = "UNKNOWN - DEFAULT BLUE";

        TELEOP_GOAL_X = BLUE_GOAL_X;
        TELEOP_GOAL_Y = BLUE_GOAL_Y;

        TELEOP_RESET_X = BLUE_RESET_X;
        TELEOP_RESET_Y = BLUE_RESET_Y;
        TELEOP_RESET_HEADING_DEG = BLUE_RESET_HEADING_DEG;
    }

    private double clip(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    @Override
    public void stop() {

        if (outtake != null) {
            outtake.stopOuttake();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        // Zero the drive powers and push them to the motors.
        if (follower != null) {
            follower.manual(new DrivePowers(0.0, 0.0, 0.0));
            follower.update();
        }
    }
}