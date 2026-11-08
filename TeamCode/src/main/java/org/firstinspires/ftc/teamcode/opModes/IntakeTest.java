package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.opModes.subClasses.ArduCam;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Brakes;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Flower;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotSettings;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@Configurable
@TeleOp(name = "Intake Test", group = "Main")
public class IntakeTest extends OpMode {

    // -------------------------------------------------------------------------
    // Pedro pose factory
    // -------------------------------------------------------------------------

    private final PoseFactory p = PoseFactory.degrees();

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    private Intake intake;
    private Outtake outtake;
    private Brakes brakes;
    private Turret turret;
    private Flower flower;
    private RobotSettings robotSettings;

    // The follower owns the mecanum drivetrain and localizer.
    private Follower follower;

    public ServoImplEx led;

    private List<LynxModule> hubs;

    // -------------------------------------------------------------------------
    // Alliance and target configuration
    // -------------------------------------------------------------------------

    private String allianceLabel = "UNKNOWN";


    public static double red = 0.310;
    public static double blue = 0.650;

    public static double BLUE_GOAL_X = 6.0;
    public static double BLUE_GOAL_Y = 138.0;

    public static double RED_GOAL_X = 138.0;
    public static double RED_GOAL_Y = 138.0;

    public static double TELEOP_RESET_X = 24.0;
    public static double TELEOP_RESET_Y = 138.0;
    public static double TELEOP_RESET_HEADING_DEG = 145.0;

    public static double TELEOP_GOAL_X = 6.0;
    public static double TELEOP_GOAL_Y = 138.0;

    public static double BLUE_RESET_X = 24.0;
    public static double BLUE_RESET_Y = 138.0;
    public static double BLUE_RESET_HEADING_DEG = 145.0;

    public static double RED_RESET_X = 120.0;
    public static double RED_RESET_Y = 138.0;
    public static double RED_RESET_HEADING_DEG = 35.0;

    // -------------------------------------------------------------------------
    // Pedro corner reset
    //
    // This reset assumes the robot localization point is at the centre of a
    // square robot. RobotConstants.ROBOT_LENGTH_MM is used for both dimensions.
    //
    // With ROBOT_LENGTH_MM = 280:
    //
    // 280 / 2 / 25.4 = approximately 5.51 inches.
    //
    // The resulting Pedro pose is approximately:
    //
    // x = 5.51
    // y = 5.51
    // heading = 180 degrees
    // -------------------------------------------------------------------------

    public static double CORNER_RESET_HEADING_DEG = 180.0;

    // -------------------------------------------------------------------------
    // Drive configuration
    // -------------------------------------------------------------------------

    // Pedro convention:
    // forward = -left stick Y
    // strafe positive = left
    // turn positive = counter-clockwise
    public static double FORWARD_SIGN = -1.0;
    public static double STRAFE_SIGN = -1.0;
    public static double TURN_SIGN = -1.0;

    public static boolean CUBIC_STRAFE_TURN = false;

    public static boolean FLOWER_ACTIVE = false;

    // -------------------------------------------------------------------------
    // Runtime state
    // -------------------------------------------------------------------------

    private boolean autoAimMode = true;
    private boolean shooterActive = false;

    // Which end of the HIVE currently holds the upward CELL.
    private boolean hiveEndRight = true;

    // These remain zero until real field-frame velocity is supplied.
    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;

    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        // Manual bulk caching gives one hub read per loop.
        hubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.MANUAL
            );
        }

        intake = new Intake();
        intake.init(hardwareMap);

        outtake = new Outtake();
        outtake.init(hardwareMap);

        brakes = new Brakes();
        brakes.init(hardwareMap);

        turret = new Turret();
        turret.init(hardwareMap);

        flower = new Flower();
        flower.init(hardwareMap);

        robotSettings = new RobotSettings();
        robotSettings.init(hardwareMap);

        // The turret's mechanical centre physically points backwards relative
        // to the robot because TURRET_ZERO_OFFSET_DEG is 180 degrees.
        turret.setCentre();

//        led = hardwareMap.get(ServoImplEx.class, "disLed");
//        led.setPwmRange(new PwmControl.PwmRange(500, 2500));
//        led.setPwmEnable();


        applyAllianceFromPoseStorage();

        updateGoalFromHive();
        updateTurretTargetFromTeleOpGoal();

        // Leave this disabled until real field-frame velocity is calculated.
        Turret.velocityCompensationActive = false;

        if (PoseStorage.currentPose != null) {

            follower.setPose(
                    PoseStorage.currentPose
            );

        } else {

            Pose resetPose = getTeleOpResetPose();

            follower.setPose(resetPose);
            PoseStorage.setPose(resetPose);
        }

        autoAimMode = true;


    }

    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------

    @Override
    public void loop() {

        clearBulkCache();

        // Read subsystem controls.
        handleControls();
        handleTurretModeControls();

        // gamepad2 dpad left/right:
        // Relocalise against the left or right field wall.
        handleRelocalise();

        // gamepad2 dpad down:
        // Reset at Pedro's bottom-left field corner.
        handleCornerPoseReset();

        // Apply drive commands and update Pedro localization.
        driveRobot();

        Pose currentPose = follower.pose();

        if (currentPose != null) {
            PoseStorage.setPose(currentPose);
        }

        // Update the selected HIVE target before aiming.
        updateGoalFromHive();
        updateTurretAim(currentPose);

        //updateDistanceCheck(turret.getDistanceToTarget());

        // Update shooter, feeder and hood controllers.
        outtake.update();

        intake.update();


        turret.updateCam();

        // Telemetry goes last so it shows the current loop's turret command.
        displayTelemetry();
    }

    // -------------------------------------------------------------------------
    // Main controls
    // -------------------------------------------------------------------------

    private void handleControls() {

        if (gamepad2.crossWasPressed()) {
            RobotConstants.camAssist = !RobotConstants.camAssist;
        }

        // =====================================================================
        // INTAKE AND FEEDER
        // =====================================================================

        boolean feederRun = false;

        if (gamepad1.right_bumper) {

            intake.intake(
                    RobotConstants.FRONT_INTAKE_SPEED,
                    RobotConstants.BACK_INTAKE_SPEED
            );

            robotSettings.block();

            RobotConstants.timer = 0;

        } else if (gamepad1.right_trigger > 0.3) {

            robotSettings.release();

            RobotConstants.timer += 0.1;

            if (RobotConstants.timer
                    > RobotConstants.delayAmount) {

                intake.intake(
                        RobotConstants.FRONT_FEED_SPEED,
                        RobotConstants.BACK_FEED_SPEED
                );

                feederRun = true;
            }

        } else if (gamepad1.leftBumperWasPressed()) {

            intake.intake(
                    -RobotConstants.FRONT_INTAKE_SPEED,
                    -RobotConstants.BACK_INTAKE_SPEED
            );

        } else {

            intake.stopIntake();

            robotSettings.block();

            RobotConstants.timer = 0;
        }

        outtake.setFeederTPS(
                feederRun && feederAllowed()
                        ? RobotConstants.FEEDER_TPS
                        : 0.0
        );

        // =====================================================================
        // OUTTAKE
        // =====================================================================

        // Intake returns the shooter to idle.
        if (gamepad1.right_bumper) {
            shooterActive = false;
        }

        // Triangle selects shooter idle.
        if (gamepad1.triangle) {
            shooterActive = false;
        }

        // Cross selects full shooting speed.
        if (gamepad1.cross) {
            shooterActive = true;
        }

        double shootTPS =
                RobotConstants.OUTTAKE_TPS;

        double idleTPS =
                shootTPS
                        * Math.max(
                        0.0,
                        RobotConstants.IDLE_FRACTION
                );

        outtake.setTargetTPS(
                shooterActive
                        ? shootTPS
                        : idleTPS
        );

        outtake.setHoodPosition(
                RobotConstants.HOOD_POSITION
        );

        // =====================================================================
        // BRAKES
        // =====================================================================

        if (gamepad1.dpad_up) {
            brakes.liftBrakes();
        }

        if (gamepad1.dpad_down) {
            brakes.lowerBrakes();
        }

        // ====================================================================
        // FLOWER
        // ====================================================================

        if (gamepad1.dpadRightWasPressed()) {
            FLOWER_ACTIVE = !FLOWER_ACTIVE;
        }

        if (FLOWER_ACTIVE) {
            flower.flowerActivate(RobotConstants.flowerDropPos);
        } else {
                flower.flowerActivate(RobotConstants.flowerLiftPos);
        }
    }

    // -------------------------------------------------------------------------
    // Turret mode controls
    //
    // gamepad1 square:
    //     Enable automatic field-centric targeting.
    //
    // gamepad2 dpad up:
    //     Disable automatic targeting and return turret to centre.
    //
    // gamepad2 Y:
    //     Swap the active HIVE end.
    // -------------------------------------------------------------------------

    private void handleTurretModeControls() {

        if (gamepad1.squareWasPressed()) {

            autoAimMode = true;

            updateGoalFromHive();
            updateTurretTargetFromTeleOpGoal();
        }

        if (gamepad2.dpadUpWasPressed()) {

            autoAimMode = false;

            // The mechanical centre physically points backwards on this robot.
            turret.setCentre();
        }

        if (gamepad2.yWasPressed()) {

            hiveEndRight = !hiveEndRight;

            updateGoalFromHive();
            updateTurretTargetFromTeleOpGoal();
        }
    }

    // -------------------------------------------------------------------------
    // Feeder permission
    // -------------------------------------------------------------------------

    /**
     * Returns whether the feeder motor may run.
     */
    private boolean feederAllowed() {

        if (!RobotConstants.FEED_REQUIRES_SPEED) {
            return true;
        }

        return shooterActive
                && outtake.isAtSpeed(
                RobotConstants.FEED_SPEED_TOLERANCE_TPS
        );
    }

    // -------------------------------------------------------------------------
    // Bulk caching
    // -------------------------------------------------------------------------

    private void clearBulkCache() {

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }

    // -------------------------------------------------------------------------
    // Drive
    // -------------------------------------------------------------------------

    private void driveRobot() {

        double strafeInput =
                gamepad1.left_stick_x;

        double turnInput =
                gamepad1.right_stick_x;

        if (CUBIC_STRAFE_TURN) {

            strafeInput =
                    Math.pow(strafeInput, 3);

            turnInput =
                    Math.pow(turnInput, 3);
        }

        double forward =
                FORWARD_SIGN
                        * gamepad1.left_stick_y
                        * RobotConstants.DRIVE_SPEED;

        double strafe =
                STRAFE_SIGN
                        * strafeInput
                        * RobotConstants.DRIVE_SPEED;

        double turn =
                TURN_SIGN
                        * turnInput
                        * RobotConstants.DRIVE_SPEED;

        follower.manual(
                new DrivePowers(
                        forward,
                        strafe,
                        turn
                )
        );

        follower.update();
    }

    private void updateDistanceCheck(double distance) {
        if (distance > 100) {
            led.setPosition(blue);
        } else {
            led.setPosition(red);
        }
    }

    // -------------------------------------------------------------------------
    // Pedro bottom-left corner reset
    //
    // gamepad2 dpad down
    // -------------------------------------------------------------------------

    private void handleCornerPoseReset() {

        if (!gamepad2.dpadDownWasPressed()) {
            return;
        }

        Pose resetPose =
                getCornerResetPose();

        follower.setPose(resetPose);
        PoseStorage.setPose(resetPose);

        updateGoalFromHive();
        updateTurretTargetFromTeleOpGoal();

        // This clears the turret's behind-target hold state.
        turret.setCentre();

        autoAimMode = true;
    }

    /**
     * Returns the Pedro pose for the robot placed in the bottom-left field
     * corner, with its localization point assumed to be at the robot's centre.
     *
     * Because the robot is approximately square, ROBOT_LENGTH_MM is used for
     * both the X and Y wall offsets.
     */
    private Pose getCornerResetPose() {

        double halfRobotIn =
                RobotConstants.ROBOT_LENGTH_MM
                        / 2.0
                        / 25.4;

        return p.of(
                halfRobotIn,
                halfRobotIn,
                CORNER_RESET_HEADING_DEG
        );
    }

    // -------------------------------------------------------------------------
    // Wall relocalisation
    //
    // gamepad2 dpad left:
    //     Robot is square against the left wall.
    //
    // gamepad2 dpad right:
    //     Robot is square against the right wall.
    // -------------------------------------------------------------------------

    private void handleRelocalise() {

        boolean leftWall =
                gamepad2.dpadLeftWasPressed();

        boolean rightWall =
                gamepad2.dpadRightWasPressed();

        if (!leftWall && !rightWall) {
            return;
        }

        double halfLengthIn =
                RobotConstants.ROBOT_LENGTH_MM
                        / 25.4
                        / 2.0;

        // Facing from the left wall toward the field is heading 0 degrees.
        // Facing from the right wall toward the field is heading 180 degrees.
        double headingDeg =
                leftWall
                        ? 0.0
                        : 180.0;

        // Reverse the expected direction if the robot front is against the wall.
        if (!RobotConstants.RESET_BACK_AGAINST_WALL) {
            headingDeg += 180.0;
        }

        headingDeg =
                normaliseDegrees0To360(headingDeg);

        double x =
                leftWall
                        ? halfLengthIn
                        : RobotConstants.FIELD_SIZE_IN - halfLengthIn;

        Pose resetPose = p.of(
                x,
                hiveY(),
                headingDeg
        );

        follower.setPose(resetPose);
        PoseStorage.setPose(resetPose);

        turret.setCentre();

        updateGoalFromHive();
        updateTurretTargetFromTeleOpGoal();

        autoAimMode = true;
    }

    // -------------------------------------------------------------------------
    // Turret aiming
    // -------------------------------------------------------------------------

    private void updateTurretAim(Pose currentPose) {

        if (currentPose == null) {
            return;
        }

        if (!autoAimMode) {
            return;
        }

        turret.setTargetPose(
                getTeleOpGoalPose()
        );

        // These velocity values remain zero until field-frame velocity is
        // calculated. Turret.velocityCompensationActive is disabled in init().
        turret.aimTurret(
                currentPose,
                estimatedFieldVelocityX,
                estimatedFieldVelocityY
        );
    }

    private void updateTurretTargetFromTeleOpGoal() {

        turret.setTargetPose(
                getTeleOpGoalPose()
        );
    }

    private Pose getTeleOpGoalPose() {

        return new Pose(
                TELEOP_GOAL_X,
                TELEOP_GOAL_Y,
                0.0
        );
    }

    // -------------------------------------------------------------------------
    // HIVE target geometry
    // -------------------------------------------------------------------------

    /**
     * Returns the Y coordinate of the current alliance's HIVE.
     * An unknown alliance defaults to blue.
     */
    private double hiveY() {

        boolean lowY =
                PoseStorage.isRed()
                        ? !RobotConstants.BLUE_IS_LOW_Y
                        : RobotConstants.BLUE_IS_LOW_Y;

        return RobotConstants.FIELD_CENTER
                + (lowY ? -1.0 : 1.0)
                * RobotConstants.HIVE_Y_OFFSET;
    }

    /**
     * Sets the aiming point to the mouth of the upward CELL at the selected end
     * of the current alliance's HIVE.
     */
    private void updateGoalFromHive() {

        TELEOP_GOAL_X =
                RobotConstants.FIELD_CENTER
                        + (hiveEndRight ? 1.0 : -1.0)
                        * RobotConstants.CELL_X_OFFSET;

        TELEOP_GOAL_Y =
                hiveY();
    }

    // -------------------------------------------------------------------------
    // TeleOp fallback pose
    // -------------------------------------------------------------------------

    private Pose getTeleOpResetPose() {

        return p.of(
                TELEOP_RESET_X,
                TELEOP_RESET_Y,
                TELEOP_RESET_HEADING_DEG
        );
    }

    // -------------------------------------------------------------------------
    // Alliance selection
    // -------------------------------------------------------------------------

    private void applyAllianceFromPoseStorage() {

        if (PoseStorage.isRed()) {

            allianceLabel = "RED";

            TELEOP_GOAL_X = RED_GOAL_X;
            TELEOP_GOAL_Y = RED_GOAL_Y;

            TELEOP_RESET_X = RED_RESET_X;
            TELEOP_RESET_Y = RED_RESET_Y;
            TELEOP_RESET_HEADING_DEG =
                    RED_RESET_HEADING_DEG;

            return;
        }

        if (PoseStorage.isBlue()) {

            allianceLabel = "BLUE";

            TELEOP_GOAL_X = BLUE_GOAL_X;
            TELEOP_GOAL_Y = BLUE_GOAL_Y;

            TELEOP_RESET_X = BLUE_RESET_X;
            TELEOP_RESET_Y = BLUE_RESET_Y;
            TELEOP_RESET_HEADING_DEG =
                    BLUE_RESET_HEADING_DEG;

            return;
        }

        allianceLabel =
                "UNKNOWN - DEFAULT BLUE";

        TELEOP_GOAL_X = BLUE_GOAL_X;
        TELEOP_GOAL_Y = BLUE_GOAL_Y;

        TELEOP_RESET_X = BLUE_RESET_X;
        TELEOP_RESET_Y = BLUE_RESET_Y;
        TELEOP_RESET_HEADING_DEG =
                BLUE_RESET_HEADING_DEG;
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void displayTelemetry() {

        // =====================================================================
        // OUTTAKE TARGETS
        // =====================================================================

        telemetry.addData(
                "Configured TPS",
                RobotConstants.OUTTAKE_TPS
        );

        telemetry.addData(
                "Shooter Mode",
                shooterActive ? "SHOOT" : "IDLE"
        );

        telemetry.addData(
                "Active TPS",
                outtake.getTargetTPS()
        );

        // =====================================================================
        // SHOOTER VELOCITY
        // =====================================================================

        telemetry.addData(
                "Current TPS filtered",
                outtake.getCurrentTPS()
        );

        telemetry.addData(
                "Raw TPS",
                outtake.getRawTPS()
        );

        telemetry.addData(
                "Error TPS",
                outtake.getErrorTPS()
        );

        telemetry.addData(
                "Commanded Power",
                outtake.getCommandedPower()
        );

        telemetry.addData(
                "At Speed",
                outtake.isAtSpeed(
                        RobotConstants.AT_SPEED_TOLERANCE_TPS
                )
        );

        // =====================================================================
        // FEEDER
        // =====================================================================

        telemetry.addData(
                "Feeder Target TPS",
                outtake.getFeederTargetTPS()
        );

        telemetry.addData(
                "Feeder Current TPS",
                outtake.getFeederCurrentTPS()
        );

        telemetry.addData(
                "Feeder Power",
                outtake.getFeederCommandedPower()
        );

        telemetry.addData(
                "Feeder At Speed",
                outtake.isFeederAtSpeed(
                        RobotConstants.AT_SPEED_TOLERANCE_TPS
                )
        );

        // =====================================================================
        // HOOD
        // =====================================================================

        telemetry.addData(
                "Hood Base",
                outtake.getHoodBase()
        );

        telemetry.addData(
                "Hood Compensation",
                outtake.getHoodCompensation()
        );

        telemetry.addData(
                "Hood Commanded",
                outtake.getHoodPosition()
        );

        // =====================================================================
        // VOLTAGE COMPENSATION
        // =====================================================================

        telemetry.addData(
                "Battery Voltage filtered",
                outtake.getVoltage()
        );

        telemetry.addData(
                "Voltage Scale",
                outtake.getVoltageScale()
        );

        // =====================================================================
        // CONTROLLER CONSTANTS
        // =====================================================================

        telemetry.addData("kF", Outtake.kF);
        telemetry.addData("kP", Outtake.kP);

        telemetry.addData(
                "Feeder kF",
                Outtake.FEEDER_kF
        );

        telemetry.addData(
                "Feeder kP",
                Outtake.FEEDER_kP
        );

        telemetry.addData(
                "Hood Comp Gain",
                Outtake.HOOD_COMP_GAIN
        );

        telemetry.addData(
                "Max Power",
                Outtake.MAX_POWER
        );

        // =====================================================================
        // INTAKE
        // =====================================================================

        telemetry.addData(
                "Front Intake Speed",
                RobotConstants.FRONT_INTAKE_SPEED
        );

        telemetry.addData(
                "Back Intake Speed",
                RobotConstants.BACK_INTAKE_SPEED
        );

        telemetry.addData(
                "Timer",
                RobotConstants.timer
        );

        // =====================================================================
        // FIELD AND TURRET
        // =====================================================================

        telemetry.addData(
                "Auto Aim",
                autoAimMode
        );

        telemetry.addData(
                "Alliance",
                allianceLabel
        );

        telemetry.addData(
                "Hive End",
                hiveEndRight
                        ? "RIGHT (high x)"
                        : "LEFT (low x)"
        );

        telemetry.addData(
                "Goal",
                "(%.1f, %.1f)",
                TELEOP_GOAL_X,
                TELEOP_GOAL_Y
        );

        telemetry.addData(
                "Distance to goal",
                "%.2f in",
                turret.getDistanceToTarget()
        );

        telemetry.addData(
                "Field target angle",
                "%.1f deg",
                Turret.lastFieldTargetAngleDeg
        );

        telemetry.addData(
                "Raw relative angle",
                "%.1f deg",
                Turret.lastRelativeAngleDeg
        );

        telemetry.addData(
                "Commanded turret angle",
                "%.1f deg",
                Turret.lastFinalAngleDeg
        );

        telemetry.addData(
                "Turret servo",
                "%.3f",
                Turret.lastServoPosition
        );

        telemetry.addData(
                "Behind hold",
                Turret.lastHoldingBehind
        );

        telemetry.addData(
                "Hold side",
                Turret.lastHoldSide > 0
                        ? "POSITIVE LIMIT"
                        : "NEGATIVE LIMIT"
        );

        telemetry.addData(
                "Turret centre direction",
                "%.1f deg from robot front",
                Turret.TURRET_ZERO_OFFSET_DEG
        );

        telemetry.addData(
                "Pose",
                follower.pose()
        );

        telemetry.addData("SensorTriggered", Intake.sensorTriggered);

        telemetry.update();
    }

    // -------------------------------------------------------------------------
    // Utilities
    // -------------------------------------------------------------------------

    private double normaliseDegrees0To360(double angleDeg) {

        angleDeg %= 360.0;

        if (angleDeg < 0.0) {
            angleDeg += 360.0;
        }

        return angleDeg;
    }

    // -------------------------------------------------------------------------
    // Stop
    // -------------------------------------------------------------------------

    @Override
    public void stop() {

        if (outtake != null) {
            outtake.stopOuttake();
        }

        if (intake != null) {
            intake.stopIntake();
        }

        if (turret != null) {
            turret.setCentre();
        }

        if (follower != null) {

            follower.manual(
                    new DrivePowers(
                            0.0,
                            0.0,
                            0.0
                    )
            );

            follower.update();
        }
    }
}