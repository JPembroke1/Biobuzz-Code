package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
@TeleOp(name = "Intake Test", group = "Main")
public class IntakeTest extends OpMode {

    // -------------------------------------------------------------------------
    // Subsystems
    // -------------------------------------------------------------------------

    private Intake intake;
    private Outtake outtake;
    private Turret turret;

    private Follower follower;


    // -------------------------------------------------------------------------
    // Drivetrain hardware
    // -------------------------------------------------------------------------

    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;


    // -------------------------------------------------------------------------
    // Block servo
    // -------------------------------------------------------------------------

    private Servo blockServo;


    // -------------------------------------------------------------------------
    // Alliance
    // -------------------------------------------------------------------------

    private String allianceLabel = "UNKNOWN";


    // -------------------------------------------------------------------------
    // Goal coordinates
    //
    // Red is on the field's X = 0 side.
    // Blue is on the opposite side.
    // -------------------------------------------------------------------------

    public static double RED_GOAL_X = 6.0;
    public static double RED_GOAL_Y = 138.0;

    public static double BLUE_GOAL_X = 138.0;
    public static double BLUE_GOAL_Y = 138.0;


    // -------------------------------------------------------------------------
    // TeleOp reset poses
    // -------------------------------------------------------------------------

    public static double BLUE_RESET_X = 24.0;
    public static double BLUE_RESET_Y = 138.0;
    public static double BLUE_RESET_HEADING_DEG = 145.0;

    public static double RED_RESET_X = 120.0;
    public static double RED_RESET_Y = 138.0;
    public static double RED_RESET_HEADING_DEG = 35.0;


    // -------------------------------------------------------------------------
    // Active reset pose
    //
    // These values are updated from PoseStorage during initialization.
    // -------------------------------------------------------------------------

    public static double TELEOP_RESET_X = BLUE_RESET_X;
    public static double TELEOP_RESET_Y = BLUE_RESET_Y;

    public static double TELEOP_RESET_HEADING_DEG =
            BLUE_RESET_HEADING_DEG;


    // -------------------------------------------------------------------------
    // Active goal
    // -------------------------------------------------------------------------

    public static double TELEOP_GOAL_X = BLUE_GOAL_X;
    public static double TELEOP_GOAL_Y = BLUE_GOAL_Y;


    // -------------------------------------------------------------------------
    // Button edge detection
    // -------------------------------------------------------------------------

    private boolean previousPoseResetButton = false;
    private boolean previousAutoAimButton = false;
    private boolean previousCentreButton = false;


    // -------------------------------------------------------------------------
    // Turret state
    // -------------------------------------------------------------------------

    private boolean autoAimMode = true;

    private double estimatedFieldVelocityX = 0.0;
    private double estimatedFieldVelocityY = 0.0;


    // -------------------------------------------------------------------------
    // Feed timing
    // -------------------------------------------------------------------------

    private long feedStartTimeMs = -1;


    // -------------------------------------------------------------------------
    // Initialization
    // -------------------------------------------------------------------------

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);


        // ---------------------------------------------------------------------
        // Intake
        // ---------------------------------------------------------------------

        intake = new Intake();
        intake.init(hardwareMap);


        // ---------------------------------------------------------------------
        // Outtake
        // ---------------------------------------------------------------------

        outtake = new Outtake();
        outtake.init(hardwareMap);


        // ---------------------------------------------------------------------
        // Alliance and turret
        // ---------------------------------------------------------------------

        applyAllianceFromPoseStorage();

        /*
         * Transfer the field coordinates into the Turret subsystem before
         * calling turret.init().
         *
         * turret.init() selects the stored alliance target.
         */
        Turret.RED_GOAL_X = RED_GOAL_X;
        Turret.RED_GOAL_Y = RED_GOAL_Y;

        Turret.BLUE_GOAL_X = BLUE_GOAL_X;
        Turret.BLUE_GOAL_Y = BLUE_GOAL_Y;

        turret = new Turret();
        turret.init(hardwareMap);


        // ---------------------------------------------------------------------
        // Drivetrain
        // ---------------------------------------------------------------------

        frontRightMotor = hardwareMap.get(
                DcMotor.class,
                "frontRight"
        );

        frontLeftMotor = hardwareMap.get(
                DcMotor.class,
                "frontLeft"
        );

        backRightMotor = hardwareMap.get(
                DcMotor.class,
                "backRight"
        );

        backLeftMotor = hardwareMap.get(
                DcMotor.class,
                "backLeft"
        );

        frontRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        backRightMotor.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        frontLeftMotor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        backLeftMotor.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        frontRightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        frontLeftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backRightMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        backLeftMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );


        // ---------------------------------------------------------------------
        // Block servo
        // ---------------------------------------------------------------------

        blockServo = hardwareMap.get(
                Servo.class,
                "blockServo"
        );

        blockServo.setDirection(
                Servo.Direction.FORWARD
        );

        blockServo.setPosition(
                RobotConstants.blockClosePos
        );


        // ---------------------------------------------------------------------
        // Initial turret state
        // ---------------------------------------------------------------------

        turret.selectAllianceGoal();
        turret.setCentre();


        // ---------------------------------------------------------------------
        // Initial localization pose
        // ---------------------------------------------------------------------

        if (PoseStorage.currentPose != null) {

            follower.setPose(
                    PoseStorage.currentPose
            );

        } else {

            Pose resetPose =
                    getTeleOpResetPose();

            follower.setPose(resetPose);
            PoseStorage.setPose(resetPose);
        }


        limitIntakeSpeeds();

        telemetry.addLine("Initialization complete");

        telemetry.addData(
                "Alliance",
                allianceLabel
        );

        telemetry.addData(
                "Selected Goal",
                turret.getSelectedGoal()
        );

        telemetry.addData(
                "Goal",
                "(%.1f, %.1f)",
                TELEOP_GOAL_X,
                TELEOP_GOAL_Y
        );

        telemetry.update();
    }


    // -------------------------------------------------------------------------
    // Start
    // -------------------------------------------------------------------------

    @Override
    public void start() {

        /*
         * Reapply the alliance in case PoseStorage was changed during the
         * initialization period.
         */
        applyAllianceFromPoseStorage();

        Turret.RED_GOAL_X = RED_GOAL_X;
        Turret.RED_GOAL_Y = RED_GOAL_Y;

        Turret.BLUE_GOAL_X = BLUE_GOAL_X;
        Turret.BLUE_GOAL_Y = BLUE_GOAL_Y;

        turret.selectAllianceGoal();

        autoAimMode = true;
    }


    // -------------------------------------------------------------------------
    // Main loop
    // -------------------------------------------------------------------------

    @Override
    public void loop() {

        /*
         * Update Pedro Pathing before reading the current pose.
         */
        follower.update();

        handleControls();
        handlePoseReset();
        handleTurretControls();

        driveRobot();

        intake.update();
        outtake.updatePF();

        Pose currentPose =
                follower.pose();

        PoseStorage.setPose(
                currentPose
        );

        updateTurretAim(
                currentPose
        );

        displayTelemetry(
                currentPose
        );
    }


    // -------------------------------------------------------------------------
    // Mechanism controls
    // -------------------------------------------------------------------------

    private void handleControls() {

        // ---------------------------------------------------------------------
        // Intake
        //
        // Right bumper: collect
        // Right trigger: open block, delay, then feed
        // Left trigger: reverse intake
        // ---------------------------------------------------------------------

        if (gamepad1.right_bumper) {

            intake.intake(
                    RobotConstants.FRONT_INTAKE_SPEED,
                    RobotConstants.BACK_INTAKE_SPEED
            );

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );

            RobotConstants.timer = 0.0;
            feedStartTimeMs = -1;

        } else if (gamepad1.right_trigger > 0.3) {

            blockServo.setPosition(
                    RobotConstants.blockOpenPos
            );

            if (feedStartTimeMs < 0) {

                feedStartTimeMs =
                        System.currentTimeMillis();
            }

            double feedElapsedSeconds =
                    (System.currentTimeMillis()
                            - feedStartTimeMs) / 1000.0;

            RobotConstants.timer =
                    feedElapsedSeconds;

            if (feedElapsedSeconds
                    >= RobotConstants.delayAmount) {

                intake.intake(
                        RobotConstants.FRONT_INTAKE_SPEED,
                        RobotConstants.BACK_INTAKE_SPEED
                );

            } else {

                intake.stopIntake();
            }

        } else if (gamepad1.left_trigger > 0.3) {

            intake.intake(
                    -RobotConstants.FRONT_INTAKE_SPEED,
                    -RobotConstants.BACK_INTAKE_SPEED
            );

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );

            RobotConstants.timer = 0.0;
            feedStartTimeMs = -1;

        } else {

            intake.stopIntake();

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );

            RobotConstants.timer = 0.0;
            feedStartTimeMs = -1;
        }


        // ---------------------------------------------------------------------
        // Outtake
        //
        // Cross: start shooter
        // Triangle: stop shooter
        // ---------------------------------------------------------------------

        if (gamepad1.cross) {

            outtake.setTargetTPS(
                    RobotConstants.OUTTAKE_TPS
            );
        }

        if (gamepad1.triangle) {

            outtake.stopOuttake();
        }
    }


    // -------------------------------------------------------------------------
    // Turret controls
    // -------------------------------------------------------------------------

    private void handleTurretControls() {

        /*
         * Left bumper toggles automatic aiming.
         *
         * Reverse intake uses left trigger, avoiding a control conflict.
         */
        boolean currentAutoAimButton =
                gamepad1.left_bumper;

        if (currentAutoAimButton
                && !previousAutoAimButton) {

            autoAimMode =
                    !autoAimMode;

            if (autoAimMode) {

                /*
                 * Select the stored alliance goal.
                 *
                 * This does not cycle to another goal.
                 */
                turret.selectAllianceGoal();

            } else {

                turret.setCentre();
            }
        }

        previousAutoAimButton =
                currentAutoAimButton;


        /*
         * Square centres the turret at servo position 0.5.
         *
         * This disables auto aim but does not change the selected goal.
         */
        boolean currentCentreButton =
                gamepad1.square;

        if (currentCentreButton
                && !previousCentreButton) {

            autoAimMode = false;
            turret.setCentre();
        }

        previousCentreButton =
                currentCentreButton;
    }


    private void updateTurretAim(
            Pose currentPose
    ) {

        if (autoAimMode) {

            turret.aimTurret(
                    currentPose,
                    estimatedFieldVelocityX,
                    estimatedFieldVelocityY
            );

        } else {

            turret.setCentre();
        }
    }


    // -------------------------------------------------------------------------
    // Pose reset
    // -------------------------------------------------------------------------

    private void handlePoseReset() {

        /*
         * Options resets Pedro localization and restores auto aim.
         */
        boolean currentPoseResetButton =
                gamepad1.options;

        if (currentPoseResetButton
                && !previousPoseResetButton) {

            Pose resetPose =
                    getTeleOpResetPose();

            follower.setPose(resetPose);
            PoseStorage.setPose(resetPose);

            /*
             * Re-select the correct alliance goal.
             *
             * This cannot switch to the opposing alliance goal.
             */
            turret.selectAllianceGoal();

            autoAimMode = true;
        }

        previousPoseResetButton =
                currentPoseResetButton;
    }


    // -------------------------------------------------------------------------
    // Drivetrain
    // -------------------------------------------------------------------------

    private void driveRobot() {

        double forward =
                gamepad1.left_stick_y
                        * RobotConstants.DRIVE_SPEED;

        double strafe =
                -gamepad1.left_stick_x
                        * RobotConstants.DRIVE_SPEED;

        double turn =
                -gamepad1.right_stick_x
                        * RobotConstants.DRIVE_SPEED;

        double denominator =
                Math.max(
                        Math.abs(forward)
                                + Math.abs(strafe)
                                + Math.abs(turn),
                        1.0
                );

        double frontLeftPower =
                (forward + strafe + turn)
                        / denominator;

        double backLeftPower =
                (forward - strafe + turn)
                        / denominator;

        double frontRightPower =
                (forward - strafe - turn)
                        / denominator;

        double backRightPower =
                (forward + strafe - turn)
                        / denominator;

        frontLeftMotor.setPower(
                frontLeftPower
        );

        frontRightMotor.setPower(
                frontRightPower
        );

        backLeftMotor.setPower(
                backLeftPower
        );

        backRightMotor.setPower(
                backRightPower
        );
    }


    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void displayTelemetry(
            Pose currentPose
    ) {

        telemetry.addLine("=== ALLIANCE AND TARGET ===");

        telemetry.addData(
                "Alliance",
                allianceLabel
        );

        telemetry.addData(
                "Selected Goal",
                turret.getSelectedGoal()
        );

        telemetry.addData(
                "Goal X",
                "%.1f",
                turret.getTargetPose().x()
        );

        telemetry.addData(
                "Goal Y",
                "%.1f",
                turret.getTargetPose().y()
        );

        telemetry.addData(
                "Distance to Goal",
                "%.1f cm",
                turret.getDistanceToTarget() * 2.54
        );


        telemetry.addLine("=== TURRET ===");

        telemetry.addData(
                "Auto Aim",
                autoAimMode
        );

        telemetry.addData(
                "Servo Position",
                "%.3f",
                turret.getServoPosition()
        );

        telemetry.addData(
                "Turret Angle",
                "%.1f deg",
                Turret.lastFinalAngleDeg
        );

        telemetry.addData(
                "Relative Target Angle",
                "%.1f deg",
                Turret.lastRelativeAngleDeg
        );

        telemetry.addData(
                "Holding At Limit",
                turret.isHoldingBehind()
        );


        telemetry.addLine("=== OUTTAKE ===");

        telemetry.addData(
                "Configured TPS",
                RobotConstants.OUTTAKE_TPS
        );

        telemetry.addData(
                "Active TPS",
                outtake.getTargetTPS()
        );

        telemetry.addData(
                "Current TPS",
                outtake.getCurrentTPS()
        );

        telemetry.addData(
                "Front TPS",
                outtake.getFrontVelocityTPS()
        );

        telemetry.addData(
                "Back TPS",
                outtake.getBackVelocityTPS()
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
                outtake.isAtSpeed(50.0)
        );

        telemetry.addData(
                "kF",
                Outtake.kF
        );

        telemetry.addData(
                "kP",
                Outtake.kP
        );

        telemetry.addData(
                "Maximum Power",
                Outtake.MAX_POWER
        );


        telemetry.addLine("=== INTAKE ===");

        telemetry.addData(
                "Front Intake Speed",
                RobotConstants.FRONT_INTAKE_SPEED
        );

        telemetry.addData(
                "Back Intake Speed",
                RobotConstants.BACK_INTAKE_SPEED
        );

        telemetry.addData(
                "Ball Sensor",
                Intake.sensorTriggered
        );

        telemetry.addData(
                "Ball Confirmed",
                Intake.ballConfirmed
        );

        telemetry.addData(
                "Feed Timer",
                "%.2f",
                RobotConstants.timer
        );


        telemetry.addLine("=== LOCALIZATION ===");

        telemetry.addData(
                "Pose X",
                "%.2f",
                currentPose.x()
        );

        telemetry.addData(
                "Pose Y",
                "%.2f",
                currentPose.y()
        );

        telemetry.addData(
                "Heading",
                "%.1f deg",
                Math.toDegrees(
                        currentPose.heading()
                )
        );

        telemetry.update();
    }


    // -------------------------------------------------------------------------
    // Alliance configuration
    // -------------------------------------------------------------------------

    private void applyAllianceFromPoseStorage() {

        if (PoseStorage.isRed()) {

            allianceLabel = "RED";

            TELEOP_GOAL_X =
                    RED_GOAL_X;

            TELEOP_GOAL_Y =
                    RED_GOAL_Y;

            TELEOP_RESET_X =
                    RED_RESET_X;

            TELEOP_RESET_Y =
                    RED_RESET_Y;

            TELEOP_RESET_HEADING_DEG =
                    RED_RESET_HEADING_DEG;

            return;
        }

        if (PoseStorage.isBlue()) {

            allianceLabel = "BLUE";

            TELEOP_GOAL_X =
                    BLUE_GOAL_X;

            TELEOP_GOAL_Y =
                    BLUE_GOAL_Y;

            TELEOP_RESET_X =
                    BLUE_RESET_X;

            TELEOP_RESET_Y =
                    BLUE_RESET_Y;

            TELEOP_RESET_HEADING_DEG =
                    BLUE_RESET_HEADING_DEG;

            return;
        }

        /*
         * Default to blue when the alliance has not been stored.
         */
        allianceLabel =
                "UNKNOWN, DEFAULT BLUE";

        TELEOP_GOAL_X =
                BLUE_GOAL_X;

        TELEOP_GOAL_Y =
                BLUE_GOAL_Y;

        TELEOP_RESET_X =
                BLUE_RESET_X;

        TELEOP_RESET_Y =
                BLUE_RESET_Y;

        TELEOP_RESET_HEADING_DEG =
                BLUE_RESET_HEADING_DEG;
    }


    private Pose getTeleOpResetPose() {

        return new Pose(
                TELEOP_RESET_X,
                TELEOP_RESET_Y,
                Math.toRadians(
                        TELEOP_RESET_HEADING_DEG
                )
        );
    }


    // -------------------------------------------------------------------------
    // Utility methods
    // -------------------------------------------------------------------------

    private void limitIntakeSpeeds() {

        RobotConstants.FRONT_INTAKE_SPEED =
                clip(
                        RobotConstants.FRONT_INTAKE_SPEED,
                        -1.0,
                        1.0
                );

        RobotConstants.BACK_INTAKE_SPEED =
                clip(
                        RobotConstants.BACK_INTAKE_SPEED,
                        -1.0,
                        1.0
                );
    }


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

        if (frontLeftMotor != null) {
            frontLeftMotor.setPower(0.0);
        }

        if (frontRightMotor != null) {
            frontRightMotor.setPower(0.0);
        }

        if (backLeftMotor != null) {
            backLeftMotor.setPower(0.0);
        }

        if (backRightMotor != null) {
            backRightMotor.setPower(0.0);
        }

        if (blockServo != null) {

            blockServo.setPosition(
                    RobotConstants.blockClosePos
            );
        }
    }
}