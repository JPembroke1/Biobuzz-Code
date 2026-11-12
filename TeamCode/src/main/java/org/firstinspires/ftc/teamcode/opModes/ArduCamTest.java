/*package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.teamcode.opModes.subClasses.ArduCam;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Slider;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "TeleOp with AprilTag")
public class ArduCamTest extends LinearOpMode {

    private ArduCam camera;
    private static final int DESIRED_TAG_ID = 21; // whichever tag you care about, or loop over all

    @Override
    public void runOpMode() {
        // ... your motor/servo hardwareMap.get() calls go here as usual ...

        camera = new ArduCam(hardwareMap, "Webcam 1")
                .setResolution(640, 480);
        camera.init();

        telemetry.addLine("Ready — waiting for camera to stream");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- your normal driving/gamepad code here ---

            // --- AprilTag telemetry / logic ---
            AprilTagDetection tag = camera.getDetection(DESIRED_TAG_ID);
            if (tag != null) {
                telemetry.addData("Tag " + tag.id, "seen");
                telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
                telemetry.addData("Bearing (deg)", "%.1f", tag.ftcPose.bearing);
                telemetry.addData("Yaw (deg)", "%.1f", tag.ftcPose.yaw);
            } else {
                telemetry.addData("Tag " + DESIRED_TAG_ID, "not visible");
            }

            telemetry.update();
        }

        camera.close();
    }
} */

package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opModes.subClasses.ArduCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


@TeleOp(name = "TeleOp with AprilTag")
public class ArduCamTest extends LinearOpMode {

    private ArduCam camera;
    private static final int DESIRED_TAG_ID = 21; // whichever tag you care about, or loop over all

    @Override
    public void runOpMode() {
        // ... your motor/servo hardwareMap.get() calls go here as usual ...

        camera = new ArduCam(hardwareMap, "Webcam 1")
                .setResolution(640, 480);
        camera.init();
        camera.streamToPanels(); // preview the raw feed live in the Panels dashboard

        telemetry.addLine("Ready — waiting for camera to stream");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- your normal driving/gamepad code here ---

            // --- AprilTag telemetry / logic ---
            AprilTagDetection tag = camera.getDetection(DESIRED_TAG_ID);
            if (tag != null) {
                telemetry.addData("Range (in)", "%.1f", tag.ftcPose.range);
                telemetry.addData("Bearing (deg)", "%.1f", tag.ftcPose.bearing);
                telemetry.addData("Yaw (deg)", "%.1f", tag.ftcPose.yaw);
            } else {
                telemetry.addData("Tag " + DESIRED_TAG_ID, "not visible");
            }

            telemetry.update();
        }

        camera.close();
    }
}