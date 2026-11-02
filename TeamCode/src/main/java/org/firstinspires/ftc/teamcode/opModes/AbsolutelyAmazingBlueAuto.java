package org.firstinspires.ftc.teamcode.opModes;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.commands.Commands.*;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.*;

import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.opModes.subClasses.Intake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Outtake;
import org.firstinspires.ftc.teamcode.opModes.subClasses.PoseStorage;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotConstants;
import org.firstinspires.ftc.teamcode.opModes.subClasses.RobotHardware;
import org.firstinspires.ftc.teamcode.opModes.subClasses.Turret;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.FileOutputStream;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Configurable
@Autonomous(name = "00 - Blue Auto", group = "Main")
public class AbsolutelyAmazingBlueAuto extends OpMode {

    private List<LynxModule> allHubs;
    private Follower follower;
    private Outtake outtake;
    private RobotHardware robotHardware;
    private Intake intake;
    private Turret turret;

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(8, 8, Math.toRadians(180));
    private final Pose pickupFirstRow = p.of(56, 8, Math.toRadians(180));
    private final Pose returnToShooter = p.of(8, 8, Math.toRadians(180));

    private Path pathToCollectFirstRow() {
        return line(start, pickupFirstRow).linear(start, pickupFirstRow);
    }

    private Path pathToReturnToShootFromFirstRow() {
        return line(pickupFirstRow, returnToShooter).linear(pickupFirstRow, returnToShooter);
    }


    public void init() {
        Scheduler.reset();

        configureBulkCaching();
        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(start);

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        outtake = new Outtake();
        intake = new Intake();
        turret = new Turret();

        robotHardware = new RobotHardware(hardwareMap);

        outtake.init(hardwareMap);
        intake.init(hardwareMap);

        intake.stopIntake();
        outtake.stopOuttake();

    }

    public void init_loop() {
        clearBulkCache();

        PoseStorage.setBlue();

        Pose currentPose = follower.pose();
        PoseStorage.setPose(currentPose);
    }

    public void start() {
        //Scheduler.reset();

        clearBulkCache();

        PoseStorage.setBlue();
        PoseStorage.setPose(start);

        turret.setTarget(0.5);

        schedule(autoRoutine());

    }

    public void loop() {

        //clearBulkCache();

        follower.update();
        Scheduler.execute();

        Pose currentPose = follower.pose();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();

    }

    private Command autoRoutine() {
        return sequential(
                followCollect(pathToCollectFirstRow()),

                follow(follower, pathToReturnToShootFromFirstRow())
        );
    }

    private Command followCollect(Path path) {
        return parallel(
                follow(follower, path),

                sequential(
                        instant(() -> {
                            robotHardware.block();
                            intake.intake(
                                    RobotConstants.FRONT_INTAKE_SPEED,
                                    RobotConstants.BACK_INTAKE_SPEED
                            );
                        })
                )
        );
    }


    private void robotPeriodic() {
        follower.update();

        Pose currentPose = follower.pose();
        PoseStorage.setPose(currentPose);

        intake.update();

        turret.aimAt(
                currentPose.x(),
                currentPose.y(),
                currentPose.heading(),
                0.0,
                0.0
        );
    }


    private void configureBulkCaching() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    private void clearBulkCache() {
        if (allHubs == null) {
            return;
        }

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

}
