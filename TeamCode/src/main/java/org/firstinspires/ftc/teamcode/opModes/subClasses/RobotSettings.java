package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotSettings {

    private Servo blockServo;

    public void init(HardwareMap hardwareMap) {

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
    }

    public void block() {
        blockServo.setPosition(
                RobotConstants.blockClosePos
        );
    }

    public void release() {
        blockServo.setPosition(
                RobotConstants.blockOpenPos
        );
    }
}
