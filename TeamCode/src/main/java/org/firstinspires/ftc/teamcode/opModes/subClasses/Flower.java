package org.firstinspires.ftc.teamcode.opModes.subClasses;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@Configurable
public class Flower {
    public Servo flower;

    public void init(HardwareMap hardwareMap) {
        flower = hardwareMap.get(Servo.class, "flower");
        flower.setPosition(RobotConstants.flowerLiftPos);
    }

    public void flowerActivate(double flowerPos) {
        flower.setPosition(flowerPos);
    }



}
