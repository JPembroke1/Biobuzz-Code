package org.firstinspires.ftc.teamcode.opModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@Configurable
@TeleOp(name = "SensorTest")
public class SensorTest extends OpMode {

        public DigitalChannel s;

        public void init() {
            s = hardwareMap.get(DigitalChannel.class, "ballSensor");
            s.setMode(DigitalChannel.Mode.INPUT);
        }

        public void loop() {
            telemetry.addData("state", s.getState());
            telemetry.update();
        }
}