package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Encoder {

    public Encoder(){}

    public void runTo(DcMotorEx motor, int position) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(1);
    }
}
