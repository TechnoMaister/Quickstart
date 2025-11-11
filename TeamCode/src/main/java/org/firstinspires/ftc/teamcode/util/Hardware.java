package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class Hardware {

    public DcMotorEx leftFront, leftRear, rightFront, rightRear, colector, leftShoot, rightShoot;
    public List<DcMotorEx> motors, shooters;

    public Hardware(HardwareMap robot){
        leftFront = robot.get(DcMotorEx.class, "leftFront");
        leftRear = robot.get(DcMotorEx.class, "leftRear");
        rightRear = robot.get(DcMotorEx.class, "rightFront");
        rightFront = robot.get(DcMotorEx.class, "rightRear");

        colector = robot.get(DcMotorEx.class, "colector");
        leftShoot = robot.get(DcMotorEx.class, "leftShoot");
        rightShoot = robot.get(DcMotorEx.class, "rightShoot");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShoot.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear, colector, leftShoot, rightShoot);
        shooters = Arrays.asList(leftShoot, rightShoot);

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }
    }
}
