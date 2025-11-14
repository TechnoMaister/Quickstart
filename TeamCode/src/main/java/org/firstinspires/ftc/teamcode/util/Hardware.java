package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

public class Hardware {

    public AprilTagWebcam aprilTagWebcam;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear, colector, leftShoot, rightShoot;
    public Servo blocker;
    public List<DcMotorEx> motors, shooters;

    public Hardware(HardwareMap hardwareMap, Telemetry telemetry){
        //aprilTagWebcam = new AprilTagWebcam();
        //aprilTagWebcam.init(hardwareMap, telemetry);

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightRear");

        colector = hardwareMap.get(DcMotorEx.class, "colector");
        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");

        blocker = hardwareMap.get(Servo.class, "blocker");

        blocker.setDirection(Servo.Direction.REVERSE);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        rightShoot.setDirection(DcMotorSimple.Direction.REVERSE);
        leftShoot.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear, colector, leftShoot, rightShoot);
        shooters = Arrays.asList(leftShoot, rightShoot);

        for (DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public double getBatteryVoltage(HardwareMap hardwareMap) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
                result = Math.min(result, voltage);
        }
        return result;
    }
}
