package org.firstinspires.ftc.teamcode.util;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Direction", group = "Util")
public class Direction extends OpMode {
    public Hardware robot;
    public TelemetryManager telemetryM;
    public Follower follower;
    public Pose startingPose;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap, telemetry);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        drive(gamepad1);

        if(gamepad1.right_bumper)
            for(DcMotorEx motor : robot.motors)
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetryM.debug("LEFT REAR(left): " + robot.leftRear.getCurrentPosition());
        telemetryM.debug("RIGHT FRONT(right): " + robot.rightFront.getCurrentPosition());
        telemetryM.debug("LEFT FRONT(strafe): " + robot.leftFront.getCurrentPosition());
        telemetryM.debug("RIGHT REAR: " + robot.rightRear.getCurrentPosition());
        telemetryM.update(telemetry);
    }

    public void drive(Gamepad gamepad){
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x
        );
        follower.update();
    }
}