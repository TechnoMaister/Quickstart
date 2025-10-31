package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Hardware;

@TeleOp
public class Drive extends OpMode {
    public Hardware robot;
    public Gamepad previousGamepad1, currentGamepad1;
    public Timer rumbling;
    public boolean direction;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap);

        rumbling = new Timer();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
    }

    @Override
    public void loop() {
        drive(gamepad1);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down) direction = !direction;

        if(direction) {
            robot.colector.setDirection(DcMotorSimple.Direction.REVERSE);
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if(rumbling.getElapsedTime() <= 250) gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
        } else {
            robot.colector.setDirection(DcMotorSimple.Direction.FORWARD);
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            rumbling.resetTimer();
        }

        if(gamepad1.cross) robot.colector.setPower(1);
        else robot.colector.setPower(0);

        if(gamepad1.circle) for(DcMotorEx motor : robot.shooters) motor.setPower(1);
        else for(DcMotorEx motor : robot.shooters) motor.setPower(0);
    }

    public void drive(Gamepad gamepad) {
        double y = gamepad.left_stick_y;
        double x = gamepad.left_stick_x;
        double rx = -gamepad.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);
    }
}
