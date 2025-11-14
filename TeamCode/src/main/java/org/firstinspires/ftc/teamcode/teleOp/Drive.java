package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.colectorPowerD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.colectorPowerN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.shooterPower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class Drive extends OpMode {
    public Hardware robot;
    public Follower follower;
    public static Pose startingPose;
    public Gamepad previousGamepad1, currentGamepad1;
    public Timer rumbling, rumbling2, block;
    public boolean direction;
    public Telemetry telemetry;
    public AprilTagDetection id20;

    @Override
    public void init() {
        robot = new Hardware(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        rumbling = new Timer();
        rumbling2 = new Timer();
        block = new Timer();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //robot.aprilTagWebcam.update();
        //id20 = robot.aprilTagWebcam.getTagBySpecificID(20);
        //robot.aprilTagWebcam.displayDetectionTelemetry(id20);

        follower.update();
        drive(gamepad1);

        telemetry.addData("voltage", "%.1f volts", new Func<Double>() { @Override public Double value() { return robot.getBatteryVoltage(hardwareMap); } });

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.left_trigger > 0 && previousGamepad1.left_trigger == 0) direction = !direction;

        if (direction) {
            robot.colector.setDirection(DcMotorSimple.Direction.REVERSE);
            gamepad1.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if (rumbling.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling2.resetTimer();
        } else {
            robot.colector.setDirection(DcMotorSimple.Direction.FORWARD);
            gamepad1.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
            if (rumbling2.getElapsedTime() <= rumblingT)
                gamepad1.rumble(Gamepad.RUMBLE_DURATION_CONTINUOUS);
            else gamepad1.stopRumble();
            rumbling.resetTimer();
        }

        if (gamepad1.left_bumper) robot.colector.setPower(colectorPowerN);
        else if (!gamepad1.right_bumper) robot.colector.setPower(0);

        if (gamepad1.right_bumper) {
            for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setPower(shooterPower);

            if(block.getElapsedTime() >= blockT) {
                robot.blocker.setPosition(blockerOpenPos);
                robot.colector.setPower(colectorPowerD);
            }
            else {
                robot.blocker.setPosition(blockerBlockedPos);
                robot.colector.setPower(0);
            }

        }
        else {
            for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setPower(0);
            robot.blocker.setPosition(blockerBlockedPos);
            block.resetTimer();
        }
    }

    public void drive(Gamepad gamepad) {
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true
        );
    }
}