package org.firstinspires.ftc.teamcode.teleOp;

import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerBlockedPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.blockerOpenPos;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.colectorPowerB;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.colectorPowerD;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.colectorPowerN;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.k;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.rumblingT;
import static org.firstinspires.ftc.teamcode.util.RobotConstants.tolerance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.BasketLauncher;
import org.firstinspires.ftc.teamcode.util.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Drive", group = "TeleOp")
public class Drive extends OpMode {
    public Hardware robot;
    public Follower follower;
    public Pose startingPose;
    public Gamepad previousGamepad1, currentGamepad1;
    public Timer rumbling, rumbling2, block;
    public boolean direction, team;
    public AprilTagDetection id;
    public BasketLauncher power;
    public double shooterPower;

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

        power = new BasketLauncher();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        robot.aprilTagWebcam.update();

        if (id != null) {
            shooterPower = power.getMotorPower(id.ftcPose.y, hardwareMap);
            telemetry.addLine("I see!");
        }

        telemetry.addData("shooterPower", shooterPower);

        drive(gamepad1);

        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.right_trigger > 0 && previousGamepad1.right_trigger == 0) direction = !direction;
        if (currentGamepad1.cross && !previousGamepad1.cross) team = !team;

        if(team) {
            id = robot.aprilTagWebcam.getTagBySpecificID(24);
            telemetry.addLine("RED");
        } else {
            id = robot.aprilTagWebcam.getTagBySpecificID(20);
            telemetry.addLine("BLUE");
        }

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

        if (gamepad1.left_bumper) robot.colector.setPower(colectorPowerD);
        else if (!gamepad1.right_bumper) robot.colector.setPower(0);

        if (gamepad1.right_bumper && id != null) {
            if(robot.aprilTagWebcam.getHorizontalOffset(id) >= -tolerance &&
                    robot.aprilTagWebcam.getHorizontalOffset(id) <= tolerance) {
                for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setPower(shooterPower);
                if(block.getElapsedTime() >= blockT) {
                    for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerOpenPos);
                    robot.colector.setPower(colectorPowerN);
                }
                else {
                    for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
                    robot.colector.setPower(colectorPowerB);
                }
            } else if(robot.aprilTagWebcam.getHorizontalOffset(id) < -tolerance) {
                robot.leftFront.setPower(-k);
                robot.leftRear.setPower(-k);
                robot.rightFront.setPower(k);
                robot.rightRear.setPower(k);
            } else {
                robot.leftFront.setPower(k);
                robot.leftRear.setPower(k);
                robot.rightFront.setPower(-k);
                robot.rightRear.setPower(-k);
            }
        } else {
            for (DcMotorEx shooterMotor : robot.shooters) shooterMotor.setPower(0);
            for(Servo blockerMotor : robot.blockers) blockerMotor.setPosition(blockerBlockedPos);
            block.resetTimer();
        }
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