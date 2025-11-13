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
    //public IMU imu;

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

        /*imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);*/
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        robot.aprilTagWebcam.update();
        id20 = robot.aprilTagWebcam.getTagBySpecificID(20);
        robot.aprilTagWebcam.displayDetectionTelemetry(id20);

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
        /*double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;


     //   double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
     //   double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
     //   double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

     //   rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
     //   double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
     //   double leftFrontPower = (rotY + rotX + rx) / denominator;
     //   double leftRearPower = (rotY - rotX + rx) / denominator;
     //   double rightFrontPower = (rotY - rotX - rx) / denominator;
     //   double rightRearPower = (rotY + rotX - rx) / denominator;

        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);*/
        follower.setTeleOpDrive(
                -gamepad.left_stick_y,
                -gamepad.left_stick_x,
                -gamepad.right_stick_x,
                true
        );
    }
}