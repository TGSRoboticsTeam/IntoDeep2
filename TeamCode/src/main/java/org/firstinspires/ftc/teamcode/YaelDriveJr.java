package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//@Disabled
@TeleOp(name = "YaelDriveJr", group = "YaelDriveJr")

public class YaelDriveJr extends LinearOpMode {
    @Override
    public void runOpMode() {
        //GamepadEx gamepadEx = new GamepadEx(gamepad2); // probably not needed...
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        hang.setDirection(DcMotorSimple.Direction.FORWARD);
        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
        linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Makes the motors output their rotation
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo armServo = hardwareMap.get(Servo.class, "arm_servo");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        Servo grabber = hardwareMap.get(Servo.class, "grabber_servo");

        Servo rightWhacker = hardwareMap.get(Servo.class, "right_whacker")
        Servo leftWhacker = hardwareMap.get(Servo.class, "left_whacker")

        // Motor Setup
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Sets the motor direction
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Makes the motors stop moving when they receive an input of 0
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up FtcDashboard telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        double changeInSpeed = 0.35;
        boolean justGrabbed = false;
        //String returnArmPos = "awaiting input...";

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // Change to left if doesn't work
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        while (!isStarted()) {

        }

        while (opModeIsActive()) {
            // Define joystick controls
            //double moveSlide = gamepad2.left_trigger - gamepad2.right_trigger;
            double moveSlide = -gamepad2.right_stick_y;
            double moveHang = -gamepad2.left_stick_y;

            boolean toggleGrabber = gamepad2.right_bumper || gamepad2.right_trigger >= 0.1;

            boolean shiftForwards = gamepad2.a;
            boolean shiftBackwards = gamepad2.b;

            boolean armPosScoop = gamepad2.dpad_right;
            boolean armPosUp = gamepad2.dpad_up;
            boolean armPosBack = gamepad2.dpad_left;
            boolean armPosDown = gamepad2.dpad_down;

            // Drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            boolean slowDown = gamepad1.right_bumper || gamepad1.right_trigger > 0.1;

            if (gamepad2.left_bumper && linearSlide.getCurrentPosition() <= 295) {// && moveSlide < 0.1) {
                linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (gamepad1.dpad_up) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bots rotation
            double strafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double drive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            strafe = -strafe; // here's the strafe inversion

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rot), 1);
            double frontLeftPower = (drive + strafe + rot) / denominator;
            double backLeftPower = (drive - strafe + rot) / denominator;
            double frontRightPower = (drive - strafe - rot) / denominator;
            double backRightPower = (drive + strafe - rot) / denominator;
            //telemetry.addData("fl, bl,fr,br: ", "%.2f %.2f %.2f %.2f",frontLeftPower,backLeftPower,frontRightPower,backRightPower);

            if (slowDown) {
                frontLeftPower *= changeInSpeed;
                frontRightPower *= changeInSpeed;
                backLeftPower *= changeInSpeed;
                backRightPower *= changeInSpeed;
            }

            double roundDown = 0.05;
            if (Math.abs(frontLeftPower) <= roundDown) {
                frontLeftPower = 0;
            }
            if (Math.abs(frontRightPower) <= roundDown) {
                frontRightPower = 0;
            }
            if (Math.abs(backLeftPower) <= roundDown) {
                backLeftPower = 0;
            }
            if (Math.abs(backRightPower) <= roundDown) {
                backRightPower = 0;
            }

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            //////////////// OTHER COMPONENTS //////////////////
            if (linearSlide.getCurrentPosition() >= 2210 && moveSlide > 0) {
                moveSlide = 0;
            } else if (linearSlide.getCurrentPosition() <= 2 && moveSlide < 0) {
                moveSlide = 0;
            } else if (moveSlide == 0 && linearSlide.getCurrentPosition() >= 0) {
                moveSlide = 0.1;
            }
            double linearSlowDown = 0.75;
            linearSlide.setPower(moveSlide * linearSlowDown);

            // Move hang
            hang.setPower(moveHang);

            telemetry.addData("Linear Slide", linearSlide.getCurrentPosition());
            telemetry.addData("Right Trigger", toggleGrabber);
            //*
            if (armPosDown) { // right (good)
                armServo.setPosition(0.21); // down 0.20 also works ok
                wristServo.setPosition(0.97); // down
            }else if (armPosUp) { // up (good)
                armServo.setPosition(0.55); // up
                wristServo.setPosition(1.0); // up
            }else if (armPosBack) { // left (good)
                armServo.setPosition(0.75); // back
                wristServo.setPosition(0.62); // back 0.62
            }else if (armPosScoop) { // down (good)
                armServo.setPosition(0.01); // scoop 0.0
                wristServo.setPosition(0.56); // scoop 0.56
            }

            //armServo.setPosition(0.3); // flat
            //wristServo.setPosition(0.8); // flat

            // Grabbing
            double grabbingPos = 1;
            if (toggleGrabber && !justGrabbed) {
                justGrabbed = true;
                if (grabber.getPosition() == grabbingPos) {
                    grabber.setPosition(0.5);
                } else {
                    grabber.setPosition(grabbingPos);
                }
            }/* else if (openGrabber) {
                grabber.setPosition(grabbingPos);
            } else if (closeGrabber) {
                grabber.setPosition(0.5);
            }*/ else if (!toggleGrabber) {
                justGrabbed = false;
            }
            //*/

            if (shiftForwards) {
                rightWhacker.setPosition(1);
                leftWhacker.setPosition(1);
            }else if (shiftBackwards){
                rightWhacker.setPosition(0);
                leftWhacker.setPosition(0);
            }

            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            telemetry.addData("Pitch (X)", "%.2f", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw (Z)", "%.2f", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}
