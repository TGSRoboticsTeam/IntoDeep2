package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "DoubleSlideDrive", group = "DoubleSlideDrive")
public class DoubleSlideDrive extends LinearOpMode {

    private static final int SLIDE_TOP_POSITION = 2088;

    @Override
    public void runOpMode() {

        DcMotor rightLinearSlide = hardwareMap.get(DcMotor.class, "right_slide");
        rightLinearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor leftLinearSlide = hardwareMap.get(DcMotor.class, "left_slide");
        leftLinearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftLinearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo leftShoulder = hardwareMap.get(Servo.class, "left_shoulder");
        Servo rightShoulder = hardwareMap.get(Servo.class, "right_shoulder");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        Servo grabber = hardwareMap.get(Servo.class, "grabber_servo");

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double changeInSpeed = 0.35;
        boolean grabberClosed = false;

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters();
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            double moveSlide = gamepad1.right_trigger - gamepad1.left_trigger;

            if (moveSlide > 0) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else if (moveSlide < 0 && bothSlidesAboveLimit(leftLinearSlide, rightLinearSlide, 0)) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else {
                leftLinearSlide.setPower(0);
                rightLinearSlide.setPower(0);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            boolean slowDown = gamepad1.right_bumper || gamepad1.right_trigger > 0.1;

            if (gamepad1.y) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double strafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double drive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            strafe = -strafe;

            double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(rot), 1);
            double frontLeftPower = (drive + strafe + rot) / denominator;
            double backLeftPower = (drive - strafe + rot) / denominator;
            double frontRightPower = (drive - strafe - rot) / denominator;
            double backRightPower = (drive + strafe - rot) / denominator;

            if (slowDown) {
                frontLeftPower *= changeInSpeed;
                frontRightPower *= changeInSpeed;
                backLeftPower *= changeInSpeed;
                backRightPower *= changeInSpeed;
            }

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            // Handle D-Pad inputs for shoulder and wrist positions
            if (gamepad1.dpad_down) {
                setShoulderAndWristPositions(leftShoulder, rightShoulder, wristServo, 0.8, 0.9);
            } else if (gamepad1.dpad_left) {
                setShoulderAndWristPositions(leftShoulder, rightShoulder, wristServo, 1.0, 0.7);
            } else if (gamepad1.dpad_up) {
                setShoulderAndWristPositions(leftShoulder, rightShoulder, wristServo, 0.55, 1.0);
            } else if (gamepad1.dpad_right) {
                setShoulderAndWristPositions(leftShoulder, rightShoulder, wristServo, 0, 0.5);
            }

            // Handle grabber toggle
            if (gamepad1.a) {
                if (grabberClosed) {
                    grabber.setPosition(0.5); // Open position
                } else {
                    grabber.setPosition(1.0); // Closed position
                }
                grabberClosed = !grabberClosed;
            }

            telemetry.addData("Left Slide Encoder", leftLinearSlide.getCurrentPosition());
            telemetry.addData("Right Slide Encoder", rightLinearSlide.getCurrentPosition());
            telemetry.addData("Left Shoulder Position", leftShoulder.getPosition());
            telemetry.addData("Right Shoulder Position", rightShoulder.getPosition());
            telemetry.addData("Wrist Position", wristServo.getPosition());
            telemetry.addData("Grabber Position", grabber.getPosition());
            telemetry.update();
        }
    }

    private void setShoulderAndWristPositions(Servo leftShoulder, Servo rightShoulder, Servo wristServo, double shoulderPosition, double wristPosition) {
        leftShoulder.setPosition(shoulderPosition);
        rightShoulder.setPosition(1.0 - shoulderPosition);
        wristServo.setPosition(wristPosition);
    }

    private boolean bothSlidesAboveLimit(DcMotor leftSlide, DcMotor rightSlide, int limit) {
        return leftSlide.getCurrentPosition() > limit && rightSlide.getCurrentPosition() > limit;
    }
}
