package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DSDrive", group = "DSDrive") // D.S Drive = Double Slide Drive
public class DSDrive extends LinearOpMode {

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

        Servo shoulder = hardwareMap.get(Servo.class, "shoulder");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        Servo grabber = hardwareMap.get(Servo.class, "grabber_servo");
        Servo panel = hardwareMap.get(Servo.class, "pushPanel");

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
        boolean panelClosed = false;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT, // Change to left if doesn't work
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            //int SLIDE_TOP_POSITION = 2088; // const
            double moveSlide = gamepad2.right_trigger - gamepad2.left_trigger;

            if (moveSlide > 0) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else if (moveSlide < 0 && (leftLinearSlide.getCurrentPosition() > 0 && rightLinearSlide.getCurrentPosition() > 0)) {
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
            if (gamepad2.dpad_down) {
                setToDegrees(shoulder,0);
                setToDegrees(wristServo, 0);
            } else if (gamepad2.dpad_left) {
                setToDegrees(shoulder,230);
                setToDegrees(wristServo, 0);
            } else if (gamepad2.dpad_up) {
                setToDegrees(shoulder,190);
                setToDegrees(wristServo, 0);
            }
            else if (gamepad2.dpad_right) {
                setToDegrees(shoulder,100);
                setToDegrees(wristServo, 180);
            }

            // Handle grabber open/close
            if (gamepad2.a) {
                grabber.setPosition(0.5); // Open position
                grabberClosed = false;
            } else if (gamepad2.b) {
                grabber.setPosition(1.0); // Closed position
                grabberClosed = true;
            }// else {
//                grabber.setPosition(1.0); // Closed position
//            }
            grabberClosed = !grabberClosed;
        }

        if (gamepad1.a) {
            panel.setPosition(0.5); // Open position
            panelClosed = false;
        } else if (gamepad1.b) {
            panel.setPosition(1.0); // Closed position
            panelClosed = true;
        }// else {
//                grabber.setPosition(1.0); // Closed position
//            }
        panelClosed = !panelClosed;
    }

       // telemetry.addData("Left Slide Encoder", leftLinearSlide.getCurrentPosition());
      //  telemetry.addData("Right Slide Encoder", rightLinearSlide.getCurrentPosition());
       // telemetry.addData(" Shoulder Position", shoulder.getPosition());
     //   telemetry.addData("Wrist Position", wristServo.getPosition());
     //   telemetry.addData("Grabber Position", grabber.getPosition());
    //    telemetry.update();
    //}

    private void setToDegrees(Servo s,double degrees) {
    double temp = degrees / 300;
    s.setPosition(temp);
    }

    private void setShoulderAndWristPositions(Servo shoulder, Servo wristServo, double shoulderPosition, double wristPosition) {
        shoulder.setPosition(shoulderPosition);
        wristServo.setPosition(wristPosition);
    }
}
