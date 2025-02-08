package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ExtendoDrive", group = "ExtendoDrive")
public class ExtendoDrive extends LinearOpMode {

    @Override
    public void runOpMode() {

        // LINEAR SLIDES
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

        // EXTENDO
        DcMotor rightExtendo = hardwareMap.get(DcMotor.class, "right_extendo");
        rightExtendo.setDirection(DcMotorSimple.Direction.REVERSE);
        rightExtendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DcMotor leftExtendo = hardwareMap.get(DcMotor.class, "left_extendo");
        leftExtendo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftExtendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftExtendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // SERVOS
        Servo shoulder = hardwareMap.get(Servo.class, "shoulder");
        Servo wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        Servo grabber = hardwareMap.get(Servo.class, "grabber_servo");

        Servo extendoBase = hardwareMap.get(Servo.class, "extendo_base");
        Servo extendoShoulder = hardwareMap.get(Servo.class, "extendo_shoulder");
        Servo grabberRotate = hardwareMap.get(Servo.class, "extendo_grabber_rotate");
        Servo extendoGrabber = hardwareMap.get(Servo.class, "extendo_grabber");

        // DRIVE
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

        boolean justGrabbed = false; // Linear slide claw vars
        boolean grabberClosed = false;

        boolean justGrabbedE = false; // Extendo claw vars
        boolean grabberClosedE = false;

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

            // Drive
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

            // Slides
            double moveSlide = gamepad1.right_trigger - gamepad1.left_trigger;

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

            // Extendo
            double moveExtendo = -gamepad2.right_stick_y;

            if (moveExtendo > 0) {
                leftExtendo.setPower(moveExtendo);
                rightExtendo.setPower(moveExtendo);
            } else if (moveExtendo < 0 && (leftExtendo.getCurrentPosition() > 0 && rightExtendo.getCurrentPosition() > 0)) {
                leftExtendo.setPower(moveExtendo);
                rightExtendo.setPower(moveExtendo);
            } else {
                leftExtendo.setPower(0);
                rightExtendo.setPower(0);
            }

            // Handle D-Pad inputs for shoulder and wrist positions
            if (gamepad1.dpad_down) {
                setShoulderAndWristPositions(shoulder, wristServo, 0.3, 0.9);
            } else if (gamepad1.dpad_left) {
                setShoulderAndWristPositions(shoulder, wristServo, 1.0, 0.7);
            } else if (gamepad1.dpad_up) {
                setShoulderAndWristPositions(shoulder, wristServo, 0.55, 1.0);
            } else if (gamepad1.dpad_right) {
                setShoulderAndWristPositions(shoulder, wristServo, 0, 0.5);
            }

            // Handle grabber open/close
            boolean toggleGrab = gamepad1.y; // Linear slide claw
            if (toggleGrab) {
                if (!justGrabbed) {
                    if (grabberClosed) {
                        grabber.setPosition(0.5);
                    }else{
                        grabber.setPosition(0);
                    }
                    grabberClosed = !grabberClosed;
                }
                justGrabbed = true;
            }else{
                justGrabbed = false;
            }

            // Handle extendo-grabber open/close
            boolean toogleGrab = gamepad2.right_trigger > 0.1 || gamepad2.right_bumper; // Extendo claw
            if (toogleGrab) {
                if (!justGrabbedE) {
                    if (grabberClosedE) {
                        extendoGrabber.setPosition(0.5);
                    }else{
                        extendoGrabber.setPosition(0);
                    }
                    grabberClosedE = !grabberClosedE;
                }
                justGrabbedE = true;
            }else{
                justGrabbedE = false;
            }

            // Extendo base rotation
            double baseRotation = gamepad2.left_stick_x;
            double baseIncrement = 1;
            if (baseRotation > 0) {
                setToDegrees(extendoBase, getDegrees(extendoBase) + baseIncrement);
            }else if (baseRotation < 0) {
                setToDegrees(extendoBase, getDegrees(extendoBase) - baseIncrement);
            }

            if (gamepad2.left_trigger > 0.1 || gamepad2.left_bumper) {
                extendoShoulder.setPosition(0.5); // Up position
            }else{
                extendoShoulder.setPosition(0); // Down position
            }

        }


        telemetry.addData("Left Slide Encoder", leftLinearSlide.getCurrentPosition());
        telemetry.addData("Right Slide Encoder", rightLinearSlide.getCurrentPosition());
        telemetry.addData("Shoulder Position", shoulder.getPosition());
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.addData("Grabber Position", grabber.getPosition());
        telemetry.update();
    }

    private void setShoulderAndWristPositions(Servo shoulder, Servo wristServo, double shoulderPosition, double wristPosition) {
        shoulder.setPosition(shoulderPosition);
        wristServo.setPosition(wristPosition);
    }

    private void setToDegrees(Servo s,double degrees) {
        double temp = degrees / 300;
        s.setPosition(temp);
    }

    private double getDegrees(Servo s) {
        return s.getPosition() * 300;
    }
}
