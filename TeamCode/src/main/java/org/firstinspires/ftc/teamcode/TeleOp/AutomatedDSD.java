package TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AutomatedDSD", group = "AutomatedDSD") // DSD = Double Slide Drive
public class AutomatedDSD extends LinearOpMode {

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

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        extendo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo extendoBase = hardwareMap.get(Servo.class, "extendo_base");
        Servo extendoShoulder = hardwareMap.get(Servo.class, "extendo_shoulder");
        Servo grabberRotate = hardwareMap.get(Servo.class, "extendo_grabber_rotate");
        Servo extendoWrist = hardwareMap.get(Servo.class, "extendo_wrist");
        Servo extendoGrabber = hardwareMap.get(Servo.class, "extendo_grabber");

        Servo shoulder = hardwareMap.get(Servo.class, "shoulder");
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
        double transferMode = 0;
        boolean justGrabbed = false;
        boolean clawGrab = false;
        double changeState = 0.0; // Used for position toggle
        int state = 0; // Used for position toggle */

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
            double moveSlide = gamepad1.right_trigger - gamepad1.left_trigger;
            double moveExtendo = gamepad2.right_stick_y;
            double extendoSpeed = 0.5;

            if (moveSlide > 0) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else if (moveSlide < 0) {// && (leftLinearSlide.getCurrentPosition() > 0 && rightLinearSlide.getCurrentPosition() > 0)) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else {
                leftLinearSlide.setPower(0);
                rightLinearSlide.setPower(0);
            }

            if (moveExtendo > 0) {
                extendo.setPower(extendoSpeed* moveExtendo);
            } else if (moveExtendo < 0) {// && (leftLinearSlide.getCurrentPosition() > 0 && rightLinearSlide.getCurrentPosition() > 0)) {
                extendo.setPower(extendoSpeed * moveExtendo);
            } else {
                extendo.setPower(0);
            }

            double baseRotation = gamepad2.left_stick_x;
            double baseIncrement = 1;
            if (baseRotation > 0) {
                setToDegrees(extendoBase, getDegrees(extendoBase) + baseIncrement);
            }else if (baseRotation < 0) {
                setToDegrees(extendoBase, getDegrees(extendoBase) - baseIncrement);
            }

            if (gamepad2.left_trigger > 0.1 || gamepad2.left_bumper) {
                setToDegrees(extendoShoulder,25);
            }else if (transferMode == 0){
                extendoShoulder.setPosition(10); // Down position
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            boolean slowDown = gamepad1.a;

            if (gamepad1.y) {
                imu.resetYaw();
            }


            if(gamepad1.x) {
                transferMode += 1;
            }

            if (transferMode == 1) {
                setToDegrees(grabber, 150);
                setToDegrees(shoulder, 110);
                setToDegrees(wristServo, 240);
                setToDegrees(extendoBase,150);
                setToDegrees(extendoShoulder,60);
                setToDegrees(extendoWrist, 90);
                setRevToDegrees(grabberRotate, 90);


            }
            else if (transferMode == 2){
                setToDegrees(grabber, 300);
                setRevToDegrees(extendoGrabber, 270);
                transferMode ++;
            } else if (transferMode == 3) {
                setToDegrees(shoulder, 235);
                setToDegrees(wristServo, 270);
                setToDegrees(extendoWrist, 90);
                transferMode = 0;
            }

            if (gamepad2.y){
                setRevToDegrees(grabberRotate, 150);
            }
            if (gamepad2.x){
                setRevToDegrees(grabberRotate, 95);
            }
            if (gamepad2.b){
                setRevToDegrees(grabberRotate, 195);
            }

            if (gamepad2.a){
                setRevToDegrees(grabberRotate, 240);
            }


            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double strafe = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double drive = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            //strafe = -strafe;

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


            // Handle grabber open/close
            boolean toggleGrabber = gamepad1.right_bumper;
            double grabbingPos = 1;
            if (toggleGrabber && !justGrabbed) {
                justGrabbed = true;
                if (grabber.getPosition() == grabbingPos) {
                    grabber.setPosition(0.5);
                } else {
                    grabber.setPosition(grabbingPos);
                }
            } else if (!toggleGrabber) {
                justGrabbed = false;
            }

            boolean toggleClaw = gamepad2.right_bumper;
            double clawPos = 1;
            if (toggleClaw && !clawGrab) {
                clawGrab = true;
                if (extendoGrabber.getPosition() == grabbingPos) {
                    setRevToDegrees(extendoGrabber, 120);
                } else {
                    extendoGrabber.setPosition(grabbingPos);
                }
            } else if (!toggleGrabber) {
                clawGrab = false;
            }

            if (gamepad1.left_bumper && changeState == 0.0) { // has not been pressed
                changeState = 0.5; // just pressed = 0.5
            } else if (!gamepad1.left_bumper) {
                changeState = 0.0; // not pressed = 0.0
            }

            if (changeState == 0.5) {
                changeState = 1.0;
                if (state == 0){
                    state = 1;
                    setToDegrees(shoulder,235);
                    setRevToDegrees(wristServo, 270);
                } else if (state == 1) {
                    state = 0;
                    setToDegrees(shoulder, 125);
                    setToDegrees(wristServo, 240);
                }
            }
            //*/

            telemetry.addData("TransferMode", transferMode);
            telemetry.update();
        }

    }

    private double getDegrees(Servo s) {
        return s.getPosition() * 300;
    }
    private void setRevToDegrees(Servo s,double degrees) {
        double temp = degrees / 270;
        s.setPosition(temp);
    }

    private void setToDegrees(Servo s,double degrees) {
        double temp = degrees / 300;
        s.setPosition(temp);
    }


}




// telemetry.addData("Left Slide Encoder", leftLinearSlide.getCurrentPosition());
//  telemetry.addData("Right Slide Encoder", rightLinearSlide.getCurrentPosition());
// telemetry.addData(" Shoulder Position", shoulder.getPosition());
//   telemetry.addData("Wrist Position", wristServo.getPosition());
//   telemetry.addData("Grabber Position", grabber.getPosition());
//    telemetry.update();
//}

