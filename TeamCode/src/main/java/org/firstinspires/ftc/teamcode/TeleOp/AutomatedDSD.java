package org.firstinspires.ftc.teamcode.TeleOp;

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
        extendo.setDirection(DcMotorSimple.Direction.FORWARD);
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

        double changeInSpeed = 0.35; // drive speed multiplier

        // Extendo vars
        int pressNum = 0; // Used for extendo arm position toggle
        boolean heldUp = false; // Used to determine if extendo arm is out of the way

        // Linear slide vars
        int linearSlideHeight = 0; // Has to be an int that's the only var-type setTargetPosition() takes

        // Grabber and arm vars
        boolean justGrabbed = false;
        double changeState = 0.0; // Used for position toggle
        int state = 0; // Used for position toggle */
        //boolean clawGrab = false; // Used for toggle grabber

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
            double moveExtendo = - gamepad2.right_stick_y;
            double extendoSpeed = 0.75;

            // Attempt to initialize wrist servo on extendo
            if (gamepad2.dpad_down){
                extendoWrist.setPosition(1);
            }

            // Move extendo in/out
            if (moveExtendo > 0) {
                extendo.setPower(extendoSpeed* moveExtendo);
            } else if (moveExtendo < 0) {// && (leftLinearSlide.getCurrentPosition() > 0 && rightLinearSlide.getCurrentPosition() > 0)) {
                extendo.setPower(extendoSpeed * moveExtendo);
            } else {
                extendo.setPower(0);
                //extendoGrabber.setPosition(1);
            }

            //rotate the extendo base
            double baseRotation = gamepad2.left_stick_x;
            double baseIncrement = 2;
            if (baseRotation > 0) {
                setToDegrees(extendoBase, getDegrees(extendoBase) + baseIncrement);
            }else if (baseRotation < 0) {
                setToDegrees(extendoBase, getDegrees(extendoBase) - baseIncrement);
            }

            // Used to pull extendo arm up
            if (gamepad2.left_trigger > 0.5) {
                heldUp = true;
                setToDegrees(extendoShoulder,85);
            } else if (gamepad2.left_trigger > 0) {
                heldUp = true;
                setToDegrees(extendoShoulder,35);
            } else {
                heldUp = false;
            }
            // Makes extendo arm hover
            if(!heldUp && pressNum == 0){
                setToDegrees(extendoShoulder, 30);
            }

            // Drive vars
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = gamepad1.right_stick_x;

            boolean slowDown = gamepad1.a;

            if (gamepad1.y) {
                imu.resetYaw();
            }

            /*// TRANSFER
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
            */

            // DRIVE
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


            //Rotate the grabber on the extendo
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

            // Handle grabber open/close
            boolean toggleGrabber = gamepad1.right_bumper;
            double grabbingPos = 1;

            if (gamepad1.x && grabber.getPosition() == grabbingPos) {
                toggleGrabber = true;
            } else if (gamepad1.b && grabber.getPosition() != grabbingPos) {
                toggleGrabber = true;
            }

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

            /*boolean toggleClaw = gamepad2.right_bumper;
            double clawPos = 1;
            if (toggleClaw && !clawGrab) {
                clawGrab = true;
                if (extendoGrabber.getPosition() == grabbingPos) {
                    setRevToDegrees(extendoGrabber, 120);
                } else {
                    extendoGrabber.setPosition(grabbingPos);
                }
            } else if (!toggleClaw) {
                clawGrab = false;
            }
            //*/

            // Handle extendo arm sequence
            if(gamepad2.right_trigger > 0.5 && pressNum == 0){
                pressNum = 1;
                setToDegrees(extendoShoulder, 0);
                setRevToDegrees(extendoGrabber, 270);
            } else if (gamepad2.right_trigger > 0.5 && pressNum == 1) {
                setRevToDegrees(extendoGrabber, 110);
                pressNum = 0;
            }

            if (gamepad1.left_bumper && changeState == 0.0) { // has not been pressed
                changeState = 0.5; // just pressed = 0.5
            } else if (!gamepad1.left_bumper) {
                changeState = 0.0; // not pressed = 0.0
            }
            //*/


            // Linear slides
            int slideHeightIncrement = 1;
            double moveSlide = gamepad1.right_trigger - gamepad1.left_trigger;
            /* if (moveSlide > 0) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else if (moveSlide < 0) {// && (leftLinearSlide.getCurrentPosition() > 0 && rightLinearSlide.getCurrentPosition() > 0)) {
                leftLinearSlide.setPower(moveSlide);
                rightLinearSlide.setPower(moveSlide);
            } else {
                leftLinearSlide.setPower(0);
                rightLinearSlide.setPower(0);
            }
            //*/// Trigger based

            // Position based
            if (moveSlide > 0) {
                linearSlideHeight += slideHeightIncrement;
            } else if (moveSlide < 0) {// && (leftLinearSlide.getCurrentPosition() > 0 && rightLinearSlide.getCurrentPosition() > 0)) {
                linearSlideHeight -= slideHeightIncrement;
            }

            if (gamepad1.dpad_down) {
                linearSlideHeight = 0;
            } else if (gamepad1.dpad_right) {
                linearSlideHeight = 850;
            } else if (gamepad1.dpad_up) {
                linearSlideHeight = 950;
            }

            setLinearSlide(rightLinearSlide, leftLinearSlide, linearSlideHeight, 1);

            //*/

            telemetry.addData("Left Encoder", leftLinearSlide.getCurrentPosition());
            telemetry.addData("Right Encoder", rightLinearSlide.getCurrentPosition());
            telemetry.addData("Extendo Encoder", extendo.getCurrentPosition());
            telemetry.addData("Left Trigger", gamepad2.left_trigger);
            telemetry.addData("state", state);
            telemetry.update();
        }


    }
    public void setLinearSlide(DcMotor rightLinearSlide, DcMotor leftLinearSlide, int position, double speed) {
        double ticks = 384.5;
        double newTarget = ticks / position;
        rightLinearSlide.setTargetPosition(position);
        leftLinearSlide.setTargetPosition(position);
        rightLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightLinearSlide.setPower(speed);
        leftLinearSlide.setPower(speed);

        /*// Wait until the motor is no longer busy
        while (leftLinearSlide.isBusy()) {

        }

        rightLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
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

