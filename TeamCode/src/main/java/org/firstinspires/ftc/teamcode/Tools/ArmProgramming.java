package org.firstinspires.ftc.teamcode.Tools;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ArmProgramming", group = "ArmProgramming")
public class ArmProgramming extends LinearOpMode {

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

        String position = "null";
        while (opModeIsActive()) {

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

            // Handle D-Pad inputs for shoulder and wrist positions
            if (gamepad1.a) {
                leftShoulder.setPosition(0);
                rightShoulder.setPosition(0);
                wristServo.setPosition(0);
            } else if (gamepad1.b) {
                leftShoulder.setPosition(1);
                rightShoulder.setPosition(1);
                wristServo.setPosition(1);
            }
        }

        telemetry.addData("Left Slide Encoder", position);
        telemetry.update();
    }
}
