package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTester", group = "Testing")
public class servoTester extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize shoulder servos
        Servo leftShoulder = hardwareMap.get(Servo.class, "left_shoulder");
        Servo rightShoulder = hardwareMap.get(Servo.class, "right_shoulder");

        // Set initial servo position
        double shoulderPosition = 0.0;
        leftShoulder.setPosition(shoulderPosition);
        rightShoulder.setPosition(1.0 - shoulderPosition);

        waitForStart();

        while (opModeIsActive()) {
            // A increases position by 0.01
            if (gamepad1.a) {
                shoulderPosition += 0.01;
            }

            // B increases position by 0.05
            if (gamepad1.b) {
                shoulderPosition += 0.05;
            }

            // X decreases position by 0.01
            if (gamepad1.x) {
                shoulderPosition -= 0.01;
            }

            // Y decreases position by 0.05
            if (gamepad1.y) {
                shoulderPosition -= 0.05;
            }

            // Clamp the position between 0 and 1
            shoulderPosition = Math.max(0, Math.min(1, shoulderPosition));

            // Update servo positions
            leftShoulder.setPosition(shoulderPosition);
            rightShoulder.setPosition(1.0 - shoulderPosition);

            // Display current position on telemetry
            telemetry.addData("Shoulder Position", shoulderPosition);
            telemetry.update();
        }
    }
}
