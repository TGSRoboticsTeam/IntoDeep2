
    /* Copyright (c) 2021 FIRST. All rights reserved.
     *
     * Redistribution and use in source and binary forms, with or without modification,
     * are permitted (subject to the limitations in the disclaimer below) provided that
     * the following conditions are met:
     *
     * Redistributions of source code must retain the above copyright notice, this list
     * of conditions and the following disclaimer.
     *
     * Redistributions in binary form must reproduce the above copyright notice, this
     * list of conditions and the following disclaimer in the documentation and/or
     * other materials provided with the distribution.
     *
     * Neither the name of FIRST nor the names of its contributors may be used to endorse or
     * promote products derived from this software without specific prior written permission.
     *
     * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
     * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
     * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
     * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
     * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
     * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
     * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
     * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
     * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
     * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
     * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
     */

    package org.firstinspires.ftc.teamcode.Auto;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    /*
     * This file contains an example of a Linear "OpMode".
     * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
     * The names of OpModes appear on the menu of the FTC Driver Station.
     * When a selection is made from the menu, the corresponding OpMode is executed.
     *
     * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
     * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
     * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
     */

    @Autonomous(name="RightSide", group="Linear OpMode")
//@Disabled
    public class RightSide extends LinearOpMode {

        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;
        private Servo armServo = null;
        private Servo wristServo = null;
        private Servo grabber = null;
        private DcMotor linearSlide = null;
        private DcMotor hang = null;
        private GoBildaPinpointDriver odo = null;
        private double heading;
        double adjustment = 0.7;

        @Override
        public void runOpMode() {

            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            hang = hardwareMap.get(DcMotor.class, "hang");
            hang.setDirection(DcMotorSimple.Direction.FORWARD);
            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
            rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

            armServo = hardwareMap.get(Servo.class, "arm_servo");
            wristServo = hardwareMap.get(Servo.class, "wrist_servo");
            grabber = hardwareMap.get(Servo.class, "grabber_servo");

            linearSlide = hardwareMap.get(DcMotor.class, "linear_slide");
            linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            // Makes the motors output their rotation
            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            hang = hardwareMap.get(DcMotor.class, "hang");
            hang.setDirection(DcMotorSimple.Direction.FORWARD);
            hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            odo.setOffsets(-84.0, -168.0);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            odo.resetPosAndIMU();
            // Wait for the game to start (driver presses START)
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            odo.update();
            heading = odo.getHeading();
            waitForStart();
            runtime.reset();

            while (opModeIsActive()) {
                // odo.update();

                // telemetry.addData("LS: ", "%d ", linearSlide.getCurrentPosition());
                // telemetry.update();
                odo.update();
                telemetry.addData("ODO yx:Move", "%.2f, %.2f ", getODOx(), getODOy());
                telemetry.update();


                runtime.reset();
                while (runtime.seconds() < 50) {
                    odo.update();
                    //hang.setPower(1);
                    //driveByTime(0,2);
                    //hang.setPower(0);
                    firstSpecimen(); //takes 6 seconds and resets
                    gotoX(20);
                    //driveByTime(-.5,.65);
                    odoMoveY(40);
                    odoMoveX(20);
                    odoMoveY(15);
                    gotoX(0);
                    telemetry.addData("final: ", "%.2f %.2f %.2f ", getODOx(),getODOy(),getHeading());
                    telemetry.update();
                    //driveByTime(0,0);;
                    //strafeByTime(0.6,2);
                    odo.resetPosAndIMU();
                    telemetry.addData("final: ", "%.2f %.2f %.2f ", getODOx(),getODOy(),getHeading());
                    telemetry.update();
                    driveByTime(0,30);

                }
                //hangSpecimen();
                unfold();
                setLinearSlide(2080,0.5);
                setArm(.55,1.0);
                telemetry.addData("LS_arm_claw: ", "%d ", linearSlide.getCurrentPosition());
                telemetry.update();
                driveByTime(0,1);
                odoMoveX(27);
                setLinearSlide(1750,0.25);
                driveByTime(0,2);
                //odoMoveX(-6);
                openClaw();
                setLinearSlide(5,0.25);
                setArm(.08,.56); //scoop
                odoMoveY(-25);
                headingCorrection(0);
                odoMoveY(-27);
                headingCorrection(0);
                odoMoveX(10);
                closeClaw();
                driveByTime(0,1);
                headingCorrection(45);
                odoMoveX(-10);
                setArm(.75,.62);



                // setLinearSlide(1500,0.25);
                //driveByTime(0,2);
                //setLinearSlide(0,0.25);
                // driveByTime(0,5);

                //  hangSpecimen();


            }

            //telemetry.addData("ODO X:", odo.getPosX());
            //telemetry.update();

            // driveByTime(0,5);
            //  odoMove(10,0);

            // telemetry.addData("ODO X:", odo.getPosX());
            // telemetry.update();
            // driveByTime(0,5);

            //krabbyPatty();
        }
        public void setLinearSlide(int turnage,double speed) {
            double ticks = 384.5;
            double newTarget = ticks / turnage;
            linearSlide.setTargetPosition(turnage);
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            linearSlide.setPower(speed);

            // Wait until the motor is no longer busy
            while (linearSlide.isBusy()) {
                // You can add a small delay here if needed
                telemetry.addData("LS_busy: ", "%d ", linearSlide.getCurrentPosition());
                telemetry.addData("LS_target: ", "%d ", linearSlide.getTargetPosition());
                telemetry.update();
            }
            telemetry.addData("LS_Complete: ", "%d ", linearSlide.getCurrentPosition());
            telemetry.update();

            // Optionally stop the motor after reaching the target
            //linearSlide.setPower(0);
        }

        public void setArm(double arm, double wrist) {
            armServo.setPosition(arm);
            wristServo.setPosition(wrist);
        }


        public void gotoX(double targetX){
            odo.update();
            double currentX = getODOx();
            odoMoveX(targetX-currentX);

        }
        public void gotoY(double targetY){
            odo.update();
            double currentY = getODOy();
            odoMoveY(targetY-currentY);
        }

        public void slidesToTop() {
            setLinearSlide(2088,.5);
        }

        public void openClaw() {
            grabber.setPosition(0.4);
        }

        public void closeClaw() {
            grabber.setPosition(1.0);
        }

        public void slidesToBottom() {
            setLinearSlide(0,.5);
        }

        public void unfold() {
            closeClaw();
            setArm(0.01, 0.56);
        }

        public void firstSpecimen(){
            unfold();
            setLinearSlide(1250,0.5);
            setArm(0.35,.75);
            driveByTime(0,0.5);
            odoMoveX(24.5);
            //setLinearSlide(300,0.5);
            setArm(0.35,1);
            driveByTime(0,0.5);
            driveByTime(-.5,.5);
            openClaw();
            driveByTime(0,0.5);
            setLinearSlide(100,0.75);
            unfold();
            driveByTime(0,0.5);
        }

        //Routine for placing a specimein if it is already in the claw and you are at the bar
        public void hangSpecimen() {
            slidesToTop();
            telemetry.addData("SlidesToTop", "%d ", linearSlide.getCurrentPosition());
            telemetry.update();
            driveByTime(1, 1); // Replace with either odo drive or drive until touch sensor
            setLinearSlide(1044,.5);
            telemetry.addData("Slides1044", "%d ", linearSlide.getCurrentPosition());
            telemetry.update();
            driveByTime(0, 2);
            openClaw();
            telemetry.addData("openclaw", "%d ", linearSlide.getCurrentPosition());
            telemetry.update();
            driveByTime(0, 2);
            driveByTime(-0.5,1);
            slidesToBottom();
            telemetry.addData("SlidesToBottom", "%d ", linearSlide.getCurrentPosition());
            telemetry.update();
            driveByTime(0, 2);

        }


        public double getODOx() {
            return odo.getPosX() / 19.89436789;
        }

        public double getODOy() {
            return odo.getPosY() / 19.89436789;
        }
        public double getHeading() {
            return odo.getHeading();
        }

        public void headingCorrection(double degrees){
            double radians = Math.toRadians(degrees);
            //radians, CCW is +
            odo.update();
            telemetry.addData("heading ", "%.2f", getHeading());
            telemetry.update();
            driveByTime(0,0);
            double power = .25;
            //ccw
            if (odo.getHeading() - radians < 0) {
                while (odo.getHeading() < radians) {
                    telemetry.addData("heading ", "%.2f", getHeading());
                    telemetry.update();
                    leftFrontDrive.setPower(-power);
                    rightFrontDrive.setPower(power);
                    leftBackDrive.setPower(-power);
                    rightBackDrive.setPower(power);
                    odo.update();

                }
                driveByTime(0,0);
            }
            else if (odo.getHeading() - radians > 0) {
                while (odo.getHeading() > radians) {
                    telemetry.addData("heading ", "%.2f", getHeading());
                    telemetry.update();
                    leftFrontDrive.setPower(power);
                    rightFrontDrive.setPower(-power);
                    leftBackDrive.setPower(power);
                    rightBackDrive.setPower(-power);
                    odo.update();

                }
                driveByTime(0,0);
            }


        }
        public void odoMoveX(double distanceX) {
            double power = .25;
            odo.update();

            double startX = getODOx();
            telemetry.addData("driveX:", "%.2f, %.2f %.2f %.2f ", getODOx(), getODOy(), distanceX, startX);
            telemetry.update();

            if (distanceX > 0.0) { //+X
                telemetry.addData("drive+X pos:", "%.2f, %.2f %.2f %.2f ", getODOx(), getODOy(), distanceX, startX);
                telemetry.update();

                while (getODOx() - startX < distanceX) {
                    if ((getODOx() - startX < 5) && (power > .2)) {
                        power *= .9;
                    }

                    telemetry.addData("ODO X test pos:Move", "%.2f, %.2f %.2f %.2f ", getODOx(), getODOy(), distanceX, startX);
                    telemetry.update();
                    leftFrontDrive.setPower(power);
                    rightFrontDrive.setPower(power);
                    leftBackDrive.setPower(power);
                    rightBackDrive.setPower(power);
                    odo.update();
                }

            }


            else if (distanceX < 0.0) {
                // power=-1*power;
                while (getODOx() - startX > distanceX) {
                    if ((getODOx() - startX > 5) && (power > .2)) {
                        power *= .9;
                    }

                    telemetry.addData("ODO X:Move", " " + getODOx(), startX, distanceX);
                    telemetry.update();
                    leftFrontDrive.setPower(-power);
                    rightFrontDrive.setPower(-power);
                    leftBackDrive.setPower(-power);
                    rightBackDrive.setPower(-power);
                    odo.update();
                }
            }
        }

        public void odoMoveY(double distanceY) {
            double power = .25;
            odo.update();
            double head = getHeading();

            double startY = getODOy();

            //move in +Y direction
            if (distanceY > 0) {
                while (getODOy() - startY < distanceY) {
                    /**if ((getODOy() - startY < 10) && (power > .2)) {
                     power *= .9;
                     }**/
                    odo.update();
                    /**double newHeading = getHeading();
                     if(Math.abs(head-newHeading)>.2){
                     headingCorrection(head);
                     }**/
                    telemetry.addData("ODO y:Move", "\"%.2f, %.2f, %.2f\" ", getODOy(), startY, distanceY);
                    //telemetry.addData("ODO y:Move", "\"%.2f, %.2f\" ", newHeading, head);

                    telemetry.update();

                    leftFrontDrive.setPower(adjustment * -power);
                    rightFrontDrive.setPower(power);
                    leftBackDrive.setPower(power);
                    rightBackDrive.setPower(adjustment * -power);

                    odo.update();
                }
            }
            else if (distanceY < 0.0) { //-Y
                while (getODOy() - startY > distanceY) {
                    /**if ((getODOy() - startY > 5) && (power > .2)) {
                     power *= .9;
                     }**/
                    odo.update();
                    /**double newHeading = getHeading();
                     if(Math.abs(head-newHeading)>.2){
                     headingCorrection(head);
                     }**/

                    telemetry.addData("ODO Y:Move", "\"%.2f, %.2f, %.2f\" ", getODOy(), startY, distanceY);
                    telemetry.addData("ODO Y heading", "%.2f",  getHeading());

                    telemetry.update();

                    leftFrontDrive.setPower(power);
                    rightFrontDrive.setPower(adjustment * -power);
                    leftBackDrive.setPower(adjustment * -power);
                    rightBackDrive.setPower(power);
                    odo.update();
                }
            }
            driveByTime(0,0);
        }


        public void driveByTime(double power, double time) {
            // Send calculated power to wheels
            runtime.reset();
            while (runtime.seconds() < time) {
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(power);
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(power);
            }


        }

        public void strafeByTime(double power, double time) {
            // Send calculated power to wheels
            //move sideways

            runtime.reset();
            if(power<0) {
                while (runtime.seconds() < time) {
                    leftFrontDrive.setPower(adjustment* -power);
                    rightFrontDrive.setPower(power);
                    leftBackDrive.setPower(power);
                    rightBackDrive.setPower(adjustment*-power);
                }
            }
            if(power>0) {
                while (runtime.seconds() < time) {
                    leftFrontDrive.setPower(power);
                    rightFrontDrive.setPower(adjustment*-power);
                    leftBackDrive.setPower(adjustment*-power);
                    rightBackDrive.setPower(power);
                }
            }

        }

        public void bestRotateByTime(double power, double time) {
            runtime.reset();
            while (runtime.seconds() < time) {
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(-power);
                leftBackDrive.setPower(power);
                rightBackDrive.setPower(-power);
            }
        }


        public void diagByTime(double power, double power2, double time) {
            // Send calculated power to wheels
            runtime.reset();
            while (runtime.seconds() < time) {
                leftFrontDrive.setPower(power);
                rightFrontDrive.setPower(-power2);
                leftBackDrive.setPower(-power2);
                rightBackDrive.setPower(power);
            }
        }

        public void rotateByTime(double power, int direction, double time) {
            // Send calculated power to wheels
            runtime.reset();
            while (runtime.seconds() < time) {
                leftFrontDrive.setPower(power * direction);
                rightFrontDrive.setPower(-power * direction);
                leftBackDrive.setPower(-power * direction);
                rightBackDrive.setPower(power * direction);
            }
        }


        ///Specific Routines*********************

        public void firstODO() {
            odoMoveX(30);
            odoMoveY(30);
            odoMoveX(15);
            odoMoveY(15);
        }

        public void droom() {
            setLinearSlide(100,.2);
            odoMoveX(25);
            odoMoveY(-20);
            odoMoveX(35);
            odoMoveY(15);
            odoMoveX(-10);
        }

        public void lightHouse() {
            strafeByTime(.25, 0.25);//right off the wall
            driveByTime(.25, 5);//forward
            strafeByTime(.25, 2);//right
            driveByTime(-.25, 2);//back
            strafeByTime(.25, 2); //right
            driveByTime(-.25, 2);//back
        }

        public void vortex() {
            driveByTime(.25, 4);
            diagByTime(.25, .75, 1.5); //back right
        }

        public void jellyfishC() {
            driveByTime(.25, 0.25);//forward off the wall
            strafeByTime(-.25, 2.5);
            driveByTime(.25, 3.25);
            strafeByTime(-.25, 1);
            diagByTime(-.5, -.3, 2); //back left
            //make the diag less steep
        }

        public void dock() {
            driveByTime(.5, 1.5);
        }

        public void fishy() {
            //drop off specimen and park
            driveByTime(.5, 1.5);
            driveByTime(0, 3); //wait
            diagByTime(.25, .5, 1.5); //back right
        }

        public void fishHook() {
            //specimen and level1 ascent
            driveByTime(.5, 1.5);
            driveByTime(0, 3);
            strafeByTime(-.5, 1.5);
            driveByTime(.5, 1);
            strafeByTime(.5, .7);
        }

        public void MrKrabs() {
            //specimen and sample and level1 ascent
            driveByTime(.5, 1.5);
            driveByTime(0, 1.5);
            strafeByTime(-.5, .7);
            driveByTime(.5, 1);
            strafeByTime(-.5, .7);
            diagByTime(-.5, -.35, 1);
            diagByTime(.5, .35, 1);
            strafeByTime(.5, 2.5);
        }

        public void krabbyPatty() {
            driveByTime(0, 5.0);
            wristServo.setPosition(0.8);
            grabber.setPosition(1.1);
            armServo.setPosition(1);
            driveByTime(0, 1.6);

            driveByTime(.25, 1.6);
            armServo.setPosition(.3);
            driveByTime(0, 0.1);
            driveByTime(0, 1.5);
            driveByTime(-.25, 1); //back away
            armServo.setPosition(.3);
            grabber.setPosition(0.5);
            wristServo.setPosition(0.5);
            strafeByTime(-.5, 1.2); //offset from start X
            driveByTime(.5, 1.2);
            strafeByTime(-.25, 0.5);
            bestRotateByTime(.25, .5);//turn a little to the right - one wheel
            driveByTime(-.5, 1.2); //c
            driveByTime(-.25, 0.5);
            driveByTime(.5, 1.4);
            strafeByTime(-.5, .4);
            driveByTime(-.5, 1.4);//b
            driveByTime(-.25, 0.5);
            driveByTime(.5, 1.5);
            strafeByTime(-.5, .3);
            strafeByTime(-.25, .3);
            driveByTime(-.5, 1.6);//A
            driveByTime(-.25, 1);
            driveByTime(.5, 1.6);
            strafeByTime(.5, 1.2);
            strafeByTime(.25, .9);
            strafeByTime(.1, 1.5);
            hang.setPower(1);
            driveByTime(0, 1);
            hang.setPower(0);

        }

        //arm = hand, grabber = wrist, wrist=arm
        public void patrick() {
            //Mr. Krabs with another sample
            wristServo.setPosition(0.8);
            grabber.setPosition(1);
            armServo.setPosition(0.5);
            driveByTime(.25, 1.6);
            armServo.setPosition(.3);
            driveByTime(0, 0.1);
            wristServo.setPosition(0.55);
            driveByTime(0, 1.5);
            strafeByTime(-.5, 1.7);
            driveByTime(.5, 1.4);
            strafeByTime(-.25, 1.3);
            bestRotateByTime(.25, .5);//turn a little to the right - one wheel
            driveByTime(-.5, 1.6); //c
            driveByTime(-.25, 0.5);
            driveByTime(.5, 1.7);
            strafeByTime(-.5, .45);
            driveByTime(-.5, 2);//b
            driveByTime(-.25, 0.5);
            driveByTime(.5, 1.7);
            strafeByTime(-.5, .4);
            strafeByTime(-.25, .2);
            driveByTime(-.5, 1.8);//A
            driveByTime(-.25, 1);
            driveByTime(.5, 1.4);
            strafeByTime(.5, 1.5);
            strafeByTime(.25, .5);
        }

        public void goFish() {
            telemetry.addData("linear slide: ", "%.1f", (double) linearSlide.getCurrentPosition());
            telemetry.update();
            double linearSlowDown = 0.15;
            //linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            grabber.setPosition(1);
            runtime.reset();
            double lt = 2;
            while (runtime.seconds() < lt) {
                linearSlide.setPower(1.0);
            }
            linearSlide.setPower(0.0);
            wristServo.setPosition(0.75);
            driveByTime(0, 3);

            armServo.setPosition(.5);
            driveByTime(.1, 1.5);
            driveByTime(0, 3);

            grabber.setPosition(0);
            driveByTime(0, 1.5);

            driveByTime(0, 1.5);

        }

        public void goSwimming() {

        }
    }





