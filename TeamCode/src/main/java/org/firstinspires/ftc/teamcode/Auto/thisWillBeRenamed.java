 package org.firstinspires.ftc.teamcode.Auto;

 import com.acmerobotics.dashboard.FtcDashboard;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.Telemetry;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.Position;
 import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
 import org.firstinspires.ftc.vision.VisionPortal;
 import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
 import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

 import java.util.List;

 @Autonomous(name="thisWillBeRenamed", group="Linear OpMode")
//@Disabled
 public class thisWillBeRenamed extends LinearOpMode {
     // Declare OpMode members for each of the 4 motors.
     private ElapsedTime runtime = new ElapsedTime();
     private DcMotor leftFrontDrive = null;
     private DcMotor leftBackDrive = null;
     private DcMotor rightFrontDrive = null;
     private DcMotor rightBackDrive = null;
     private Servo rightShoulder = null;
     private Servo leftShoulder = null;
     private Servo wristServo = null;
     private Servo grabber = null;
     private Servo panel = null;
     private DcMotor linearSlide = null;
     private GoBildaPinpointDriver odo = null;
     private double heading;
     double adjustment = 0.7;
     double odoConversion = 23.49;
     private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
     private Position cameraPosition = new Position(DistanceUnit.INCH,
             0, 0, 0, 0);
     private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
             0, -90, 0, 0);
     private AprilTagProcessor aprilTag;
     private VisionPortal visionPortal;


     @Override
     public void runOpMode() {
         initAprilTag();
         FtcDashboard dashboard = FtcDashboard.getInstance();
         Telemetry dashboardTelemetry = dashboard.getTelemetry();

         // Initialize the hardware variables. Note that the strings used here must correspond
         // to the names assigned during the robot configuration step on the DS or RC devices.
         leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
         leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
         rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
         rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

         Servo leftShoulder = hardwareMap.get(Servo.class, "left_shoulder");
         Servo rightShoulder = hardwareMap.get(Servo.class, "right_shoulder");
         Servo wristServo = hardwareMap.get(Servo.class, "wrist_servo");
         Servo grabber = hardwareMap.get(Servo.class, "grabber_servo");

         Servo panel = hardwareMap.get(Servo.class, "pushPanel")

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
             odo.update();
             telemetryAprilTag();



         }
     }


     // Method to drive the robot forward by a specified distance (in inches) with slowdown
     public void driveInches(double targetDistance) {
         // Reset encoders
         odo.update();
         double currentY = getODOx();

         // Use a PID controller or other control mechanism to drive to the target distance
         // This example uses a simple proportional control with a slowdown factor
         while (getDistance(currentY) < targetDistance && opModeIsActive()) {
             odo.update();
             double remainingDistance = targetDistance - getDistance(currentY);
             double power = 0.35 * (remainingDistance / targetDistance); // Proportional control with slowdown
             telemetry.addData("Remaining", "%.2f", remainingDistance);
             telemetry.addData("Current ODO", "%.2f", getODOx());
             telemetry.addData("target", "%.2f", targetDistance);
             telemetry.update();
             // Ensure minimum power to avoid stalling
             if (power < 0.1) {
                 power = 0.1;
             }
             if(power>.4){
                 power = .4;
             }
             setMotorPowers(power, power, power, power); // Drive forward
         }
         setMotorPowers(0, 0, 0, 0); // Stop the robot
     }

     public void panelUp(){
         panel.setPosition(0.33);
     }

     public void panelDown() {
         panel.setPosition(0);
     }

     public double getDistance(double initialPos){
         odo.update();
         double currentPos = getODOx();
         double distance = currentPos - initialPos;
         // telemetry.addData("DistanceTravelled", "%.2f", distance);
         // telemetry.addData("Current Pos", "%.2f", currentPos);
         // telemetry.addData("Initial Pos", "%.2f", initialPos);
         // telemetry.update();
         return distance;

     }

     public void rotate90(int direction){
         //if direction is 1 rotate 90 degrees counterclockwise
         //if direction is -1 rotate -90 degrees clockwise
         if(direction == 1){
             headingCorrection(90);
         }else if(direction == -1){
             headingCorrection(-90);
         }
     }
     public void rotate(double degrees){
         //CONVERT DEGREES TO RADIANS
         double radians = Math.toRadians(degrees);
         double targetAngle = getAngle() + radians; // Calculate target angle

         // Use a PID controller or other control mechanism to rotate to the target angle
         // You'll need to read the angle from your odometry pods and adjust motor powers accordingly
         // This is a simplified example that assumes you have a getAngle() method
         telemetry.addData("Angle", "%.2f", getAngle());
         telemetry.addData("Target Angle", "%.2f", targetAngle);

         telemetry.update();
         while (Math.abs(getAngle() - targetAngle) > .1 && opModeIsActive()) {
             //add telemetry for angle
             telemetry.addData("Angle", "%.2f", getAngle());
             telemetry.addData("Target Angle", "%.2f", targetAngle);

             telemetry.update();
             double power = (targetAngle - getAngle()) * 0.01; // Proportional control
             setMotorPowers(power, -power, power, -power); // Rotate
         }

         setMotorPowers(0, 0, 0, 0); // Stop the robot
     }
     public void rotateTo(double targetAngle){
         telemetry.addData("Angle", "%.2f", getAngle());
         telemetry.addData("Target Angle", "%.2f", targetAngle);

         telemetry.update();
         if (targetAngle < getAngle()) {
             while (Math.abs(getAngle() - targetAngle) > .1 && opModeIsActive()) {
                 //add telemetry for angle
                 telemetry.addData("Angle", "%.2f", getAngle());
                 telemetry.addData("Target Angle", "%.2f", targetAngle);

                 telemetry.update();
                 double power = (targetAngle - getAngle()) * 0.01; // Proportional control
                 setMotorPowers(power, -power, power, -power); // Rotate
             }
         }
         else {
             while (Math.abs(getAngle() - targetAngle) > .1 && opModeIsActive()) {
                 //add telemetry for angle
                 telemetry.addData("Angle", "%.2f", getAngle());
                 telemetry.addData("Target Angle", "%.2f", targetAngle);

                 telemetry.update();
                 double power = (targetAngle - getAngle()) * 0.01; // Proportional control
                 setMotorPowers(-power, power, -power, power); // Rotate
             }
         }
     }

     public void setMotorPowers(double power1,double power2, double power3, double power4){
         //sets each motor to the power value
         leftFrontDrive.setPower(power1);
         rightFrontDrive.setPower(power2);
         leftBackDrive.setPower(power3);
         rightBackDrive.setPower(power4);
     }
     public double getAngle(){
         //gets angle from odo odopod
         odo.update();
         return odo.getHeading();
     }




     public void gogogo(){
         odo.update();
         if (aprilTag.getDetections().size() != 0){
             driveByCamera(-30);
         }
         else {
             odoMoveX(-15);
         }
         //move forward
         driveByTime(0, 2);
         //hang specimen
         if (aprilTag.getDetections().size() != 0){
             strafeByCamera(-30);
         }
         else {
             odoMoveY(30);
         }
         //move to the right
         odoMoveX(-20); //move forward
         if (aprilTag.getDetections().size() != 0){
             strafeByCamera(-48);
         }
         else {
             odoMoveY(20);
         }
         //move to in front of block
         rotateTo(0);

         if (aprilTag.getDetections().size() != 0){
             driveByCamera(-72);
         }
         else {
             odoMoveX(-30);
         }
     }
     /*
      driveByTime()
      if (aprilTag.getDetections().size()) {
          double xd = detection.robotPose.getPosition().x;
          double yd = detection.robotPose.getPosition().y;
          double angle = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
          odo.resetPosAndIMU();
      }

  }

  public void placeSpecimen(){
      odoMoveX(-10);
      //fill in later
      driveByTime(0,2);

  }
*/
     public void setLinearSlide(int turnage, double speed) {
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


     public void gotoX(double targetX) {
         odo.update();
         double currentX = getODOx();
         odoMoveX(targetX - currentX);

     }

     public void gotoY(double targetY) {
         odo.update();
         double currentY = getODOy();
         odoMoveY(targetY - currentY);
     }

     private void setShoulderAndWristPositions(Servo leftShoulder, Servo rightShoulder, Servo wristServo, double shoulderPosition, double wristPosition) {
         leftShoulder.setPosition(shoulderPosition);
         rightShoulder.setPosition(1.0 - shoulderPosition);
         wristServo.setPosition(wristPosition);
     }

     public void slidesToTop() {
         setLinearSlide(2088, .5);
     }

     public void openClaw() {
         grabber.setPosition(0.4);
     }

     public void closeClaw() {
         grabber.setPosition(1.0);
     }

     public void slidesToBottom() {
         setLinearSlide(0, .5);
     }



     //Routine for placing a specimein if it is already in the claw and you are at the bar
     public void hangSpecimen() {
     }


     public double getODOx() {
         return odo.getPosX() / odoConversion;
     }

     public double getODOy() {
         return odo.getPosY() / odoConversion;
     }

     public double getHeading() {
         return odo.getHeading();
     }

     public void setBearing(double degrees){
         double radians = Math.toRadians(degrees);
         odo.update();
         if(radians > 0){
             while(odo.getHeading() < radians){

             }
         }


     }

     public void headingCorrection(double degrees) {
         double radians = Math.toRadians(degrees);
         //radians, CCW is +
         odo.update();
         telemetry.addData("heading ", "%.2f", getHeading());
         telemetry.update();
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
             driveByTime(0, 0);
         } else if (odo.getHeading() - radians > 0) {
             while (odo.getHeading() > radians) {
                 telemetry.addData("heading ", "%.2f", getHeading());
                 telemetry.update();
                 leftFrontDrive.setPower(power);
                 rightFrontDrive.setPower(-power);
                 leftBackDrive.setPower(power);
                 rightBackDrive.setPower(-power);
                 odo.update();

             }
             driveByTime(0, 0);
         }


     }

     private void initAprilTag() {

         // Create the AprilTag processor.
         aprilTag = new AprilTagProcessor.Builder()

                 // The following default settings are available to un-comment and edit as needed.
                 //.setDrawAxes(false)
                 //.setDrawCubeProjection(false)
                 //.setDrawTagOutline(true)
                 //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                 //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                 //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                 .setCameraPose(cameraPosition, cameraOrientation)

                 // == CAMERA CALIBRATION ==
                 // If you do not manually specify calibration parameters, the SDK will attempt
                 // to load a predefined calibration for your camera.
                 //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                 // ... these parameters are fx, fy, cx, cy.

                 .build();

         // Adjust Image Decimation to trade-off detection-range for detection-rate.
         // eg: Some typical detection data using a Logitech C920 WebCam
         // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
         // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
         // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
         // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
         // Note: Decimation can be changed on-the-fly to adapt during a match.
         //aprilTag.setDecimation(3);

         // Create the vision portal by using a builder.
         VisionPortal.Builder builder = new VisionPortal.Builder();

         // Set the camera (webcam vs. built-in RC phone camera).
         if (USE_WEBCAM) {
             builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
         } else {
             builder.setCamera(BuiltinCameraDirection.BACK);
         }

         // Choose a camera resolution. Not all cameras support all resolutions.
         //builder.setCameraResolution(new Size(640, 480));

         // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
         //builder.enableLiveView(true);

         // Set the stream format; MJPEG uses less bandwidth than default YUY2.
         //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

         // Choose whether or not LiveView stops if no processors are enabled.
         // If set "true", monitor shows solid orange screen if no processors enabled.
         // If set "false", monitor shows camera view without annotations.
         //builder.setAutoStopLiveView(false);

         // Set and enable the processor.
         builder.addProcessor(aprilTag);

         // Build the Vision Portal, using the above settings.
         visionPortal = builder.build();

         // Disable or re-enable the aprilTag processor at any time.
         //visionPortal.setProcessorEnabled(aprilTag, true);

     }   // end method initAprilTag()

     private void telemetryAprilTag() {

         List<AprilTagDetection> currentDetections = aprilTag.getDetections();
         telemetry.addData("# AprilTags Detected", currentDetections.size());

         // Step through the list of detections and display info for each one.
         for (AprilTagDetection detection : currentDetections) {
             if (detection.metadata != null) {
                 telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                 telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                         detection.robotPose.getPosition().x,
                         detection.robotPose.getPosition().y,
                         detection.robotPose.getPosition().z));
                 telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                         detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                         detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                         detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
             } else {
                 telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                 telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
             }
         }   // end for() loop

         // Add "key" information to telemetry
         telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
         telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

     }   // end method telemetryAprilTag()

     public void strafeByCamera(double xLocation){
         AprilTagDetection detection = aprilTag.getDetections().get(0);
         double xd = detection.robotPose.getPosition().x;
         double yd = detection.robotPose.getPosition().y;
         double angle = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
         double changeX = xLocation - xd;
         telemetry.addData("x, y, change, location", "%.2f, %.2f %.2f %.2f ", xd, yd, changeX, xLocation);
         telemetry.update();
         driveByTime(0,3);
         while(changeX<0) {
             odoMoveY(-2);
             double yd2 = detection.robotPose.getPosition().y;
             double changeY = yd2 - yd;
             double angle2 = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
             double changeAngle = angle2 - angle;
             telemetry.addData("y2, change", "%.2f, %.2f ", yd2, changeY);
             telemetry.addData("x, y, change, location", "%.2f, %.2f %.2f %.2f ", xd, yd, changeX, xLocation);
             telemetry.update();
             if (changeAngle > 15) {
                 rotateTo(0);
             }
             if (changeY > 3) {
                 odoMoveX(.5);
             }
             xd = detection.robotPose.getPosition().x;
         }
         while(changeX>0) {
             odoMoveY(2);
             double yd2 = detection.robotPose.getPosition().y;
             double changeY = yd2 - yd;
             double angle2 = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
             double changeAngle = angle2 - angle;
             telemetry.addData("y2, change", "%.2f, %.2f ", yd2, changeY);
             telemetry.addData("x, y, change, location", "%.2f, %.2f %.2f %.2f ", xd, yd, changeX, xLocation);
             telemetry.update();
             if (changeAngle> 15) {
                 rotateTo(0);
             }
             if(changeY>3) {
                 odoMoveX(.5);
             }
             xd = detection.robotPose.getPosition().x;
         }
         if (aprilTag.getDetections().size()==0){
             odoMoveToY(xLocation);
             telemetry.addData("Doesn't see camera. Go to:", "%.2f ", xLocation);
         }
     }
     public void driveByCamera(double yLocation){
         AprilTagDetection detection = aprilTag.getDetections().get(0);
         double xd = detection.robotPose.getPosition().x;
         double yd = detection.robotPose.getPosition().y;
         double angle = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
         double changeY = yLocation - yd;
         telemetry.addData("x, y, change, location", "%.2f, %.2f %.2f %.2f ", xd, yd);
         telemetry.update();
         driveByTime(0,3);
         while(changeY<0) {
             odoMoveY(-2);
             telemetry.addData("x, y, change, location", "%.2f, %.2f %.2f %.2f ", xd, yd);
             telemetry.update();
             yd = detection.robotPose.getPosition().x;
         }
         while(changeY>0) {

             odoMoveY(2);
             telemetry.addData("x, y, change, location", "%.2f, %.2f %.2f %.2f ", xd, yd);
             telemetry.update();
             yd = detection.robotPose.getPosition().x;
         }
     }

     public void odoMoveX(double distanceX) {
         double power = .4;
         odo.update();

         double startX = getODOx();
         telemetry.addData("driveX:", "%.2f, %.2f %.2f %.2f ", getODOx(), getODOy(), distanceX, startX);
         telemetry.update();

         if (distanceX > 0) { //+X
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

         } else if (distanceX < 0.0) {
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

     public void odoMoveToX(double gotox){
         getODOx();
         odoMoveX(getODOx() - gotox);
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
         } else if (distanceY < 0.0) { //-Y
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
                 telemetry.addData("ODO Y heading", "%.2f", getHeading());

                 telemetry.update();

                 leftFrontDrive.setPower(power);
                 rightFrontDrive.setPower(adjustment * -power);
                 leftBackDrive.setPower(adjustment * -power);
                 rightBackDrive.setPower(power);
                 odo.update();
             }
         }
         driveByTime(0, 0);
     }

     public void odoMoveToY(double gotoy){
         getODOy();
         odoMoveY(getODOy() - gotoy);
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
         if (power < 0) {
             while (runtime.seconds() < time) {
                 leftFrontDrive.setPower(adjustment * -power);
                 rightFrontDrive.setPower(power);
                 leftBackDrive.setPower(power);
                 rightBackDrive.setPower(adjustment * -power);
             }
         }
         if (power > 0) {
             while (runtime.seconds() < time) {
                 leftFrontDrive.setPower(power);
                 rightFrontDrive.setPower(adjustment * -power);
                 leftBackDrive.setPower(adjustment * -power);
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


 }
