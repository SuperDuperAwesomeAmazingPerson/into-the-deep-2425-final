 /* Copyright (c) 2017 FIRST. All rights reserved.
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

 package org.firstinspires.ftc.teamcode.autos;

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.util.ElapsedTime;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.Range;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.teamcode.notUsing.GoBildaPinpointDriver;

 /*
  * This OpMode illustrates the concept of driving a path based on encoder counts.
  * The code is structured as a LinearOpMode
  *
  * The code REQUIRES that you DO have encoders on the wheels,
  *   otherwise you would use: RobotAutoDriveByTime;
  *
  *  This code ALSO requires that the drive Motors have been configured such that a positive
  *  power command moves them forward, and causes the encoders to count UP.
  *
  *   The desired path in this example is:
  *   - Drive forward for 48 inches
  *   - Spin right for 12 Inches
  *   - Drive Backward for 24 inches
  *   - Stop and close the claw.
  *
  *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
  *  that performs the actual movement.
  *  This method assumes that each movement is relative to the last stopping place.
  *  There are other ways to perform encoder based moves, but this method is probably the simplest.
  *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
  *
  * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
  * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
  */

 @Autonomous(name="Experiment", group="Robot")

 public class Experiment extends LinearOpMode {

     /* Declare OpMode members. */
     private DcMotor frontleft   = null;
     private DcMotor frontright  = null;
     private DcMotor backleft  = null;
     private DcMotor backright  = null;
     private DcMotor droppie = null;
     private DcMotor intakie = null;
     private Servo flipity = null;
     private Servo flopity = null;
     private CRServo indulgey = null;
     private CRServo bobby = null;
     GoBildaPinpointDriver odo;

     private double          headingError  = 0;

     private ElapsedTime     runtime = new ElapsedTime();

     // Calculate the COUNTS_PER_INCH for your specific drive train.
     // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     // This is gearing DOWN for less speed and more torque.
     // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
     static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: GoBilda 312 RPM Yellow Jacket
     static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
     static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
     static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
     static final double     DRIVE_SPEED             = 0.575;
     static final double     TURN_SPEED              = 0.5;
     static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
     // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
     // Define the Proportional control coefficient (or GAIN) for "heading control".
     // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
     // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
     // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
     static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable.
     static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

     @Override
     public void runOpMode() {

         // Initialize the drive system variables.
         frontleft  = hardwareMap.get(DcMotor.class, "FL");
         backleft = hardwareMap.get(DcMotor.class, "BL");
         frontright  = hardwareMap.get(DcMotor.class, "FR");
         backright = hardwareMap.get(DcMotor.class, "BR");
         droppie = hardwareMap.get(DcMotor.class, "droppie");
         intakie = hardwareMap.get(DcMotor.class, "intakie");
         flipity = hardwareMap.get(Servo.class, "flipity");
         flopity = hardwareMap.get(Servo.class, "flopity");
         bobby = hardwareMap.get(CRServo.class, "bobby");
         indulgey = hardwareMap.get(CRServo.class, "indulgey");
         odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");


         // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
         // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
         // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
         frontleft.setDirection(DcMotor.Direction.REVERSE);
         frontright.setDirection(DcMotor.Direction.FORWARD);
         backleft.setDirection(DcMotor.Direction.REVERSE);
         backright.setDirection(DcMotor.Direction.FORWARD);

         backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         intakie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         droppie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         intakie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         droppie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         intakie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         odo.resetPosAndIMU();

         // Send telemetry message to indicate successful Encoder reset
         telemetry.addData("Starting at",  "%7d :%7d",
                 frontleft.getCurrentPosition(),
                 frontright.getCurrentPosition()
                 ,backleft.getCurrentPosition(),
                 backright.getCurrentPosition());
         telemetry.update();

         // Wait for the game to start (driver presses START)
         waitForStart();
//      *******************************
//       The actual movements go here!
//      *******************************
         telemetry.addData("Path", "Complete");
         telemetry.update();
         sleep(1000);  // pause to display final telemetry message.
     }

     /*
      *  Method to perform a relative move, based on encoder counts.
      *  Encoders are not reset as the move is based on the current position.
      *  Move will stop if any of three conditions occur:
      *  1) Move gets to the desired position
      *  2) Move runs out of time
      *  3) Driver stops the OpMode running.
      */

     public double getSteeringCorrection(double targetHeading, double proportionalGain) {

         // Determine the heading current error
         headingError = targetHeading - odo.getHeading(AngleUnit.DEGREES);

         // Normalize the error to be within +/- 180 degrees
         while (headingError > 180)  headingError -= 360;
         while (headingError <= -180) headingError += 360;

         // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
         return Range.clip(headingError * proportionalGain, -1, 1);
     }

     public void turnToHeading(double speed,
                               double angle) {
         // Run getSteeringCorrection() once to pre-calculate the current error
         getSteeringCorrection(angle, P_DRIVE_GAIN);

         // keep looping while we are still active, and not on heading.
         while (opModeIsActive() && headingError > 1) {
             frontleft.setPower(-TURN_SPEED);
             frontright.setPower(TURN_SPEED);
             backleft.setPower(TURN_SPEED);
             backright.setPower(-TURN_SPEED);
             odo.update();
             }
         while (opModeIsActive() && headingError < -1) {
             frontleft.setPower(TURN_SPEED);
             frontright.setPower(-TURN_SPEED);
             backleft.setPower(-TURN_SPEED);
             backright.setPower(TURN_SPEED);
             odo.update();
         }
     }

     public void encoderDrive(double speed,
                              double leftInches, double rightInches, //double heading,
                              double timeoutS) {
         int newLeftTarget;
         int newRightTarget;

         // Ensure that the OpMode is still active
         if (opModeIsActive()) {

             // Determine new target position, and pass to motor controller
             newLeftTarget = backleft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
             newRightTarget = backright.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

             backleft.setTargetPosition(newLeftTarget);
             backright.setTargetPosition(newRightTarget);
             frontright.setTargetPosition(newRightTarget);
             frontleft.setTargetPosition(newLeftTarget);

             // Turn On RUN_TO_POSITION
             frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

             // reset the timeout time and start motion.
             runtime.reset();
             frontright.setPower(Math.abs(speed));
             frontleft.setPower(Math.abs(speed));
             backleft.setPower(Math.abs(speed));
             backright.setPower(Math.abs(speed));

             // keep looping while we are still active, and there is time left, and both motors are running.
             // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
             // its target position, the motion will stop.  This is "safer" in the event that the robot will
             // always end the motion as soon as possible.
             // However, if you require that BOTH motors have finished their moves before the robot continues
             // onto the next step, use (isBusy() || isBusy()) in the loop test.
             while (opModeIsActive() &&
                     (runtime.seconds() < timeoutS) &&
                     (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy())) {

                 // Display it for the driver.
                 telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                 telemetry.addData("Currently at",  " at %7d :%7d",
                         backleft.getCurrentPosition(), backright.getCurrentPosition(), frontleft.getCurrentPosition(), frontright.getCurrentPosition());
                 telemetry.update();
             }

             // Stop all motion;
             frontright.setPower(0);
             frontleft.setPower(0);
             backright.setPower(0);
             backleft.setPower(0);

             // Turn off RUN_TO_POSITION
             frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         }

     }

     public void encoderStrafe(double speed,
                               double inchesToStrafeLeft,
                               double inchesToStrafeRight,
                               int timeoutS) {
         double FlInches = inchesToStrafeLeft;
         double FrInches = -inchesToStrafeRight;
         double BlInches = -inchesToStrafeLeft;
         double BrInches = inchesToStrafeRight;


         int newFrontLeftTarget;
         int newFrontRightTarget;
         int newBackLeftTarget;
         int newBackRightTarget;

         // Ensure that the OpMode is still active
         if (opModeIsActive()) {

             // Determine new target position, and pass to motor controller
             newFrontLeftTarget = frontleft.getCurrentPosition() + (int)(FlInches * COUNTS_PER_INCH);
             newFrontRightTarget = frontright.getCurrentPosition() + (int)(FrInches * COUNTS_PER_INCH);
             newBackRightTarget = backright.getCurrentPosition() + (int)(BrInches * COUNTS_PER_INCH);
             newBackLeftTarget = backleft.getCurrentPosition() + (int)(BlInches * COUNTS_PER_INCH);

             backleft.setTargetPosition(newBackLeftTarget);
             backright.setTargetPosition(newBackRightTarget);
             frontright.setTargetPosition(newFrontRightTarget);
             frontleft.setTargetPosition(newFrontLeftTarget);

             // Turn On RUN_TO_POSITION
             frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);



             // reset the timeout time and start motion.
             runtime.reset();
             frontright.setPower(Math.abs(speed));
             frontleft.setPower(Math.abs(speed));
             backleft.setPower(Math.abs(speed));
             backright.setPower(Math.abs(speed));

             // keep looping while we are still active, and there is time left, and both motors are running.
             // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
             // its target position, the motion will stop.  This is "safer" in the event that the robot will
             // always end the motion as soon as possible.
             // However, if you require that BOTH motors have finished their moves before the robot continues
             // onto the next step, use (isBusy() || isBusy()) in the loop test.
             while (opModeIsActive() &&
                     (runtime.seconds() < timeoutS) &&
                     (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy())) {

                 // Display it for the driver.
                 telemetry.addData("Test %7d", backright.getCurrentPosition());
//                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                        backleft.getCurrentPosition(), backright.getCurrentPosition(), frontleft.getCurrentPosition(), frontright.getCurrentPosition());
                 telemetry.update();
             }

             // Stop all motion;
             frontright.setPower(0);
             frontleft.setPower(0);
             backright.setPower(0);
             backleft.setPower(0);

             // Turn off RUN_TO_POSITION
             frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

             frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }
     }
     public void encoderDiagonal(double speed,
                                 double diagonalLeft,
                                 double diagonalRight,
                                 int timeoutS) {
         double Flinches = diagonalRight;
         double Frinches = diagonalLeft;
         double Blinches = diagonalLeft;
         double Brinches = diagonalRight;

         int newFrontLeftTarget;
         int newFrontRightTarget;
         int newBackLeftTarget;
         int newBackRightTarget;

         if (opModeIsActive()) {

             // Determine new target position, and pass to motor controller
             newFrontLeftTarget = frontleft.getCurrentPosition() + (int) (Flinches * COUNTS_PER_INCH);
             newFrontRightTarget = frontright.getCurrentPosition() + (int) (Frinches * COUNTS_PER_INCH);
             newBackRightTarget = backright.getCurrentPosition() + (int) (Brinches * COUNTS_PER_INCH);
             newBackLeftTarget = backleft.getCurrentPosition() + (int) (Blinches * COUNTS_PER_INCH);

             backleft.setTargetPosition(newBackLeftTarget);
             backright.setTargetPosition(newBackRightTarget);
             frontright.setTargetPosition(newFrontRightTarget);
             frontleft.setTargetPosition(newFrontLeftTarget);

             // Turn On RUN_TO_POSITION
             frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


             // reset the timeout time and start motion.
             runtime.reset();
             frontright.setPower(Math.abs(speed));
             frontleft.setPower(Math.abs(speed));
             backleft.setPower(Math.abs(speed));
             backright.setPower(Math.abs(speed));

             // keep looping while we are still active, and there is time left, and both motors are running.
             // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
             // its target position, the motion will stop.  This is "safer" in the event that the robot will
             // always end the motion as soon as possible.
             // However, if you require that BOTH motors have finished their moves before the robot continues
             // onto the next step, use (isBusy() || isBusy()) in the loop test.
             while (opModeIsActive() &&
                     (runtime.seconds() < timeoutS) &&
                     (backleft.isBusy() && backright.isBusy() && frontleft.isBusy() && frontright.isBusy())) {

                 // Display it for the driver.
                 telemetry.addData("Test %7d", backright.getCurrentPosition());
//                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                        backleft.getCurrentPosition(), backright.getCurrentPosition(), frontleft.getCurrentPosition(), frontright.getCurrentPosition());
                 telemetry.update();
             }

             // Stop all motion;
             frontright.setPower(0);
             frontleft.setPower(0);
             backright.setPower(0);
             backleft.setPower(0);

             // Turn off RUN_TO_POSITION
             frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
             backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

             frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
             backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         }
     }


     public void makeDroppieWork(int position){
         droppie.setTargetPosition(position); //-1400
         droppie.setPower(-0.75);
         droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }

     public void makeIntakieWork(int pos){
         intakie.setTargetPosition(pos);//800
         intakie.setPower(0.8);//0.8);
         intakie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     }

     public void makeBobbyWork(double power){
         bobby.setPower(power);//-0.6
     }

     public void makeFlipityWork(double pos){
         flipity.setPosition(pos);//0.8387);
     }

     public void makeFlopityWork(double pos){
         flopity.setPosition(pos);//0.8387);
     }

     public void makeIndulgeyWork(double power){
         indulgey.setPower(power);
     }
 }
