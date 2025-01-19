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

package org.firstinspires.ftc.teamcode.notUsing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name="KDC_Left", group="Robot")

public class KDC_Left extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         frontleft   = null;
    private DcMotor         frontright  = null;
    private DcMotor         backleft  = null;
    private DcMotor         backright  = null;
    private DcMotor droppie = null;
    private DcMotor intakie = null;
    private Servo flipity = null;
    private Servo flopity = null;
    private CRServo indulgey = null;
    private CRServo bobby = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1200 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.4;

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

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                          frontleft.getCurrentPosition(),
                          frontright.getCurrentPosition()
                ,backleft.getCurrentPosition(),
                backright.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        //First Basket
        makeFlipityWork(0.4);
        encoderDrive(DRIVE_SPEED, -7, -7, 4.0);// S3: Reverse 24 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED,   -5, 5, 4.0);
        makeDroppieWork(-3000);
        sleep(1500);//2500
        makeFlopityWork(-0.8);
        sleep(1000); //2000
        encoderDrive(DRIVE_SPEED, 4, 4, 4.0);
        makeDroppieWork(0);
        //Drive to the spike mark and pick up a sample
        encoderDrive(0.3, 9.9, 9.9, 4.0);
        makeFlopityWork(0.6);
        encoderDrive(0.2, -17.9, 17.9, 4.0);
        encoderStrafe(0.3, 6, 6, 4);
        makeIntakieWork(-600);
        sleep(250);
        makeFlipityWork(0.95);
        sleep(750);
        makeIntakieWork(-1900);
        makeIndulgeyWork(1);
        sleep(1000);
        makeFlipityWork(0.1);
        makeIntakieWork(0);
        sleep(750);
        makeIndulgeyWork(-1);
        //Second Basket
        encoderStrafe(DRIVE_SPEED, -3, -3, 4);
        makeIndulgeyWork(0);
        makeFlipityWork(0.4);
        encoderDrive(TURN_SPEED, 15.3, -15.3, 4.0);
        makeDroppieWork(-3000);
        encoderDrive(DRIVE_SPEED, -13.35, -13.35, 4.0);
        makeFlopityWork(-0.8);
        sleep(1000);
        //Park
        makeFlopityWork(0.6);
        encoderDrive(DRIVE_SPEED, 5, 5, 4.0);
        makeDroppieWork(-2000);
        encoderDrive(0.4, -15.5, 15.5, 4.0);
        encoderStrafe(0.6, -7, -7, 4);
        encoderStrafe(0.6, 2.5, 2.5, 4);
        encoderDrive(0.6, -25, -25, 4.0);
        sleep(750);
        makeDroppieWork(0);
        encoderDrive(0.4, -15, -15, 4.0);

//        encoderDrive(TURN_SPEED, -5, 5, 4.0);
//        encoderDrive(DRIVE_SPEED, 24, 24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//        encoderDrive(TURN_SPEED, -13, 13, 4.0);
//        makeDroppieWork(-1400);
//        sleep(3000);
//        encoderDrive(DRIVE_SPEED, -5, -5, 4.0);
//        makeDroppieWork(-1200);
//        makeFlopityWork(-0.8);
//        sleep(10000);


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
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
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

            sleep(250);   // optional pause after each move.
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

            sleep(250);   // optional pause after each move.
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
