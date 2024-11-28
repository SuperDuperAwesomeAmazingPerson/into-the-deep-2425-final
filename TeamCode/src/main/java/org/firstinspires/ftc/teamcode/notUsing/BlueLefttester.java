/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.notUsing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.

The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@Autonomous(name="BlueLefttester", group="Linear OpMode")

public class BlueLefttester extends LinearOpMode {

    private DcMotor FRMotor = null;
    private DcMotor FLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor droppie = null;
    private CRServo bobby = null;

    //private DcMotor intakie;  // Motor for the extending/retracting mechanism

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    //Starting location
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalH = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        FRMotor  = hardwareMap.get(DcMotor.class, "FR");
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        droppie = hardwareMap.get(DcMotor.class, "droppie");
        bobby = hardwareMap.get(CRServo.class, "bobby");

        //intakie = hardwareMap.get(DcMotor.class, "intakie");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);

        //intakie.setDirection(DcMotor.Direction.FORWARD);


        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        droppie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        droppie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //intakie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        Set the odometry pod positions relative to the point that the odometry computer tracks around.
        The X pod offset refers to how far sideways from the tracking point the
        X (forward) odometry pod is. Left of the center is a positive number,
        right of center is a negative number. the Y pod offset refers to how far forwards from
        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
        backwards is a negative number.
         */
        //  odo.setOffsets(-84.0, -224.0); //these are tuned for 3110-0002-0001 Product Insight #1
        //odo.setOffsets(-153.71, -215.019);
        odo.setOffsets(-192, -68);
        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);


        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        //odo.recalibrateIMU();
        odo.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        //goToPos(3000, 0, Math.toRadians(0), .4, 25, Math.toRadians(5));
        //goToPos(0, 500 , Math.toRadians(0), .6, 15, Math.toRadians(5));
        //Y OFFSET SHOULD BE 76.7 mm

        //*******************************************
        //HANG A SPECIMEN AND PARK WITH A SAMPLE!!!
        //*******************************************

        //Lift goes up
        droppie.setTargetPosition(-1700);
        droppie.setPower(-0.8);
        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        //Robot drives forward (Movement #1)
        goToPos(-760, 127 , Math.toRadians(0), 0.6, 30, Math.toRadians(2));
        telemetry.addData("Finished",0);
        telemetry.update();
        sleep(3000);
        //Lift goes on and specimen hooks onto the bar
        droppie.setTargetPosition(-1400);
        droppie.setPower(-0.6);
        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Wait
        //sleep(500);
        //Claw releases specimen
        bobby.setPower(-0.6);
        sleep(2000);
        bobby.setPower(0);
        goToPos(-650.6, -127 , Math.toRadians(0), 0.6, 25, Math.toRadians(2));
        sleep(2000);
        //Robot moves to diagonal midpoint (Movement #2)
        goToPos(-88.9, -914.4 , Math.toRadians(0), 0.6, 25, Math.toRadians(5));
//        goToPos(-609.6, 374.65 , Math.toRadians(-180), .35, 25, Math.toRadians(5));
        sleep(2000);
        //Lift drops down all the way
        droppie.setTargetPosition(0);
        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
//        //Robot moves to first spike mark (Movement #3)
//        goToPos(-1295.4, 914.4 , Math.toRadians(180), .35, 25, Math.toRadians(2));
//        sleep(2000);
//        //Robot pushes sample into Observation Zone (Movement #4)
//        goToPos(-88.9, 914.4 , Math.toRadians(180), .35, 25, Math.toRadians(2));


        // Motor power is based on gyro angle/rotation
        // sleep(5000);
        //goToPos(-670, -110 , Math.toRadians(0), .5, 15, Math.toRadians(1));
        //goToPos(1092.2, 673.1 , Math.toRadians(180), .6, 15, Math.toRadians(5));
        //673.1-91.4 = 581.7
        //goToPos(1092.2, 581.7 , Math.toRadians(180), .6, 15, Math.toRadians(5));
//        resetRuntime();

//        /*
//        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
//         */
//        Pose2D pos = odo.getPosition();
//        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Position", data);
//
//        /*
//        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
//         */
//        Pose2D vel = odo.getVelocity();
//        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//        telemetry.addData("Velocity", velocity);
//
//
//        /*
//        Gets the Pinpoint device status. Pinpoint can reflect a few states. But we'll primarily see
//        READY: the device is working as normal
//        CALIBRATING: the device is calibrating and outputs are put on hold
//        NOT_READY: the device is resetting from scratch. This should only happen after a power-cycle
//        FAULT_NO_PODS_DETECTED - the device does not detect any pods plugged in
//        FAULT_X_POD_NOT_DETECTED - The device does not detect an X pod plugged in
//        FAULT_Y_POD_NOT_DETECTED - The device does not detect a Y pod plugged in
//        */
//        telemetry.addData("Status", odo.getDeviceStatus());
//
//        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
//
//        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
//        telemetry.update();

    }
    // used to mantain angle values between Pi and -Pi
    public double angleWrapRad(double angle)
    {
        while (angle > Math.PI)
        {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI)
        {
            angle += Math.PI * 2;
        }

        return angle;
    }

    public void refresh(){
        odo.update();
        Pose2D pos = odo.getPosition();
        GlobalX = pos.getX(DistanceUnit.MM);
        GlobalY = pos.getY(DistanceUnit.MM);
        GlobalH = -pos.getHeading(AngleUnit.RADIANS);
    }

    public void goToPosSingle(double x, double y, double h, double speed){

        refresh();

        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteAngleToTarget = Math.atan2(x - GlobalX, y - GlobalY);
        double reletiveAngleToTarget = angleWrapRad(absoluteAngleToTarget - GlobalH-Math.toRadians(90));
        double reletiveXToTarget = -Math.cos(reletiveAngleToTarget) * distanceToTarget;
        double reletiveYToTarget = -Math.sin(reletiveAngleToTarget) * distanceToTarget;

        //slow down ensures the robot does not over shoot the target
        double slowDown = Range.clip(distanceToTarget / 3, -speed, speed);

        //calculate the vector powers for the mecanum math
        double movementXpower = (reletiveXToTarget / (Math.abs(reletiveXToTarget) + Math.abs(reletiveYToTarget))) * slowDown;
        double movementYpower = (reletiveYToTarget / (Math.abs(reletiveYToTarget) + Math.abs(reletiveXToTarget))) * slowDown;

        double reletiveTurnAngle = angleWrapRad(h - GlobalH);
        double movementTurnPower = Range.clip(reletiveTurnAngle / Math.toRadians(10), -speed, speed);

        FLMotor.setPower(-movementYpower - movementXpower - movementTurnPower);
        BLMotor.setPower(-movementYpower + movementXpower - movementTurnPower);
        FRMotor.setPower(-movementYpower + movementXpower + movementTurnPower);
        BRMotor.setPower(-movementYpower - movementXpower + movementTurnPower);

    }

    public void goToPos(double x, double y, double h, double speed, double moveAccuracy, double angleAccuracy){
        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        while(Math.abs(x-GlobalX) > moveAccuracy || Math.abs(y-GlobalY) > moveAccuracy || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy) {
            // while(true){
            goToPosSingle(x, y, h, speed);

            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.update();


        }

        //stop all movement at the end of while loop
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);

    }}

// vertical distance 43 inches 109.22 cm - 1092.2 mm
// horizontal distance  odometer at 26.5 inches 67.31 cm - 673.1 mm
//                      start of wheel 31 inches 78.74 - 787.4 mm
//Angle 180 degrees - pi