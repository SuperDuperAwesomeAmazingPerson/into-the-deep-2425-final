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

package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.notUsing.GoBildaPinpointDriver;

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

@Autonomous(name="LQ_Right", group="Linear OpMode")
//@Disabled

public class LQ_Right extends LinearOpMode {

    private DcMotor FRMotor = null;
    private DcMotor FLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor BLMotor = null;

    private DcMotor droppie = null;
    private DcMotor intakie = null;

    private Servo flipity = null;
    private Servo flopity = null;
    private CRServo indulgey = null;
    private CRServo bobby = null;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    //Starting location
    public double GlobalX = 0;
    public double GlobalY = 0;
    public double GlobalH = 0;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        FRMotor  = hardwareMap.get(DcMotor.class, "FR");
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");

        droppie = hardwareMap.get(DcMotor.class, "droppie");
        intakie = hardwareMap.get(DcMotor.class, "intakie");

        flipity = hardwareMap.get(Servo.class, "flipity");
        flopity = hardwareMap.get(Servo.class, "flopity");
        indulgey = hardwareMap.get(CRServo.class, "indulgey");
        bobby = hardwareMap.get(CRServo.class, "bobby");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

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
        odo.setOffsets(-210, -150);
        //New Offsets (x-201.61, y-173.04)
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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);


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
        Pose2D pos = odo.getPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        String data = String.format(Locale.US, "X: %.3f, Y: %.3f, H: %.3f", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        //*******************************************
        //SAMPLE PICK AND DEPOSIT MODE!!!
        //*******************************************
/*
        //Specimen #1
        goToPos(0, 750, Math.toRadians(0), 0.7, 30, 20, Math.toRadians(10));
        goToPos(0, 950, Math.toRadians(0), 0.7, 30, 205, Math.toRadians(10));
        goToPosStop();
        sleep(1000);

        //Pickup sample #1 from spike marks
        goToPos(500, 600, Math.toRadians(40), 0.7, 40,40, Math.toRadians(10));
        goToPosStop();
        sleep(1000);

        //Deposit sample #1
        goToPos(500, 600, Math.toRadians(-60), 0.7, 40,40, Math.toRadians(20));
        goToPosStop();
        sleep(1000);

        // Pickup specimen #2;
        goToPos(1000, 0, Math.toRadians(-179), 0.7, 40,20, Math.toRadians(10));
        goToPos(1000, -200, Math.toRadians(-179), 0.7, 40,205, Math.toRadians(10));
        goToPosStop();
        sleep(1000);

        //Place specimen #2
        goToPos(0, 750, Math.toRadians(0), 0.7, 30, 20, Math.toRadians(10));
        goToPos(0, 950, Math.toRadians(0), 0.7, 30, 205, Math.toRadians(10));
        goToPosStop();
        sleep(1000);

        //Pickup specimen #3
        goToPos(1000, 10, Math.toRadians(-179), 0.75, 40,40, Math.toRadians(10));
        goToPosShortDis(1000, 0, Math.toRadians(-179), 0.3, 20, 10, Math.toRadians(10));
        goToPosStop();
        sleep(1000);

        //Place specimen #3
        goToPos(0, 750, Math.toRadians(0), 0.7, 30, 20, Math.toRadians(10));
        goToPos(0, 950, Math.toRadians(0), 0.7, 30, 205, Math.toRadians(10));
        goToPosStop();
        sleep(1000);

        //Park
        goToPos(1000, 10, Math.toRadians(0), 0.8, 30,30, Math.toRadians(20));
        goToPosStop();

*/

        //*******************************************
        //SAMPLE PUSH MODE!! (1+2)
        //*******************************************

        //Specimen #1 (Preload)
        makeFlipityWork(0.45);
        makeDroppieWork(-1500);
        makeFlopityWork(0.6);
//        goToStart(0, 650, Math.toRadians(0), 0.6, 30, 20, Math.toRadians(10), 3);
        goToPos(0, 740, Math.toRadians(0), 0.6, 30, 20, Math.toRadians(10), 3);
        goToPos(0, 950, Math.toRadians(0), 0.4, 30, 210, Math.toRadians(10), 1);
        goToPosStop();
        makeBobbyWork(-0.5);
        makeDroppieWork(-1050);
        sleep(100);
        makeBobbyWork(1);
        sleep(1000);
        makeBobbyWork(0);

        //Push
        makeDroppieWork(-250);
//        goToStart(0, 450, Math.toRadians(0), 0.6, 100, 100, Math.toRadians(20), 3);
        goToPos(0, 450, Math.toRadians(0), 0.6, 100, 100, Math.toRadians(20), 3);
        goToPos(630, 450, Math.toRadians(90), 0.6, 100, 100, Math.toRadians(20), 3);
        goToPos(650, 600, Math.toRadians(90), 0.6, 100, 100, Math.toRadians(20), 3);
        goToPos(750, 1100, Math.toRadians(90), 0.6, 100, 100, Math.toRadians(20), 3);
        goToPos(850, 1300, Math.toRadians(90), 0.4, 50, 50, Math.toRadians(20), 3);
        goToPosStop();
//        goToStart(850, 600, Math.toRadians(90), 0.9, 150, 200, Math.toRadians(20), 3);
        goToPos(850, 150, Math.toRadians(90), 0.9, 150, 200, Math.toRadians(20), 3);
        goToPos(950, 1250, Math.toRadians(90), 0.6, 120,120, Math.toRadians(20), 3);
        goToPos(1210, 1320, Math.toRadians(90), 0.4, 150, 150, Math.toRadians(20), 3);
        goToPosStop();
//        goToStart(1150, 600, Math.toRadians(90), 0.8, 150, 150, Math.toRadians(20), 3);
        goToPos(1200, 320, Math.toRadians(90), 0.8, 150, 150, Math.toRadians(20), 3);

        // Pickup specimen #2
//        goToStart(1000, 300, Math.toRadians(181), 0.6, 50,20, Math.toRadians(10), 3);
        goToPos(1000, 300, Math.toRadians(181), 0.6, 50,20, Math.toRadians(10), 3);
        goToPos(1000, -200, Math.toRadians(181), 0.6, 50, 205, Math.toRadians(10), 3);
        goToPosStop();
        makeBobbyWork(-1);
        sleep(750);
        makeBobbyWork(-0.5);

        //Place specimen #2
        makeDroppieWork(-1500);
//        goToStart(-60, 200, Math.toRadians(-10), 0.7, 50, 40, Math.toRadians(20), 3);
        goToPos(-60, 450, Math.toRadians(-10), 0.7, 50, 40, Math.toRadians(20), 3);
        goToPos(-60, 750, Math.toRadians(0), 0.6, 50, 20, Math.toRadians(10), 3);
        goToPos(-60, 945, Math.toRadians(0), 0.4, 50, 190, Math.toRadians(10), 1);
        goToPosStop();
        makeDroppieWork(-1050);
        makeBobbyWork(1);
        sleep(1000);
        makeBobbyWork(0);

        //Pickup specimen #3
        makeDroppieWork(-250);
//        goToStart(1000, 600, Math.toRadians(90), 0.6, 50, 30, Math.toRadians(20), 3);
        goToPos(1000, 600, Math.toRadians(90), 0.6, 50, 30, Math.toRadians(20), 3);
        goToPos(1000, 600, Math.toRadians(195), 0.6, 75, 50, Math.toRadians(15), 3);
        goToPos(1000, 5, Math.toRadians(195), 0.6, 20, 30, Math.toRadians(10), 1);
        goToPos(1000, -320, Math.toRadians(200), 0.6, 50, 310, Math.toRadians(10), 1);
        goToPosStop();
        makeBobbyWork(-1);
        sleep(750);
        makeBobbyWork(-0.5);

        //Place specimen #3
        makeDroppieWork(-1500);
//        goToStart(-100, 450, Math.toRadians(-10), 0.7, 50, 40, Math.toRadians(20), 3);
        goToPos(-100, 450, Math.toRadians(-10), 0.7, 50, 40, Math.toRadians(20), 3);
        goToPos(-150, 750, Math.toRadians(0), 0.6, 50, 20, Math.toRadians(10), 3);
        goToPos(-150, 945, Math.toRadians(0), 0.4, 50, 190, Math.toRadians(10), 1);
        goToPosStop();
        makeDroppieWork(-1050);
        makeBobbyWork(1);
        sleep(1000);
        makeBobbyWork(0);

        //Park
        makeDroppieWork(0);
//        goToStart(200, 500, Math.toRadians(-30), 1, 30, 30, Math.toRadians(20), 3);
        makeIntakieWork(-2200);
        makeFlipityWork(0.65);
        goToPos(500, 400, Math.toRadians(-35), 1, 30, 30, Math.toRadians(20), 3);
//        goToPos(600, 300, Math.toRadians(-40), 1, 30, 30, Math.toRadians(20), 3);
        goToPosStop();
        sleep(2000);



//        goToPos(50, 0, 0, 0.6, 1, Math.toRadians(5));

//        //Lift goes up
//        droppie.setTargetPosition(-1700);
//        droppie.setPower(-0.8);
//        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sleep(1000);
//        //Robot drives forward (Movement #1)
//        goToPos(-760, -127 , Math.toRadians(0), .35, 30, Math.toRadians(2));
//        telemetry.addData("Finished",0);
//        telemetry.update();
//        sleep(1000);
//        //Lift goes on and specimen hooks onto the bar
//        droppie.setTargetPosition(-1250);
//        droppie.setPower(-0.6);
//        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //Wait
//        //sleep(500);
//        //Claw releases specimen
//        bobby.setPower(-0.6);
//        sleep(1500);
//        bobby.setPower(0);
//        goToPos(-650.6, -127 , Math.toRadians(0), .35, 25, Math.toRadians(2));
//        sleep(1000);
//        //Lift drops down all the way
//        droppie.setTargetPosition(0);
//        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sleep(1000);
//        //Robot moves to diagonal midpoint (Movement #2)
//        goToPos(-150, -150 , Math.toRadians(0), .35, 25, Math.toRadians(5));
//        sleep(1000);
//        goToPos(-88.9, 950 , Math.toRadians(0), .35, 25, Math.toRadians(5));
////        goToPos(-609.6, 374.65 , Math.toRadians(-180), .35, 25, Math.toRadians(5));
//        sleep(1000);

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

    public void moveForward(double x, double speed) {
        while((x - GlobalH) > Math.toRadians(5)) {
            FLMotor.setPower(-speed);
            BLMotor.setPower(-speed);
            FRMotor.setPower(speed);
            BRMotor.setPower(speed);
            refresh();
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);
            telemetry.addData("GlobalH", GlobalH);
            telemetry.update();
        }
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);
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
        GlobalH = pos.getHeading(AngleUnit.RADIANS);
    }

    double integralSum = 0;
    double feedfoward = 0;
    double Kp = 0.6;
    double Ki = 0.32;
    double Kd = 0.17;
    double Kf = 0.25;
    private double lastError = 0;

    double integralSumX = 0;
    double KpX=0.04;
    double KiX=0.002;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdX=0.008;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardX = 0;
    private double lastErrorX = 0;

    double integralSumY = 0;
    double KpY=0.04;
    double KiY=0.002;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdY=0.008;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardY = 0;
    private double lastErrorY = 0;

    double correctFactorCoeff = 300;
    double initialDistanceToTarget = 0;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timerX = new ElapsedTime();
    ElapsedTime timerY = new ElapsedTime();

    public double PIDControlH(double reference, double state) {
        double error = angleWrapRad(reference - state);
        integralSum += error*timer.seconds();
        double derivative = (error - lastError) / (timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error*Kp) + (derivative*Kd) + (integralSum*Ki) + (feedfoward*Kf);
        return output;
    }

    public double PIDControlX(double reference, double state) {
        double error = reference - state;
        integralSumX += error*timerX.seconds();
        double derivative = (error - lastErrorX) / (timerX.seconds());
        lastErrorX = error;
        timerX.reset();
        double output = (error*KpX) + (derivative*KdX) + (integralSumX*KiX);
        return output;
    }

    public double PIDControlY(double reference, double state) {
        double error = reference - state;
        integralSumY += error*timerY.seconds();
        double derivative = (error - lastErrorY) / (timerY.seconds());
        lastErrorY = error;
        timerY.reset();
        double output = (error*KpY) + (derivative*KdY) + (integralSumY*KiY);
        return output;
    }


    public void goToPosSingle(double x, double y, double h, double speed){

        refresh();
        //math to calculate distances to the target
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x- GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h-GlobalH);

        double correctFactor = correctFactorCoeff;
        if (initialDistanceToTarget>1200) { correctFactor = 3.5*correctFactorCoeff;}
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor*Math.abs(relativeTurnAngle) ;

//        double movementXpower = relativeXToTarget / maxPower * speed;
//        double movementYpower = relativeYToTarget / maxPower * speed;

        double PIDX = PIDControlX(x, GlobalX)*Math.signum(Math.cos(GlobalH));
        double PIDY = PIDControlY(y, GlobalY)*Math.signum(Math.cos(GlobalH));
        double PIDH = PIDControlH(h, GlobalH);
        double movementXpower = PIDX * speed * (Math.abs(relativeXToTarget)/maxPower) ;
        double movementYpower = PIDY * speed * (Math.abs(relativeYToTarget)/maxPower);
        double movementTurnPower = PIDH * speed * (correctFactor*Math.abs(relativeTurnAngle)/maxPower);


        telemetry.addData("distanceToTarget", distanceToTarget);
        telemetry.addData("movementXpower", movementXpower);
        telemetry.addData("movementYpower", movementYpower);
        telemetry.addData("movementTurnPower", movementTurnPower);
        telemetry.addData("relativeYToTarget", relativeYToTarget);
        telemetry.addData("absoluteAngleToTarget", absoluteTurnAngle);
        telemetry.addData("relativeAngleToTarget", relativeAngleToTarget);
        telemetry.addData("GlobalX", GlobalX);
        telemetry.addData("GlobalY", GlobalY);
        telemetry.addData("GlobalH", Math.toDegrees(GlobalH));
        telemetry.update();

        FLMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
        FRMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
        BLMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
        BRMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));

    }

    public void goToPos(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS) {
        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        integralSum = 0;
        integralSumX = 0;
        integralSumY = 0;
        refresh();
        feedfowardX = x - GlobalX;
        feedfowardY = y - GlobalY;
        feedfoward = h - GlobalH;

        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);
        double correctFactor = correctFactorCoeff;
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);
        double initialSpeed = 0.2;
        double movementXpower = initialSpeed * relativeXToTarget / maxPower;
        double movementYpower = initialSpeed * relativeYToTarget / maxPower;
        double movementTurnPower = initialSpeed * correctFactor * relativeTurnAngle / maxPower;

        runtime.reset();
        FLMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -initialSpeed, initialSpeed));
        FRMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -initialSpeed, initialSpeed));
        BLMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -initialSpeed, initialSpeed));
        BRMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -initialSpeed, initialSpeed));
//        sleep(5);
        initialDistanceToTarget = distanceToTarget;
        while (((Math.abs(x - GlobalX) > moveAccuracyX || Math.abs(y - GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy)) && opModeIsActive() && (runtime.seconds() < timeoutS)) {
            // while(true){

            goToPosSingle(x, y, h, speed);

//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//            telemetry.update();


        }
    }

    public void goToTest(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy, double timeoutS) {
        //while loop makes the code keep running till the desired location is reached. (within the accuracy constraints)
        integralSum = 0;
        integralSumX = 0;
        integralSumY = 0;
        refresh();
        feedfowardX = x - GlobalX;
        feedfowardY = y - GlobalY;
        feedfoward = h - GlobalH;

        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);
        double correctFactor =  correctFactorCoeff;
        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);
//        double initialSpeed=0.2;
        double movementXpower = relativeXToTarget / maxPower ;
        double movementYpower = relativeYToTarget / maxPower ;
        double movementTurnPower = correctFactor * relativeTurnAngle / maxPower;

        runtime.reset();
        FLMotor.setPower(movementXpower - movementYpower - movementTurnPower);
        FRMotor.setPower(movementXpower + movementYpower + movementTurnPower);
        BLMotor.setPower(movementXpower + movementYpower - movementTurnPower);
        BRMotor.setPower(movementXpower - movementYpower + movementTurnPower);
//        sleep(5);
        initialDistanceToTarget = distanceToTarget;
        while (((Math.abs(x - GlobalX) > moveAccuracyX || Math.abs(y - GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy)) && opModeIsActive() && (runtime.seconds() < timeoutS)){
            // while(true){

            goToPosSingle(x, y, h, speed);

//            Pose2D pos = odo.getPosition();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//            telemetry.addData("Position", data);
//            telemetry.update();


        }

        //stop all movement at the end of while loop
    }

    public void goToPosShortDis(double x, double y, double h, double speed, double moveAccuracyX, double moveAccuracyY, double angleAccuracy){

            refresh();
            double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
            double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
            double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
            double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
            double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
            double relativeTurnAngle = angleWrapRad(h - GlobalH);

            double correctFactor=300;
            double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);

            double movementXpower = speed * relativeXToTarget / maxPower ;
            double movementYpower = speed * relativeYToTarget / maxPower ;
            double movementTurnPower = speed * correctFactor * relativeTurnAngle / maxPower;

            while (Math.abs(x - GlobalX) > moveAccuracyX || Math.abs(y - GlobalY) > moveAccuracyY || Math.abs(angleWrapRad(h - GlobalH)) > angleAccuracy) {
                // while(true){

                FLMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
                FRMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
                BLMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
                BRMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));

                telemetry.addData("movementXpower", movementXpower);
                telemetry.addData("movementYpower", movementYpower);
                telemetry.addData("movementTurnPower", movementTurnPower);
                telemetry.addData("relativeYToTarget", relativeYToTarget);
                telemetry.addData("sign:", Math.signum(Math.cos(GlobalH)));
                telemetry.addData("maxPower", maxPower);
                telemetry.addData("GlobalX", GlobalX);
                telemetry.addData("GlobalY", GlobalY);
                telemetry.addData("GlobalH", Math.toDegrees(GlobalH));
                telemetry.update();
                refresh();
            }
    }

    public void goToPosStop (){
        FLMotor.setPower(0);
        BLMotor.setPower(0);
        FRMotor.setPower(0);
        BRMotor.setPower(0);
    }
    /*
    public void goToStart(double x, double y, double h, double speed, int sleep_time){

        refresh();
        double distanceToTarget = Math.hypot(x - GlobalX, y - GlobalY);
        double absoluteTurnAngle = Math.atan2(y - GlobalY, x - GlobalX);
        double relativeAngleToTarget = angleWrapRad(absoluteTurnAngle - GlobalH);
        double relativeXToTarget = distanceToTarget * Math.cos(relativeAngleToTarget);
        double relativeYToTarget = distanceToTarget * Math.sin(relativeAngleToTarget);
        double relativeTurnAngle = angleWrapRad(h - GlobalH);

        double maxPower = Math.abs(relativeXToTarget) + Math.abs(relativeYToTarget) + correctFactor * Math.abs(relativeTurnAngle);

        double movementXpower = speed * relativeXToTarget / maxPower ;
        double movementYpower = speed * relativeYToTarget / maxPower ;
        double movementTurnPower = speed * correctFactor * relativeTurnAngle / maxPower;

            FLMotor.setPower(Range.clip(movementXpower - movementYpower - movementTurnPower, -speed, speed));
            FRMotor.setPower(Range.clip(movementXpower + movementYpower + movementTurnPower, -speed, speed));
            BLMotor.setPower(Range.clip(movementXpower + movementYpower - movementTurnPower, -speed, speed));
            BRMotor.setPower(Range.clip(movementXpower - movementYpower + movementTurnPower, -speed, speed));
        sleep(sleep_time);
    }
*/
    public void makeDroppieWork(int position){
        droppie.setTargetPosition(position); //-1400
        droppie.setPower(-0.9);
        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void makeIntakieWork(int pos){
        intakie.setTargetPosition(pos);//800
        intakie.setPower(1);//0.8);
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


// vertical distance 43 inches 109.22 cm - 1092.2 mm
// horizontal distance  odometer at 26.5 inches 67.31 cm - 673.1 mm
//                      start of wheel 31 inches 78.74 - 787.4 mm
//Angle 180 degrees - pi