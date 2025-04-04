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

@Autonomous(name="3_3", group="Linear OpMode")
//@Disabled

public class Three_Three extends LinearOpMode {

    private DcMotor FRMotor = null;
    private DcMotor FLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor BLMotor = null;

    private DcMotor droppie = null;
    private DcMotor intakie = null;
    private DcMotor revie = null;

    private Servo flipity = null;
    private Servo flopity = null;
    private CRServo indulgey = null;
    private Servo bobby = null;

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
        revie = hardwareMap.get(DcMotor.class, "revy");

        flipity = hardwareMap.get(Servo.class, "flipity");
        flopity = hardwareMap.get(Servo.class, "flopity");
        indulgey = hardwareMap.get(CRServo.class, "indulgey");
        bobby = hardwareMap.get(Servo.class, "bobby2");

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
        revie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        droppie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


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

//        //Pickup Sample #1
//        makeFlipityWork(0.5);
//        makeIntakieWork(-700);
//        goToPos(150, 500,Math.toRadians(35), 0.7, 30, 30, Math.toRadians(20), 1);
//        makeFlipityWork(0.837);
//        makeIndulgeyWork(-1);
//        makeIntakieWork(-2400);
//        goToPos(300, 625, Math.toRadians(35), 0.55, 20, 20, Math.toRadians(20), 2);
//        goToPosStop();
//
//        //Spit Sample #1
//        makeIndulgeyWork(0);
//        makeIntakieWork(-1800);
//        goToPos(300, 625, Math.toRadians(-60), 0.85, 30, 30, Math.toRadians(25), 2);
//        goToPosStop();
//        makeIndulgeyWork(1);
//        sleep(300);
//
//        //Pickup Sample #2
//        goToPos(300, 625, Math.toRadians(30), 0.65, 20, 20, Math.toRadians(20), 2);
//        makeIndulgeyWork(0);
//        makeIndulgeyWork(-1);
//        makeIntakieWork(-2600);
//        goToPos(500, 625, Math.toRadians(30), 0.7, 15, 15, Math.toRadians(15), 2);
//        makeIndulgeyWork(0);
//
//        //Spit Sample #2
//        makeIntakieWork(-1900);
//        goToPos(450, 625, Math.toRadians(-60), 0.85, 30, 30, Math.toRadians(20), 2);
//        goToPosStop();
//        makeIndulgeyWork(1);
//        sleep(300);
//
//        //Pickup Sample #3
//        goToPos(400, 500, Math.toRadians(30), 0.75, 20, 20, Math.toRadians(15), 2);
//        makeIndulgeyWork(0);
//        makeIntakieWork(-2600);
//        makeIndulgeyWork(-1);
//        goToPos(650, 625, Math.toRadians(30), 0.65, 15, 15, Math.toRadians(15), 2);
//        goToPosStop();
//        makeIndulgeyWork(0);
//
//        //Spit Sample #3
//        makeIntakieWork(-2400);
//        goToPos(500, 400, Math.toRadians(-55), 0.8, 30, 30, Math.toRadians(20), 2);
//        goToPosStop();
//        makeIndulgeyWork(1);
//        sleep(250);

//        ********************
//           5 Specimens!!!
//        ********************
//        makeRevieWork(1);
//        goToPos(745, 0, Math.toRadians(0), 0.8, 50, 100, Math.toRadians(10), 2);
//        makeRevieWork(-1);
//        goToPos(945, 0, Math.toRadians(0), 0.8, 190, 100, Math.toRadians(10), 1);
//        makeBobbyWork(-1);
//        goToPos(750, 0, Math.toRadians(0), 0.8, 50, 100, Math.toRadians(10), 2);
//        makeRevieWork(0);
//        makeBobbyWork(0);




        makeIntakieWork(0);
        makeFlipityWork(0.2);
        makeBobbyWork(0.15);

        //Push #1
        goToPos(600, -450, Math.toRadians(0), 0.6, 200, 200, Math.toRadians(15), 1);
        goToPos(1250, -475, Math.toRadians(0), 0.6, 200, 150, Math.toRadians(15), 2);
        goToPos(1300, -600, Math.toRadians(0), 0.7, 200, 150, Math.toRadians(15), 2);
        goToPosStop();
        goToPos(175, -625, Math.toRadians(0), 0.8, 200, 200, Math.toRadians(15), 2);

        //Push #2
        goToPos(1200, -625, Math.toRadians(0), 0.7, 200,150, Math.toRadians(15), 2);
        goToPos(1320, -800, Math.toRadians(-10), 0.6, 200, 200, Math.toRadians(15), 2);
        goToPosStop();
        goToPos(190, -800, Math.toRadians(-10), 0.8, 200, 200, Math.toRadians(15), 2);

        //Push #3
        goToPos(1300, -1050, Math.toRadians(0), 0.7, 200,150, Math.toRadians(15), 2);
        goToPos(1400, -1150, Math.toRadians(0), 0.6, 200, 200, Math.toRadians(15), 1);
        goToPosStop();
        goToPos(100, -1200, Math.toRadians(0), 0.8, 200, 200, Math.toRadians(15), 1.5);

        //Pickup #1
        goToPos(200, -450, Math.toRadians(0), 0.5, 100, 50, Math.toRadians(10), 1);
        goToPos(-200, -450, Math.toRadians(0), 0.5, 205, 50, Math.toRadians(10), 1);
        goToPosStop();
        makeBobbyWork(1);
        sleep(750);
        makeRevieWork(1);

        //Place #1
        goToPos(400, 550, Math.toRadians(0), 0.7, 100, 100, Math.toRadians(10), 2);
        goToPos(750, 540, Math.toRadians(0), 0.7, 100, 100, Math.toRadians(10), 2);
        goToPos(945, 525, Math.toRadians(0), 0.5, 190, 100, Math.toRadians(10), 1);
        sleep(500);
        makeRevieWork(-1);
        sleep(250);
        makeBobbyWork(0.15);
        sleep(500);
        goToPosStop();

        //Pickup #2
        goToPos(750, 490, Math.toRadians(0), 0.7, 50, 50, Math.toRadians(10), 2);
        makeRevieWork(-0.7);
        goToPos(-200, -400, Math.toRadians(0), 0.7, 210, 50, Math.toRadians(10), 2);
        goToPosStop();
        makeBobbyWork(1);
        sleep(750);
        makeRevieWork(0.8);

        //Place #2
        goToPos(400, 640, Math.toRadians(0), 0.7, 100, 100, Math.toRadians(10), 2);
        goToPos(750, 650, Math.toRadians(0), 0.7, 100, 100, Math.toRadians(10), 2);
        goToPos(945, 630, Math.toRadians(0), 0.5, 190, 100, Math.toRadians(10), 1);
        sleep(500);
        makeRevieWork(-1);
        sleep(250);
        makeBobbyWork(0.15);
        sleep(500);
        goToPosStop();

//        //Pickup #3
        goToPos(750, 530, Math.toRadians(0), 0.7, 50, 50, Math.toRadians(10), 2);
        makeRevieWork(-0.7);
        goToPos(-200, -400, Math.toRadians(0), 0.7, 210, 50, Math.toRadians(10), 2);
        goToPosStop();
        makeBobbyWork(1);
        sleep(750);
        makeRevieWork(0.8);

//        //Place #3
        goToPos(400, 670, Math.toRadians(0), 0.7, 100, 100, Math.toRadians(10), 2);
        goToPos(750, 680, Math.toRadians(0), 0.7, 100, 100, Math.toRadians(10), 2);
        goToPos(945, 700, Math.toRadians(0), 0.5, 190, 100, Math.toRadians(10), 1);
        sleep(500);
        makeRevieWork(-1);
        sleep(250);
        makeBobbyWork(0.15);
        sleep(500);
        goToPosStop();

//
//        //Pickup #4
//        goToPos(750, 800, Math.toRadians(0), 0.8, 50, 50, Math.toRadians(10), 2);
//        makeBobbyWork(0);
//        makeRevieWork(0);
//        goToPos(-200, -300, Math.toRadians(0), 0.8, 205, 100, Math.toRadians(10), 2);
//        goToPosStop();
//        makeBobbyWork(1);
//        sleep(500);
//        makeBobbyWork(0.5);
//        makeRevieWork(0.8);
//
//        //Place #4
//        goToPos(400, 480, Math.toRadians(0), 0.8, 50, 100, Math.toRadians(10), 2);
//        goToPos(750, 680, Math.toRadians(0), 0.8, 50, 100, Math.toRadians(10), 2);
//        makeRevieWork(-1);
//        goToPos(945, 680, Math.toRadians(0), 0.8, 190, 100, Math.toRadians(10), 1);
//        goToPosStop();
//        makeBobbyWork(-1);
//
//        //Pickup #5
//        goToPos(750, 800, Math.toRadians(0), 0.8, 50, 50, Math.toRadians(10), 2);
//        makeBobbyWork(0);
//        makeRevieWork(0);
//        goToPos(-200, -300, Math.toRadians(0), 0.8, 205, 100, Math.toRadians(10), 2);
//        goToPosStop();
//        makeBobbyWork(1);
//        sleep(500);
//        makeBobbyWork(0.5);
//        makeRevieWork(0.8);
//
//        //Place #5
//        goToPos(400, 560, Math.toRadians(0), 0.8, 50, 100, Math.toRadians(10), 2);
//        goToPos(750, 740, Math.toRadians(0), 0.8, 50, 100, Math.toRadians(10), 2);
//        makeRevieWork(-1);
//        goToPos(945, 740, Math.toRadians(0), 0.8, 190, 50, Math.toRadians(10), 1);
//        goToPosStop();
//        makeBobbyWork(-1);
//
        //Park
        goToPos(500, 650, Math.toRadians(0), 1, 200, 200, Math.toRadians(10), 2);
        makeRevieWork(0);
        goToPos(100, -300, Math.toRadians(0), 1, 250, 250, Math.toRadians(10), 2);
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
    double Kp = 0.475;
    double Ki = 0.0;
    double Kd = 0.3;
    double Kf = 0.2;
    private double lastError = 0;

    double integralSumX = 0;
    double KpX=0.04;
    double KiX=0.0000;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdX=0.005;// KXf/KYf ratio is affected by the robot weight balance
    double feedfowardX = 0;
    private double lastErrorX = 0;

    double integralSumY = 0;
    double KpY=0.04;
    double KiY=0.000;   //Kxp/KYp ratio is affected by the robot weight balance
    double KdY=0.005;// KXf/KYf ratio is affected by the robot weight balance
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
    public void makeRevieWork(double power){
        revie.setPower(power);
    }

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

    public void makeBobbyWork(double pos){
        bobby.setPosition(pos);//-0.6
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