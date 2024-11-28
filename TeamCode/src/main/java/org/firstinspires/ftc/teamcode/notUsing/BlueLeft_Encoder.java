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
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="BlueLeft_Encoder", group="Linear OpMode")
public class BlueLeft_Encoder extends LinearOpMode{

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

    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    // strafing movement
    //
    Double meccyBias = 0.6;

    Double conversion = cpi * bias;
    Boolean exit = false;

    public void runOpMode(){

        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        droppie = hardwareMap.get(DcMotor.class, "droppie");
        intakie = hardwareMap.get(DcMotor.class, "intakie");
        flipity = hardwareMap.get(Servo.class, "flipity");
        flopity = hardwareMap.get(Servo.class, "flopity");
        bobby = hardwareMap.get(CRServo.class, "bobby");
        indulgey = hardwareMap.get(CRServo.class, "indulgey");



        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);



        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        droppie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        droppie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        FLMotor.setPower(0);
        FRMotor.setPower(0);
        BLMotor.setPower(0);
        BRMotor.setPower(0);



        waitForStart();
        //
        moveToPosition(18,.2);
        sleep(1000);
        turnWithEncoder(45);
        sleep(2000);
        makeDroppieWork(-4400);
        sleep(2000);
        makeFlopityWork(-1);
        sleep(1000);
        moveToPosition(-2,.2);
        sleep(1000);
       makeDroppieWork(0);
        sleep(1000);
        makeFlopityWork(1);
        sleep(1000);





//        makeIntakieWork(800);
//
//        makeBobbyWork(-0.6);
//
//        makeFlipityWork(0.8387);
//
//        makeFlopityWork(0.8387);
//
//        makeIndulgeyWork(-0.6);

    }

    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition()  + move);
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() + move);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + move);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() + move);
        //
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        FLMotor.setPower(speed);
        BLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BRMotor.setPower(speed);
        //
        while (FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){
            if (exit){
                FRMotor.setPower(0);
                FLMotor.setPower(0);
                BRMotor.setPower(0);
                BLMotor.setPower(0);
                return;
            }
        }
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        return;
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        BLMotor.setTargetPosition(BLMotor.getCurrentPosition() + move);
        FLMotor.setTargetPosition(FLMotor.getCurrentPosition() - move);
        BRMotor.setTargetPosition(BRMotor.getCurrentPosition() + move);
        FRMotor.setTargetPosition(FRMotor.getCurrentPosition() - move);
        //
        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        FLMotor.setPower(speed);
        BLMotor.setPower(speed);
        FRMotor.setPower(speed);
        BRMotor.setPower(speed);
        //
        while (FLMotor.isBusy() && FRMotor.isBusy() && BLMotor.isBusy() && BRMotor.isBusy()){}
        FRMotor.setPower(0);
        FLMotor.setPower(0);
        BRMotor.setPower(0);
        BLMotor.setPower(0);
        return;
    }
    //
    /*

    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input){

        double angleWrapped = angleWrapRad(input);
//        double angleWrapped = calculateAngle(input);
        FLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        FLMotor.setPower(angleWrapped);
        BLMotor.setPower(angleWrapped);
        FRMotor.setPower(-angleWrapped);
        BRMotor.setPower(-angleWrapped);

        //calculateAngle(FLMotor.getCurrentPosition())
    }

    public double calculateAngle(double target_angle){
    //        int revolutions = encoderTicks/cpr;
    //        //double angle = revolutions*360;
    //        double angleNormalized = revolutions%360;
    //
    //        return angleNormalized;

        double targetTicks = (target_angle/360)*cpr;
        return targetTicks;

    }

    public void makeDroppieWork(int position){
        droppie.setTargetPosition(position); //-1400
        droppie.setPower(-0.6);
        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void makeIntakieWork(int pos){
        intakie.setTargetPosition(pos);//800
        intakie.setPower(0.8);//0.8);
        intakie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void makeBobbyWork(double power){
        bobby.setPower(power);//-0.6
        sleep(1500);
        bobby.setPower(0);
    }

    public void makeFlipityWork(double pos){
        flipity.setPosition(pos);//0.8387);
    }

    public void makeFlopityWork(double pos){
        flopity.setPosition(pos);//0.8387);
    }

    public void makeIndulgeyWork(double power){
        indulgey.setPower(power);//-0.6
        sleep(1500);
        indulgey.setPower(0);
    }

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

}
