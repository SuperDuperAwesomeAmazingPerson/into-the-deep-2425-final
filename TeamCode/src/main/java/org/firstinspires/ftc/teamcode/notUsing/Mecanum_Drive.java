package org.firstinspires.ftc.teamcode.notUsing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Mecanum_Drive", group="Iterative OpMode")

public class Mecanum_Drive extends OpMode
{
//put variables here
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor backleft = null;
    private DcMotor backright = null;

//    double FRPower;
//    double FLPower;
//    double BRPower;
//    double BLPower;
//    double MotorPower = 0.75;
    double stopBuffer = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Init in progress");
//put initialization code here
        frontleft = hardwareMap.get(DcMotor.class,"FL");
        frontright = hardwareMap.get(DcMotor.class, "FR");
        backleft   = hardwareMap.get(DcMotor.class, "BL");
        backright  = hardwareMap.get(DcMotor.class, "BR");

        frontright.setDirection(DcMotor.Direction.REVERSE);
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backright.setDirection(DcMotor.Direction.FORWARD);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
//put main loop code here
        double forward = Math.pow(gamepad1.left_stick_y, -3);
        double right = Math.pow(gamepad1.left_stick_x, -3);
        double turn = Math.pow(gamepad1.right_stick_x, 3);

        if (gamepad1.dpad_right){
            frontleft.setPower(0.6);
            frontright.setPower(0.6);
            backleft.setPower(0.6);
            backright.setPower(0.6);
        }
        else if ( gamepad1.dpad_up) {
            frontleft.setPower(-0.6);
            frontright.setPower(-0.6);
            backleft.setPower(-0.6);
            backright.setPower(-0.6);
        }
        else if (gamepad1.dpad_down) {
            frontleft.setPower(0.6);
            backright.setPower(0.6);
            frontright.setPower(-0.6);
            backleft.setPower(-0.6);
        }
        else if (gamepad1.dpad_left) {
            frontleft.setPower(-0.6);
            backright.setPower(-0.6);
            frontright.setPower(0.6);
            backleft.setPower(0.6);
        }

        double FLPower = forward + right + turn;
        double BLPower = forward - right + turn;
        double FRPower = forward - right - turn;
        double BRPower = forward + right - turn;
        double[] powers = {FLPower, BLPower, FRPower, BRPower};

        boolean needToScale = false;
        for (double power : powers) {
            if (Math.abs(power) > 1) {
                needToScale = true;
                break;
            }
        }
        if (needToScale) {
            double greatest = 0;
            for (double power : powers) {
                if (Math.abs(power) > greatest) {
                    greatest = Math.abs(power);
                }
            }
            FLPower/= greatest;
            BLPower/= greatest;
            FRPower/= greatest;
            BRPower/= greatest;
        }

        boolean Stop = true;
        for (double power : powers) {
            if (Math.abs(power) > stopBuffer) {
                Stop = false;
                break;
            }
        }
        if (Stop) {
            FLPower= 0;
            BLPower= 0;
            FRPower= 0;
            BRPower= 0;
        }

        frontleft.setPower(FLPower);
        backleft.setPower(BLPower);
        frontright.setPower(FRPower);
        backright.setPower(BRPower);

    }
}


