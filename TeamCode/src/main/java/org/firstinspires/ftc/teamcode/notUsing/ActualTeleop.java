package org.firstinspires.ftc.teamcode.notUsing;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="ActualTeleop", group="Iterative OpMode")
public class ActualTeleop extends OpMode {
    //put variables here
    private DcMotor FRMotor = null;
    private DcMotor FLMotor = null;
    private DcMotor BRMotor = null;
    private DcMotor BLMotor = null;

    private DcMotor intakie;  // Motor for the extending/retracting mechanism

    double oldTime = 0;


    double FRPower;
    double FLPower;
    double BRPower;
    double BLPower;
    double speedMode = 0.7;


    double stopBuffer = 0;


    @Override
    public void init() {
        telemetry.addData("Status", "Init in progress");
//put initialization code here
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");

        intakie = hardwareMap.get(DcMotor.class, "intakie");

// Set the motor direction and zero power behavior


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        FRMotor.setDirection(DcMotor.Direction.REVERSE);
        FLMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.FORWARD);

//        intakie.setDirection(DcMotor.Direction.FORWARD);


        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        intakie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Initialized");

//        // Initialize the hardware variables. Note that the strings used here must correspond
//        // to the names assigned during the robot configuration step on the DS or RC devices.
////        /*
////        Set the odometry pod positions relative to the point that the odometry computer tracks around.
////        The X pod offset refers to how far sideways from the tracking point the
////        X (forward) odometry pod is. Left of the center is a positive number,
////        right of center is a negative number. the Y pod offset refers to how far forwards from
////        the tracking point the Y (strafe) odometry pod is. forward of center is a positive number,
////        backwards is a negative number.
////         */
//        //odo.setOffsets(-84.0, -224.0); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setOffsets(-153.71, -215.019);
////        /*
////        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
////        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
////        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
////        number of ticks per mm of your odometry pod.
////         */
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderResolution(13.26291192);
////
////
////        /*
////        Set the direction that each of the two odometry pods count. The X (forward) pod should
////        increase when you move the robot forward. And the Y (strafe) pod should increase when
////        you move the robot to the left.
////         */
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
////
////
////        /*
////        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
////        The IMU will automatically calibrate when first powered on, but recalibrating before running
////        the robot is a good idea to ensure that the calibration is "good".
////        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
////        This is recommended before you run your autonomous, as a bad initial calibration can cause
////        an incorrect starting value for x, y, and heading.
////         */
//        odo.recalibrateIMU();
//        odo.resetPosAndIMU();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("X offset", odo.getXOffset());
//        telemetry.addData("Y offset", odo.getYOffset());
//        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
//        telemetry.addData("Device Scalar", odo.getYawScalar());
//        telemetry.update();
    }

    @Override
    public void loop() {
//put main loop code here
        double forward = Math.pow(gamepad1.left_stick_y, 3);
        double right = Math.pow(gamepad1.left_stick_x, 3);
        double turn = Math.pow(gamepad1.right_stick_x, 3);
// Get the left stick Y-axis input from gamepad1 (values range from -1 to 1)
//        double extendArm = gamepad2.left_stick_y;


        double FLPower = speedMode * forward - right - turn;
        double BLPower = speedMode * forward + right - turn;
        double FRPower = speedMode * forward + right + turn;
        double BRPower = speedMode * forward - right + turn;
        double[] powers = {FLPower, BLPower, FRPower, BRPower};

//    /*
//        Request an update from the Pinpoint odometry computer. This checks almost all outputs
//        from the device in a single I2C read.
//         */
//        odo.update();
//
//        /*
//        Optionally, you can update only the heading of the device. This takes less time to read, but will not
//        pull any other data. Only the heading (which you can pull with getHeading() or in getPosition().
//         */
//        odo.update(GoBildaPinpointDriver.readData.ONLY_UPDATE_HEADING);
//
//
        if (gamepad1.a) {
//        odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
//    }
//
            if (gamepad1.b) {
//        odo.recalibrateIMU(); //recalibrates the IMU without resetting position
//    }

        /*
        This code prints the loop frequency of the REV Control Hub. This frequency is effected
        by I²C reads/writes. So it's good to keep an eye on. This code calculates the amount
        of time each cycle takes and finds the frequency (number of updates per second) from
        that cycle time.
         */
                double newTime = getRuntime();
                double loopTime = newTime - oldTime;
                double frequency = 1 / loopTime;
                oldTime = newTime;

                //Move forward
//    intakie.setPower(extendArm);
                // Telemetry to monitor arm motor power
//                telemetry.addData("Arm Motor Power Extend", extendArm);
//                telemetry.update();

//        /*
//        gets the current Position (x & y in mm, and heading in degrees) of the robot, and prints it.
//         */
//                Pose2D pos = odo.getPosition();
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
//                telemetry.addData("Position", data);
//
//        /*
//        gets the current Velocity (x & y in mm/sec and heading in degrees/sec) and prints it.
//         */
//    Pose2D vel = odo.getVelocity();
//    String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
//    telemetry.addData("Velocity", velocity);
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
//                telemetry.addData("Status", odo.getDeviceStatus());

//                telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint

                telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
                telemetry.update();

                if (gamepad1.left_trigger > 0.5) {
                    speedMode = 0.4;
                } else if (gamepad1.right_trigger > 0.5) {
                    speedMode = 1;
                } else {
                    speedMode = .6;
                }

                //Added by Leo for Game d pad -- Begin
                if (gamepad1.dpad_down) {
                    FRMotor.setPower(speedMode);
                    FLMotor.setPower(speedMode);
                    BLMotor.setPower(speedMode);
                    BRMotor.setPower(speedMode);
                } else if (gamepad1.dpad_up) {
                    FLMotor.setPower(-speedMode);
                    FRMotor.setPower(-speedMode);
                    BLMotor.setPower(-speedMode);
                    BRMotor.setPower(-speedMode);
                } else if (gamepad1.dpad_left) {
                    FLMotor.setPower(speedMode);
                    BRMotor.setPower(speedMode);
                    FRMotor.setPower(-speedMode);
                    BLMotor.setPower(-speedMode);
                } else if (gamepad1.dpad_right) {
                    FLMotor.setPower(-speedMode);
                    BRMotor.setPower(-speedMode);
                    FRMotor.setPower(speedMode);
                    BLMotor.setPower(speedMode);
                } else {
                    //Added by Leo for Game d pad -- End
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
                        FLPower /= greatest;
                        BLPower /= greatest;
                        FRPower /= greatest;
                        BRPower /= greatest;
                    }


                    boolean Stop = true;
                    for (double power : powers) {
                        if (Math.abs(power) > stopBuffer) {
                            Stop = false;
                            break;
                        }
                    }
                    if (Stop) {
                        FLPower = 0;
                        BLPower = 0;
                        FRPower = 0;
                        BRPower = 0;
                    }


                    FLMotor.setPower(FLPower * (speedMode + .2));
                    BLMotor.setPower(BLPower * (speedMode + .2));
                    FRMotor.setPower(FRPower * (speedMode + .2));
                    BRMotor.setPower(BRPower * (speedMode + .2));


                }

            }
        }
    }
}