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

package org.firstinspires.ftc.teamcode;

import static java.lang.Math.round;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="REAL_TELEOP", group="Linear OpMode")
public class REAL_TELEOP extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;

    private DcMotor intakie = null;
    private DcMotor droppie = null;
    private DcMotor hangie = null;
    private DcMotor mike = null;

    private Servo flipity = null;
    private Servo flopity = null;
    private Servo logan = null;
    private CRServo indulgey = null;
    private CRServo bobby = null;

    double FRPower;
    double FLPower;
    double BRPower;
    double BLPower;
    double speedMode = 0.6;

    double stopBuffer = 0;

//    public void intakieLimit(double power){
//        //Get current position
//        int motor_pos = intakie.getCurrentPosition();
//        int extended_pos = -1500;
//        int retracted_pos = 0;
//        if ((motor_pos<= extended_pos) && (motor_pos>= retracted_pos))
//        {
//            intakie.setPower(power);
//        }
//        else if (motor_pos> extended_pos){
//            //set to max range
//            intakie.setPower(Range.clip(power, 0, -1));
//        }
//        else{
//            //set to max range
//            intakie.setPower(Range.clip(power, 1, 0));
//        }
//
//
//    }


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");

        intakie = hardwareMap.get(DcMotor.class, "intakie");
        droppie = hardwareMap.get(DcMotor.class, "droppie");
        hangie = hardwareMap.get(DcMotor.class, "hangie");
        mike = hardwareMap.get(DcMotor.class, "mike");

        flipity = hardwareMap.get(Servo.class, "flipity");
        flopity = hardwareMap.get(Servo.class, "flopity");
        indulgey = hardwareMap.get(CRServo.class, "indulgey");
        bobby = hardwareMap.get(CRServo.class, "bobby");
        logan = hardwareMap.get(Servo.class, "logan");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        FLMotor.setDirection(DcMotor.Direction.REVERSE);
        BLMotor.setDirection(DcMotor.Direction.REVERSE);
        FRMotor.setDirection(DcMotor.Direction.FORWARD);
        BRMotor.setDirection(DcMotor.Direction.FORWARD);

        FLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        droppie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mike.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        droppie.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int droppiePos = 0;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

//        droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        droppie.setTargetPosition(startPos);
//        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double extendArm = gamepad2.right_stick_y/1.5;
            double extendLeg = gamepad2.left_stick_y;

//            if (gamepad2.y) {
//                droppie.setTargetPosition(specimenRack);
//                droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                droppie.setPower(0.5);
//            }else if(gamepad2.b) {
//                droppie.setTargetPosition(wall);
//                droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                droppie.setPower(0.5);
//            }else if(gamepad2.a) {
//                droppie.setTargetPosition(topBasket);
//                droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                droppie.setPower(0.5);
//
////            }else if(gamepad2.left_stick_y) {
////                extendLeg = gamepad2.left_stick_y;
//            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = speedMode * (axial + lateral + yaw);
            double rightFrontPower = speedMode * (axial - lateral - yaw);
            double leftBackPower = speedMode * (axial - lateral + yaw);
            double rightBackPower = speedMode * (axial + lateral - yaw);

            double[] powers = {FLPower, BLPower, FRPower, BRPower};

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }


            if (gamepad1.right_bumper) {
                speedMode = 0.4;
            } else if (gamepad1.right_trigger > 0.5) {
                speedMode = 1;
            } else {
                speedMode = 0.7;
            }

//                //Added by Leo for Game d pad -- Begin
//                if (gamepad1.dpad_down) {
//                    FRMotor.setPower(speedMode);
//                    FLMotor.setPower(speedMode);
//                    BLMotor.setPower(speedMode);
//                    BRMotor.setPower(speedMode);
//                } else if (gamepad1.dpad_up) {
//                    FLMotor.setPower(-speedMode);
//                    FRMotor.setPower(-speedMode);
//                    BLMotor.setPower(-speedMode);
//                    BRMotor.setPower(-speedMode);
//                } else if (gamepad1.dpad_left) {
//                    FLMotor.setPower(speedMode);
//                    BRMotor.setPower(speedMode);
//                    FRMotor.setPower(-speedMode);
//                    BLMotor.setPower(-speedMode);
//                } else if (gamepad1.dpad_right) {
//                    FLMotor.setPower(-speedMode);
//                    BRMotor.setPower(-speedMode);
//                    FRMotor.setPower(speedMode);
//                    BLMotor.setPower(speedMode);
//                } else {
//                    //Added by Leo for Game d pad -- End
//                    boolean needToScale = false;
//                    for (double power : powers) {
//                        if (Math.abs(power) > 1) {
//                            needToScale = true;
//                            break;
//                        }
//
//                    }
//                    if (needToScale) {
//                        double greatest = 0;
//                        for (double power : powers) {
//                            if (Math.abs(power) > greatest) {
//                                greatest = Math.abs(power);
//                            }
//                        }
//                        FLPower /= greatest;
//                        BLPower /= greatest;
//                        FRPower /= greatest;
//                        BRPower /= greatest;
//
//
//                    }


//            {
//                if (gamepad2.y) {
//                    droppie.setTargetPosition(specimenRack);
//                    droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    droppie.setPower(0.5);
//                }
//                if (gamepad2.b) {
//                    droppie.setTargetPosition(wall);
//                    droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    droppie.setPower(0.5);
//                }
//                if (gamepad2.a) {
//                    droppie.setTargetPosition(topBasket);
//                    droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    droppie.setPower(0.5);
//
//                }
//
//            }

            //Added by Aish

            intakie.setPower(extendArm);
            droppie.setPower(extendLeg);
//            droppiePos += round(gamepad2.left_stick_y * 6);
//            droppie.setTargetPosition(droppiePos);
//            droppie.setPower(-1);
//            droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//            if (gamepad2.right_trigger > 0.3) {
//                indulgey.setPower(gamepad2.right_trigger);
//            }else if (gamepad2.left_trigger > 0.3) {
//                indulgey.setPower(gamepad2.left_trigger);
//            }else{
//                indulgey.setPower(0);
//            }

            if (gamepad2.dpad_up) {
                flopity.setPosition(0.1);
            } else if (gamepad2.dpad_down) {
                flopity.setPosition(0.6);
            }


            if (gamepad2.dpad_left) {
                flipity.setPosition(0.95);
            } else if (gamepad2.dpad_right) {
                flipity.setPosition(0.1);
            }


            if (gamepad2.left_bumper) {
                bobby.setPower(0.7);
            } else if (gamepad2.right_bumper) {
                bobby.setPower(-0.7);
            } else {
                bobby.setPower(0);
            }

            if (gamepad2.left_trigger > 0.35) {
                indulgey.setPower(-1);
            } else if (gamepad2.right_trigger > 0.3) {
                indulgey.setPower(1);
            } else {
                indulgey.setPower(0);
            }

            if (gamepad1.x) {
                logan.setPosition(.7);
            } else if (gamepad1.a) {
                logan.setPosition(-1);
            }


            if (gamepad1.y) {
                hangie.setPower(1);
            } else if (gamepad1.b) {
                hangie.setPower(-1);
            } else {
                hangie.setPower(0);
            }

            if (gamepad1.left_trigger > 0.3) {
                mike.setPower(1);
            } else if (gamepad1.left_bumper) {
                mike.setPower(-1);
            } else {
                mike.setPower(0);
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            FLMotor.setPower(leftFrontPower);
            FRMotor.setPower(rightFrontPower);
            BLMotor.setPower(leftBackPower);
            BRMotor.setPower(rightBackPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();


        }

    }

//    public void preset() throws InterruptedException {
//
//        int startPos = 0;
//        int specimenRack = 700;
//        int wall = 300;
//        int topBasket = 1500;
//
//        droppie.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        droppie.setTargetPosition(startPos);
//        droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            if (gamepad2.y) {
//                droppie.setTargetPosition(specimenRack);
//                droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                droppie.setPower(0.5);
//            }
//            if (gamepad2.b) {
//                droppie.setTargetPosition(wall);
//                droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                droppie.setPower(0.5);
//            }
//            if (gamepad2.a) {
//                droppie.setTargetPosition(topBasket);
//                droppie.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                droppie.setPower(0.5);
//
//            }
//
//        }
}