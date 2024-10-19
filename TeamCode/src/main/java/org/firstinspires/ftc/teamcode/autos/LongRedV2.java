package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "WebShortRed")
public class LongRedV2 extends LinearOpMode {

    OpenCvWebcam webcam;
    private DcMotor frontleft = null;
    private DcMotor frontright = null;
    private DcMotor backleft = null;
    private DcMotor backright = null;

    private DcMotor liftLeft = null;

    private CRServo servo1 = null;
    private CRServo servo2 = null;
    private CRServo servo3 = null;

    private DcMotor intake = null;

    private CRServo intakeservo = null;

    private DcMotor Suspension = null;

    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.125;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    // strafing movement
    //
    Double meccyBias = 0.6;
    Double conversion = cpi * bias;
    Boolean exit = false;
    public static int position = -1;
    public static boolean realBot = false;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.clearAll();

        backleft = hardwareMap.get(DcMotor.class, "Bl");
        frontleft = hardwareMap.get(DcMotor.class, "Fl");
        backright = hardwareMap.get(DcMotor.class, "Br");
        frontright = hardwareMap.get(DcMotor.class, "Fr");

//        liftLeft = hardwareMap.get(DcMotor.class, "Ll");
//
//        servo1 = hardwareMap.get(CRServo.class, "S1");
//        servo2 = hardwareMap.get(CRServo.class, "S2");
//        servo3 = hardwareMap.get(CRServo.class, "S3");
//
//        intake = hardwareMap.get(DcMotor.class, "I");
//
//        intakeservo = hardwareMap.get(CRServo.class, "Iservo");
//
//        Suspension = hardwareMap.get(DcMotor.class, "Sus");


        backleft.setDirection(DcMotor.Direction.FORWARD);
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        frontright.setDirection(DcMotor.Direction.REVERSE);


        liftLeft.setDirection(DcMotor.Direction.FORWARD);


        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        frontleft.setPower(0);
        frontright.setPower(0);
        backleft.setPower(0);
        backright.setPower(0);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        webcam.setPipeline(new SamplePipeline());
        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //This will be called if the camera could not be opened
            }
        });

        waitForStart();
         {

            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if (gamepad1.a) {
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }
            for (int i = 0; i < 10; i++) {
                sleep(100);
            }

            if (position == 0) {
                webcam.stopStreaming();
                strafeToPosition(-48, 0.4);
                moveToPosition(-15, 0.5);
                strafeToPosition(4, 0.5);
                moveToPosition(58, 0.5);
                strafeToPosition(-16,.4);
                Lifty(17, 0.4);
                sleep(3000);
                disy(-0.4);
                sleep(3000);
                disy(0);
                Lifty(-10, 0.3);
                sleep(3000);
                strafeToPosition(30, 0.4);
                sleep(15000);
            } else if (position == 1) {
                webcam.stopStreaming();
                strafeToPosition(-55, 0.5);
                strafeToPosition(12, 0.5);
                moveToPosition(42, 0.5);
                Lifty(17, 0.4);
                sleep(3000);
                disy(-0.3);
                sleep(3000);
                disy(0);
                Lifty(-10, 0.3);
                sleep(3000);
                strafeToPosition(25, 0.4);
//                sleep(15000);
            } else {
                webcam.stopStreaming();
                moveToPosition(10, 0.4);
                strafeToPosition(-53, 0.4);
                strafeToPosition(15, 0.4);
                moveToPosition(35, 0.4);
                Lifty(17, 0.3);
                sleep(3000);
                disy(-0.4);
                sleep(3000);
                disy(0);
                Lifty(-10, 0.3);
                sleep(3000);
                strafeToPosition(20, 0.4);
                sleep(15000);
            }
        }
    }

    class SamplePipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {
            Point LeftBox_topLeftCorner = new Point(0, 100);
            int LeftBoxWidth = 60;
            int LeftBoxHeight = 60;

            Point MiddleBox_topLeftCorner = new Point(80, 80);
            int MidBoxWidth = 60;
            int MidBoxHeight = 60;

            Point RightBox_topRightCorner = new Point(320, 80);
            int RightBoxWidth = 60;
            int RightBoxHeight = 60;

            Point LeftTop = LeftBox_topLeftCorner; // I"m too laxy to change the names below
            Point MiddleTop = MiddleBox_topLeftCorner;
            Point RightTop = RightBox_topRightCorner;

            Point LeftBottom = new Point(LeftBox_topLeftCorner.x + LeftBoxWidth,
                                      LeftBox_topLeftCorner.y + LeftBoxHeight); // bottomright corner
            Point MiddleBottom = new Point(MiddleBox_topLeftCorner.x + MidBoxWidth,
                                        MiddleBox_topLeftCorner.y + MidBoxHeight); // bottomright corner
            Point RightBottom = new Point(RightBox_topRightCorner.x - RightBoxWidth,
                                        RightBox_topRightCorner.y + RightBoxHeight); // bottomleft corner

//            Point LeftTop = new Point(0, 100); // topleft corner
//            Point LeftBottom = new Point(60, 160); // bottomright corner
//            Point MiddleTop = new Point(170, 80); // topleft ccorner
//            Point MiddleBottom = new Point(230, 140); // bottomright corner
//            Point RightTop = new Point(320, 100); // topright corner (???????? why though)
//            Point RightBottom = new Point(260, 160); // bottomleft corner
            int red = 0;
            int green = 0;
            int blue = 0;
            int total = 0;
            int totalRedLeft = 0;
            int totalRedMiddle = 0;
            int totalRedRight = 0;

            for (int x = (int) LeftTop.x; x < LeftBottom.x; x++) {
                for (int y = (int) LeftTop.y; y < LeftBottom.y; y++) {
                    red += input.get(y, x)[0];
                    green += input.get(y, x)[1];
                    blue += input.get(y, x)[2];
                    total++;
                }
            }

            blue /= total;
            green /= total;
            totalRedLeft = (red / total) - blue - green;
            red = 0;
            total = 0;
            blue = 0;
            green = 0;

            //Middle
            for (int x = (int) MiddleTop.x; x < MiddleBottom.x; x++) {
                for (int y = (int) MiddleTop.y; y < MiddleBottom.y; y++) {
                    red += input.get(y, x)[0];
                    green += input.get(y, x)[1];
                    blue += input.get(y, x)[2];
                    total++;
                }
            }

            blue /= total;
            green /= total;
            totalRedMiddle = (red / total) - blue - green;
            red = 0;
            total = 0;
            blue = 0;
            green = 0;

            //Right
            for (int x = (int) RightBottom.x; x < RightTop.x; x++) {
                for (int y = (int) RightTop.y; y < RightBottom.y; y++) {
                    red += input.get(y, x)[0];
                    green += input.get(y, x)[1];
                    blue += input.get(y, x)[2];
                    total++;
                }
            }

            blue /= total;
            green /= total;
            totalRedRight = (red / total) - blue - green;

           // if (totalBlueLeft > totalBlueMiddle && totalBlueLeft > totalBlueRight) {
//                Imgproc.rectangle(
//                        input,
//                        LeftTop,
//                        LeftBottom,
//                        new Scalar(0, 0, 255), 4);
//          //  }
//
//           // if (totalBlueMiddle > totalBlueLeft && totalBlueMiddle > totalBlueRight) {
//                Imgproc.rectangle(
//                        input,
//                        MiddleTop,
//                        MiddleBottom,
//                        new Scalar(255, 0, 255), 4);
//           // }
//
////            if (totalBlueRight > totalBlueLeft && totalBlueRight > totalBlueMiddle) {
//                Imgproc.rectangle(
//                        input,
//                        RightTop,
//                        RightBottom,
//                        new Scalar(255, 0, 0), 4);
            //}
            Imgproc.putText(input, " rightRed: " + totalRedRight, new Point(0, 25), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar (0, 255, 0));
            Imgproc.putText(input, " rightMid: " + totalRedMiddle, new Point(0, 50), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar (0, 255, 0));
            Imgproc.putText(input, " rightLeft: " + totalRedLeft, new Point(0, 75), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar (0, 255, 0));

            if(totalRedRight > -60) {
                Imgproc.rectangle(input, RightTop, RightBottom, new Scalar(0, 0, 255), 4);
                position = 2;
            }
            else if(totalRedMiddle > -60) {
                Imgproc.rectangle(input, MiddleTop, MiddleBottom, new Scalar(255, 0, 255), 4);
                position = 1 ;
            }
            else {
                Imgproc.rectangle(input, LeftTop, LeftBottom, new Scalar(255, 0, 0), 4);
                position = 0 ;
            }

            return input;
        }

        @Override
        public void onViewportTapped() {

            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                webcam.pauseViewport();
            } else {
                webcam.resumeViewport();
            }
        }
    }

    public void strafeDrive(float distance, double speed) {

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (realBot == true) {
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + (int) (43.47343 * -distance));
            frontright.setTargetPosition(frontright.getCurrentPosition() + (int) (43.47343 * distance));
            backleft.setTargetPosition(backleft.getCurrentPosition() + (int) (43.47343 * distance));
            backright.setTargetPosition(backright.getCurrentPosition() + (int) (43.47343 * -distance));
        } else {
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + (int) (48.30381 * -distance)); //39.12609
            frontright.setTargetPosition(frontright.getCurrentPosition() + (int) (48.30381 * distance));
            backleft.setTargetPosition(backleft.getCurrentPosition() + (int) (48.30381 * distance));
            backright.setTargetPosition(backright.getCurrentPosition() + (int) (48.30381 * -distance));
        }

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);
    }

    public void straightDrive(float distance, double speed) {

        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (realBot == true) {
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + (int) (43.47343 * -distance));
            frontright.setTargetPosition(frontright.getCurrentPosition() + (int) (43.47343 * -distance));
            backleft.setTargetPosition(backleft.getCurrentPosition() + (int) (43.47343 * -distance));
            backright.setTargetPosition(backright.getCurrentPosition() + (int) (43.47343 * -distance));
        } else {
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + (int) (48.30381 * -distance));
            frontright.setTargetPosition(frontright.getCurrentPosition() + (int) (48.30381 * -distance));
            backleft.setTargetPosition(backleft.getCurrentPosition() + (int) (48.30381 * -distance));
            backright.setTargetPosition(backright.getCurrentPosition() + (int) (48.30381 * -distance));
        }

        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setPower(speed);
        frontright.setPower(speed);
        backleft.setPower(speed);
        backright.setPower(speed);
    }

    public void moveToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * conversion));
        //
        double a = (backleft.getCurrentPosition() + move);
        double b = (frontleft.getCurrentPosition() + move);
        double c = (backright.getCurrentPosition() + move);
        double d = (frontright.getCurrentPosition() + move);

        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            if (exit) {
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() - move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    public void Lifty(double inches, double speed) {
        int move = (int) (Math.round(inches * cpi * meccyBias));
        liftLeft.setTargetPosition(liftLeft.getCurrentPosition() + move);
        liftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftLeft.setPower(speed);
        while (liftLeft.isBusy()) {
            liftLeft.setPower(speed);
        }


        //


    }

    //
    /*

    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }


    public void disy ( double speed){
        servo1.setPower(speed);
        servo2.setPower(speed);
        servo3.setPower(speed);
    }
}

