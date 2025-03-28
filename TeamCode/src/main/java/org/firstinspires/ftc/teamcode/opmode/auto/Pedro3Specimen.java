package org.firstinspires.ftc.teamcode.opmode.auto;

import static android.os.SystemClock.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */
@Autonomous(name = "Pedro3Specimen Auto Blue", group = "Examples")
public class Pedro3Specimen extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** This is our claw subsystem.
     * We call its methods to manipulate the servos that it has within the subsystem. */
//    public ClawSubsystem claw;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(2, 29, Math.toRadians(0));

    private final Pose preloadScoreControlPose = new Pose(2, 20, 0);
//    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose push1ControlPose = new Pose(27, 16, Math.toRadians(-90));

    /** Middle (Second) Sample from the Spike Mark */

    private final Pose push1Pose = new Pose(30, 50, Math.toRadians(-90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose obs1ControlPose = new Pose(30, 20, Math.toRadians(-90));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose obs1Pose = new Pose(33, 10, Math.toRadians(-90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose obs2ControlPose = new Pose(38, 20, Math.toRadians(-90));

    private final Pose push2Pose = new Pose(38, 49, Math.toRadians(-90));

    private final Pose push2ControlPose = new Pose(33, 49, Math.toRadians(-90));

    private final Pose obs2Pose = new Pose(38, 10, Math.toRadians(-90));

    private final Pose prePickup2ControlPose = new Pose(38, 8, Math.toRadians(-135));

    private final Pose prePickup3ControlPose = new Pose(20, 10, Math.toRadians(-90));

    private final Pose prePickupPose = new Pose(38, 5, Math.toRadians(-179));

    private final Pose pickupPose = new Pose(38, 0, Math.toRadians(-179));

    private final Pose pickupControlPose = new Pose(38, 5, Math.toRadians(-90));

    private final Pose scoreControlPose = new Pose( 10, 20, 0);

    private final Pose parkControlPose = new Pose(20, 20, 0);

    private final Pose parkPose = new Pose(50, 0, 0);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain pushSample1, pushToObs1, pushSample2, pushToObs2, prePickupSpecimen2, prePickupSpecimen3, pickupSpecimen, scoreSpecimen, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */


        scorePreload = new Path(new BezierCurve(new Point(startPose), new Point(preloadScoreControlPose), new Point(scorePose)));
        scorePreload.setConstantHeadingInterpolation(startPose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our pushSample1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushSample1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(push1ControlPose), new Point(push1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), push1ControlPose.getHeading(), push1Pose.getHeading())
                .build();

        pushToObs1 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push1Pose), new Point(obs1ControlPose), new Point(obs1Pose)))
                .setLinearHeadingInterpolation(push1Pose.getHeading(), obs1ControlPose.getHeading(), obs1Pose.getHeading())
                .build();

//        test = new Path(new BezierLine(new Point(obs1Pose), new Point(testPose)));
//        test.setConstantHeadingInterpolation(obs1Pose.getHeading());

        pushSample2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obs1Pose), new Point(push2ControlPose), new Point(push2Pose)))
                .setLinearHeadingInterpolation(obs1Pose.getHeading(), push2ControlPose.getHeading(), push2Pose.getHeading())
                .build();

        pushToObs2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(push2Pose), new Point(obs2ControlPose), new Point(obs2Pose)))
                .setLinearHeadingInterpolation(push2Pose.getHeading(), obs2ControlPose.getHeading(), obs2Pose.getHeading())
                .build();

        pickupSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(prePickupPose), new Point(pickupControlPose), new Point(pickupPose)))
                .setLinearHeadingInterpolation(prePickupPose.getHeading(), pickupControlPose.getHeading(), pickupPose.getHeading())
                .build();

        prePickupSpecimen2 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(obs2Pose), new Point(prePickup2ControlPose), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(obs2Pose.getHeading(), prePickup2ControlPose.getHeading(), prePickupPose.getHeading())
                .build();

        prePickupSpecimen3 = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(prePickup3ControlPose), new Point(prePickupPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3ControlPose.getHeading(), prePickupPose.getHeading())
                .build();

        scoreSpecimen = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickupPose), new Point(scoreControlPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), scoreControlPose.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkControlPose.getHeading(), parkPose.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(push1ControlPose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(push1ControlPose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//                .build();
//
//        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//                .build();
//
//        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
//        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
//        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}" (Though, I don't recommend this because it might not return due to holdEnd
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */
//                    claw.scoringClaw();
//                    claw.openClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    sleep(1000);
                    follower.followPath(pushSample1);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the push1ControlPose's position */
                if (follower.getPose().getX() > (push1Pose.getX() - 1) && follower.getPose().getY() > (push1Pose.getY() - 1)) {
                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushToObs1);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (obs1Pose.getX() - 1) && follower.getPose().getY() > (obs1Pose.getY() - 1)) {
                    /* Score Sample */
//                    claw.scoringClaw();
//                    claw.openClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pushSample2);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (follower.getPose().getX() > (push2Pose.getX() - 1) && follower.getPose().getY() > (push2Pose.getY() - 3)) {
                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pushToObs2);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (obs2Pose.getX() - 1) && follower.getPose().getY() > (obs2Pose.getY() - 1)) {
                    /* Score Sample */
//                    claw.scoringClaw();
//                    claw.openClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(prePickupSpecimen2, true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if (follower.getPose().getX() > (prePickupPose.getX() - 2) && follower.getPose().getY() > (prePickupPose.getY() - 2)) {
                    /* Grab Sample */
//                    claw.groundClaw();
//                    claw.closeClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    sleep(500);
                    follower.followPath(pickupSpecimen, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {
                    /* Score Sample */
//                    claw.scoringClaw();
//                    claw.openClaw();
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    sleep(1000);
                    follower.followPath(scoreSpecimen, true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Put the claw in position to get a level 1 ascent */
//                    claw.startClaw();
//                    claw.closeClaw();

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    sleep(1000);
                    follower.followPath(prePickupSpecimen3, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (follower.getPose().getX() > (prePickupPose.getX() - 1) && follower.getPose().getY() > (prePickupPose.getY() - 1)) {

                    sleep(500);
                    follower.followPath(pickupSpecimen);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.getPose().getX() > (pickupPose.getX() - 1) && follower.getPose().getY() > (pickupPose.getY() - 1)) {

                    sleep(1000);
                    follower.followPath(scoreSpecimen);
                    setPathState(11);
                }
                break;
            case 11:
                if (follower.getPose().getX() > (scorePose.getX() - 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {

                    sleep(1000);
                    follower.followPath(park);
                    setPathState(-1);
                }
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

//        claw = new ClawSubsystem(hardwareMap);

        // Set the claw to positions for init
//        claw.closeClaw();
//        claw.startClaw();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}