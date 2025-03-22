package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.notUsing.GoBildaPinpointDriver;

@TeleOp(name="robocentric", group="Linear OpMode")
public class robocentric extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FLMotor = null;
    private DcMotor BLMotor = null;
    private DcMotor FRMotor = null;
    private DcMotor BRMotor = null;

    double FRPower;
    double FLPower;
    double BRPower;
    double BLPower;
    double speedMode = 0.7;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        FLMotor = hardwareMap.get(DcMotor.class, "FL");
        BLMotor = hardwareMap.get(DcMotor.class, "BL");
        FRMotor = hardwareMap.get(DcMotor.class, "FR");
        BRMotor = hardwareMap.get(DcMotor.class, "BR");

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

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;


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


            if (gamepad1.left_trigger > 0.5) {
                speedMode = 0.4;
            } else if (gamepad1.right_trigger > 0.5) {
                speedMode = 1;
            } else {
                speedMode = 0.7;
            }

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
}
