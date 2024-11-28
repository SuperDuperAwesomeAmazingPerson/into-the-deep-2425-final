package org.firstinspires.ftc.teamcode.TestAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import static java.lang.Thread.sleep;

@Autonomous(name="stolenOdo", group="Robot")

public class IntoDeepAuto extends LinearOpMode {

    public Chassis driveChassis;


    @Override
    public void runOpMode() throws InterruptedException {
        driveChassis = new Chassis();
        driveChassis.init(hardwareMap, telemetry);


        waitForStart();

        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 0), telemetry);
        driveChassis.driveToPosition(new Pose2D(DistanceUnit.INCH, 72, 0, AngleUnit.DEGREES, -45), telemetry);



    }

}