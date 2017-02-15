package org.firstinspires.ftc.teamcode.testfunctionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.iowaprograms.IowaAutoBase;

/*
 * Created by Owner on 2/14/2017.
 */
@Autonomous(name = "Consistency Test Runner", group = "Test")
public class TestConsistency extends IowaAutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        initialization(); waitForStart();

        gyroPivotalTurnRight(200, 0.6);

        telemetry.addData("Gyro Heading", gyro.getIntegratedZValue());
        telemetry.update();

        sleep(1000);

        telemetry.addData("Gyro Heading", gyro.getIntegratedZValue());
        telemetry.update();

        sleep(10000);
    }
}
