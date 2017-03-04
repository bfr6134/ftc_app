package org.firstinspires.ftc.teamcode.testfunctionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.iowaprograms.IowaAutoBase;

/*kjhfds
 * Created by Owner on 2/14/2017.
 */
@Autonomous(name = "Consistency Test Runner", group = "Test")
public class TestConsistency extends IowaAutoBase {

    double rightLatencyFac = 10;
    double leftLatencyFac  = 6;

    @Override
    public void runOpMode() throws InterruptedException {
        initialization(); waitForStart();

        gyroPivotalTurnLeft(200-leftLatencyFac, 0.6);
        // sleep(250);

        /*gyroPivotalTurnLeft(90-5, 0.6);
        sleep(250);

        reverseGyroPivotalTurnRight(180-5, 0.6);
        sleep(250);

        reverseGyroPivotalTurnLeft(135-5, 0.6);*/

        telemetry.addData("Gyro Heading", gyro.getIntegratedZValue());
        telemetry.update();

        sleep(1000);

        telemetry.addData("Gyro Heading", gyro.getIntegratedZValue());
        telemetry.update();

        sleep(10000);
    }
}
