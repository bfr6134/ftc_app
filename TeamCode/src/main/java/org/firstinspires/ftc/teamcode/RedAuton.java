package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jimly2024 on 2/8/2017.
 */
@Autonomous (name = "Red Alliance [] Auto", group = "Autonomous")
public class RedAuton extends AutonBase {

    @Override
    public void setAuton() {
        shooterSpeed = 8250;
        shootingPower = 0.9;
        shootDrivePower = 0.3;
        shootDrive = 1;
        drive1 = 52 - 3;
        drive2 = 33 - 6;
        parkDrive = -66;
        pTurn1 = 58;
        pTurn2 = 52 + 2;
        pTurn3 = 50;
        leftK = 0.96;
        rightK = 0.70;
        optimalWallDist = 3.5;
        lThreshold = 40;
        driveHeading1 = -58;
        driveHeading2 = 0 + 10 + 5;
        driveHeading3 = -50;
        drivePower = 0.60;
        lineDrivePower = 0.2 + 0.1;
        turnPower = 0.6;
        mTurnPower = 0.45;
        alterDistances[0] = 1.5;
        alterDistances[1] = 1.7;
        alterDistances[2] = -8;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Configures all hardware and autonomous stats
        initialization(); setAuton();

        waitForStart();

        driveForward(shootDrivePower, shootDrive, (int) driveHeading2, true);

        autonomousShootBall(shootingPower, shooterSpeed, 5, 0, 0, 2);

        encoderLeftPTurn(turnPower, (int) pTurn1, leftK);

        driveForward(drivePower, drive1, (int) driveHeading1, true);

        gyroPivotalTurnRight(pTurn2, turnPower + 0.2);

        preManeuverDegrees = getManeuverDegrees(optimalWallDist);
        if(preManeuverDegrees != 0)
            gyroPivotalTurnLeft(preManeuverDegrees, mTurnPower);

        if(preManeuverDegrees != 0)
            gyroPivotalTurnRight(preManeuverDegrees*0.75, mTurnPower);

        if (preManeuverDegrees != 0)
        {
            double backDistance = ROBOT_WIDTH*Math.sin(preManeuverDegrees*Math.PI/180);
            driveBackward(lineDrivePower, -backDistance, (int) driveHeading2, true);
        }

        driveToLine(lineDrivePower, lThreshold);

        pushBeacon(ALLIANCES.RED_ALLIANCE, alterDistances[0], alterDistances[1]);

        driveForward(drivePower+0.2, drive2, (int) driveHeading2, true);

        preManeuverDegrees2 = getManeuver2Degrees();

        if(preManeuverDegrees2 != 0) {
            gyroPivotalTurnLeft(preManeuverDegrees2, mTurnPower);
        }

        if(preManeuverDegrees2 != 0)
            gyroPivotalTurnRight(preManeuverDegrees2*rightK*0.8, mTurnPower);

        driveToLine(lineDrivePower, lThreshold);

        pushBeacon(ALLIANCES.RED_ALLIANCE, alterDistances[0], alterDistances[1]);

        reverseGyroPivotalTurnLeft(pTurn3, turnPower + 0.4);

        driveBackward(drivePower+0.2, parkDrive, (int) driveHeading3, true);

        telemetry.addData("SUCCESS", "Hey! Dat's Pretty Gooood");
        telemetry.update();

    }
}
