
package org.firstinspires.ftc.teamcode.iowaprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * Created by Black Frog Coders on 2/8/2017.
 */
@Autonomous (name = "Red Alliance [] Auto", group = "Autonomous")
// @Disabled
public class IowaRedAuto extends IowaAutoBase {

    int latencyTurnFactor = 5;
    double backUpPower = 0.2  ;

    @Override
    public void setAuton() {


        /* General unmodified auto measurements */
        shooterSpeed    = 8250  ;
        shootingPower   = 0.9   ;
        shootDrivePower = 0.3   ;
        shootDrive      = 1     ;
        drive1          = 45+8-3;
        drive2          = 37    ;
        parkDrive       = -66   ;

        // Gyro error corrector turns
        pTurn1   = 45-latencyTurnFactor ;
        pTurn2   = 45-latencyTurnFactor ;
        pTurn3   = 50-latencyTurnFactor ;

        /* Properties of motion and sensing variables */

        leftK           = 0.96 ;
        rightK          = 0.70 ;
        rightLatencyFac = 10   ;
        leftLatencyFac  = 6    ;
        optimalWallDist = 3.5  ;
        lThreshold      = 40   ;
        driveHeading1   = -58  ;
        driveHeading2   = 15   ;
        driveHeading3   = -45  ;
        drivePower      = 0.6  ;
        lineDrivePower  = 0.3  ;
        turnPower       = 0.6  ;
        mTurnPower      = 0.45 ;


        // For beacon pushing
        alterDistances[0] = 1.5;
        alterDistances[1] = 1.7;
        alterDistances[2] = -8 ;
    }

    @Override
    public void driveToLine(double speed, double threshold) throws InterruptedException {

        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (lineSensor.getLightDetected() < threshold / 100) {
            setPowerLR(0.24, 0.36);
            idle();
        }
        setPowerLR(0, 0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Configures all hardware and autonomous stats
        initialization(); setAuton();

        waitForStart();

        driveForward(shootDrivePower, shootDrive, (int) driveHeading2, true);

        autonomousShootBall(shootingPower, shooterSpeed, 5, 0, 0, 2);

        // encoderLeftPTurn(turnPower, (int) pTurn1, leftK);
        gyroPivotalTurnLeft(pTurn1, turnPower);

        driveForward(drivePower, drive1, (int) driveHeading1, true);

        gyroPivotalTurnRight(pTurn2, turnPower + 0.2);

        preManeuverDegrees = getManeuverDegrees(optimalWallDist);
        if(preManeuverDegrees != 0)
            gyroPivotalTurnLeft(preManeuverDegrees, mTurnPower);

        if(preManeuverDegrees != 0)
            gyroPivotalTurnRight(preManeuverDegrees*0.7, mTurnPower);

        if (preManeuverDegrees != 0)
        {
            double backDistance = ROBOT_WIDTH*Math.sin(preManeuverDegrees*Math.PI/180);
            driveBackward(backUpPower, -backDistance, (int) driveHeading2, true);
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
