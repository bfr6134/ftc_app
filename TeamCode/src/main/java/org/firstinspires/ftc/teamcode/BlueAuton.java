package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by jimly2024 on 2/8/2017.
 */
@Autonomous (name = "Blue Alliance [] Auto", group = "Autonomous")
public class BlueAuton extends AutonBase {

    @Override
    public void setAuton() {
        shooterSpeed    = 6500   ;
        shootingPower   = 0.9    ;
        drive1         = 9      ;
        drive2          = -52 + 2    ;
        drive3          = -33    ;
        parkDrive       = -33    ;
        turn1           = 119.5  ;
        pTurn1          = 60.5   ;
        pTurn2          = 50     ;
        pTurn3          = 48.5     ;
        drive4          = 33     ;
        leftK           = 0.8    ;
        rightK          = 0.96   ;
        optimalWallDist = 3.5   ;
        lThreshold      = 40     ;
        driveHeading1   = 0      ;
        driveHeading2   = -119.5 ;
        driveHeadingBeacon = -180-5;
        drivePower      = 0.6    ;
        drivePowerR     = 0.7    ;
        lineDrivePower  = -0.2 -0.1; //+ 0.05   ;
        capDrivePower   = 1      ;
        turnPower       = 0.7    ;
        mTurnPower      = 0.45   ;
        alterDistances[0] = 1.5  ;
        alterDistances[1] = 1.7  ;
        alterDistances[2] = 8    ;
    }


    @Override
    public void runOpMode() throws InterruptedException {
        // Configures all hardware and autonomous stats
        initialization();  setAuton();

        waitForStart();

        driveForward(0.6, drive1, (int) driveHeading1, true);

        autonomousShootBall(shootingPower, shooterSpeed, 5, 1.6, 3, 1);

        gyroTurnLeft(turn1, turnPower, turnPower);

        driveBackward(drivePowerR, drive2, (int) driveHeading2, true);

        reverseGyroPivotalTurnLeft(pTurn1, turnPower);

        preManeuverDegrees = getManeuverDegrees(optimalWallDist);
        if(preManeuverDegrees != 0)
            reverseGyroPivotalTurnRight(preManeuverDegrees, mTurnPower);

        if(preManeuverDegrees != 0)
            reverseGyroPivotalTurnLeft(preManeuverDegrees * 0.9, mTurnPower);

        if (preManeuverDegrees != 0) {
            double backDistance = ROBOT_WIDTH*Math.sin(preManeuverDegrees*Math.PI/180);
            driveBackward(-lineDrivePower, backDistance, (int) driveHeadingBeacon, true);
        }

        driveToLine(lineDrivePower, lThreshold);

        pushBeacon(ALLIANCES.BLUE_ALLIANCE, 1.5, 1.7);

        driveBackward(drivePower, drive3, (int) driveHeadingBeacon,true);

        preManeuverDegrees2 = getManeuver2Degrees();

        if(preManeuverDegrees2 != 0)
            reverseGyroPivotalTurnRight(preManeuverDegrees2, mTurnPower);

        if(preManeuverDegrees2 != 0)
            reverseGyroPivotalTurnLeft(preManeuverDegrees2 * 0.8, mTurnPower);

        if (preManeuverDegrees2 != 0)
        {
            double backDistance = ROBOT_WIDTH*Math.sin(preManeuverDegrees2*Math.PI/180);
            driveBackward(drivePower, backDistance, (int) driveHeading2, true);
        }

        driveToLine(lineDrivePower, lThreshold);

        pushBeacon(ALLIANCES.BLUE_ALLIANCE, 1.5, 1.7);

        driveForward(drivePower, 4.5, 0, true);

        gyroPivotalTurnRight(45, turnPower);

        driveForward(capDrivePower, drive4, -135, true);

        gyroTurnLeft(180, turnPower, turnPower);

        driveBackward(capDrivePower, parkDrive, -135, true);

        telemetry.addData("SUCCESS", "Hey! Dat's Pretty Gooood");
        telemetry.update();

        leftMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotor.setPower(0);
        rightMotorR.setPower(0);
    }

}
