
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Created by The BFR Coderz on 11/28/2016.
 */

@Autonomous(name="Auto Red: [LEFT]", group = "Cyber Young Jr")
// @Disabled

public class ARLeft00 extends StatesAutoRedBase {

    @Override
    public void setAuton() {
        shooterSpeed    = 8500   ;
        shootingPower   = 0.9    ;
        drive1          = 52     ;
        drive2          = 30     ;
        parkDrive       = -66    ;
        pTurn1          = 55     ;
        pTurn2          = 52     ;
        pTurn3          = 50     ;
        leftK           = 0.96   ;
        rightK          = 0.96   ;
        optimalWallDist = 4.75   ;
        lThreshold      = 40     ;
        driveHeading1   = -55    ;
        driveHeading2   = 0      ;
        driveHeading3   = -50    ;
        drivePower      = 0.70   ;
        lineDrivePower  = 0.2    ;
        turnPower       = 0.6    ;
        alterDistances[0] = 1.5  ;
        alterDistances[1] = 1.7  ;
    }
}
