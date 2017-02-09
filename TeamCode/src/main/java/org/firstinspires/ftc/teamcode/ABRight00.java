package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Owner on 12/9/2016.
 */
@Autonomous(name = "Auto Blue: [Right]", group = "Cyber Young Jr")
public class ABRight00 extends StatesAutoBlueBase {
    @Override
    public void setAuton() {
        shooterSpeed    = 7500   ;
        shootingPower   = 0.9    ;
        drive1          = 9      ;
        drive2          = -49    ;
        drive3          = -45    ;
        parkDrive       = -33    ;
        turn1           = 119.5  ;
        pTurn1          = 60.5   ;
        pTurn2          = 50     ;
        pTurn3          = 50     ;
        drive4          = 33     ;
        leftK           = 0.96   ;
        rightK          = 0.96   ;
        optimalWallDist = 4.75   ;
        lThreshold      = 40     ;
        driveHeading1   = 0      ;
        driveHeading2   = -119.5 ;
        drivePower      = 0.6    ;
        drivePowerR     = 0.7    ;
        lineDrivePower  = -0.2   ;
        turnPower       = 0.7    ;
        alterDistances[0] = 1.5  ;
        alterDistances[1] = 1.7  ;
    }
}
