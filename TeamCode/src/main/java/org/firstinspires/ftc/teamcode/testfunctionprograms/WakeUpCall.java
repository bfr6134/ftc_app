/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.testfunctionprograms;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Call: WAAAAAAKE UUUUP!!!!", group="Cyber Young Jr")  // @Autonomous(...) is the other common choice
// @Disabled
public class WakeUpCall extends LinearOpMode {
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    byte[] rangeAcache;
    // byte[] rangeCcache;
    byte[] colorAb;
    // byte[] colorCb;
    byte[] colorAr;
    // byte[] colorCr;

    ModernRoboticsI2cGyro gyro;
    OpticalDistanceSensor lineSensor;
    I2cDevice rangeA;
    // I2cDevice rangeC;
    I2cDevice colorA;
    // I2cDevice colorC;
    I2cDeviceSynch colorAreader;
    // I2cDeviceSynch colorCreader;
    I2cDeviceSynch rangeAreader;
    // I2cDeviceSynch rangeCreader;

    String gcolorA = "NONE";
    String gcolorC = "NONE";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        rangeA = hardwareMap.i2cDevice.get("rrange");
        // rangeC = hardwareMap.i2cDevice.get("brange");
        colorA = hardwareMap.i2cDevice.get("rcolor");
        // colorC = hardwareMap.i2cDevice.get("bcolor");

        rangeAreader = new I2cDeviceSynchImpl(rangeA, I2cAddr.create8bit(0x48), false);
        // rangeCreader = new I2cDeviceSynchImpl(rangeC, I2cAddr.create8bit(0x28), false);
        colorAreader = new I2cDeviceSynchImpl(colorA, I2cAddr.create8bit(0x4c), false);
        // colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);

        rangeAreader.engage();
        // rangeCreader.engage();
        colorAreader.engage();
        // colorCreader.engage();
        lineSensor = hardwareMap.opticalDistanceSensor.get("optic");
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        gyro.calibrate();

        while (gyro.isCalibrating()) {
            Thread.sleep(50);
            idle();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                colorAreader.write8(0x03, 0);
                // colorCreader.write8(0x03, 0);
            }

            rangeAcache = rangeAreader.read(0x04, 2);  //Read 2 bytes starting at 0x04
            // rangeCcache = rangeCreader.read(0x04, 2);
            colorAb = colorAreader.read(0x07, 1);
            // colorCb = colorCreader.read(0x07, 1);
            colorAr = colorAreader.read(0x05, 1);
            // olorCr = colorCreader.read(0x05, 1);

            // int RUS = rangeCcache[0] & 0xFF;   //Ultrasonic value is at index 0. & 0xFF creates a value between 0 and 255 instead of -127 to 128
            int LUS = rangeAcache[0] & 0xFF;
            // int RODS = rangeCcache[1] & 0xFF;
            int LODS = rangeAcache[1] & 0xFF;

            if (colorAr[0] > colorAb[0])
                gcolorA = "RED";

            else if (colorAb[0] > colorAr[0])
                gcolorA = "BLUE";

            else
                gcolorA = "NONE";


            //display values
            telemetry.addData("1 A US", LUS);
            telemetry.addData("2 A ODS", LODS);
            // telemetry.addData("3 C US", RUS);
            // telemetry.addData("4 C ODS", RODS);
            telemetry.addData("5 #A", gcolorA);
            telemetry.addData("6 #C", gcolorC);
            telemetry.addData("7 GG", gyro.getIntegratedZValue());
            telemetry.addData("8 ODS", lineSensor.getLightDetected());
            telemetry.update();
            idle();
        }
    }

    private void cool() {
    }

    private void killCrap() {
    }
}