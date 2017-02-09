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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the 
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="BFR: Encoder Gyro Test", group="Pushbot")
// @Disabled
public class BfrAutoDriveByEncAndGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor leftMotor   = null;
    DcMotor rightMotor  = null;
    DcMotor leftMotorR  = null;
    DcMotor rightMotorR = null;
    ModernRoboticsI2cGyro gyro = null;
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final int        SLOW_POINT              = 730; //counts //1024 870
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double     SLOW_DOWN_SPEED         = 0.4;
    static final double     GDM                     = 0.0035;// 0.001 0.01 0.005
    static final double     GDM2                    = 0.004;


    void safeSleep(long sleepTimeInMS)
    {
        try {
            Thread.sleep(sleepTimeInMS);
        }
        catch (InterruptedException ex) {
            returnNothing(0);
        }
    }

    int returnNothing (int nothing) {
        return nothing;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        leftMotor = hardwareMap.dcMotor.get("lf");
        rightMotor = hardwareMap.dcMotor.get("rf");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotorR = hardwareMap.dcMotor.get("lr");
        rightMotorR = hardwareMap.dcMotor.get("rr");

        leftMotorR.setDirection(DcMotor.Direction.FORWARD);
        rightMotorR.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                          rightMotor.getCurrentPosition(),
                          rightMotor.getCurrentPosition());
        telemetry.update();

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        telemetry.addData("Status", "Gyro Calibraerwr cfchhytfftfumv   fc");
        telemetry.update();
        while (gyro.isCalibrating()) {
            safeSleep(50);
            idle();
        }
        telemetry.addData("Status", "*CLEAR*");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        driveForward(DRIVE_SPEED, 78, 0, true);
        safeSleep(250);
        driveBackward(DRIVE_SPEED, 78, 0, true);

        //safeSleep(1000);     // pause for servos to move

        int gyroAngle = gyro.getIntegratedZValue();

        telemetry.addData("Path", "Complete. Gyro Heading is: " + gyroAngle);
        telemetry.update();


        //driveBackward(DRIVE_SPEED, 78, 0);

        safeSleep(5000);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void driveForward0(double power, double inches)
    {
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int rightInitialPosition = rightMotor.getCurrentPosition();
            int leftInitialPosition = leftMotor.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            int leftNewTarget = leftInitialPosition + (int)(inches * COUNTS_PER_INCH);
            int rightNewTarget = rightInitialPosition + (int)(inches * COUNTS_PER_INCH);

            /*leftMotor.setPower(0.145*0.795); // 16%= 20
            rightMotor.setPower(0.145*1.205);//25%= -15
            leftMotorR.setPower(0.145*0.795);
            rightMotorR.setPower(0.145*1.205);*/
            leftMotor.setPower(0.6);
            rightMotor.setPower(0.6);
            leftMotorR.setPower(0.6);
            rightMotorR.setPower(0.6);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && leftMotor.getCurrentPosition() < leftNewTarget
                                    && rightMotor.getCurrentPosition() < rightNewTarget) {
                leftMotor.setPower(0.6);
                rightMotor.setPower(0.6);
                leftMotorR.setPower(0.6);
                rightMotorR.setPower(0.6);
            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotorR.setPower(0);
        }
    }


    //Input:
    // power:  should always be positive
    //inches: should always be positive
    //targetHeading: global gyro heading
    public void driveForward(double power, double inches, int targetHeading, boolean rampDown) {
        //Make sure the power is always negative for forward move
        power = power < 0 ? -power : power;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int initialPosition = rightMotor.getCurrentPosition();
            // int leftInitialPosition = leftMotor.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            // int leftNewTarget = leftInitialPosition + (int)(inches * COUNTS_PER_INCH);
            int newTargetPosition = initialPosition + (int) (Math.signum(power) * inches * COUNTS_PER_INCH);
            //  int newTargetPosition = initialPosition + (int)inches * COUNTS_PER_INCH);
            // leftMotor.setTargetPosition(leftNewTarget);

            // rightMotor.setTargetPosition(rightNewTarget);

            // leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //double steerFactor = .125;

            //     double leftPower = (power*0.795) - 0.001;
            //    double rightPower = (power*1.205) + 0.001;

            double steeringFactor = 0.124;
            if (power > .6)
                steeringFactor = 0.131;

            double leftPower = power - steeringFactor;
            double rightPower = power + steeringFactor;

            if (rightPower > 1)
                rightPower = 1;

            leftMotor.setPower(leftPower); // 16%= 20
            rightMotor.setPower(rightPower);//25%= -15
            leftMotorR.setPower(leftPower);
            rightMotorR.setPower(rightPower);

            // keep looping while we are still active, and there is time left, and both motors are running.
           /* while (opModeIsActive() && leftMotor.getCurrentPosition() < leftNewTarget
                    && rightMotor.getCurrentPosition() < rightNewTarget)*/
            double headingError;
            double powerAdjustment;

            while (opModeIsActive() &&
                    (rightMotor.getCurrentPosition() < newTargetPosition)) {

                headingError = targetHeading - gyro.getIntegratedZValue();
                powerAdjustment = headingError * GDM;

                //Don't do this. It will keep increasing or
                //decreasing thr right power, making it out of
                //balance. The robit will spin.

                //rightPower -= powerAdjustment;
                //leftPower  += powerAdjustment;

                leftMotor.setPower(leftPower);
                leftMotorR.setPower(leftPower);
                rightMotor.setPower(rightPower - powerAdjustment);
                rightMotorR.setPower(rightPower - powerAdjustment);

                if (rampDown && (newTargetPosition - rightMotor.getCurrentPosition()) < SLOW_POINT) {
                    rightPower = SLOW_DOWN_SPEED + (steeringFactor * 0.875);
                    leftPower  = SLOW_DOWN_SPEED - (steeringFactor * 0.875);
                    rampDown = false;
                }

                telemetry.addData("Actual Current Pos", Math.signum(power) * rightMotor.getCurrentPosition());
                telemetry.addData("Target Pos", newTargetPosition);
                telemetry.addData("Heading Error", headingError);
                telemetry.update();

                idle();

            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotorR.setPower(0);
        }
    }

    //Input:
    // power:  should be positive
    //inches: should always be positive
    //targetHeading: global gyro heading
    public void driveBackward(double power, double inches, int targetHeading, boolean rampDown)
    {
        //Make sure the power is always negative for backward move
        power = power < 0 ? power : -power;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            int initialPosition = rightMotor.getCurrentPosition();
            // int leftInitialPosition = leftMotor.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            // int leftNewTarget = leftInitialPosition + (int)(inches * COUNTS_PER_INCH);
            int newTargetPosition = (int)(initialPosition -  inches * COUNTS_PER_INCH);


            double steeringFactor = 0.121;

            if (power < -.6)
                steeringFactor = 0.1195;

            double leftPower = power + steeringFactor;
            double rightPower = power - steeringFactor;

            if (rightPower < -1)
                rightPower = -1;

            leftMotor.setPower(leftPower); // 16%= 20
            rightMotor.setPower(rightPower);//25%= -15
            leftMotorR.setPower(leftPower);
            rightMotorR.setPower(rightPower);


            double headingError;
            double powerAdjustment;
            while (opModeIsActive() &&
                    (rightMotor.getCurrentPosition() >  newTargetPosition)) {

                headingError = targetHeading - gyro.getIntegratedZValue();
                powerAdjustment = headingError * GDM2;

                leftMotor.setPower(leftPower);
                leftMotor.setPower(leftPower);
                rightMotor.setPower(rightPower - powerAdjustment);
                rightMotorR.setPower(rightPower - powerAdjustment);

                if (rampDown && (rightMotor.getCurrentPosition() - newTargetPosition) < SLOW_POINT) {
                    leftPower = -SLOW_DOWN_SPEED + (steeringFactor * 0.87);
                    rightPower = -SLOW_DOWN_SPEED  - (steeringFactor * 0.87);
                    rampDown = (1 == 0);// In other words "false" god dang it
                }

                telemetry.addData("Actual Current Pos" , Math.signum(power)*rightMotor.getCurrentPosition());
                telemetry.addData("Target Pos" , newTargetPosition);
                telemetry.addData("Heading Error" , headingError);
                telemetry.update();

                idle();
            }

            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotorR.setPower(0);
        }
    }
}
