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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="STATES_TELEOP", group="Cyber Young Jr")  // @Autonomous(...) is the other common choice
//@Disabled
public class StatesTeleop extends OpMode {

    /* Declare OpMode members. */
    // Declare DcMotors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftMotorR;
    DcMotor rightMotorR;

    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    DcMotor intakeMotor;

    DcMotor forkMotor;

    /* Declare Motor Variables */
    float leftDrive;
    float rightDrive;
    static final double DRIVE_POWER = 1.0;
    static final double CAP_POWER = 0.6;
    static final int MAX_SHOOTER_SPEED = 6000; // Encoder ticks for shooter per second when power = 0.4
    static final double SHOOTER_POWER  = 0.8;

    double maxSpeed = DRIVE_POWER;
    boolean capModeFlag = false;

    // Declare servos
    Servo dropFork;
    Servo bTrigBlue;
    Servo bTrigRed;

    CRServo conveyorServos;

    double bTrigPosR = RED_HOME;
    double bTrigPosB = BLUE_HOME;

    static final double SERVO_DELTA = 0.01;
    static final double RED_HOME    = 0.53;
    static final double BLUE_HOME   = 0.39;
    static final double TRAVEL      = 0.2 ;
    static final double HOME_FORK   = 0.95  ;

    // ModernRoboticsI2cRangeSensor redRange;
    //  ModernRoboticsI2cRangeSensor blueRange;

    double wallDist = 0;

    private ElapsedTime runtime = new ElapsedTime();




    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialization ...
        leftMotor         = hardwareMap.dcMotor.get("lf");
        rightMotor        = hardwareMap.dcMotor.get("rf");
        leftMotorR        = hardwareMap.dcMotor.get("lr");
        rightMotorR       = hardwareMap.dcMotor.get("rr");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotorR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftShooterMotor  = hardwareMap.dcMotor.get("ls");
        rightShooterMotor = hardwareMap.dcMotor.get("rs");

        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor       = hardwareMap.dcMotor.get("intake");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        forkMotor          = hardwareMap.dcMotor.get("fork");

        forkMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        try {
            conveyorServos = hardwareMap.crservo.get("conveyor");
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        try {
            bTrigBlue      = hardwareMap.servo.get("bbt");
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        try {
            bTrigRed      = hardwareMap.servo.get("rbt");
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        try {
            dropFork       = hardwareMap.servo.get("forkservo");
        } catch (Exception p_exception) {
            DbgLog.msg(p_exception.getLocalizedMessage());
        }

        conveyorServos    = hardwareMap.crservo.get("conveyor");
        dropFork          = hardwareMap.servo.get("forkservo");
        bTrigRed          = hardwareMap.servo.get("rbt");
        bTrigBlue         = hardwareMap.servo.get("bbt");


        bTrigBlue.setPosition(BLUE_HOME);
        bTrigRed.setPosition(RED_HOME);
        dropFork.setPosition(HOME_FORK);
       // blueRange = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "brange");

        runtime.reset();
    }

    @Override
    public void loop() {


        leftDrive = -gamepad1.left_stick_y;
        rightDrive = -gamepad1.right_stick_y;

        //DbgLog.msg("TeleOp - right stick ***  %.02f", -gamepad1.left_stick_y);
        //DbgLog.msg("TeleOp - **** left stick   %.02f", -gamepad1.right_stick_y);

        leftDrive  = (float) Range.clip(leftDrive, -maxSpeed, maxSpeed);
        rightDrive = (float) Range.clip(rightDrive, -maxSpeed, maxSpeed);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Range", wallDist);
        telemetry.addData("Speed LR", leftShooterMotor.getMaxSpeed()
                + ":" + rightShooterMotor.getMaxSpeed());
        telemetry.update();

        rightMotor.setPower(rightDrive);
        rightMotorR.setPower(rightDrive);
        leftMotor.setPower(leftDrive);
        leftMotorR.setPower(leftDrive);

        //DbgLog.msg("TeleOp - right power  %.02f", rightDrive);
        //DbgLog.msg("TeleOp - left power  %.02f", leftDrive);
        //DbgLog.msg("TeleOp - left shooter power %7d", leftShooterMotor.getCurrentPosition());
        //DbgLog.msg("TeleOp - right shooter power %7d", rightShooterMotor.getCurrentPosition());


        if (gamepad2.right_bumper) {
            conveyorServos.setPower(-1.0);
        } else if (gamepad2.left_bumper) {
            conveyorServos.setPower(1);
        } else {
            conveyorServos.setPower(0);
        }

        if (gamepad2.right_trigger > 0) {
            leftShooterMotor.setMaxSpeed(MAX_SHOOTER_SPEED);
            rightShooterMotor.setMaxSpeed(MAX_SHOOTER_SPEED);
            leftShooterMotor.setPower(SHOOTER_POWER);
            rightShooterMotor.setPower(SHOOTER_POWER);
        } else {
            leftShooterMotor.setPower(0.0);
            rightShooterMotor.setPower(0.0);
        }

        if (gamepad2.left_trigger > 0) {
            intakeMotor.setPower(0.8);
        } else {
            intakeMotor.setPower(0);
        }

        if (gamepad2.dpad_up) {
            forkMotor.setPower(1);
        } else if (gamepad2.dpad_down) {
            forkMotor.setPower(-0.3);
        } else {
            forkMotor.setPower(0);
        }

        if (gamepad2.a) {
            dropFork.setPosition(0);
        }
        if (gamepad2.y) {
            dropFork.setPosition(0.95);
        }

        if (gamepad1.b)
            bTrigPosR -= SERVO_DELTA;

        else
            bTrigPosR = RED_HOME;


        bTrigPosR = Range.clip(bTrigPosR, RED_HOME - TRAVEL, RED_HOME);

        bTrigRed.setPosition(bTrigPosR);

        if (gamepad1.x)
            bTrigPosB += SERVO_DELTA;

        else
            bTrigPosB = BLUE_HOME;

        bTrigPosB = Range.clip(bTrigPosB, BLUE_HOME, BLUE_HOME + TRAVEL);

        bTrigBlue.setPosition(bTrigPosB);
    }

    public void setShooterON(int speed, double power) {
        leftShooterMotor.setMaxSpeed(speed);
        rightShooterMotor.setMaxSpeed(speed);
        leftShooterMotor.setPower(power);
        rightShooterMotor.setPower(power);
    }

    public void setShooterOFF() {
        leftShooterMotor.setMaxSpeed(0);
        rightShooterMotor.setMaxSpeed(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
    }
}





