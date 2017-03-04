
package org.firstinspires.ftc.teamcode.testfunctionprograms;

/*
 * Created by Joshua on 2/18/2017.
 */

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Test Shooter Speed", group = "Test")
// @Disabled

public class TestGetShooterSpeed extends LinearOpMode {

    DcMotor       rightShooterMotor = null; // Power controller
    DcMotor       rightEncoder      = null; // Enc data receiver

    ElapsedTime timer = new ElapsedTime();  // Time receiver

    static final double TARGET_SPEED        = 2000 ;   // User input speed in rpm
    static final double IIR_SMOOTHER        = 10   ;   // Smoothing factor in the IIR Smoothing calc
    static final double CPR                 = 28   ;   // Enc counts per motor rev
    static final double POWER_REFERENCE     = 0.24 ;   // Power referred to for correcting
    static final double CORRECTION_FACTOR    = 3000;   // Scales down the correction of speed

    /* Speed variables */
    double firstTime, secondTime;
    double firstPosition, secondPosition;
    double shooterPower, speedError;

    double currentSpeed;
    double averageSpeed = 2000;


    @Override
    public void runOpMode() throws InterruptedException {
        rightShooterMotor = hardwareMap.dcMotor.get("rs");
        rightEncoder      = hardwareMap.dcMotor.get("re");
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        timer.reset();
        telemetry.addData(">", "Initialized");
        telemetry.update();

        waitForStart();
        rightShooterMotor.setPower(1);
        //rightShooterMotor.setMaxSpeed(8250);

        sleep(275);

        shooterPower = POWER_REFERENCE;
        rightShooterMotor.setPower(shooterPower);

        firstPosition = -rightEncoder.getCurrentPosition();
        firstTime = timer.milliseconds();

        sleep(100);



        while (opModeIsActive()) {
            secondPosition = -rightEncoder.getCurrentPosition(); // Second data collection
            secondTime     = timer.milliseconds(); // "..."

            // ΔPosition divided by ΔTime is speed
            currentSpeed = (secondPosition-firstPosition)/(secondTime-firstTime);
            currentSpeed *= 60*1000/CPR; // Conversion to rpm

            /* IIR Smoother Calc */
            averageSpeed = (currentSpeed/ IIR_SMOOTHER)
                    + (averageSpeed*(IIR_SMOOTHER-1)/ IIR_SMOOTHER);


            /* Log Data */
            telemetry.addData(">", "Current Speed %.02f rpm", averageSpeed);
            telemetry.addData("Power", shooterPower);
            telemetry.update();
            DbgLog.msg("Current Speed %.02f rpm", averageSpeed);

            /* Update first data collect */
            firstPosition = secondPosition;
            firstTime     = secondTime;

            /* Speed change calc */
            speedError    = TARGET_SPEED - averageSpeed;
            shooterPower = POWER_REFERENCE + speedError/CORRECTION_FACTOR;

            rightShooterMotor.setPower(shooterPower); // Apply

            sleep(50); // Loop delay
            idle();
        }

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Coast
    }
}


