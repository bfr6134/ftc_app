
package org.firstinspires.ftc.teamcode.misc;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * Created by the BFR Coderz on 11/26/2016.
 */
public class StatesAutoShooting extends LinearOpMode {
    // Auton navigational parts
    double  alterDist, drive1, drive2, drive3, drive4, parkDrive;
    double  driveHeading1, driveHeading2;
    double  leftK, rightK;
    double  drivePower, drivePowerR, turnPower, lineDrivePower, shootingPower;
    double  lThreshold, optimalWallDist;
    double[] alterDistances = new double[2];
    double     pTurn1, pTurn2, pTurn3;
    int     shooterSpeed;
    double     turn1;
    int     stateController = 0;
    String stateDescription;

    enum ALLIANCES {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }



    // Declare DcMotors
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor leftMotorR;
    DcMotor rightMotorR;

    DcMotor leftShooterMotor;
    DcMotor rightShooterMotor;

    DcMotor intakeMotor;

    DcMotor forkMotor;
    
    // Encoder properties ...
    static final double COUNTS_PER_MOTOR_REV = 560;    // eg: Andymark Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double ROBOT_WIDTH = 16.5;
    static final double FULL_TURN_CIRCUMFERENCE = 3.1415 * ROBOT_WIDTH * 2;
    static final double FULL_TURN_COUNTS = FULL_TURN_CIRCUMFERENCE * COUNTS_PER_INCH;
    static final double GYRO_DRIVE_MOTOR_MULTIPLIER = 0.0001;
    static final int GYRO_READING_COUNT = 10;
    static final double GYRO_READING_DELAY_FACTOR = 0.942;
    static final double GYRO_LINEAR_TURN_ANGLE = 150;
    static final double GYRO_ANTI_TURN_STALL_POWER = .3;
    
    // Declare servos
    Servo dropFork;
    Servo bTrigBlue;
    Servo bTrigRed;

    CRServo conveyorServos;
    
    // Servo init positions ...
    static final double TRAVEL      = 0.32;
    static final double RED_HOME    = 0.51;
    static final double RED_PUSH    = RED_HOME - TRAVEL;
    static final double BLUE_HOME   = 0.36;
    static final double BLUE_PUSH   = BLUE_HOME + TRAVEL;
    static final double HOME_RETRACT = 0.95;
    
    // Declare sensors and timer
    OpticalDistanceSensor lineSensor;
    ModernRoboticsI2cGyro gyro;
    byte[] rangeCache;
    byte[] colorBlue;
    byte[] colorRed;

    String gColor = "NONE";

    I2cDevice rangeR;
    // I2cDevice rangeB;
    I2cDevice colorR;
    // I2cDevice colorB;
    I2cDeviceSynch colorRreader;
    // I2cDeviceSynch colorBreader;
    I2cDeviceSynch rangeRreader;
    // I2cDeviceSynch rangeBreader;
    double wallOffset;
    /*ColorSensor redColor;
    ColorSensor blueColor;
    ModernRoboticsI2cRangeSensor rangeSensor;*/
    static final double CONVERT_FACTOR           = 0.393701;
    static final double RANGE_ROBOT_MISALIGNMENT = 1;

    ElapsedTime timeKeeper  = new ElapsedTime();
    
    @Override
    public void runOpMode() throws InterruptedException {
        setAuton();
        
        // Initialization ...
        leftMotor         = hardwareMap.dcMotor.get("lf");
        rightMotor        = hardwareMap.dcMotor.get("rf");
        leftMotorR        = hardwareMap.dcMotor.get("lr");
        rightMotorR       = hardwareMap.dcMotor.get("rr");

        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftMotorR.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftShooterMotor  = hardwareMap.dcMotor.get("ls");
        rightShooterMotor = hardwareMap.dcMotor.get("rs");

        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor       = hardwareMap.dcMotor.get("intake");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        forkMotor          = hardwareMap.dcMotor.get("fork");

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

        // conveyorServos    = hardwareMap.crservo.get("conveyor");
        // beaconServo       = hardwareMap.servo.get("beacon");
        // dropFork          = hardwareMap.servo.get("forkservo");

        bTrigBlue.setPosition(BLUE_HOME);
        bTrigRed.setPosition(RED_HOME);
        dropFork.setPosition(HOME_RETRACT);

        lineSensor  = hardwareMap.opticalDistanceSensor.get("optic");
        gyro        = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        rangeR = hardwareMap.i2cDevice.get("rrange");
        // rangeB = hardwareMap.i2cDevice.get("brange");
        colorR = hardwareMap.i2cDevice.get("rcolor");
        // colorB = hardwareMap.i2cDevice.get("bcolor");
        rangeRreader = new I2cDeviceSynchImpl(rangeR, I2cAddr.create8bit(0x48), false);
        // rangeBreader = new I2cDeviceSynchImpl(rangeB, I2cAddr.create8bit(0x28), false);
        colorRreader = new I2cDeviceSynchImpl(colorR, I2cAddr.create8bit(0x4c), false);
        // colorBreader = new I2cDeviceSynchImpl(colorB, I2cAddr.create8bit(0x3c), false);

        rangeRreader.engage();
        // rangeBreader.engage();
        colorRreader.engage();
        // colorBreader.engage();

        sleep(25);
        Thread.sleep(25);

        gyro.calibrate();

        while (gyro.isCalibrating()) {

            Thread.sleep(50);

            telemetry.addData(">", "Gyro Calibrating, Do not move!");
            telemetry.update();
            idle();
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            switch (stateController) {

                case 0: {
                    stateDescription = "Shooting Particles"; displayState();
                    autonomousShootBall(shootingPower, shooterSpeed, 5, 1.6, 3, 1);
                    stateController++; break;
                }
                case 1: {
                    stateDescription = "Proceeding To Knock Off Cap Ball and Park"; displayState();
                    gyroDriveStraight(drivePower, 57, 0);
                }
                default: {
                    telemetry.addData("SUCCESS", "Hey! Dat's Pretty Gooood");
                    leftMotor.setPower(0);
                    leftMotorR.setPower(0);
                    rightMotor.setPower(0);
                    rightMotorR.setPower(0);
                    break;
                }
            }
            idle();
        }
    }


    
    public void setAuton() {
        
    }

    public void setPowerLR(double left, double right) throws InterruptedException {
        leftMotor.setPower(left);
        leftMotorR.setPower(left);
        rightMotor.setPower(right);
        rightMotorR.setPower(right);
    }

    public void setMotorsMode(DcMotor.RunMode mode){
        leftMotor.setMode(mode);
        //leftMotorR.setMode(mode);
        rightMotor.setMode(mode);
        //rightMotorR.setMode(mode);
    }

    public void encoderDrive(double speed,
                             double leftInches,
                             double rightInches) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTarget = rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                leftMotor.setTargetPosition(newLeftTarget);
                rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            setPowerLR(Math.abs(speed), Math.abs(speed));

                if (leftInches < 0){
                    leftMotorR.setPower(-speed);
                    rightMotorR.setPower(-speed);
                }

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {
                idle();
            }

            // Stop all motion;
            setPowerLR(0, 0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public void encoderRightPTurnR(double power, float degrees, double k) throws InterruptedException{
        double dDecimal = degrees / 360;
        double counts   = dDecimal * FULL_TURN_COUNTS;

        int newLeftTarget;
        int newRightTarget;
        int leftCurrent;
        int rightCurrent;

        double leftPower;
        double rightPower;

        double distanceError;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftCurrent = leftMotor.getCurrentPosition();
            rightCurrent = rightMotor.getCurrentPosition();
            newLeftTarget = leftCurrent;
            newRightTarget = (int) (rightCurrent - (counts));
            newRightTarget  *= k;
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            rightMotor.setPower(Math.abs(power));
            rightMotorR.setPower(Math.abs(power));
            leftMotor.setPower(0);
            leftMotorR.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                (rightMotor.getCurrentPosition() > newRightTarget)) {

                distanceError = rightMotor.getCurrentPosition() - newRightTarget;


                if (distanceError >= 1100) {
                    rightMotor.setPower(0.5);
                    rightMotorR.setPower(0.5);
                }
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            /*  leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void encoderLeftPTurn(double power, float degrees, double k) throws InterruptedException{
        double dDecimal = degrees / 360;
        double counts   = dDecimal * FULL_TURN_COUNTS;

        int newLeftTarget;
        int newRightTarget;
        int leftCurrent;
        int rightCurrent;

        double leftPower;
        double rightPower;

        double distanceError;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftCurrent = leftMotor.getCurrentPosition();
            rightCurrent = rightMotor.getCurrentPosition();
            newLeftTarget = leftCurrent;
            newRightTarget = (int) (rightCurrent +  (counts));
            newRightTarget  *= k;
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            rightMotor.setPower(Math.abs(power));
            rightMotorR.setPower(Math.abs(power));
            leftMotor.setPower(0);
            leftMotorR.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (rightMotor.getCurrentPosition() < newRightTarget)) {

                distanceError = -rightMotor.getCurrentPosition() + newRightTarget;


                if (distanceError <= 1100) {
                    rightMotor.setPower(0.5);
                    rightMotorR.setPower(0.5);
                }
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            /*  leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void encoderLeftPTurnR(double power, float degrees, double k) throws InterruptedException{
        double dDecimal = degrees / 360;
        double counts   = dDecimal * FULL_TURN_COUNTS;

        int newLeftTarget;
        int newRightTarget;
        int leftCurrent;
        int rightCurrent;

        double leftPower;
        double rightPower;

        double distanceError;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftCurrent = leftMotor.getCurrentPosition();
            rightCurrent = rightMotor.getCurrentPosition();
            newRightTarget = leftCurrent;
            newLeftTarget = (int) (rightCurrent - (counts));
            newLeftTarget  *= k;
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setPower(Math.abs(power));
            leftMotorR.setPower(Math.abs(power));
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (leftMotor.getCurrentPosition() > newLeftTarget)) {

                distanceError = leftMotor.getCurrentPosition() - newLeftTarget;


                if (distanceError >= 1100) {
                    leftMotor.setPower(0.5);
                    leftMotorR.setPower(0.5);
                }
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            /*  leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void encoderRightPTurn(double power, double degrees, double k) throws InterruptedException{
        double dDecimal = degrees / 360;
        double counts   = dDecimal * FULL_TURN_COUNTS;

        int newLeftTarget;
        int newRightTarget;
        int leftCurrent;
        int rightCurrent;

        double leftPower;
        double rightPower;

        double distanceError;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            leftCurrent = leftMotor.getCurrentPosition();
            rightCurrent = rightMotor.getCurrentPosition();
            newRightTarget = leftCurrent;
            newLeftTarget = (int) (rightCurrent +  (counts));
            newLeftTarget  *= k;
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            leftMotor.setPower(Math.abs(power));
            leftMotorR.setPower(Math.abs(power));
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (leftMotor.getCurrentPosition() < newLeftTarget)) {

                distanceError = -leftMotor.getCurrentPosition() + newLeftTarget;


                if (distanceError <= 1100) {
                    leftMotor.setPower(0.5);
                    leftMotorR.setPower(0.5);
                }
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            /*  leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void gyroAbsoluteLeft(double speed) throws InterruptedException {

        while (opModeIsActive() && gyro.getHeading() > (0 * 1.06)) {
            setPowerLR(0, speed);
            idle();
        }

        setPowerLR(0,0);
    }

    public void pushBeacon(ALLIANCES alliances, double alterDist, double alterDist2) throws InterruptedException{
        if (opModeIsActive()) {
            timeKeeper.reset();
            if (alliances == ALLIANCES.RED_ALLIANCE) {
                if (getColor(colorRreader).equals("RED")) {
                    gyroDriveStraight(drivePower, alterDist, 0);
                    while (timeKeeper.time() < 1.9)
                        bTrigRed.setPosition(RED_PUSH);
                } else if (getColor(colorRreader).equals("BLUE")) {
                    encoderDrive(drivePower, -alterDist2, 0);
                    while (timeKeeper.time() < 1.9)
                        bTrigRed.setPosition(RED_PUSH);
                }

            } else {
                if (getColor(colorRreader).equals("BLUE")) {
                    gyroDriveStraight(drivePower, -alterDist, 0);
                    while (timeKeeper.time() < 1.9)
                        bTrigRed.setPosition(RED_PUSH);
                } else if (getColor(colorRreader).equals("RED")) {
                    gyroDriveStraight(drivePower, alterDist2, 0);
                    while (timeKeeper.time() < 1.9)
                        bTrigRed.setPosition(RED_PUSH);
                }
            }

        }

        timeKeeper.reset();
        while (timeKeeper.time() < 1.9) {
            bTrigRed.setPosition(RED_HOME);
            bTrigBlue.setPosition(BLUE_HOME);
        }
    }
    public void driveToLine(double speed, double threshold) throws InterruptedException {

        while (lineSensor.getLightDetected() < threshold / 100) {
            setPowerLR(speed, speed);
            idle();
        }
        setPowerLR(0, 0);
    }
    public void gyroDriveStraight(double speed, double driveInches, double targetHeading) throws InterruptedException {
        int newLeftTarget;
        int newRightTarget;
        int leftCurrent;
        int rightCurrent;
        double currentHeading;
      //  boolean distanceFlag = false;
        double headingError;
        double powerAdjustment = 0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            currentHeading = gyro.getIntegratedZValue();
            leftCurrent = leftMotor.getCurrentPosition();
            rightCurrent = rightMotor.getCurrentPosition();
            newLeftTarget = leftCurrent + (int) (driveInches * COUNTS_PER_INCH);
            newRightTarget = rightCurrent + (int) (driveInches * COUNTS_PER_INCH);

            headingError = targetHeading - currentHeading;
            powerAdjustment = headingError * GYRO_DRIVE_MOTOR_MULTIPLIER;

            leftMotor.setPower(Math.signum(driveInches) * speed);
            rightMotor.setPower(Math.signum(driveInches) * speed);
            leftMotorR.setPower(Math.signum(driveInches) * speed);
            rightMotorR.setPower(Math.signum(driveInches) * speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive()) {

                currentHeading = gyro.getIntegratedZValue();
                leftCurrent = leftMotor.getCurrentPosition();
                rightCurrent = rightMotor.getCurrentPosition();

                headingError = targetHeading - currentHeading;
                powerAdjustment = headingError * GYRO_DRIVE_MOTOR_MULTIPLIER;

                //    leftMotor.setPower(speed + powerAdjustment);
                rightMotor.setPower(Math.signum(driveInches) * (speed) - powerAdjustment);
                rightMotorR.setPower(Math.signum(driveInches) * (speed) - powerAdjustment);

                if (driveInches > 0) {
                    if (leftCurrent >= newLeftTarget || rightCurrent >= newRightTarget) {
                        break;
                    }
                }
                else
                {
                    if (leftCurrent <= newLeftTarget || rightCurrent <= newRightTarget) {
                        break;
                    }
                }

                DbgLog.msg("GYRO - Current Heading %7d", gyro.getIntegratedZValue());
                DbgLog.msg("GYRO - Current Heading Error %.02f", headingError);
                DbgLog.msg("GYRO - Current Power Adjustment %.02f", powerAdjustment);

                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            leftMotorR.setPower(0);
            rightMotor.setPower(0);
            rightMotorR.setPower(0);

            // Turn off RUN_TO_POSITION
            /*  robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //leftMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //rightMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move

        }
    }

    double gyroReading() throws InterruptedException
    {
        double gRead = 0;
        for (int i = 0; i < GYRO_READING_COUNT; i++) {
            gRead += gyro.getIntegratedZValue();
        }

        gRead /= GYRO_READING_COUNT ;

        return gRead;
    }

    //degree - always positive
    // leftPower - always positive
    //rightPower - always positive, for pivotal turn, it is 0
    public void gyroTurnRight(double degree, double leftPower, double rightPower) throws InterruptedException {

        double currentAngle = gyroReading();
        double targetAngle = currentAngle + GYRO_READING_DELAY_FACTOR*degree;

        double slowTurnAngle = GYRO_LINEAR_TURN_ANGLE;
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && error > 0)
        {
            //   DbgLog.msg("Current Gyro Reading %7d", gyroReading());
            //   DbgLog.msg("Current Turn Error %.02f", error);

            double newLeftPower = Range.clip(leftPower * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newLeftPower < GYRO_ANTI_TURN_STALL_POWER)
                newLeftPower = GYRO_ANTI_TURN_STALL_POWER;

            double newRightPower = Range.clip(rightPower * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newRightPower < GYRO_ANTI_TURN_STALL_POWER)
                newRightPower = GYRO_ANTI_TURN_STALL_POWER;

            leftMotor.setPower(newLeftPower);
            leftMotorR.setPower(newLeftPower);
            rightMotor.setPower(-newRightPower);
            rightMotorR.setPower(-newRightPower);

            idle();

            currentAngle = gyroReading();
            error = targetAngle - currentAngle;
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotorR.setPower(0);

    }

    public void gyroPivotalTurnLeft(double degree, double power) throws InterruptedException {
        leftMotorR.setPower(0);
        leftMotor.setPower(0);

        double currentAngle = gyroReading();
        double targetAngle = currentAngle - GYRO_READING_DELAY_FACTOR*degree;

        double slowTurnAngle = GYRO_LINEAR_TURN_ANGLE;
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && error < 0)
        {

            leftMotorR.setPower(0);
            leftMotor.setPower(0);

            //   DbgLog.msg("Current Gyro Reading %7d", gyroReading());
            //   DbgLog.msg("Current Turn Error %.02f", error);

            double newRightPower = Range.clip(power * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newRightPower < GYRO_ANTI_TURN_STALL_POWER)
                newRightPower = GYRO_ANTI_TURN_STALL_POWER;

            rightMotor.setPower(newRightPower);
            rightMotorR.setPower(newRightPower);

            idle();

            currentAngle = gyroReading();
            error = targetAngle - currentAngle;
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotorR.setPower(0);
    }

    public void gyroPivotalTurnRight(double degree, double power) throws InterruptedException {
        //gyroTurnRight(degree, power, 0);

        rightMotorR.setPower(0);
        rightMotor.setPower(0);

        double currentAngle = gyroReading();
        double targetAngle = currentAngle + GYRO_READING_DELAY_FACTOR*degree;

        double slowTurnAngle = GYRO_LINEAR_TURN_ANGLE;
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && error > 0)
        {

            rightMotorR.setPower(0);
            rightMotor.setPower(0);

            //   DbgLog.msg("Current Gyro Reading %7d", gyroReading());
            //   DbgLog.msg("Current Turn Error %.02f", error);

            double newLeftPower = Range.clip(power * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newLeftPower < GYRO_ANTI_TURN_STALL_POWER)
                newLeftPower = GYRO_ANTI_TURN_STALL_POWER;

            leftMotor.setPower(newLeftPower);
            leftMotorR.setPower(newLeftPower);

            idle();

            currentAngle = gyroReading();
            error = targetAngle - currentAngle;
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotorR.setPower(0);
    }

    public void reverseGyroPivotalTurnRight(double degree, double power) throws InterruptedException {
        //gyroTurnRight(degree, power, 0);

        leftMotorR.setPower(0);
        leftMotor.setPower(0);

        double currentAngle = gyroReading();
        double targetAngle = currentAngle + GYRO_READING_DELAY_FACTOR*degree;

        double slowTurnAngle = GYRO_LINEAR_TURN_ANGLE;
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && error > 0)
        {

          //  leftMotorR.setPower(0);
          //  leftMotor.setPower(0);

            //   DbgLog.msg("Current Gyro Reading %7d", gyroReading());
            //   DbgLog.msg("Current Turn Error %.02f", error);

            double newRigtPower = Range.clip(power * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newRigtPower < GYRO_ANTI_TURN_STALL_POWER)
                newRigtPower = GYRO_ANTI_TURN_STALL_POWER;

            rightMotor.setPower(-newRigtPower);
            rightMotorR.setPower(-newRigtPower);

            idle();

            currentAngle = gyroReading();
            error = targetAngle - currentAngle;
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotorR.setPower(0);
    }



    public void reverseGyroPivotalTurnLeft(double degree, double power) throws InterruptedException {
        rightMotorR.setPower(0);
        rightMotor.setPower(0);

        double currentAngle = gyroReading();
        double targetAngle = currentAngle - GYRO_READING_DELAY_FACTOR*degree;

        double slowTurnAngle = GYRO_LINEAR_TURN_ANGLE;
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && error < 0)
        {

         //   rightMotorR.setPower(0);
         //   rightMotor.setPower(0);

            //   DbgLog.msg("Current Gyro Reading %7d", gyroReading());
            //   DbgLog.msg("Current Turn Error %.02f", error);

            double newLeftPower = Range.clip(power * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newLeftPower < GYRO_ANTI_TURN_STALL_POWER)
                newLeftPower = GYRO_ANTI_TURN_STALL_POWER;

            leftMotor.setPower(-newLeftPower);
            leftMotorR.setPower(-newLeftPower);

            idle();

            currentAngle = gyroReading();
            error = targetAngle - currentAngle;
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotorR.setPower(0);
    }


    //degree - always positive
    // leftPower - always positive, for pivotal turn, it is 0
    //rightPower - always positive
    public void gyroTurnLeft(double degree, double leftPower, double rightPower) throws InterruptedException {

        double currentAngle = gyroReading();
        double targetAngle = currentAngle - GYRO_READING_DELAY_FACTOR*degree;

        double slowTurnAngle = GYRO_LINEAR_TURN_ANGLE;
        double error = targetAngle - currentAngle;

        while (opModeIsActive() && error < 0)
        {
            //   DbgLog.msg("Current Gyro Reading %7d", gyroReading());
            //   DbgLog.msg("Current Turn Error %.02f", error);

            double newLeftPower = Range.clip(leftPower * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newLeftPower < GYRO_ANTI_TURN_STALL_POWER)
                newLeftPower = GYRO_ANTI_TURN_STALL_POWER;

            double newRightPower = Range.clip(rightPower * Math.abs(error) / slowTurnAngle, 0, 1);
            if (newRightPower < GYRO_ANTI_TURN_STALL_POWER)
                newRightPower = GYRO_ANTI_TURN_STALL_POWER;

            leftMotor.setPower(-newLeftPower);
            leftMotorR.setPower(-newLeftPower);
            rightMotor.setPower(newRightPower);
            rightMotorR.setPower(newRightPower);

            idle();

            currentAngle = gyroReading();
            error = targetAngle - currentAngle;
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        leftMotorR.setPower(0);
        rightMotorR.setPower(0);

    }

    public void autonomousShootBall(double shootPower,
                                    int shooterSpeed,
                                    int totalTime,
                                    double pauseStarter,
                                    int pauseEnder,
                                    int conveyorStart) throws InterruptedException {

        double stopTime = timeKeeper.time() + totalTime;
        double restartTime = timeKeeper.time() + pauseEnder;
        double startPause = timeKeeper.time() + pauseStarter;
        double startConveyor = timeKeeper.time() + conveyorStart;

        while (opModeIsActive() && timeKeeper.time() <= startConveyor) {

            leftShooterMotor.setMaxSpeed(shooterSpeed);
            rightShooterMotor.setMaxSpeed(shooterSpeed);
            leftShooterMotor.setPower(shootPower);
            rightShooterMotor.setPower(shootPower);
            idle();
        }
        while (opModeIsActive() && timeKeeper.time() <= stopTime){
            conveyorServos.setPower(1);
            idle();
        }
        /*while (opModeIsActive() && timeKeeper.time() <= restartTime && timeKeeper.time() >= startPause)
        {
            conveyorServos.setPower(0);
        }
        while (opModeIsActive() && timeKeeper.time() <= stopTime && timeKeeper.time() >= restartTime) {
            conveyorServos.setPower(1);
            idle();
        }*/
        conveyorServos.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
    }

    public void displayState() throws InterruptedException {
        telemetry.addData("Status", "Running Step " +
                (stateController + 1) + ": " +
                stateDescription);
        telemetry.update();
    }

    public double getWallDist(I2cDeviceSynch device) throws InterruptedException {
        rangeCache = device.read(0x04, 2);  //Read 2 bytes starting at 0x04
        int US = rangeCache[0] & 0xFF;
        return (US * CONVERT_FACTOR) - RANGE_ROBOT_MISALIGNMENT;
    }

    public double getManeuverDegrees() throws InterruptedException {
        wallOffset = getWallDist(rangeRreader) - optimalWallDist;
        return Math.acos((ROBOT_WIDTH - wallOffset) / ROBOT_WIDTH);
    }

    public String getColor(I2cDeviceSynch device) throws InterruptedException {
        if (opModeIsActive()) {
            colorBlue = device.read(0x07, 1);
            colorRed  = device.read(0x05, 1);

            if (colorRed[0] > colorBlue[0])
                gColor = "RED";

            else if (colorBlue[0] > colorRed[0])
                gColor = "BLUE";

            else
                gColor = "NONE";

            return gColor;
        }
        return null;
    }
}