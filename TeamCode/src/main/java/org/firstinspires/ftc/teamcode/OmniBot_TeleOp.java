package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Owner on 1/7/2017.
 */
@TeleOp(name = "OmniBot TeleOp", group = "OmniBot")
// @Disabled
public class OmniBot_TeleOp extends OpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("lf");
        rightMotor = hardwareMap.dcMotor.get("rf");
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftMotor.setPower(-gamepad1.left_stick_y);
        rightMotor.setPower(-gamepad1.right_stick_y);
    }
}
