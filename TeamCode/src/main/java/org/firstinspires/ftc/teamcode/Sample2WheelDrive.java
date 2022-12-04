package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class Sample2WheelDrive extends OpMode {

    // creates 2 empty objects for motors
    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    DcMotor frontLMotor = null;
    DcMotor frontRMotor = null;

    public void init() {
        // passings hardware setups to the motors
        leftMotor = hardwareMap.get(DcMotor.class, "backL");
        rightMotor = hardwareMap.get(DcMotor.class, "backR");

        frontLMotor = hardwareMap.get(DcMotor.class, "frontL");
        frontRMotor = hardwareMap.get(DcMotor.class, "frontR");
    }

    public void loop() {

        // assign speed modifier
        int leftMod = 2;
        int rightMod = 2;

        if (gamepad1.left_bumper) {
            leftMod = 1;
        }
        if (gamepad1.right_bumper) {
            rightMod = 1;
        }


        // makes motor run at power of left, right control sticks
        // Implements tank steering

        leftMotor.setPower(gamepad1.left_stick_y / leftMod);
        rightMotor.setPower(gamepad1.right_stick_y / rightMod);

        frontLMotor.setPower(gamepad1.left_trigger / leftMod);
        frontRMotor.setPower(gamepad1.right_trigger / rightMod);
        telemetry.addData("Testing", "");



        //change all build variant to stock debug
        // mecanum wheels


    }
}
