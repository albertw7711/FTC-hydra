package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class MecanumDrive extends OpMode {

    // creates 2 empty objects for motorsj
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
        int speedMod = 2;


        if (gamepad1.right_bumper) {
            speedMod = 1;
        }
        if (gamepad1.left_bumper) {
            speedMod = 3;
        }

        // Mecanum Drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
        double robotAngle = Math.atan2(- 1 * gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_y;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        frontLMotor.setPower(v1 / speedMod);
        frontRMotor.setPower(v2 / speedMod);
        leftMotor.setPower(v3 / speedMod);
        rightMotor.setPower(v4 / speedMod);

        /*from https://ftcforum.firstinspires.org/forum/
        ftc-technology/android-studio/6361-mecanum-wheels-drive-code-example
         */

    }

}
