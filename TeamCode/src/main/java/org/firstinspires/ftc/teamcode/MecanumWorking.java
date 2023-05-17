package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp()
public class MecanumWorking extends OpMode{

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    DcMotor frontLMotor = null;
    DcMotor frontRMotor = null;

    //INTRODUCE VARIABLES HERE

    public void init() {

        leftMotor = hardwareMap.get(DcMotor.class, "backL");
        rightMotor = hardwareMap.get(DcMotor.class, "backR");

        frontLMotor = hardwareMap.get(DcMotor.class, "frontL");
        frontRMotor = hardwareMap.get(DcMotor.class, "frontR");

        telemetry.setAutoClear(false);

    }



    public void loop() {
        telemetry.clear();


        // Drive --------------------------------------------------------------------
        // assign speed modifier
        int speedModB = 2;

        if (gamepad1.right_bumper) {
            speedModB = 1;
        }
        if (gamepad1.left_bumper) {
            speedModB = 3;
        }

        // Mecanum Drive
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.right_stick_x);
        double robotAngle = Math.atan2(- 1 * gamepad1.right_stick_x, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.left_stick_y;
        final double v1 = r * Math.cos(-robotAngle) + rightX; //back left
        final double v2 = r * Math.sin(robotAngle) - rightX; //front right
        final double v3 = r * Math.sin(robotAngle) + rightX; //front left
        final double v4 = r * Math.cos(-robotAngle) - rightX; //back right

        frontLMotor.setPower(-v3 / speedModB);
        frontRMotor.setPower(-v2 / speedModB);
        leftMotor.setPower(-v1 / speedModB);
        rightMotor.setPower(-v4 / speedModB);
    }
}
