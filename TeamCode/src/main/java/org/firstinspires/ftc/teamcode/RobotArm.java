package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp()
public class RobotArm extends OpMode{

    //CRServo clawRot;
    // Servo clawRot;
    // Servo claw;
    //DcMotor roboArm = null;
    //DcMotor roboArmUp = null;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    DcMotor frontLMotor = null;
    DcMotor frontRMotor = null;

    DcMotor ArmMotor1 = null;
    DcMotor ArmMotor2 = null;
    float RoboArmNum = 0;


    //INTRODUCE VARIABLES HERE

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    static final double LIFT_SYNC_KP = 0.001;
    static final double LIFT_POSITION_TOLERANCE = 50;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    /*EDIT IF NEEDED!!!*/

    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    //https://github.com/FTCLib/FTCLib
    //https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Java-Sample-Op-Mode-for-TensorFlow-Object-Detection
    public void init() {
        //clawRot = hardwareMap.get(CRServo.class, "Servo");
        // clawRot = hardwareMap.get(Servo.class, "Servo");
        // claw = hardwareMap.get(Servo.class, "Servo2");


        leftMotor = hardwareMap.get(DcMotor.class, "backL");
        rightMotor = hardwareMap.get(DcMotor.class, "backR");

        frontLMotor = hardwareMap.get(DcMotor.class, "frontL");
        frontRMotor = hardwareMap.get(DcMotor.class, "frontR");


        ArmMotor1 = hardwareMap.get(DcMotor.class, "Arm1");
        ArmMotor2 = hardwareMap.get(DcMotor.class, "Arm2");


        ArmMotor1.setTargetPosition(0);
        ArmMotor2.setTargetPosition(0);

    }

    public void setSyncMotorPosition(DcMotor motor1, DcMotor motor2, int targetPosition, double power)
    {
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setTargetPosition(targetPosition);
        motor2.setTargetPosition(targetPosition);
        boolean isOnTarget = false;
        while (!isOnTarget && this.getRuntime() > 0.1)
        {
            telemetry.addData("ArmMotor1 Position: ", motor1.getCurrentPosition());
            telemetry.addData("ArmMotor2 Position: ", motor2.getCurrentPosition());
            telemetry.update();
            double differentiatePower = (motor2.getCurrentPosition() - motor1.getCurrentPosition())*LIFT_SYNC_KP;
            motor1.setPower(Range.clip(power + differentiatePower, -1.0, 1.0));
            motor2.setPower(Range.clip(power, -1.0, 1.0));
            isOnTarget = Math.abs(targetPosition - motor1.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                    Math.abs(targetPosition - motor2.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
            this.resetRuntime();
        }
        motor1.setPower(0.0);
        motor2.setPower(0.0);
    }

    public void loop() {
        double MIN_POSITION = 0.0;
        double MAX_POSITION = 1.0;

        //if (claw.getDirection() == Servo.Direction.FORWARD) {
            //double clawPos = claw.getDirection();
        //} else if (claw.getDirection() == Servo.Direction.REVERSE) {

        //}

            double RotPos = 1;
            int speedMod = 2;

            // Claw rotation
            if (gamepad2.dpad_up) {
                // clawRot.setPosition(RotPos);
                telemetry.addData("action:", "rightstick, claw rotation");
            } else if (gamepad2.dpad_down) {
                // clawRot.setPosition(-RotPos);
                telemetry.addData("action:", "reverse rotation");
            } else {
                // clawRot.setPosition(0);
            }


            // Clwa open / close
        /*
            if (gamepad2.left_trigger != 0) {
                claw.setPosition(1);
                telemetry.addData("action:", "claw open");
            } else if (gamepad1.left_bumper) {
                claw.setPosition(-1);
                telemetry.addData("action:", "claw close");
            }

         */

            /* Previous code
            //Robot arm
            if (gamepad2.right_bumper) {
                speedMod = 1;
            }

            roboArm.setPower(gamepad2.left_stick_y/speedMod);
            roboArm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            roboArmUp.setPower(gamepad2.right_stick_y/speedMod);
            roboArmUp.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            */
            int RoboArmMax = 500;
            int RoboArmTop = 500;
            int RoboArmMid = 250;
            int RoboArmBot = 100;
            double RoboArmPower = 1.0;
            RoboArmNum += gamepad2.left_stick_y * 4;
            if (gamepad2.y) {
                RoboArmNum = RoboArmTop;
            } else if (gamepad2.b) {
                RoboArmNum = RoboArmMid;
            } else if (gamepad2.a) {
                RoboArmNum = RoboArmBot;
            }
            // min, max values
            RoboArmNum = Math.max(RoboArmMax, RoboArmNum);
            RoboArmNum = Math.min(0, RoboArmNum);

            // set power, position
            setSyncMotorPosition(ArmMotor1, ArmMotor2, Math.round(RoboArmNum), RoboArmPower);
            // telemetry.addData("Motor position: ", RoboArmNum);
            // telemetry.update();


                if(gamepad2.left_bumper) {
                    //open
                    // claw.setPosition(0.05);
                    telemetry.addData("Claw servo set position:", "0.05");
                } else if (gamepad2.right_bumper) {
                    // claw.setPosition(0.3);
                    telemetry.addData("Claw servo set position:", "0.3");
                }
                telemetry.update();



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

            frontLMotor.setPower(v3 / speedModB);
            frontRMotor.setPower(v2 / speedModB);
            leftMotor.setPower(v1 / speedModB);
            rightMotor.setPower(v4 / speedModB);
    }
}
