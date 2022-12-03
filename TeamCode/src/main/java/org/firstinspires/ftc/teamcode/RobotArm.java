package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp()
public class RobotArm extends OpMode{
    Servo claw;

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    DcMotor frontLMotor = null;
    DcMotor frontRMotor = null;

    DcMotorEx ArmMotor1 = null;
    DcMotorEx ArmMotor2 = null;
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
        claw = hardwareMap.get(Servo.class, "Servo");

        leftMotor = hardwareMap.get(DcMotor.class, "backL");
        rightMotor = hardwareMap.get(DcMotor.class, "backR");

        frontLMotor = hardwareMap.get(DcMotor.class, "frontL");
        frontRMotor = hardwareMap.get(DcMotor.class, "frontR");


        ArmMotor1 = hardwareMap.get(DcMotorEx.class, "Arm1");
        ArmMotor2 = hardwareMap.get(DcMotorEx.class, "Arm2");


        ArmMotor1.setTargetPosition(0);
        ArmMotor2.setTargetPosition(0);
        ArmMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        ArmMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        ArmMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor1.setVelocity(500);
        ArmMotor2.setVelocity(500);

        telemetry.setAutoClear(false);

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
        while (!isOnTarget)
        {
            double differentiatePower = (motor2.getCurrentPosition() - motor1.getCurrentPosition())*LIFT_SYNC_KP;
            motor1.setPower(Range.clip(power + differentiatePower, -1.0, 1.0));
            motor2.setPower(Range.clip(power, -1.0, 1.0));
            isOnTarget = Math.abs(targetPosition - motor1.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE &&
                    Math.abs(targetPosition - motor2.getCurrentPosition()) <= LIFT_POSITION_TOLERANCE;
            telemetry.clear();
            telemetry.addLine()
                    .addData("targetPosition", targetPosition)
                    .addData("armMotor1Position", motor1.getCurrentPosition())
                    .addData("armMotor2Position", motor2.getCurrentPosition())
                    .addData("isOnTarget", isOnTarget)
                    .addData("armMotor1Power", motor1.getPower())
                    .addData("armMotor2Power", motor2.getPower());
            telemetry.update();
        }
    }

    public void loop() {
        double MIN_POSITION = 0.0;
        double MAX_POSITION = 1.0;
        String currentClawState = "closed";

        // Claw Code
        if(gamepad2.b) {
            // open
            currentClawState = "open";
            claw.setPosition(0.4);
        } else if (gamepad2.a) {
            // closed
            currentClawState = "closed";
            claw.setPosition(0);
        }

        telemetry.addData("Claw:", currentClawState);



        int RoboArmMax = 5000;
        int RoboArmTop = 5000;
        int RoboArmMid = 2500;
        int RoboArmBot = 1000;

        RoboArmNum -= gamepad2.left_stick_y * 4;
//            if (gamepad2.y) {
//                RoboArmNum = RoboArmTop;
//            } else if (gamepad2.b) {
//                RoboArmNum = RoboArmMid;
//            } else if (gamepad2.a) {
//                RoboArmNum = RoboArmBot;
//            }
//            // min, max values
//            RoboArmNum = Math.min(RoboArmMax, RoboArmNum);
//            RoboArmNum = Math.max(0, RoboArmNum);

        // set power, position
        // setSyncMotorPosition(ArmMotor1, ArmMotor2, Math.round(RoboArmNum), RoboArmPower);
        // telemetry.addData("Motor position: ", RoboArmNum);
        // telemetry.update();

        ArmMotor1.setTargetPosition(Math.round(RoboArmNum));
        ArmMotor2.setTargetPosition(Math.round(RoboArmNum));
        telemetry.clear();
        telemetry.addLine()
                .addData("targetPosition", RoboArmNum)
                .addData("armMotor1Position", ArmMotor1.getCurrentPosition())
                .addData("armMotor2Position", ArmMotor2.getCurrentPosition());
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
