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

    int maxArmPos;
    int minArmPos;
    boolean runtoSet = false;

    String currentClawState = "closed";

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


        // Arm Motor
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

        ArmMotor1.setVelocity(1200);
        ArmMotor2.setVelocity(1200);

        ArmMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ArmMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void velocityCurve(DcMotor motor1, DcMotor motor2, float RoboArmNum) {
        int currentArmPos = (ArmMotor1.getCurrentPosition() + ArmMotor2.getCurrentPosition())/2;
    }


    public void loop() {
        telemetry.clear();

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

        // RoboArmNum = Math.min(RoboArmMax, RoboArmNum);
        // RoboArmNum = Math.max(0, RoboArmNum);

        // set power, position
        // setSyncMotorPosition(ArmMotor1, ArmMotor2, Math.round(RoboArmNum), RoboArmPower);
        // telemetry.addData("Motor position: ", RoboArmNum);
        // telemetry.update();


        // Sets up max and min positions
        if(gamepad2.dpad_up) {
            maxArmPos = (ArmMotor1.getCurrentPosition() + ArmMotor2.getCurrentPosition())/2;
            runtoSet = true;
        } else if (gamepad2.dpad_down) {
            minArmPos = (ArmMotor1.getCurrentPosition() + ArmMotor2.getCurrentPosition())/2;
        }

        // If the max and min values have been set
        if(runtoSet) {
            if(gamepad2.right_trigger > 0.5) {
                RoboArmNum = maxArmPos;

                // Make zoom zoom
                ArmMotor1.setVelocity(1200);
                ArmMotor2.setVelocity(1200);
            } else if (gamepad2.left_trigger > 0.5) {
                RoboArmNum = minArmPos;

                // Make not so zoom zoom so servo doesn't die
                ArmMotor1.setVelocity(800);
                ArmMotor2.setVelocity(800);
            }
        }

        telemetry.addData("Right Trigger:", gamepad2.right_trigger);

        telemetry.addData("Max Pos:", maxArmPos);
        telemetry.addData("Min Pos:", minArmPos);


        ArmMotor1.setTargetPosition(Math.round(RoboArmNum));
        ArmMotor2.setTargetPosition(Math.round(RoboArmNum));

        telemetry.addData("Arm Driver 1:", ArmMotor1.getCurrentPosition());
        telemetry.addData("Arm Driver 2:", ArmMotor2.getCurrentPosition());

        telemetry.update();



        // Drive --------------------------------------------------------------------
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
    }
}
