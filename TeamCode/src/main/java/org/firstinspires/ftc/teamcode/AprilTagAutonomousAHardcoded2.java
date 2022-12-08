/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="AutonomousAHardcoded2", group="Robot")
public class AprilTagAutonomousAHardcoded2 extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    Servo claw = null;
    DcMotorEx ArmMotor1 = null;
    DcMotorEx ArmMotor2 = null;
    float RoboArmNum = 0;

    static final double FEET_PER_METER = 3.28084;

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
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    int FORWARD2 = 0;
    int BACK1 = 0;
    int LEFT1 = 0;
    int RIGHT1 = 0;
    int CLOCKWISE45 = 0;
    int COUNTERCLOCKWISE45 = 0;
    int TOP = 0;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "backL");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "frontL");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "frontR");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "backR");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        claw = hardwareMap.get(Servo.class, "Servo");

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

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        claw.setPosition(0.4);
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            park1();
        }else if(tagOfInterest.id == MIDDLE){
            park2();
        }else{
            park3();
        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void left(long sec) {
        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightBackDrive.setPower(-0.5);
        sleep(sec);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void right(long sec) {
        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightBackDrive.setPower(0.5);
        sleep(sec);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void back(long sec) {
        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        sleep(sec);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void forward(long sec) {
        leftFrontDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        sleep(sec);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void cclockwise(long sec) {
        leftFrontDrive.setPower(-0.5);
        rightFrontDrive.setPower(0.5);
        leftBackDrive.setPower(-0.5);
        rightBackDrive.setPower(0.5);
        sleep(sec);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void score() {
        ArmMotor1.setTargetPosition(-595);
        ArmMotor2.setTargetPosition(-595);
        while (ArmMotor1.isBusy() || ArmMotor2.isBusy()) {
            idle();
        }
        sleep(500);
        forward(100);
        sleep(500);
        right(1450);
        sleep(500);
        forward(150);
        sleep(500);
        right(250);
        sleep(500);
        claw.setPosition(0);
        sleep(500);
        sleep(500);
        back(100);
        sleep(500);
    }

    void park1() {
        score();
        left(650);
        forward(600);
    }

    void park2() {
        score();
    }

    void park3() {
        score();
        left(650);
        cclockwise(200);
        back(550);
    }
}