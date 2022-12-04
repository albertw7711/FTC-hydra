package org.firstinspires.ftc.teamcode;


//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


//Use apriltag (image scanner, not neural network)
@Autonomous
public class Auto extends OpMode {

    DcMotor leftMotor = null;
    DcMotor rightMotor = null;

    //SampleMecanumDrive drive;
    DcMotor frontLMotor = null;
    DcMotor frontRMotor = null;

    //Drive Constants
    public static final double TICKS_PER_REV = 537.6;
    public static final double MAX_RPM = 312;



    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "backL");
        rightMotor = hardwareMap.get(DcMotor.class, "backR");

        frontLMotor = hardwareMap.get(DcMotor.class, "frontL");
        frontRMotor = hardwareMap.get(DcMotor.class, "frontR");
        //drive = new SampleMecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        //Roadrunner code
       // Trajectory untitled0 = drive.trajectoryBuilder(new Pose2d(-42.17, 59.50, Math.toRadians(6.01)))
                //.splineTo(new Vector2d(19.00, 46.67), Math.toRadians(223.73))
               // .splineTo(new Vector2d(-50.67, 28.50), Math.toRadians(39.77))
               // .splineTo(new Vector2d(0.33, 50.17), Math.toRadians(-36.03))
                //.splineTo(new Vector2d(4.67, 46.83), Math.toRadians(270.00))
               // .build();

        // move right
        double X = 0.1;
        frontLMotor.setPower(X);
        frontRMotor.setPower(-X);
        leftMotor.setPower(-X);
        rightMotor.setPower(X);
    }
}
