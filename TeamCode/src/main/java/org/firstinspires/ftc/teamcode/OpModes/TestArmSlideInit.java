package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Disabled
@TeleOp(name = "TestArmSlideInit")
public class TestArmSlideInit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Put constants here
        int armMaxPosition = 100000;
        int armBucketPosition = 150;
        int armTargetPosition = 0;
        int armMinPosition = 0;
        int armRotationSpeed = 10;
        int slideMaxPosition = 100000;
        int slideBucketPosition = 150;
        int slideTargetPosition = 0;
        int slideMinPosition = 0;
        int slideMovementSpeed = 10;
        double clawTargetPosition = 0.5;
        double clawSpeed = 1.0/100.0;
        double clawMax = 1.0;
        double clawMin = 0.46;
        double wristTargetPosition = 0.5;
        double wristSpeed = 1.0/300.0;
        double wristMax = 0.89;
        double wristMin = 0.03;
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightBack");
//        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
//        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftBack");
//        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
//        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        DcMotor arm = hardwareMap.dcMotor.get("pivot");
        Servo claw = hardwareMap.servo.get("claw");
        Servo wrist = hardwareMap.servo.get("wrist");


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        slide.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setPower(0.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        tickLimit = maxVal;

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setPower(0.0);
//        if(reverse) {arm.setDirection(DcMotorSimple.Direction.REVERSE);}
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        tickLimit = maxVal;

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad2.a){
                arm.setPower(-0.3);
            } else if(gamepad2.y){
                arm.setPower(-0.5);
            }  else {
                arm.setPower(0.0);
            }
            if(gamepad2.b){
                slide.setPower(-0.03);
            } else {
                slide.setPower(0.0);
            }

            telemetry.addData("armPosition", arm.getCurrentPosition());
            telemetry.addData("slidePosition", slide.getCurrentPosition());
            telemetry.addData("armTargetPosition", armTargetPosition);
            telemetry.addData("slideTargetPosition", slideTargetPosition);
            telemetry.addData("wristTargetPosition", wristTargetPosition);
            telemetry.addData("clawTargetPosition", clawTargetPosition);

            telemetry.update();
        }
    }
}