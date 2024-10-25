package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.BuildConfig;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pose2dWrapper;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.teamcode.ComputerVision;

import java.util.Arrays;

@Config
@Autonomous(name = "MecanumAuto", group="8628")
public final class MecanumAuto extends LinearOpMode {
    public static double INITIAL_X = -6;
    public static double INITIAL_Y = 55;

    public static double ROTATIONINITIAL = 0;

    public static double PLACEONE_X = -45;
    public static double PLACEONE_Y = 54;

    public static double PLACETWO_X = -40;
    public static double PLACETWO_Y = 43;

    public static double PLACETHREE_X = -40;
    public static double PLACETHREE_Y = 60;

    public static double PLACEFOUR_X = -46;
    public static double PLACEFOUR_Y = 60;

    public static double ROTATIONFOUR = 180;

    public static double PLACEFIVE_X = -46;
    public static double PLACEFIVE_Y = 52;

    public static double PLACESIX_X = 0;
    public static double PLACESIX_Y = 52;
    public static double ROTATIONSIX = 0;

    public static double PLACESEVEN_X = 0;
    public static double PLACESEVEN_Y = 43;

    ComputerVision vision;
    AprilTagPoseFtc[] aprilTagTranslations = new AprilTagPoseFtc[11];
    //InputHandler inputHandler;

    boolean inputComplete = false;
    Pose2d robotPose;

    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime inputTimer = new ElapsedTime();
    int startDelay = 0;
    int i = 1; //used as an iterator for outputLog()

    @Override
    public void runOpMode() throws InterruptedException {

        //runtime.reset();
        //inputHandler = InputAutoMapper.normal.autoMap(this);
        /*while (inputComplete == false) {


            if (inputHandler.up("D1:X")) {
                inputComplete = true;
                inputTimer.reset();
            }

        }
        telemetry.addData("-----Initialization-----", "");
        telemetry.addLine();
        telemetry.addData("-----Modifications-----", "");
        telemetry.addLine();
        telemetry.addData("Press X to finalize values", inputComplete);
        telemetry.update();


        vision = new ComputerVision(hardwareMap);
        while (vision.visionPortal.getCameraState() == OPENING_CAMERA_DEVICE) {}

        vision.setActiveCameraOne(); */
        Pose2dWrapper startPose = new Pose2dWrapper(INITIAL_X, INITIAL_Y, Math.toRadians(-90));
        Pose2dWrapper poseOne = new Pose2dWrapper(PLACEONE_X, PLACEONE_Y, Math.toRadians(-90));
        Pose2dWrapper poseTwo = new Pose2dWrapper(PLACETWO_X, PLACETWO_Y, Math.toRadians(90));
        Pose2dWrapper poseThree = new Pose2dWrapper(PLACETHREE_X, PLACETHREE_Y, Math.toRadians(90));
        Pose2dWrapper poseFour = new Pose2dWrapper(PLACEFOUR_X, PLACEFOUR_Y, Math.toRadians(90));
        Pose2dWrapper poseFive = new Pose2dWrapper(PLACEFIVE_X, PLACEFIVE_Y, Math.toRadians(90));
        Pose2dWrapper poseSix = new Pose2dWrapper(PLACESIX_X, PLACESIX_Y, Math.toRadians(90));
        Pose2dWrapper poseSeven = new Pose2dWrapper(PLACESEVEN_X, PLACESEVEN_Y, Math.toRadians(90));

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(40.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        VelConstraint curveVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(15.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-20.0, 50.0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose.toPose2d());

        if (opModeInInit()) {
            String compilationDate = BuildConfig.COMPILATION_DATE;
            telemetry.addData("Compiled on:", compilationDate);
            telemetry.update();
        }

        waitForStart();
//                        .splineToConstantHeading(new Vector2d(-33.0, 30.00), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)

        Actions.runBlocking(
                drive.actionBuilder(startPose.toPose2d())
                        .strafeToConstantHeading(poseOne.toPose2d().position, baseVelConstraint,baseAccelConstraint)
//                        .strafeToConstantHeading(new Vector2d(PLACEONE_X, PLACEONE_Y), baseVelConstraint,baseAccelConstraint)
//                        .splineToConstantHeading(new Vector2d(PLACEONE_X, PLACEONE_Y), Math.toRadians(-90.0), baseVelConstraint,baseAccelConstraint)
                        //.strafeTo(poseOne.toPose2d().position)
                        //.strafeTo(poseTwo.toPose2d().position)
                        //.strafeTo(poseThree.toPose2d().position)
                        //.strafeTo(poseFour.toPose2d().position)
                        //.strafeTo(poseFive.toPose2d().position)
                        .build());

        if (isStopRequested()) return;
        telemetry.addData("Started Running", " ");
        telemetry.update();
        sleep(startDelay);
        outputLog(drive); //1

    }
    public void outputLog (MecanumDrive drive){
        RobotLog.d("WAY: Current Robot Pose Estimate and time: X: %.03f Y: %.03f Heading: %.03f ms: %.03f iteration: %d", drive.pose.position.x, drive.pose.position.y, Math.toDegrees(drive.pose.heading.real), runtime.milliseconds(), i);
        i++;
    }
}
