package org.firstinspires.ftc.teamcode.RR.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RR.drive.SampleMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
public class LocalizationTestNewBot extends LinearOpMode {
    public DcMotor rightf, leftf, rightb, leftb, ringPickup2, ringPickup;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    double leftfrPower;
    double leftbackPower;
    double rightfrPower;
    double rightbackPower;
    double numberOfRings = 0;
    double leftPower;
    double rightPower;
    double wobbleArmPower;
    boolean sawRing = false;
    boolean shootingRing = false;

    double pickupState = 0;
    double shootInputState = 0;


    boolean zeroBrake = true;
    boolean slowMode = false;
    double lastBrake, lastSlow, lastGrip;
    double lastIntake = 0;
    boolean isIntakeOn;

    public double highGoalPower = -1 * 1600;
    public double powerShotPower = -1 * 1350;

    double lastChangePowershot = 0;
    boolean powerShotBeenShot = false;
    boolean powerShotMode = true;
    double lastPowerShotMode = 0;
    double lastWing = 0;


    double counter = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ringPickup2 = hardwareMap.get(DcMotor.class, "ringPickup2");
        ringPickup = hardwareMap.get(DcMotor.class, "ringPickup");


        runtime.reset();

        waitForStart();

        while (!isStopRequested()) {


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                // Setup a variable for each drive wheel to save power level for telemetry
                double straifPower = gamepad1.right_stick_x * -1;
                double tankPower = -gamepad1.right_stick_y;
                double turnPower = gamepad1.left_stick_x * -1;
                telemetry.addData("X: ", gamepad1.x);
                telemetry.addData("Y: ", gamepad1.y);

                if (gamepad1.x == true) {
                    isIntakeOn = true;
                }
                if (gamepad1.y == true) {
                    isIntakeOn = false;
                }


                if (isIntakeOn == true) {
                    ringPickup2.setPower(1);
                    ringPickup.setPower(-1);
                } else {
                    ringPickup2.setPower(0);
                    ringPickup.setPower(0);

                }


                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());
                telemetry.update();
            }
        }
    }
}