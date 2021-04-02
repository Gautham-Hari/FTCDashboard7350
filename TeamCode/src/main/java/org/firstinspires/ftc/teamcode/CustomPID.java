package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class CustomPID extends LinearOpMode {
    public DcMotorEx ringShooter;
    Servo ejectoSetoCus;
    public static PIDCoefficients testPID = new PIDCoefficients(700,4,100);
    ElapsedTime PIDTimer = new ElapsedTime();
    FtcDashboard dashboard;
    /*D is watching sudden change of position within the fraction time to give feedback. It does’t matter where the target is.
So ..
1. Set P for the smooth and sluggish curve.
2. Set I to help add power to P if it is taking too much time to get to the target.
3. Set D to push the power back if  I  is doing too much work and excessive acceleration.
P I’ll try to get to the target.
I   I’ll give you extra power because you are taking too much time.
D I’ll slow you down because I is giving too much power and detected sudden movement in short period of time
     */
    public static double VELOCITY=-1600;
    public void runOpMode(){
        ringShooter=hardwareMap.get(DcMotorEx.class,"ringShooter");
        ejectoSetoCus = hardwareMap.get(Servo.class,"ringEjector");
        ringShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dashboard = FtcDashboard.getInstance();
        waitForStart();
        while(opModeIsActive())
        {
            //telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());
            TelemetryPacket packet = new TelemetryPacket();
            if(gamepad1.right_trigger > 0.5)
            {
                ejectoSetoCus.setPosition(1);
            }
            else
            {
                ejectoSetoCus.setPosition(0);
            }
            packet.put("AbsoluteVelocity",Math.abs(VELOCITY));
            packet.put("ShooterVelocity",Math.abs(ringShooter.getVelocity()));
            runShooterMotor(VELOCITY);
            dashboard.sendTelemetryPacket(packet);
        }
    }
    void runShooterMotor(double velo){
        ringShooter.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,testPID);
        ringShooter.setVelocity(velo);
    }

}
