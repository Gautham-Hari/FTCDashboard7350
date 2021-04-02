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
public class NoPID extends LinearOpMode {
    public DcMotorEx ringShooter;
    Servo ejectoSetoCus;
   // public static PIDCoefficients testPID = new PIDCoefficients(700,4,100);
    ElapsedTime PIDTimer = new ElapsedTime();
    FtcDashboard dashboard;

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
            packet.put("AbsoluteVelocity",VELOCITY);
            packet.put("ShooterVelocity",ringShooter.getVelocity());
            runShooterMotor(VELOCITY);
            dashboard.sendTelemetryPacket(packet);
        }
    }
    void runShooterMotor(double velo){
        ringShooter.setVelocity(velo);
    }

}
