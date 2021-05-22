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
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

@Config
@TeleOp
public class NoPID extends LinearOpMode {
    public DcMotorEx ringShooter;
    Servo ejectoSetoCus;
   // public static PIDCoefficients testPID = new PIDCoefficients(700,4,100);
    ElapsedTime PIDTimer = new ElapsedTime();
    FtcDashboard dashboard;
    File velocityFile = new File("C:\\Users\\gauth_7ruqbi8\\Desktop\\ftc-dashboard-master\\ftc-dashboard-master\\TeamCode\\src\\main\\java\\org\\firstinspires\\ftc\\teamcode");


    int i=0;

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
                i++;
            }
            else
            {
                ejectoSetoCus.setPosition(0);
            }
            if (gamepad2.right_bumper) {
                NewTeleOpEjectSequence(3,VELOCITY,packet);
            }//

            packet.put("AbsoluteVelocity",VELOCITY);
            packet.put("ShooterVelocity",ringShooter.getVelocity());
            runShooterMotor(VELOCITY);
            dashboard.sendTelemetryPacket(packet);
        }
    }
    void runShooterMotor(double velo){
        ringShooter.setVelocity(velo);
    }
    public static void writeVelocityValues(double velo, int ringNum)
    {
        File file = AppUtil.getInstance().getSettingsFile("VelocityValues.txt");
        String value = "Velocity: "+velo;
        ReadWriteFile.writeFile(file,value);
    }

    public void NewTeleOpEjectSequence(int rings,double velo,TelemetryPacket packet){
        int i=1;
        boolean isejectDone;
        sleep(250);

        while(i<=rings && !gamepad2.left_bumper) {
            isejectDone=false;
            while (!isejectDone && !gamepad2.left_bumper) {
                //-1600-100
                if(ringShooter.getVelocity()<=(velo)) {
                    sleep(50);
                    ejectoSetoCus.setPosition(1);
                    if(ejectoSetoCus.getPosition()>0.7)
                    {
                        writeVelocityValues(ringShooter.getVelocity(),i);
                    }
                    if (ejectoSetoCus.getPosition() > 0.9) {
                        sleep(500);
                        while (!isejectDone && !gamepad2.left_bumper) {
                            ejectoSetoCus.setPosition(0);
                            if (ejectoSetoCus.getPosition() < 0.2) {
                                sleep(500);
                                isejectDone = true;
                            }
                        }
                    }
                }

            }
            i++;
        }
        telemetry.update();
        sleep(4000);
    }
    public int getAverageVelo(double time)
    {
        int sum;
    ElapsedTime timer = new ElapsedTime();
    timer.startTime();
    while(timer.seconds()<=time)
    {


    }
    return 0;

    }



}
