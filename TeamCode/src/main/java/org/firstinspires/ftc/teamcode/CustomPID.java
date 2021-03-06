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
    public static PIDFCoefficients testPID = new PIDFCoefficients(40,0.1,40,16);
    ElapsedTime PIDTimer = new ElapsedTime();
    FtcDashboard dashboard;
    double velo;
    /*D is watching sudden change of position within the fraction time to give feedback. It does’t matter where the target is.
So ..
1. Set P for the smooth and sluggish curve.
2. Set I to help add power to P if it is taking too much time to get to the target.
3. Set D to push the power back if  I  is doing too much work and excessive acceleration.
P I’ll try to get to the target.
I   I’ll give you extra power because you are taking too much time.
D I’ll slow you down because I is giving too much power and detected sudden movement in short period of time
     */
    public static double VELOCITY=-1800;
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
            if (gamepad2.right_bumper) {
                TeleOpEjectSequence(3);
            }

            packet.put("AbsoluteVelocity",VELOCITY);
            packet.put("ShooterVelocity",ringShooter.getVelocity());
            runShooterMotor(VELOCITY);
            dashboard.sendTelemetryPacket(packet);
        }
    }
    void runShooterMotor(double velo){
        ringShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,testPID);
        ringShooter.setVelocity(velo);
    }

    public void TeleOpEjectSequence(int rings){
        int i=1;
        boolean isejectDone;
        sleep(250);
        while(i<=rings && !gamepad2.left_bumper) {
            isejectDone=false;
            while (!isejectDone && !gamepad2.left_bumper) {
                if(ringShooter.getVelocity()<=(VELOCITY)) {
                    sleep(50);
                    ejectoSetoCus.setPosition(1);
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


    }





}
