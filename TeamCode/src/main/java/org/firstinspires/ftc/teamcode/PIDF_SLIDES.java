package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="PIDF_SLIDES", group="LinearOpMode")
@Config
public class PIDF_SLIDES extends LinearOpMode {
    public static double kp = 0.02;
    public static double ki = 0;
    public static double kd = 0.0001;
    public static double kf = 0.2;
    public static double targetpos = 0;
    ElapsedTime timer = new ElapsedTime();

    public static DcMotorEx leftslide;
    public static DcMotorEx rightslide;

    double integralsum = 0;
    double lasterror = 0;




    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
        rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");

        leftslide.setDirection(DcMotorEx.Direction.REVERSE);
        rightslide.setDirection(DcMotorEx.Direction.FORWARD);

        leftslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightslide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while(opModeIsActive()) {
            double power;
            power = PIDF_Control(targetpos, leftslide.getCurrentPosition());



            leftslide.setPower(power);
            rightslide.setPower(power);

            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("power", power);
            packet.put("targetpos", targetpos);
            packet.put("leftslidepos", leftslide.getCurrentPosition());
            packet.put("rightslidepos", rightslide.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("power", power);
            telemetry.addData("targetpos", targetpos);
            telemetry.addData("leftslidepos", leftslide.getCurrentPosition());
            telemetry.addData("rightslidepos", rightslide.getCurrentPosition());
            telemetry.update();

        }



    }




        public double PIDF_Control(double reference, double state)
        {
            double error = reference-state;
            integralsum += error * timer.seconds();
            double derivative = (error-lasterror) / timer.seconds();
            lasterror = error;
            timer.reset();
            double output = (error*kp) + (derivative * kd) + (integralsum * ki) + kf;
            return output;

        }



}
