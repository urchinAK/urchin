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


@Autonomous(name="PIDF_ARM", group="LinearOpMode")
@Config
public class PIDF_ARM extends LinearOpMode {
    public static double kp = 0.002;
    public static double ki = 0;
    public static double kd = 0.0001;
    public static double kf = 0.3;
    public static double targetpos = 0;
    ElapsedTime timer = new ElapsedTime();

    public static double EncoderOffsetLeft = 0;
    public static double EncoderOffsetRight = 0;

    public static DcMotorEx leftmotor;
    public static DcMotorEx rightmotor;
    double integralsum = 0;
    double lasterror = 0;





    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        leftmotor = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightmotor = hardwareMap.get(DcMotorEx.class, "rightmotor");
        rightmotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightmotor.setDirection(DcMotorEx.Direction.FORWARD);

        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while(opModeIsActive()) {
            double power;
            power = PIDF_Control(targetpos, leftmotor.getCurrentPosition());
            leftmotor.setPower(power);
            rightmotor.setPower(power);





            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("powerL", power);
            packet.put("targetpos", targetpos);
            packet.put("leftslidepos", leftmotor.getCurrentPosition());
            packet.put("rightslidepos", rightmotor.getCurrentPosition());

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("powerL", power);
            telemetry.addData("targetpos", targetpos);
            telemetry.addData("leftmotorpos", leftmotor.getCurrentPosition()+EncoderOffsetLeft);
            telemetry.addData("rightmotorpos", rightmotor.getCurrentPosition()+EncoderOffsetRight);
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
