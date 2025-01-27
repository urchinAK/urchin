package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(name = "BlueSideSample", group = "Autonomous")
public class BlueSideSample extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(38, 62, Math.toRadians(270));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Pose2d CurrentPose = drive.pose;
        Slides slides = new Slides(hardwareMap);
        Wheels wheels = new Wheels(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Arm arm = new Arm(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(50, 50), Math.toRadians(225))
        .stopAndAdd(new SequentialAction(
                wrist.wristup(),
                new SleepAction(1),
                arm.rotateup(),
                new SleepAction(2),
                slides.liftup(),
                new SleepAction(1.7),
                wheels.outake(),
                new SleepAction(3),
                slides.liftdown(),
                new SleepAction(1),
                wrist.wristup(),
                new SleepAction(1),
                arm.rotatedown(),
                new SleepAction(2),
                wheels.outake())


      )
                .turnTo(Math.toRadians(-90))
       .stopAndAdd(new SequentialAction(
          wrist.wristup(),
                new SleepAction(1),
                slides.liftup(),
                new SleepAction(1),
                wrist.wristdown(),
                new SleepAction(1),
                wheels.intake(),
                new SleepAction(1),
                slides.liftdown(),
                new SleepAction(1),
                wrist.wristup(),
                new SleepAction(1),
                arm.rotateup(),
                new SleepAction(1),
                slides.liftup())


        );






        


        waitForStart();
        Action trajectoryActionChosen1 = tab1.build();
        Actions.runBlocking(trajectoryActionChosen1);



        if (isStopRequested()) return;
    }
}




