package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Base64;

public class Arm {
    public static double kpA = 0.002;
    public static double kiA = 0;
    public static double kdA = 0.0001;
    public static double kfA = 0.3;
    public static double targetposA = 0;
    ElapsedTime timerA = new ElapsedTime();
    double integralsumA = 0;
    double lasterrorA = 0;
    private DcMotorEx leftmotor;
    private DcMotor rightmotor;
    public static double EncoderSpecimenHeight = 400;




    public Arm(HardwareMap hardwareMap) {
        leftmotor = hardwareMap.get(DcMotorEx.class, "leftmotor");
        rightmotor = hardwareMap.get(DcMotorEx.class, "rightmotor");
        leftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftmotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightmotor.setDirection(DcMotorEx.Direction.FORWARD);

    }

    public class RotateUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            targetposA = 400;
            double power = PIDF_ControlA(targetposA, leftmotor.getCurrentPosition());
            leftmotor.setPower(power);
            rightmotor.setPower(power);
            return false;


        }
    }

        public class RotateDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetposA = 200;
                double power = PIDF_ControlA(targetposA, leftmotor.getCurrentPosition());
                leftmotor.setPower(power);
                rightmotor.setPower(power);
                return false;       

            }
        }

        public Action rotatedown() {
            return new RotateDown();
        }


        public Action rotateup() {
            return new RotateUp();
        }


        public class SpecimenHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                double power = PIDF_ControlA(EncoderSpecimenHeight, leftmotor.getCurrentPosition());
                leftmotor.setPower(power);
                rightmotor.setPower(power);
                return true;
            }
        }

        public Action ArmToSpecimenHeight() {
            return new SpecimenHeight();
        }


        public double PIDF_ControlA(double reference, double state) {
            double error = reference - state;
            integralsumA += error * timerA.seconds();
            double derivative = (error - lasterrorA) / timerA.seconds();
            lasterrorA = error;
            timerA.reset();
            double output = (error * kpA) + (derivative * kdA) + (integralsumA * kiA);
            return output;


        }


    }


