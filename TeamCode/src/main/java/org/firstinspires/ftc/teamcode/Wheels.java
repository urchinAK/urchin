package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wheels {
    private CRServo leftwheel;
    private CRServo rightwheel;
    private ElapsedTime timer = new ElapsedTime();

    public Wheels(HardwareMap hardwareMap) {
        leftwheel = hardwareMap.get(CRServo.class, "leftwheel");
        rightwheel = hardwareMap.get(CRServo.class, "rightwheel");
    }

    public class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            leftwheel.setPower(1);
            rightwheel.setPower(1);

            return false;
        }
    }


    public class Outake implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            timer.reset();
            if(timer.time()>3)
            {
                return false;
            }
            else {
                leftwheel.setPower(-1);
                rightwheel.setPower(1);
            }
            return false;

        }
    }

    public Action intake() {
        return new Intake();
    }

    public Action outake(){
        return new Outake();
    }
}