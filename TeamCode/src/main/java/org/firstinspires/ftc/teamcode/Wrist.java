package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wrist {
    private Servo leftwrist;
    private Servo rightwrist;
    private ElapsedTime timer = new ElapsedTime();

    public Wrist(HardwareMap hardwareMap) {
        leftwrist = hardwareMap.get(Servo.class, "leftwrist");
        rightwrist = hardwareMap.get(Servo.class, "rightwrist");
    }

    public class WristUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftwrist.setPosition(0.75);
            rightwrist.setPosition(0.24);
            return false;
        }
    }

    public Action wristup() {
        return new WristUp();
    }


    public class WristDown implements Action
    {
        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            leftwrist.setPosition(1);
            rightwrist.setPosition(1);
            return false;
        }
    }
    public Action wristdown()
    {
        return new WristDown();
    }
}