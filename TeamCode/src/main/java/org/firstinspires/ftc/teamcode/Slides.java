    package org.firstinspires.ftc.teamcode;

    import androidx.annotation.NonNull;

    import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    import com.acmerobotics.roadrunner.Action;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.util.ElapsedTime;

    public class Slides {
        public static double kpS = 0.02;
        public static double kiS = 0;
        public static double kdS = 0.0001;
        public static double kfS = 0.2;
        public static double targetposS = 2150;
        ElapsedTime timerS = new ElapsedTime();
        double integralsumS = 0;
        double lasterrorS = 0;
        private DcMotorEx leftslide;
        private DcMotorEx rightslide;



        public Slides(HardwareMap hardwareMap) {
            leftslide = hardwareMap.get(DcMotorEx.class, "leftslide");
            rightslide = hardwareMap.get(DcMotorEx.class, "rightslide");
            leftslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightslide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rightslide.setDirection(DcMotorEx.Direction.FORWARD);
            leftslide.setDirection(DcMotorSimple.Direction.REVERSE);
            leftslide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rightslide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class LiftUp implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetposS = 2125;;
                double power = PIDF_ControlS(targetposS, leftslide.getCurrentPosition());
                leftslide.setPower(power);
                rightslide.setPower(power);
                return false;
            }
        }
        public class SpeciScoreHeight implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetposS = 1200;
                double power = PIDF_ControlS(targetposS, leftslide.getCurrentPosition());
                leftslide.setPower(power);
                rightslide.setPower(power);
                return true;


            }
        }
        public Action speciscoreheight()
        {
            return new SpeciScoreHeight();
        }


        public Action liftup() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                targetposS = 0;
                double power = PIDF_ControlS(targetposS, leftslide.getCurrentPosition());
                leftslide.setPower(power);
                rightslide.setPower(power);
                return false;
            }

        }
        public Action liftdown() {
            return new LiftDown();
        }

        public double PIDF_ControlS(double reference, double state)
        {
            double error = reference-state;
            integralsumS += error * timerS.seconds();
            double derivative = (error-lasterrorS) / timerS.seconds();
            lasterrorS = error;
            timerS.reset();
            double output = (error*kpS) + (derivative * kdS) + (integralsumS* kiS);
            return output;


        }

    }


