package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@Disabled
public class AutoMethods extends LinearOpMode {

    public OpenCvCamera webcam;
    public boolean camError = false;
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode op;
    public int baza = 1;

    //Железо
    public DcMotor m1, m2, m3, m4, m5, m6, m7, led;
    public DistanceSensor r1, r2;
    public Servo s1, s2, s3, s4;
    private BNO055IMU imu;
    private DigitalChannel touch;

    //Переменные моторов
    private double zm1, zm2, zm3, zm4, zm5, zm6, zm7;
    private double zs1 = 0.71;

    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0, last_moment_auto_down = 0.0, last_moment_auto_sides = 0.0, last_moment_auto_up = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free, moment_diff_auto_down, moment_diff_auto_sides, moment_diff_auto_up;
    private boolean auto_mode = true, free_mode = false;
    private double a, a_telescope, vyr, turn;
    private int strela_req_level;
    private double strela_level;
    private Orientation angles;
    private String teleservo = "неактивен";
    private String telemode = "автозахват";
    private String telestable = "стабилен";
    private String telespeed = "стабильная";
    private String pressed = "не нажат";
    private boolean pos_servoscop = false;
    private boolean last_press_servoscop = false;
    private float dgr = 0;
    double LastAngle = 0;
    int telescopePos = 0;
    int rot_vyr;
    private double volt = 0;
    private boolean reinit = false;
    private boolean reverse = false;
    private boolean auto = false;
    private double threshc = 0.02;

    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt"); //Файл с позицией робота

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initIMU(OpMode op) {
        this.op = op;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Инициализируем железо
    public void initC(OpMode op) {
        this.op = op;
        m1 = op.hardwareMap.get(DcMotor.class, "m1");
        m2 = op.hardwareMap.get(DcMotor.class, "m2");
        m3 = op.hardwareMap.get(DcMotor.class, "m3");
        m4 = op.hardwareMap.get(DcMotor.class, "m4");
        m5 = op.hardwareMap.get(DcMotor.class, "m5");

        s1 = op.hardwareMap.get(Servo.class, "s1");
        initIMU(op);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touch = op.hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void camStart(OpMode op) {
        try {
            this.op = op;
            int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.openCameraDevice();
            webcam.setPipeline(new Detector(op.telemetry));
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        } catch (OpenCvCameraException e1) {camError = true;}
        catch (NullPointerException e2) {camError = true;}
    }

    public void camStop() {
        if (!camError) {webcam.stopStreaming();}
    }

    public void getPos() {
        if (!camError) {
            switch (Detector.getLocation()) {
                case BLUE:
                    baza = 1;
                    break;
                case YELLOW:
                    baza = 2;
                    break;
                case STRIPES:
                    baza = 3;
                    break;
            }
        }
    }

    //Запись угла в файл
    public void writeAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, String.valueOf(angles.firstAngle));
    }

    public void dvizh(double speed, double x, double y, double ang, double timeout, boolean reset) {

        double alpha, m1v, m2v, m3v, m4v, ugol, xv, yv, xc, yc, xl, yl;

        if (reset) {
            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        xc = m1.getCurrentPosition();
        yc = m2.getCurrentPosition();

        xl = x-xc;
        yl = y-yc;

        runtime.reset();

        while (opModeIsActive() && !isStopRequested() && (Math.abs(xl) > 1 || Math.abs(yl) > 1) && (runtime.milliseconds() - timeout * 1000) < 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            ugol = 360 + angles.firstAngle;

            alpha = 0;

            xl = x-xc;
            yl = y-yc;

            if (yl > 0) {
                if (xl >= 0) {
                    alpha = Math.atan(xl/yl);
                }
                if (xl < 0) {
                    alpha = 3.141592653589*2 - Math.abs(Math.atan(xl/yl));
                }
            }

            if (yl < 0) {
                if (x >= 0) {
                    alpha = 3.141592653589 - Math.abs(Math.atan(xl/yl));
                }
                if (xl < 0) {
                    alpha = 3.141592653589 + Math.atan(xl/yl);
                }
            }

            if (yl == 0) {
                if (xl >= 0) {
                    alpha = 3.141592653589/2;
                }
                if (xl < 0) {
                    alpha = -(3.141592653589/2);
                }
            }

            xv = Math.sin(alpha);
            yv = Math.cos(alpha);

            m1v = yv - xv;
            m2v = xv + yv;
            m3v = xv - yv;
            m4v = -xv - yv;

            xc = m1.getCurrentPosition();
            yc = m2.getCurrentPosition();

            vyr = (angles.firstAngle + ang) / 18;

            if (Math.abs(xl) < 175 && Math.abs(yl) < 175) {
                speed = 0.2 + (Math.abs(xl) + Math.abs(yl)) / 830;
            }

            m1.setPower((m1v - vyr) * speed);
            m2.setPower((m2v - vyr) * speed);
            m3.setPower((m3v - vyr) * speed);
            m4.setPower((m4v - vyr) * speed);

            //op.telemetry.addData("Угол альфа (в градусах)", Math.toDegrees(alpha));
            //op.telemetry.addData("Угол альфа (в радианах)", alpha);
            op.telemetry.addData("Энкодер оси x", m1.getCurrentPosition());
            op.telemetry.addData("Энкодер оси y", m2.getCurrentPosition());
            op.telemetry.addData("Энкодер стрелы", m5.getCurrentPosition());
            op.telemetry.update();
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        sleep(100);
    }

    public void dvizh_passive() {

        double x = 0;
        double y = 0;
        double speed = 0;

        double alpha, m1v, m2v, m3v, m4v, ugol, xv, yv, xc, yc, xl, yl;

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        xc = m1.getCurrentPosition();
        yc = m2.getCurrentPosition();

        xl = x-xc;
        yl = y-yc;

        while (opModeIsActive() && !isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            ugol = 360 + angles.firstAngle;

            alpha = 0;

            xl = x-xc;
            yl = y-yc;

            if (yl > 0) {
                if (xl >= 0) {
                    alpha = Math.atan(xl/yl);
                }
                if (xl < 0) {
                    alpha = 3.141592653589*2 - Math.abs(Math.atan(xl/yl));
                }
            }

            if (yl < 0) {
                if (x >= 0) {
                    alpha = 3.141592653589 - Math.abs(Math.atan(xl/yl));
                }
                if (xl < 0) {
                    alpha = 3.141592653589 + Math.atan(xl/yl);
                }
            }

            if (yl == 0) {
                if (xl >= 0) {
                    alpha = 3.141592653589/2;
                }
                if (xl < 0) {
                    alpha = -(3.141592653589/2);
                }
            }

            xv = Math.sin(alpha);
            yv = Math.cos(alpha);

            m1v = yv - xv;
            m2v = xv + yv;
            m3v = xv - yv;
            m4v = -xv - yv;

            xc = m1.getCurrentPosition();
            yc = m2.getCurrentPosition();

            vyr = angles.firstAngle / 18;

            //op.telemetry.addData("Угол альфа (в градусах)", Math.toDegrees(alpha));
            //op.telemetry.addData("Угол альфа (в радианах)", alpha);
            op.telemetry.addData("Энкодер оси x", m1.getCurrentPosition());
            op.telemetry.addData("Энкодер оси y", m2.getCurrentPosition());
            op.telemetry.addData("Энкодер стрелы", m5.getCurrentPosition());
            op.telemetry.update();
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        sleep(100);
    }

    public void rotate(double ugolok) {

        int vyr_znak;

        ugolok = -ugolok;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > 0.3) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            vyr = (angles.firstAngle - ugolok) / 150;
            vyr_znak = (int) (vyr/Math.abs(vyr));

            m1.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m2.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m3.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m4.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    public void rotate_rough(double ugolok) {

        int vyr_znak;

        ugolok = -ugolok;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > 1.5) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            vyr = (angles.firstAngle - ugolok) / 100;
            vyr_znak = (int) (vyr/Math.abs(vyr));

            m1.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
            m2.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
            m3.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
            m4.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    public void tele(double pos) {

        if (m5.getCurrentPosition() < pos) {
            while (opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() < pos) {
                m5.setPower(-0.8);
            }
            m5.setPower(0);
        }

        if (m5.getCurrentPosition() > pos) {
            while (opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() > pos) {
                m5.setPower(0.3);
            }
            m5.setPower(0);
        }
    }
}
