package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@TeleOp(name="Telepop", group="Telepop")
//@Disabled
public class Telepop extends LinearOpMode {

    //Железо
    private DcMotor m1, m2, m3, m4, m5, m6, m7, m8, led;
    public DistanceSensor r1, r2;
    private Servo s1, s2, s3, s4;
    private BNO055IMU imu;
    private DigitalChannel touch;

    //Переменные моторов
    private double zm1, zm2, zm3, zm4, zm5, zm6, zm7, zm8;
    private double zs1 = 0.74;

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
    private ElapsedTime runtime = new ElapsedTime();
    private double threshc = 0.02;

    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt"); //Файл с отклонением угла в конце автономки от начального угла

    //Гироскоп
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Инициализируем железо
    public void initC() {
        //Инициализация
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");
        m5 = hardwareMap.get(DcMotor.class, "m5");

        s1 = hardwareMap.get(Servo.class, "s1");
        s1.setPosition(0.72);
        initIMU();

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

        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {

                    m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    LastAngle = Double.parseDouble(ReadWriteFile.readFile(angleFile).trim()); //Отклонение угла от начального

                    while (!isStopRequested() & opModeIsActive()) {

                        //ТЕЛЕЖКА

                        //Угол выравнивания
                        LastAngle = !reinit ? LastAngle : 0;

                        //Выравнивание тележки
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        if (gamepad1.a == true) {
                            vyr = ((angles.firstAngle + LastAngle + 90) / 25);
                        }
                        else {
                            vyr = 0;
                        }

                        //Коэффицент скорости робота
                        a = gamepad1.left_trigger < 0.15 ? 0.5 : 1;

                        //Поворот
                        turn = gamepad1.right_stick_x;

                        //Мощность моторов тележки
                        zm1 = Range.clip((-gamepad1.left_stick_y - gamepad1.left_stick_x - turn - vyr) * a, -1, 1);
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip((-gamepad1.left_stick_y + gamepad1.left_stick_x - turn - vyr) * a, -1, 1);
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = Range.clip((gamepad1.left_stick_y + gamepad1.left_stick_x - turn - vyr) * a, -1, 1);
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip((gamepad1.left_stick_y - gamepad1.left_stick_x - turn - vyr) * a, -1, 1);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }

                        //Мощность телескопа
                        if (gamepad2.left_stick_y < 0.05) {
                            a_telescope = 0.8;
                        } else if (gamepad2.left_stick_y > 0.05) {
                            a_telescope = 0.25;
                        } else {
                            a_telescope = 0;
                        }

                        zm5 = Range.clip(gamepad2.left_stick_y * a_telescope, -1, 1);

                        //Захват конусов
                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;

                        //Переключение режимов Автоматический-Ручной
                        if (gamepad2.back == true && moment_diff_switch > 200) {
                            if (auto_mode == false) {
                                auto_mode = true;
                            } else {
                                auto_mode = false;
                            }
                            last_moment_switch = runtime.milliseconds();
                        }

                        //Ручной захват
                        if (gamepad2.x == true && moment_diff_serv > 200) {
                            if (pos_servoscop == false) {
                                zs1 = 0.66;
                                pos_servoscop = true;
                            } else {
                                zs1 = 0.74;
                                pos_servoscop = false;
                            }
                            last_moment_serv = runtime.milliseconds();
                        }

                        //Состояние скорости робота
                        if (a == 0.5) {
                            telespeed = "стабильная";
                        } else {
                            telespeed = "форсаж";
                        }

                        //Состояние захвата
                        if (auto_mode == false) {
                            telemode = "ручной";
                        } else {
                            telemode = "автозахват";
                        }

                        //Состояние ограничений
                        if (free_mode == false) {
                            telestable = "стабилен";
                        } else {
                            telestable = "без ограничений";
                        }

                        //Состояние захвата
                        if (pos_servoscop == false) {
                            teleservo = "неактивен";
                            zs1 = 0.66;
                            s1.setPosition(zs1);
                        } else {
                            teleservo = "активен";
                            zs1 = 0.72;
                            s1.setPosition(zs1);
                        }

                        //Нажатие на датчик касания
                        if (touch.getState() == true) {
                            pressed = "не нажат";
                        } else {
                            pressed = "нажат";
                        }

                    }

                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();
                }
            }
            public void start_c() {
                if (c == null) {
                    c = new Thread(this, "Calc thread");
                    c.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск подпроцессов
        CalcThread C1 = new CalcThread();
        C1.start_c();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            m1.setPower(zm1);
            m2.setPower(zm2);
            m3.setPower(zm3);
            m4.setPower(zm4);
            m5.setPower(zm5);

            if (gamepad1.y) { initIMU(); reinit = true; } //Обнуление гироскопа

            telemetry.addData("Состояние захвата", teleservo);
            telemetry.addData("Энкодер телескопа", m5.getCurrentPosition());
            telemetry.addData("Датчик захвата", pressed);
            telemetry.addData("Режим захвата", telemode);
            telemetry.addData("Режим", telestable);
            telemetry.addData("Скорость", telespeed);
            telemetry.addData("Энкодер 1", m1.getCurrentPosition());
            telemetry.addData("Энкодер 2", m2.getCurrentPosition());
            telemetry.addData("Энкодер 3", m3.getCurrentPosition());
            telemetry.update();

        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, Double.toString(angles.firstAngle));
    }
}