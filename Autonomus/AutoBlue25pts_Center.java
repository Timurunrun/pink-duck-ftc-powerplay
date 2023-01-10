package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoBlue25pts_Center", group="AutoBlue")
//@Disabled
public class AutoBlue25pts_Center extends LinearOpMode {

    public AutoMethods bot = new AutoMethods();

    @Override
    public void runOpMode() throws InterruptedException {

        bot.initC(this);
        bot.camStart(this);

        waitForStart();

        bot.start();
        bot.getPos();

        //Захватываем предзагруженный конус
        bot.s1.setPosition(0.73);

        //Едем к высокому узлу в центре
        bot.dvizh(0.85, -5, 466, 0, 5, false);

        //Поворачиваемся к высокому узлу
        bot.rotate(-90);

        //Поднимаем стрелу
        bot.tele(2262);

        //Подъезжаем к высокому узлу
        bot.dvizh(0.45, 0, 32, -90, 1.5, true);

        //Отпускаем конус
        sleep(300);
        bot.s1.setPosition(0.66);
        sleep(300);

        //Отъезжаем от высокого узла
        bot.dvizh(0.45, 0, 0, -90, 2.5, false);

        //Опускаем телескоп
        bot.tele(1000);

        bot.tele(0);

        //Паркуемся
        if (bot.baza == 1) {
            bot.dvizh(0.8, -340, 0, -90, 1.5, false);
            bot.dvizh(0.8, 0, 200, -90, 1, true);
        }
        if (bot.baza == 2) {
            bot.dvizh(0.8, -340, 0, -90, 1.5, false);
        }
        if (bot.baza == 3) {
            bot.dvizh(0.8, -340, 0, -90, 1.5, false);
            bot.dvizh(0.8, 0, -200, -90, 1, true);
        }

        bot.writeAngle();

    }
}