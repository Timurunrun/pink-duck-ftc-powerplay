package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoBlue30pts", group="AutoBlue")
//@Disabled
public class AutoBlue30pts extends LinearOpMode {

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

        //Отъезжаем вбок
        bot.dvizh(0.65, -71, 0, -90, 3.5, false);

        //Поворачиваемся к стопке конусов
        bot.rotate_rough(0);
        bot.rotate_rough(45);
        bot.rotate(90);

        //Поддерживаем уровень телескопа
        bot.tele(1000);

        //Подъезжаем к стопке конусов
        bot.dvizh(0.4, 0, 222, 90, 2, true);

        //Берём конус
        bot.tele(240);
        sleep(200);
        bot.s1.setPosition(0.73);
        sleep(300);

        //Поднимаем стрелу
        bot.tele(1000);

        //Отъезжаем от стопки конусов
        bot.dvizh(0.65, 0, -159, 90, 4.5, false);

        //Подъезжаем к высокому узлу
        bot.dvizh(0.5, -71, -159, 90, 4.5, false);
        bot.tele(2262);
        bot.dvizh(0.4, -71, -129, 90, 1.5, false);

        //Ставим конус
        sleep(300);
        bot.s1.setPosition(0.66);
        sleep(300);

        //Отъезжаем
        bot.tele(2262);
        bot.dvizh(0.7, -71, -159, 90, 3, false);
        bot.tele(0);

        //Паркуемся
        if (bot.baza == 1) {
            bot.dvizh(0.8, 225, -159, 90, 1.5, false);
            bot.dvizh(0.8, 0, 390, 90, 1, true);
        }
        if (bot.baza == 2) {
            bot.dvizh(0.8, 225, -159, 90, 1.5, false);
            bot.dvizh(0.8, 0, 180, 90, 1, true);
        }
        if (bot.baza == 3) {
            bot.dvizh(0.8, 225, -159, 90, 1.5, false);
        }

        bot.writeAngle();

    }
}