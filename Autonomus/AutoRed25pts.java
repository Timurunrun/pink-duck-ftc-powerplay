package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoRed25pts", group="AutoRed")
//@Disabled
public class AutoRed25pts extends LinearOpMode {

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

        //Едем к правому высокому узлу
        bot.dvizh(0.6, 0, 190, 0, 1.5, false);
        bot.dvizh(0.7, 296, 190, 0, 2, false);

        //Поднимаем стрелу
        bot.tele(2262);

        //Подъезжаем к высокому узлу
        bot.dvizh(0.45, 0, 30, 0, 1.5, true);

        //Отпускаем конус
        sleep(300);
        bot.s1.setPosition(0.66);
        sleep(300);

        //Отъезжаем от высокого узла
        bot.tele(2262);
        bot.dvizh(0.45, 0, 16, 0, 2.5, false);

        //Опускаем телескоп
        bot.tele(0);

        //Паркуемся
        if (bot.baza == 1) {
            bot.dvizh(0.7, -460, 16, 0, 3, false);
        }
        if (bot.baza == 2) {
            bot.dvizh(0.7, -300, 16, 0, 2, false);
        }
        if (bot.baza == 3) {
            bot.dvizh(0.7, -140, 16, 0, 1, false);
        }

        bot.writeAngle();

    }
}

