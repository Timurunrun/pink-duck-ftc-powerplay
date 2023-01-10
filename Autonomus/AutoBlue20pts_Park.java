package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoBlue20pts_Park", group="AutoBlue")
//@Disabled
public class AutoBlue20pts_Park extends LinearOpMode {

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

        //Паркуемся
        if (bot.baza == 1) {
            bot.dvizh(0.6, 0, 210, 0, 6, false);
            bot.dvizh(0.8, -190, 210, 0, 6, false);
        }
        if (bot.baza == 2) {
            bot.dvizh(0.6, 0, 210, 0, 6, false);
        }
        if (bot.baza == 3) {
            bot.dvizh(0.6, 0, 210, 0, 6, false);
            bot.dvizh(0.8, 190, 210, 0, 6, false);
        }

        bot.writeAngle();

    }
}