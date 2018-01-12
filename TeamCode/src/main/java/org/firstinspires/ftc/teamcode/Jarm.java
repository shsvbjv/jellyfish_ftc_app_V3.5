package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Testing Motors
 */

@Autonomous (name = "Jarm")
public class Jarm extends LinearOpMode {
    hMap robot = new hMap();

    boolean forward;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.armServo.setPosition(robot.UP_JARM_POS);

        robot.jarmEXT.setPosition(0.5);

        waitForStart();

        robot.armServo.setPosition(robot.DOWN_JARM_POS);

        sleep(1000);

        forward = isJewelRedFinal();

        if(forward) {
            robot.jarmEXT.setPosition(0);
            sleep(500);
            robot.armServo.setPosition(robot.UP_JARM_POS);
        } else {
            robot.jarmEXT.setPosition(1);
            sleep(200);
            robot.armServo.setPosition(robot.UP_JARM_POS);
            sleep(200);
            robot.jarmEXT.setPosition(0);
            sleep(500);
        }
    }

    public boolean isJewelRed() {
        if (robot.color_sensor.red() > robot.color_sensor.blue()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isJewelRedFinal() {
        int red = 0;
        int blue = 0;
        boolean isRed = false;

        for (int i = 0; i < 20; i++) {
            if (isJewelRed()) {
                red++;
            } else {
                blue++;
            }
        }

        if (red < blue) {
            isRed = false;
        } else if (blue < red) {
            isRed = true;
        }

        return isRed;

    }

}
