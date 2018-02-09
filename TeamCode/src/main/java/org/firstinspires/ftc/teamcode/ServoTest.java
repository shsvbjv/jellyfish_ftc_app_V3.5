package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Testing Motors
 */

@TeleOp (name = "Servo Test")
public class ServoTest extends LinearOpMode {
    boolean topServo = false;
    boolean botServo = false;

    double pos = 0.5;

    hMap robot = new hMap();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.x) {
                pos -= 0.1;
                robot.armServo.setPosition(pos);
                sleep(500);
            } else if(gamepad1.b) {
                pos += 0.1;
                robot.armServo.setPosition(pos);
                sleep(500);
            }

            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }

}
