package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Testing Motors
 */

@TeleOp (name = "Motor Test")
public class MotorTest extends LinearOpMode {
    hMap robot = new hMap();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.botServL.setPosition(robot.GRAB_CHOP_POS_A);
        robot.botServR.setPosition(robot.GRAB_CHOP_POS_B);
        robot.topServL.setPosition(robot.GRAB_CHOP_POS_B + 0.1);
        robot.topServR.setPosition(robot.GRAB_CHOP_POS_A - 0.4);

        waitForStart();

        while(opModeIsActive()) {
            robot.lSpat.setPower(gamepad1.left_stick_y/2);
            robot.rSpat.setPower(gamepad1.left_stick_y/2);

            telemetry.addData("lSpat", robot.lSpat.getCurrentPosition());
            telemetry.addData("rSpat", robot.rSpat.getCurrentPosition());
            telemetry.update();
        }
    }
}
