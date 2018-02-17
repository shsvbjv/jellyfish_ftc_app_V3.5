package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Testing Motors
 */

@Autonomous (name = "GlyphTest")
public class GlyphTest extends LinearOpMode {
    hMap robot = new hMap();
    int rev = 720;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.chop("OPEN");

        waitForStart();

        intake("ON");

        VerticalDriveDistance(1, 7*rev);

        sleep(1000);

        robot.chop("GRAB");

        intake("OFF");

        VerticalDriveDistance(-1, -7*rev);


    }

    void VerticalDrive(double power) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }


    void rotateRight(double power) {
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
    }

    void intake(String power) {
        if(power == "ON") {
            //robot.inL.setPower(0.7);
            //robot.inR.setPower(-0.7);
        } else if(power == "OFF") {
            //robot.inL.setPower(0);
            //robot.inR.setPower(0);
        }
    }

    void VerticalDriveDistance(double power, int distance) throws InterruptedException {
        //reset encoders
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setTargetPosition(distance);
        robot.frontRight.setTargetPosition(distance);
        robot.backLeft.setTargetPosition(distance);
        robot.backRight.setTargetPosition(distance);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        VerticalDrive(power);

        while (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
        }

        //StopDriving();
    }

    void RotateDistance(double power, int distance) throws InterruptedException {
        {
            //reset encoders
            robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.frontLeft.setTargetPosition(distance);
            robot.frontRight.setTargetPosition(-distance);
            robot.backLeft.setTargetPosition(distance);
            robot.backRight.setTargetPosition(-distance);

            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rotateRight(power);

            while (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
                //wait until robot stops
            }

            //          StopDriving();
        }
    }
}
