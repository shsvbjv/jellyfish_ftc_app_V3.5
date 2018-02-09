package org.firstinspires.ftc.teamcode;

/**
 * Created by oliversun on 11/16/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

     @TeleOp(name = "Vex")
     public class Vex extends LinearOpMode {
         private CRServo vexL;
         private CRServo vexR;

         double lPow = 0;
         double rPow = 0;

         public void runOpMode() throws InterruptedException {
             vexL = hardwareMap.crservo.get("vexL");
             vexR = hardwareMap.crservo.get("vexR");

             waitForStart();

             while(opModeIsActive()) {
                 lPow = Range.clip(-gamepad1.left_stick_y, -0.5, 0.5);
                 rPow = Range.clip(-gamepad1.right_stick_y, -0.5, 0.5);

                 vexL.setPower(lPow);
                 vexR.setPower(rPow);
             }
         }
     }

