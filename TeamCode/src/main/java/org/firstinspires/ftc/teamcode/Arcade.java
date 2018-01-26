package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Manual with Arcade Drive
 */

@TeleOp(name = "Arcade")
public class Arcade extends LinearOpMode {
    boolean topServo = false;
    boolean botServo = false;
    boolean relServo = false;
    boolean isIn     = false;
    boolean spat     = false;
    boolean over     = false;
    boolean rkao     = false;
    double lPow = 0;
    double rPow = 0;
    double flStr;
    double frStr;
    double blStr;
    double brStr;
    double power = 0;
    double turn = 0;
    double FL, FR, BL, BR;

    hMap robot = new hMap();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.color_sensor.enableLed(false);
        robot.armServo.setPosition(robot.UP_JARM_POS);


        robot.botServL.setPosition(robot.START_CHOP_POS_A);
        robot.botServR.setPosition(robot.START_CHOP_POS_B);
        robot.topServL.setPosition(robot.START_CHOP_POS_B + 0.1);
        robot.topServR.setPosition(robot.START_CHOP_POS_A - 0.4);
        robot.intake.setPosition(robot.START_INTAKE_POS);
        robot.in = false;
        robot.tChop = false;
        robot.bChop = false;
        robot.relic.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {

            servo();
            intake();
            spatula();
            relic();

            if(gamepad1.x) {
                robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            power = scaleInput(Range.clip(-gamepad1.right_stick_y, -1, 1));
            turn = scaleInput(Range.clip(-gamepad1.left_stick_x, -1, 1));

            FL = power + turn;
            BL = power + turn;
            FR = power - turn;
            BR = power - turn;

            if (gamepad1.right_bumper) {
                FL /= 2;
                BL /= 2;
                FR /= 2;
                BR /= 2;
            } else if(gamepad1.dpad_right) {
                FL = -0.5;
                BL = -0.5;
                FR = 0.5;
                BR = 0.5;
            } else if(gamepad1.dpad_left) {
                FL = 0.5;
                BL = 0.5;
                FR = -0.5;
                BR = -0.5;
            }

            robot.frontLeft.setPower(FL);
            robot.backLeft.setPower(BL);
            robot.frontRight.setPower(FR);
            robot.backRight.setPower(BR);

            if(gamepad2.dpad_right) {
                robot.winch(0.7);
            } else if(gamepad2.dpad_left) {
                robot.winch(-0.4);
            } else {
                robot.winch(0);
            }

            if(gamepad2.dpad_up) {
                //robot.pitch(0.6);
            } else if(gamepad2.dpad_down) {
                //robot.pitch(0);
            } else {
                //robot.pitch(0.05);
            }


            telemetry.addData("Motors", "FL (%.2f), FR (%.2f), BL (%.2f), BR (%.2f)", FL, FR, BL, BR);
            telemetry.addData("Servos", "TL (%.2f), TR (%.2f), BL (%.2f), BR (%.2f)", robot.topServL.getPosition(), robot.topServR.getPosition(), robot.botServL.getPosition(), robot.botServR.getPosition());
            telemetry.addData("FL", robot.frontLeft.getCurrentPosition());
            telemetry.addData("BL", robot.backLeft.getCurrentPosition());
            telemetry.addData("FR", robot.frontRight.getCurrentPosition());
            telemetry.addData("BR", robot.backRight.getCurrentPosition());
            telemetry.addData("Slow", gamepad1.left_bumper);
            telemetry.addData("Spatula", robot.spatula);
            telemetry.addData("lSpat", robot.lSpat.getCurrentPosition());
            telemetry.addData("rSpat", robot.rSpat.getCurrentPosition());
            telemetry.update();
        }
    }

    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }


    void servo() {

        if (!robot.bChop) {
            if (!botServo && gamepad2.b) {
                robot.chop("GRAB");
            }
        } else {
            if (!botServo && gamepad2.b) {
                robot.chop("OPEN");
            }
        }

        /*if (!robot.tChop) {
            if (!topServo && gamepad2.x) {
                robot.topServL.setPosition(robot.GRAB_CHOP_POS_B - 0.2);
                robot.topServR.setPosition(robot.GRAB_CHOP_POS_A);
                robot.tChop = true;
            }
        } else {
            if (!topServo && gamepad2.x) {
                robot.topServL.setPosition(robot.START_CHOP_POS_B - 0.1);
                robot.topServR.setPosition(robot.START_CHOP_POS_A - 0.1);
                robot.tChop = false;
            }
        }*/

        botServo = gamepad2.b;
        //topServo = gamepad2.x;
    }

    void intake() {
        lPow = Range.clip(-gamepad2.left_stick_y, -0.7, 0.7);
        rPow = Range.clip(-gamepad2.right_stick_y, -0.7, 0.7);

        robot.vexL.setPower(-lPow);
        robot.vexR.setPower(rPow);

        if(!robot.in) {
            if(!isIn && gamepad2.right_bumper) {
                robot.intake.setPosition(robot.FINAL_INTAKE_POS);
                robot.in = true;
            }
        } else {
            if(!isIn && gamepad2.right_bumper) {
                robot.intake.setPosition(robot.START_INTAKE_POS);
                robot.in = false;
            }
        }

        isIn = gamepad2.right_bumper;
    }

    void spatula() {
        if(!robot.spatula) {
            if(!spat && gamepad2.a) {
                robot.chop("OPEN");
                robot.lSpat.setTargetPosition(robot.DOWN_SPAT_POS);
                robot.rSpat.setTargetPosition(robot.DOWN_SPAT_POS);
                robot.lSpat.setPower(0.4);
                robot.rSpat.setPower(0.4);
                robot.spatula = true;
                sleep(400);
                robot.chop("GRAB");
                robot.ov = false;
            }
        } else {
            if(!spat && gamepad2.a) {
                robot.intake.setPosition(robot.START_INTAKE_POS);
                robot.in = false;
                robot.chop("GRAB");
                robot.lSpat.setTargetPosition(robot.UP_SPAT_POS);
                robot.rSpat.setTargetPosition(robot.UP_SPAT_POS);
                robot.lSpat.setPower(-0.7);
                robot.rSpat.setPower(-0.7);
                robot.spatula = false;
                robot.ov = false;
            }
        }
        spat = gamepad2.a;

        if(!robot.ov) {
            if (!over && gamepad2.y && !robot.spatula) {
                robot.intake.setPosition(robot.START_INTAKE_POS);
                robot.in = false;
                robot.chop("GRAB");
                robot.lSpat.setTargetPosition(robot.OVER_SPAT_POS);
                robot.rSpat.setTargetPosition(robot.OVER_SPAT_POS);
                robot.lSpat.setPower(-0.7);
                robot.rSpat.setPower(-0.7);
                robot.spatula = true;
                robot.ov = true;
            }
        } else {
            if(!over && gamepad2.y) {
                robot.lSpat.setTargetPosition(robot.UP_SPAT_POS);
                robot.rSpat.setTargetPosition(robot.UP_SPAT_POS);
                robot.lSpat.setPower(0.3);
                robot.rSpat.setPower(0.3);
                robot.spatula = true;
                robot.ov = false;
            }
        }
        over = gamepad2.y;

        if(!robot.rko) {
            if(!rkao && gamepad2.left_trigger != 0) {
                robot.chop("GRAB");
                robot.lSpat.setTargetPosition(robot.RKO_SPAT_POS);
                robot.rSpat.setTargetPosition(robot.RKO_SPAT_POS);
                robot.lSpat.setPower(-0.7);
                robot.rSpat.setPower(-0.7);
                robot.ov = false;
                robot.spatula = true;
            }
        } else {
            if(!rkao && gamepad2.left_trigger != 0) {
                robot.chop("GRAB");
                robot.lSpat.setTargetPosition(robot.DOWN_SPAT_POS);
                robot.rSpat.setTargetPosition(robot.DOWN_SPAT_POS);
                robot.lSpat.setPower(0.3);
                robot.rSpat.setPower(0.3);
                robot.spatula = false;
                robot.ov = false;
            }
        }

        if(gamepad2.left_trigger!= 0) {
            rkao = true;
        } else {
            rkao = false;
        }
    }

    void relic() {
        if(!robot.rel) {
            if(!relServo && gamepad2.x){
                robot.relic.setPosition(0.3);
                robot.rel = true;
            }
        } else {
            if(!relServo && gamepad2.x) {
                robot.relic.setPosition(0.5);
                robot.rel = false;
            }
        }
        relServo = gamepad2.x;
    }
}
