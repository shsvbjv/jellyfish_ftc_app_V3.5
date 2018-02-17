package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


import java.util.Locale;


//For testing distance moved
/**
 * Created by Feranno and Kyle on 9/23/17. 123
 */

@Autonomous(name = "AutoTester")
public class AutoTester extends LinearOpMode {

    //heading for gyro
    double heading;
    Orientation angles;


    VuforiaLocalizer vuforia;

    hMap robot = new hMap();

    //diameter of mecanum wheels = 4in
    //Circumference = 12.5663706144in
    //1 revolution=7 encoder values
    //1 rev = 12.56637036144in = 1.0471975512ft or 12.5663706144in
    int rev = 1120;
    int winchrev = 560;
    boolean found = false;
    String cryptobox_column;
    boolean spat = true;
    boolean over = false;
    boolean botServo = false;
    int drive = 0;
    int turn;


    @Override
    public void runOpMode() throws InterruptedException {

        // Set up our telemetry dashboard
        composeTelemetry();

        robot.init(hardwareMap);

        setHeadingToZero();

        robot.color_sensor.enableLed(true);

        waitForStart();

        gyroToGo(270);

        sleep(500);

        gyroToGo(359);
    }


    String format (OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



    //------------------------------------------------------------------------------------------------------------------------------
    //Driving Power Functions

    //distance=rate*duration duration=distance/rate
    //power drives forward, -power drives backward
    void VerticalDrive(double power) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }


    public void rotateRight(double power) {
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
    }
    public void rotateLeft(double power) {
        rotateRight(-power);
    }



    //------------------------------------------------------------------------------------------------------------------------------
    //Encoder Functions


    void VerticalDriveDistance(double power, int distance) throws InterruptedException {
        //reset encoders
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        VerticalDrive(power);

        if(distance > 0) {
            while (robot.frontLeft.getCurrentPosition() < distance && robot.frontRight.getCurrentPosition() < distance && robot.backLeft.getCurrentPosition() < distance && robot.backRight.getCurrentPosition() < distance) {
            }
        } else {
            while (robot.frontLeft.getCurrentPosition() > distance && robot.frontRight.getCurrentPosition() > distance && robot.backLeft.getCurrentPosition() > distance && robot.backRight.getCurrentPosition() > distance) {
            }
        }

        StopDriving();
    }


    void RotateDistanceRight(double power, int distance) throws InterruptedException {
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

    void RotateDistanceLeft(double power, int distance) throws InterruptedException {
        RotateDistanceRight(-power,-distance);
    }
    //------------------------------------------------------------------------------------------------------------------------------
//rotate using gyro Functions
    void StopDriving() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }
    public void waitUntilStable() throws InterruptedException {
        sleep(1000);
    }
    static class RangeResult {
        public double distance;
        public int position;
    }

    RangeResult inRange(double angle, double offset) {
        RangeResult range = new RangeResult();
        telemetry.update();
        double degree = heading;
        range.distance = Math.abs(angle - degree);
        double right = angle - offset;
        double left = angle + offset;
        if (right < 0) {
            right = right + 360;
            if (degree > right || degree < left) {
                range.position = 0;
            } else {
                range.position = 1;
            }
        } else if (left >= 360) {
            left = left - 360;
            if (degree < left || degree > right) {
                range.position = 0;
            } else {
                range.position = -1;
            }
        } else {
            if (degree > left) {
                range.position = 1;
            } else if (degree < right) {
                range.position = -1;
            } else {

                range.position = 0;
            }
        }
        if (range.distance > 180) {
            range.distance = range.distance - 180;
            if (range.position == -1) {
                range.position = 1;
            } else {
                range.position = -1;
            }
        }
        return range;
    }

    //turn left when -1
    //turn right when 1
    public void gyroToGo(double angle) throws InterruptedException {
        double angleoffset = 2;
        RangeResult rangeresult = inRange(angle, angleoffset);
        int position = rangeresult.position;
        int previousposition = rangeresult.position;
        double distance = rangeresult.distance;
        double previouspower = 0.5;
        double powerlevel = 0.5;
        while (true) {
            //update rangeresult
            rangeresult = inRange(angle, angleoffset);
            position = rangeresult.position;
            distance = rangeresult.distance;

            //adjust power level
            if (distance > 30) {
                powerlevel = 0.6;
            }
            else if(distance<10){
                powerlevel = 0.340;
            }
            else{
                powerlevel = 0.385;
            }

            //turn or stop
            if (position == 0) {
                StopDriving();
                waitUntilStable();
                rangeresult = inRange(angle, angleoffset);
                if (rangeresult.position == 0) {
                    break;
                }
            } else if (position == 1) {
                if (previouspower != powerlevel || previousposition != position) {
                    rotateRight(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                }
            } else if (position == -1) {
                if (previouspower != powerlevel || previousposition != position) {
                    rotateLeft(powerlevel);
                    previousposition = position;
                    previouspower = powerlevel;
                }
            }
        }
    }


//------------------------------------------------------------------------------------------------------------------------------
    //Winching functions

    void grabBottom() throws InterruptedException {
        robot.botServL.setPosition(robot.GRAB_CHOP_POS_A + 0.1);
        robot.botServR.setPosition(robot.GRAB_CHOP_POS_B);
        robot.bChop = true;
        sleep(300);
    }

    void startBottom() throws InterruptedException {
        robot.botServL.setPosition(robot.START_CHOP_POS_A);
        robot.botServR.setPosition(robot.START_CHOP_POS_B + 0.1);
        robot.bChop = false;
        sleep(300);
    }

    void grabTop() throws InterruptedException {
        robot.topServL.setPosition(robot.GRAB_CHOP_POS_B - 0.2);
        robot.topServR.setPosition(robot.GRAB_CHOP_POS_A);
        robot.tChop = true;
        sleep(300);
    }

    void startTop() throws InterruptedException {
        robot.topServL.setPosition(robot.START_CHOP_POS_B - 0.1);
        robot.topServR.setPosition(robot.START_CHOP_POS_A - 0.1);
        robot.tChop = false;
        sleep(300);
    }

//------------------------------------------------------------------------------------------------------------------------------
    //isJewelRed

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


//------------------------------------------------------------------------------------------------------------------------------


    void composeTelemetry() {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //robot.gravity = robot.imu.getGravity();
            }
        });

        /*telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });
                */

        telemetry.addLine()
                //rotating left adds to the heading, while rotating right makes the heading go down.
                //when heading reaches 180 it'll become negative and start going down.

                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {

                        //heading is a string, so the below code makes it a long so it can actually be used
                        heading = Double.parseDouble(formatAngle(robot.angles.angleUnit, robot.angles.firstAngle));
                        if(heading<0){
                            heading=heading+180;
                        }


                        return formatAngle(robot.angles.angleUnit, heading);

                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    //The two functions below are for gyro
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }



    void setHeadingToZero() {
        robot.gyroInit();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    void spatula() {
        if(!robot.spatula) {
            if(!spat && gamepad1.a) {
                robot.rSpat.setTargetPosition(robot.DOWN_SPAT_POS);
                robot.rSpat.setPower(0.4);
                robot.spatula = true;
                sleep(200);
                robot.botServL.setPosition(robot.GRAB_CHOP_POS_A);
                robot.botServR.setPosition(robot.GRAB_CHOP_POS_B);
                robot.topServL.setPosition(robot.GRAB_CHOP_POS_B + 0.1);
                robot.topServR.setPosition(robot.GRAB_CHOP_POS_A - 0.4);
                robot.bChop = true;
                if(robot.rSpat.getCurrentPosition() > -20) {
                    robot.botServL.setPosition(robot.START_CHOP_POS_A);
                    robot.botServR.setPosition(robot.START_CHOP_POS_B);
                    robot.topServL.setPosition(robot.START_CHOP_POS_B + 0.1);
                    robot.topServR.setPosition(robot.START_CHOP_POS_A - 0.4);
                    robot.bChop = false;
                }
            }
        } else {
            if(!spat && gamepad1.a) {
                robot.botServL.setPosition(robot.GRAB_CHOP_POS_A);
                robot.botServR.setPosition(robot.GRAB_CHOP_POS_B);
                robot.topServL.setPosition(robot.GRAB_CHOP_POS_B + 0.1);
                robot.topServR.setPosition(robot.GRAB_CHOP_POS_A - 0.4);
                robot.bChop = true;
                robot.rSpat.setTargetPosition(robot.UP_SPAT_POS);
                robot.rSpat.setPower(-0.7);
                robot.spatula = false;
            }
        }
        spat = gamepad1.a;

        if(!over && gamepad1.y) {
            robot.botServL.setPosition(robot.GRAB_CHOP_POS_A);
            robot.botServR.setPosition(robot.GRAB_CHOP_POS_B);
            robot.topServL.setPosition(robot.GRAB_CHOP_POS_B + 0.1);
            robot.topServR.setPosition(robot.GRAB_CHOP_POS_A - 0.4);
            robot.bChop = true;
            robot.rSpat.setTargetPosition(robot.OVER_SPAT_POS);
            robot.rSpat.setPower(-0.7);
            robot.spatula = false;
        }
        over = gamepad1.y;

        if(gamepad1.dpad_up) {
            robot.rSpat.setPower(-0.5);
            robot.spatula = false;
        } else if(gamepad1.dpad_down) {
            robot.rSpat.setPower(0.5);
            robot.spatula = false;
        }
    }

    void servo() {

        if (!robot.bChop) {
            if (!botServo && gamepad1.b) {
                robot.botServL.setPosition(robot.GRAB_CHOP_POS_A);
                robot.botServR.setPosition(robot.GRAB_CHOP_POS_B);
                robot.topServL.setPosition(robot.GRAB_CHOP_POS_B + 0.1);
                robot.topServR.setPosition(robot.GRAB_CHOP_POS_A - 0.4);
                robot.bChop = true;
            }
        } else {
            if (!botServo && gamepad1.b) {
                robot.botServL.setPosition(robot.START_CHOP_POS_A);
                robot.botServR.setPosition(robot.START_CHOP_POS_B);
                robot.topServL.setPosition(robot.START_CHOP_POS_B + 0.1);
                robot.topServR.setPosition(robot.START_CHOP_POS_A - 0.4);
                robot.bChop = false;
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

        botServo = gamepad1.b;
        //topServo = gamepad2.x;
    }

}