package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Created by Ferannow and Kyle on 9/23/17. 123
 */

@Autonomous(name = "AutoBlueBot")
public class AutoBlueBot extends LinearOpMode {

    //heading for gyro
    double heading;
    double temp;

    ElapsedTime runtime = new ElapsedTime();

    VuforiaLocalizer vuforia;

    hMap robot = new hMap();

    //diameter of mecanum wheels = 4in
    //Circumference = 12.5663706144in
    //1 revolution=7 encoder values
    //1 rev = 12.56637036144in = 1.0471975512ft or 12.5663706144in
    int rev = 1120;
    int run360=5476;
    int winchrev = 560;
    boolean forward;
    boolean found = false;
    String cryptobox_column;


    @Override
    public void runOpMode() throws InterruptedException {
        composeTelemetry();
        robot.init(hardwareMap);
        setHeadingToZero();
        robot.color_sensor.enableLed(true);

        robot.armServo.setPosition(robot.UP_JARM_POS);


//------------------------------------------------------------------------------------------------------------------------------
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        /*
         * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
         * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
         * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
         * web site at https://developer.vuforia.com/license-manager.
         *
         * Vuforia license keys are always 380 characters long, and look as if they contain mostly
         * random data. As an example, here is a example of a fragment of a valid key:
         *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
         * Once you've obtained a license key, copy the string from the Vuforia web site
         * and paste it in to your code onthe next line, between the double quotes.
         */
        parameters.vuforiaLicenseKey = "ATwJ9+j/////AAAAGWKRoGTF3EXjjiUONUpE/FEwHMBGsRsjSjnKHLRm/QkTZrfBTDWGmxaODJswltGeGHE/NewaAKjI9tFnnLg4uFGaQVAgYWNmHvi7RFMfMiQKKWXbwL6KjW7hFcPyZClckV+wfMPtW0EYe2if1IfwAx/C82Z2TqAbFLHWgz2QMf2h+LatQz5jgAJA+N46A+fNjDu4Ixf5VPiTL8Rffdho5FdLh0mWvrW7fnIjJvVfmHIaX+VSSRmWlK+rvmZN9fiD2Yi7jD99mArgXvQBq8fUBvUouzPNw5iRh1tiy8PiytQl0a39zXo9xpseGJ/HnpFDjklAvntMXQTIn2nl1bg9J9N3WZkEST4ymb+7CpgKYyp0";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        robot.chop("GRAB");

        waitForStart();

        runtime.reset();

        relicTrackables.activate();

        robot.jarmEXT.setPosition(0.63);

        robot.armServo.setPosition(robot.DOWN_JARM_POS);


//------------------------------------------------------------------------------------------------------------------------------
        //start Autonomous
        while (!found) {
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                cryptobox_column = vuMark.toString();
                found = true;
                telemetry.addData("VuMark", "%s visible", vuMark);
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            } else {
                telemetry.addData("VuMark", "not visible");
            }
            telemetry.update();
            if (runtime.seconds() > 1) {
                break;
            }
        }

        sleep(1000);

        forward = isJewelRed();

        if (forward) {
            robot.jarmEXT.setPosition(0);
            sleep(300);
            robot.armServo.setPosition(robot.MID_JARM_POS);
        } else {
            robot.jarmEXT.setPosition(1);
            sleep(500);
            robot.armServo.setPosition(robot.MID_JARM_POS);
        }

        sleep(500);

        VerticalDrive(0.2);


        while (Double.isNaN(robot.distance_sensor.getDistance(DistanceUnit.CM))) {
            telemetry.addData("Fencepost", robot.distance_sensor.getDistance(DistanceUnit.CM));
            telemetry.addData("DistanceFront", robot.ods_sensor_front.getVoltage());
            telemetry.addData("DistanceBack", robot.ods_sensor_back.getVoltage());
            telemetry.update();
        }

        StopDriving();

        robot.jarmEXT.setPosition(0.5);
        robot.armServo.setPosition(robot.UP_JARM_POS);

        sleep(300);

        if (cryptobox_column == "LEFT") {
            VerticalDriveDistance(0.3, rev / 3 + 70);
        } else if (cryptobox_column == "CENTER") {
            VerticalDriveDistance(0.3, rev);
        } else {
            VerticalDriveDistance(0.3, 4 * rev / 3 + 300);
        }

        sleep(400);

        telemetry.addData("DistanceFront", robot.ods_sensor_front.getVoltage());
        telemetry.addData("DistanceBack", robot.ods_sensor_back.getVoltage());
        telemetry.update();

        sleep(1000);

        RotateDistance(-0.7, -11*rev/9);

        VerticalDriveDistance(-0.5, -rev/2);
        robot.rSpat.setTargetPosition(robot.UP_SPAT_POS);
        robot.rSpat.setPower(-0.7);
        runtime.reset();
        while(robot.rSpat.isBusy()) {
            if(runtime.seconds() > 2) {
                break;
            }
        }
        robot.chop("OPEN");
        VerticalDriveDistance(-0.5, -rev);
        sleep(400);
        VerticalDriveDistance(0.3, rev/3);
        robot.rSpat.setTargetPosition(robot.DOWN_SPAT_POS);
        robot.rSpat.setPower(0.3);
        sleep(500);
        robot.chop("GRAB");
        VerticalDriveDistance(-0.3, -rev/3);
        VerticalDriveDistance(0.2, rev/4);
        /*sleep(500);
        robot.chop("OPEN");
        robot.inL.setPower(0.5);
        robot.inR.setPower(-0.5);
        VerticalDriveDistance(0.6, 3*rev);
        robot.chop("GRAB");
        VerticalDriveDistance(-0.6, -3*rev);
        robot.rSpat.setTargetPosition(robot.UP_SPAT_POS);
        robot.rSpat.setPower(-0.7);
        runtime.reset();
        while(robot.rSpat.isBusy()) {
            if(runtime.seconds() > 2) {
                break;
            }
        }
        robot.chop("OPEN");
        VerticalDriveDistance(-0.3, -rev/3);
        VerticalDriveDistance(0.2, rev/4);*/
    }
    //


    String format (OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



    //------------------------------------------------------------------------------------------------------------------------------
    //Driving Power Functions
    void StopDriving() {
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    //distance=rate*duration duration=distance/rate
    //power drives forward, -power drives backward
    void VerticalDrive(double power) {
        robot.frontLeft.setPower(power);
        robot.frontRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.backRight.setPower(power);
    }

    void rotateRight(double power) {
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);
        robot.frontRight.setPower(power);
        robot.backRight.setPower(power);
    }

    void rotateLeft(double power) {
        rotateRight(-power);
    }


    //------------------------------------------------------------------------------------------------------------------------------
    //Encoder Functions


    void VerticalDriveDistance(double power, int distance) throws InterruptedException {
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
            while (robot.frontLeft.getCurrentPosition() < distance &&
                    robot.frontRight.getCurrentPosition() < distance &&
                    robot.backLeft.getCurrentPosition() < distance &&
                    robot.backRight.getCurrentPosition() < distance) {
            }
        } else {
            while (robot.frontLeft.getCurrentPosition() > distance &&
                    robot.frontRight.getCurrentPosition() > distance &&
                    robot.backLeft.getCurrentPosition() > distance &&
                    robot.backRight.getCurrentPosition() > distance) {
            }
        }

        StopDriving();
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

            StopDriving();
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


    public void waitUntilStable() throws InterruptedException {
        telemetry.update();
        double degree = heading;
        double previousreading = 0;
        boolean stable = false;
        while (stable == false) {
            sleep(10);
            previousreading = heading;
            if (Math.abs(degree - previousreading) < 0.1) {
                stable = true;
            }
        }
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
        double angleoffset = 4;
        RangeResult rangeresult = inRange(angle, angleoffset);
        int position = rangeresult.position;
        int previousposition = rangeresult.position;
        double distance = rangeresult.distance;
        double previouspower = 0.5;
        double powerlevel = 0.5;
        double k=0.7;
        while (true) {
            //update rangeresult
            rangeresult = inRange(angle, angleoffset);
            position = rangeresult.position;
            distance = rangeresult.distance;

            //adjust power level
            if (distance > 40) {
                powerlevel = 0.7;
            }
            else{
                powerlevel = k-3;
            }

            //turn or stop
            if (position == 0) {
                StopDriving();
                waitUntilStable();
                rangeresult = inRange(angle, angleoffset);
                if (rangeresult.position == 0) {
                    break;
                }
                //position is left of heading, rotate right
            } else if (position == 1) {
                if (previouspower != powerlevel || previousposition != position) {
                    int deg= Math.round((float) (run360/360)*(float)(distance));
                    RotateDistance(powerlevel, deg);
                    previousposition = position;
                    previouspower = powerlevel;
                }
                //position is right of heading, rotate left
            } else if (position == -1) {
                if (previouspower != powerlevel || previousposition != position) {
                    int deg= Math.round((float) (run360/360)*(float)(distance));
                    RotateDistance(-powerlevel, -deg);                    previousposition = position;
                    previouspower = powerlevel;

                }
            }
        }
    }

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
                        if (heading < 0) {
                            heading = heading + 360;
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

//------------------------------------------------------------------------------------------------------------------------------
    //isJewelRed

    public boolean isJewelRed() {
        if (robot.color_sensor.red() > robot.color_sensor.blue()) {
            telemetry.addData("IsRed", true);
            telemetry.update();
            return true;
        } else {
            telemetry.addData("IsRed", false);
            telemetry.update();
            return false;
        }
    }

    public boolean isJewelRedFinal() {
        int red = 0;
        int blue = 0;
        boolean isRed = false;

        for (int i = 0; i < 5; i++) {
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

        telemetry.addData("IsRed", isRed);
        telemetry.update();
        return isRed;
    }
}