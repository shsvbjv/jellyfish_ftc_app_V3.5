package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by oliversun on 10/7/17.
 */

public class hMap {

    //Gyro

    // The IMU sensor object
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;


    /* Motors */

    //Wheels
    public DcMotor frontLeft   ;
    public DcMotor frontRight  ;
    public DcMotor backLeft    ;
    public DcMotor backRight   ;

    //Spatula
    public DcMotor lSpat       ;
    public DcMotor rSpat       ;

    //Spatula Lift

    public DcMotor spatLeft;
    public DcMotor spatRight;

    /* Servos */

    //Chopstick Servos
    public Servo topServL    ;
    public Servo topServR    ;
    public Servo botServL    ;
    public Servo botServR    ;

    //Intake Servos
    public CRServo vexL      ;
    public CRServo vexR      ;
    public Servo intake      ;

    //Sensor Arm Servo, for jewel arm
    public Servo armServo    ;
    public Servo jarmEXT     ;

    /* Sensors */
    public ColorSensor color_sensor;

    //Values for encoders and servos
    public static final double START_CHOP_POS_A  = 0.1;
    public static final double START_CHOP_POS_B  = 0.9;
    public static final double GRAB_CHOP_POS_A   = 0.4;
    public static final double GRAB_CHOP_POS_B   = 0.6;
    public static final double START_INTAKE_POS  = 0;
    public static final double FINAL_INTAKE_POS  = 1;

    //Start and end positions for spatula
    public static final int UP_SPAT_POS = 1000;
    public static final int DOWN_SPAT_POS = 0;

    //Start and end positions for the jewel arm
    public static final double UP_JARM_POS = 0;
    public static final double DOWN_JARM_POS = 0.75;

    //boolean for functions
    public boolean tChop;
    public boolean bChop;
    public boolean in   ;
    public boolean spatula;

    HardwareMap hwMap;

    public hMap() {}

    public void init(HardwareMap ahwMap) {



        hwMap = ahwMap;

        /* Motors */

        //Wheels
        frontLeft       = hwMap.get(DcMotor.class, "frontLeft")       ;
        frontRight      = hwMap.get(DcMotor.class, "frontRight")      ;
        backLeft        = hwMap.get(DcMotor.class, "backLeft")        ;
        backRight       = hwMap.get(DcMotor.class, "backRight")       ;

        spatLeft = hwMap.get(DcMotor.class, "spatLeft");
        spatRight = hwMap.get(DcMotor.class, "spatRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE)            ;
        backRight .setDirection(DcMotor.Direction.REVERSE)            ;

        spatLeft.setDirection(DcMotor.Direction.REVERSE)            ;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //RUN_TO_POSITION = set encoder value and motor will go there and hold
        spatRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spatLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        //Spatula
        lSpat           = hwMap.get(DcMotor.class, "lSpat"  )           ;
        rSpat           = hwMap.get(DcMotor.class, "rSpat"  )           ;

        rSpat.setDirection(DcMotor.Direction.REVERSE)                  ;


        /* Servos */

        //Chopsticks
        topServL        = hwMap.get(Servo.class,  "topServL")          ;
        topServR        = hwMap.get(Servo.class,  "topServR")          ;
        botServL        = hwMap.get(Servo.class,  "botServL")          ;
        botServR        = hwMap.get(Servo.class,  "botServR")          ;

        //Jarm
        armServo        = hwMap.get(Servo.class,  "armServo")          ;
        jarmEXT         = hwMap.get(Servo.class,  "jarmEXT" )          ;

        //Intake
        intake          = hwMap.get(Servo.class,  "intake"  )          ;
        vexL            = hwMap.get(CRServo.class,"vexL"    )          ;
        vexR            = hwMap.get(CRServo.class,"vexR"    )          ;

        /* Sensors */
        color_sensor    = hwMap.get(ColorSensor.class, "color_sensor");
    }

    public void gyroInit() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
}