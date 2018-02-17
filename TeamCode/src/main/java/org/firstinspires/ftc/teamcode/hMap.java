package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
    public DcMotor rSpat       ;

    //Relic\
    public DcMotor winch       ;

    /* Servos */

    //Chopstick Servos
    public Servo topServL    ;
    public Servo topServR    ;
    public Servo botServL    ;
    public Servo botServR    ;

    //Intake
    public DcMotor inL      ;
    public DcMotor inR      ;
    public Servo intake     ;

    //Sensor Arm Servo, for jewel arm
    public Servo armServo    ;
    public Servo jarmEXT     ;

    //Relic Servo
    public Servo relic       ;
    public Servo wrist       ;

    /* Sensors */
    public ColorSensor color_sensor;
    public DistanceSensor distance_sensor;
    public AnalogInput ods_sensor_front;
    public AnalogInput ods_sensor_back;

    //Values for encoders and servos
    public static final double START_CHOP_POS_A  = 0.9;
    public static final double START_CHOP_POS_B  = 0.1;
    public static final double GRAB_CHOP_POS_A   = 1;
    public static final double GRAB_CHOP_POS_B   = 0;
    public static final double START_INTAKE_POS  = 0;
    public static final double FINAL_INTAKE_POS  = 1;

    //Start and end positions for spatula
    public static final int OVER_SPAT_POS = -900;
    public static final int UP_SPAT_POS = -760;
    public static final int DOWN_SPAT_POS = 0;
    public static final int RKO_SPAT_POS = -1600;

    //Start and end positions for the jewel arm
    public static final double UP_JARM_POS = 0.1;
    public static final double DOWN_JARM_POS = 0.75;
    public static final double MID_JARM_POS = 0.57;


    //boolean for functions
    public boolean tChop;
    public boolean bChop;
    public boolean in   ;
    public boolean spatula = false;
    public boolean ov   ;
    public boolean rel = true ;
    public boolean rko  ;

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

        frontRight.setDirection(DcMotor.Direction.REVERSE)            ;
        backRight .setDirection(DcMotor.Direction.REVERSE)            ;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        //Spatula
        rSpat           = hwMap.get(DcMotor.class, "rSpat"  )           ;

        rSpat.setDirection(DcMotor.Direction.REVERSE)                               ;
        rSpat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSpat.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Relic
        winch           = hwMap.get(DcMotor.class, "winch"  )           ;
        relic           = hwMap.get(Servo.class,   "relic"  )           ;
        wrist           = hwMap.get(Servo.class,   "wrist"  )           ;

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
        inL             = hwMap.get(DcMotor.class,"inL"     )          ;
        inR             = hwMap.get(DcMotor.class,"inR"     )          ;

        /* Sensors */
        color_sensor    = hwMap.get(ColorSensor.class, "color_sensor"      );
        distance_sensor = hwMap.get(DistanceSensor.class, "distance_sensor");
        ods_sensor_front= hwMap.get(AnalogInput.class, "ods_sensor_front"  );
        ods_sensor_back = hwMap.get(AnalogInput.class, "ods_sensor_back"   );
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

    public void chop(String command) {
        if(command == "OPEN") {
            botServL.setPosition(START_CHOP_POS_A);
            botServR.setPosition(START_CHOP_POS_B);
            topServL.setPosition(START_CHOP_POS_B + 0.1);
            topServR.setPosition(START_CHOP_POS_A - 0.4);
            bChop = false;
        } else if(command == "GRAB") {
            botServL.setPosition(GRAB_CHOP_POS_A);
            botServR.setPosition(GRAB_CHOP_POS_B);
            topServL.setPosition(GRAB_CHOP_POS_B + 0.1);
            topServR.setPosition(GRAB_CHOP_POS_A - 0.4);
            bChop = true;
        }
    }

    public void winch(double power) {
        winch.setPower(power);
    }

}
