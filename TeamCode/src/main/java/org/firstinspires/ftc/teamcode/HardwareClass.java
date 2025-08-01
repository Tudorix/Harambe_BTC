package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HardwareClass {

    //Trash
    public int CAMERA_MAX_RIGHT = 0;
    public int CAMERA_MAX_LEFT = 0;
    public int ROBOT_MAX_LEFT = 0;
    public int ROBOT_MAX_RIGHT = 0;
    public double SPECIMEN_CLOSED = 0.53 , SPECIMEN_OPEN = 0.3;

    public DcMotorEx Push;

    public DcMotor FR, FL , BR , BL;
    public DcMotorEx LS , RS, Extendo;
    public double SLIDES_MAX_POWER = 1;

    /** SLIDES PID VALUES */

    public double IN_ROBOT = 0, HIGH_BASKET = 910 ,LOW_BASKET = 350, HIGH_S_BASKET = 800;
    public double PREP_SPECIMEN = 330;
    public double PLACE_SPECIMEN = 330;

    public double HATZ_SPECIMEN = 90;

    /** AUTO COORDS*/

    public double X = 0 , Y = 0, HEAD = 0;

    /**  EXTENDO PID VALUES  */

    public double IN = 0, MAX = 1000;
    public double CLOSE = 300;
    public ColorRangeSensor IntakeSensor;

    //Servo
    public Servo ClawIn , ClawOut , IntakeRotate, OuttakeRotate, MobiDick , Cam;
    public CRServo Sweep;
    // Intake claw
   public double SPEC_CLAW_OPEN = 0.25 ,SPEC_CLAW_CLOSE = 0.5;
    public double PIVOT_MAX_RIGHT = 0.3 , PIVOT_RIGHT_TOP = 0.4 , PIVOT_LEFT_TOP = 0.1 , PIVOT_PERPENDICULAR = 0.54;
    public double CAM_OUT = 0.47 ,CAM_IN = 0.13;


    public double MD_OUT = 0.13 ,MD_IN = 0.43;
    // Intake rotation
    public double INTAKE_ROTATION_DOWN = 0.95 , INTAKE_ROTATION_UP = 0.4 , INTAKE_ROTATION_PREP = 0.42,INTAKE_ROTATION_SEE = 0.39;
    public double OUTTAKE_ROTATION_TAKE = 0.1, OUTTAKE_ROTATION_PLACE = 1, OUTTAKE_ROTATION_VERTICAL = 0.8 , OUTTAKE_ROTATION_SPEC = 0.9   , OUTTAKE_ROTATION_HIGH_VERT = 0.62,OUTTAKE_ROTATION_TAKE_LOW = 0.04;
    public double SWEEP_START = -1;
    // Intake Pivot
    public double CLAW_OUT_CLOSED = 0.6,CLAW_OUT_OPEN = 0.4, CLAW_OUT_SPEC = 0.57;
    public double CLAW_IN_CLOSED = 0.5 ,CLAW_IN_OPEN = 0.7 , CLAW_IN_ADJUST = 0.43;

    // Outtake claw rotation

    //Singleton
    private static HardwareClass hardwareClass = null;

    public HardwareClass(HardwareMap hardwareMap){
        //Chassy
        this.FR = hardwareMap.get(DcMotor.class , "FR");
        this.FL = hardwareMap.get(DcMotor.class , "FL");
        this.BR = hardwareMap.get(DcMotor.class , "BR");
        this.BL = hardwareMap.get(DcMotor.class , "BL");

        //Slides
        this.RS = hardwareMap.get(DcMotorEx.class,"RS");
        this.LS = hardwareMap.get(DcMotorEx.class,"LS");
        this.Extendo = hardwareMap.get(DcMotorEx.class , "EX");
        //Servos
        this.ClawIn = hardwareMap.get(Servo.class, "CI");
        this.ClawOut = hardwareMap.get(Servo.class, "CO");
        this.IntakeRotate = hardwareMap.get(Servo.class, "RI");
        this.OuttakeRotate = hardwareMap.get(Servo.class, "RO");
        this.Sweep = hardwareMap.get(CRServo.class, "SW");
        this.MobiDick = hardwareMap.get(Servo.class, "MD");

        this.IntakeSensor = hardwareMap.get(ColorRangeSensor.class,"HATZ");
    }

    public static synchronized HardwareClass getInstance(HardwareMap hardwareMap){
        if(hardwareClass == null)
            hardwareClass = new HardwareClass(hardwareMap);
        return hardwareClass;
    }
}
