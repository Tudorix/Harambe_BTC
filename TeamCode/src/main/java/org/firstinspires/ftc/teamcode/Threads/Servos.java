package org.firstinspires.ftc.teamcode.Threads;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareClass;

public class Servos {

    //Declarations
    public Servo ClawIn , ClawOut , IntakeRotate, OuttakeRotate, Specimen, SpecClaw, MobyDick,Pivot;
    public CRServo Sweep;
    public  Servo SpecArm;
    private HardwareClass hardwareClass;

    int a  = 1, y = 1 , x = 1 , b = 1;

    //Singleton
    private static Servos single_instance = null;
    private boolean running = false;
    Thread thread = null;

    double SpecP , RIP , ROP , CIP, COP, PVP ;

    int trans = 0;

    public Servos(HardwareClass hardwareClass, Telemetry telemetry , HardwareMap hardwareMap){
        this.ClawIn = hardwareClass.ClawIn;
        this.ClawOut = hardwareClass.ClawOut;
        //this.Extendo = hardwareClass.Extendo;
        this.IntakeRotate = hardwareClass.IntakeRotate;
        this.OuttakeRotate = hardwareClass.OuttakeRotate;
        this.Sweep = hardwareClass.Sweep;
        this.MobyDick = hardwareClass.MobiDick;
        this.Pivot = hardwareClass.Pivot;
        this.hardwareClass = hardwareClass;
    }

    public void intake(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        this.IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
    }

    public void rotPiv(double poz){
        Pivot.setPosition(poz);
    }

    public void intake_OFF(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        this.IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
    }

    public void intakePrep(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_PREP);
    }

    public void transfer(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        wait(50);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(100);
    }

    public void transfer2(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        wait(100);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(100);
    }

    public void help(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_RIGHT);
    }

    public void invert(){
        wait(500);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        Pivot.setPosition(hardwareClass.PIVOT_MAX_LEFT);
    }

    public void outtake(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(70);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(70);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE);
    }

    public void placeInBasket(){
        OuttakeRotate.setPosition(1);
        wait(150);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        wait(180);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }

    public void mobyDickOut(){
        MobyDick.setPosition(hardwareClass.MD_OUT);
    }

    public void mobyDickIn(){
        MobyDick.setPosition(hardwareClass.MD_IN);
    }

    /** TeleOP */

    public void camIn(){
        hardwareClass.Cam.setPosition(hardwareClass.CAM_IN);
    }

    public void camOut(){
        hardwareClass.Cam.setPosition(hardwareClass.CAM_OUT);
    }

    public void transferShort(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(80);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        //adjust();
    }

    public void transferTO(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN - 0.02);
        wait(20);
        ClawIn.setPosition(hardwareClass.CLAW_IN_CLOSED);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        //adjust();
    }

    public void outtakeDown(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
    }

    public void outtakeVERYDown(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE_LOW);
    }

    public void transferSpec() {
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
        wait(100);
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        wait(100);
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
        wait(100);
        //adjust();
    }

    public void outtakeTransf(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE);
    }

    public void outtakeS(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(100);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_HIGH_VERT);
    }

    public void outtakeSpec(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_SPEC);
        wait(170);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_SPEC);
    }

    public void see(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_SEE);
    }

    public void placeInBasket_S(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN - 0.09);
        wait(100);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }

    public void prepOuttake(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
        wait(200);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
        wait(200);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }

    public void releaseSpecimen(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
    }

    public void closeSpecimen(){
        Specimen.setPosition(hardwareClass.SPECIMEN_CLOSED);
    }

    public void openSpecimen(){
        Specimen.setPosition(hardwareClass.SPECIMEN_OPEN);
    }

    public void specArmOpen(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_OPEN);
    }
    public void specArmClose(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_CLOSE);
    }


    public void specArmUpFar(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_CLOSE);
    }

    public void specClawOpen(){
        SpecClaw.setPosition(hardwareClass.SPEC_CLAW_OPEN);
    }


    /** Fine state */
    public void intakeDown_noPivot(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_DOWN);
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_TAKE);
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
    }

    public void intakeTake(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_ADJUST);
    }

    public void intakeUp(){
        IntakeRotate.setPosition(hardwareClass.INTAKE_ROTATION_UP);
    }

    public void transferClose(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_CLOSED);
    }

    public void transferGive(){
        ClawIn.setPosition(hardwareClass.CLAW_IN_OPEN);
    }

    public void outtakePut(){
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_PLACE - 0.05);
    }

    public void outtakePlace(){
        ClawOut.setPosition(hardwareClass.CLAW_OUT_OPEN);
    }

    public void outtakeReturn(){
        OuttakeRotate.setPosition(hardwareClass.OUTTAKE_ROTATION_VERTICAL);
    }




    public void wait(int sec){
        try {
            Thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void waitThr(int sec){
        try {
            thread.sleep(sec);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setup(){

    }

    public void resume(){
        running = true;
    }

    public void stop(){
        running = false;
    }

    public boolean getStatus(){
        return running;
    }

    public static synchronized Servos getInstance(HardwareMap hardwareMap , Telemetry telemetry ){
        if(single_instance == null){
            single_instance = new Servos(HardwareClass.getInstance(hardwareMap), telemetry , hardwareMap);
        }
        return single_instance;
    }
}
