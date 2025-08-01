package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Extendo;
import org.firstinspires.ftc.teamcode.Threads.Holonomic;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Slides;
import org.opencv.core.Mat;

/**
 * This opmode demonstrates how one can augment driver control by following Road Runner arbitrary
 * Road Runner trajectories at any time during teleop. This really isn't recommended at all. This is
 * not what Trajectories are meant for. A path follower is more suited for this scenario. This
 * sample primarily serves as a demo showcasing Road Runner's capabilities.
 * <p>
 * This bot starts in driver controlled mode by default. The player is able to drive the bot around
 * like any teleop opmode. However, if one of the select buttons are pressed, the bot will switch
 * to automatic control and run to specified location on its own.
 * <p>
 * If A is pressed, the bot will generate a splineTo() trajectory on the fly and follow it to
 * targetA (x: 45, y: 45, heading: 90deg).
 * <p>
 * If B is pressed, the bot will generate a lineTo() trajectory on the fly and follow it to
 * targetB (x: -15, y: 25, heading: whatever the heading is when you press B).
 * <p>
 * If Y is pressed, the bot will turn to face 45 degrees, no matter its position on the field.
 * <p>
 * Pressing X will cancel trajectory following and switch control to the driver. The bot will also
 * cede control to the driver once trajectory following is done.
 * <p>
 * The following may be a little off with this method as the trajectory follower and turn
 * function assume the bot starts at rest.
 * <p>
 * This sample utilizes the SampleMecanumDriveCancelable.java and TrajectorySequenceRunnerCancelable.java
 * classes. Please ensure that these files are copied into your own project.
 */
@TeleOp(name="TeleOp Solo", group = "Solo")
public class TeleOpAugmentedDriving extends LinearOpMode {
    // Define 2 states, drive control or automatic control

    //Holonomic chassy = null;
    Slides slides = null;
    Servos servos = null;
    Extendo extendo = null;
    HardwareClass hardwareClass = null;
    Holonomic holonomic = null;

    // TeleOp variables

    String Style = "Drive";

    int decl = 1;
    int pivotType = 1;

    int ExtendoPosition = 0;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // The coordinates we want the bot to automatically go to when we press the A button
    Vector2d targetAVector = new Vector2d(-60, -56);
    // The heading we want the bot to end on for targetA
    double targetAHeading = Math.toRadians(45);

    // The location we want the bot to automatically go to when we press the B button
    Vector2d targetBVector = new Vector2d(-15, 25);

    // The angle we want to align to when we press Y
    double targetAngle = Math.toRadians(45);

    Pose2d pose = null;

    int automatic = -1;

    @Override
    public void runOpMode() throws InterruptedException {
        slides = Slides.getInstance(hardwareMap, telemetry);
        servos = Servos.getInstance(hardwareMap , telemetry);
        extendo = Extendo.getInstance(hardwareMap,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);
        holonomic = Holonomic.getInstance(hardwareMap , gamepad1);

        //Setup Individual Motors
        hardwareClass.Extendo.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.LS.setDirection(DcMotorSimple.Direction.REVERSE);

        hardwareClass.FL.setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareClass.BL.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        //servos.camIn();

        if(!slides.getStatus()){
            slides.setup();
        }

        if(!extendo.getStatus()){
            extendo.setup();
        }

        if(!holonomic.getStatus()){
            holonomic.start();
        }

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            switch (Style){
                case "Drive" : {
                    //Intake mode
                    if(gamepad1.right_bumper){
                        servos.intake();
                        ExtendoPosition = (int) hardwareClass.CLOSE;
                        holonomic.decel();
                        servos.placeInBasket();
                        delay(100);
                        slides.setCoefs(0.003 ,0 ,0);
                        slides.goToPosition(hardwareClass.IN_ROBOT);
                        //pivotType = 1;
                        Style = "Intake";
                    }

                    if(gamepad1.right_trigger > 0.5){
                        servos.intake();
                        servos.mobyDickOut();
                        ExtendoPosition = (int) hardwareClass.CLOSE;
                        holonomic.decel();
                        servos.placeInBasket();
                        delay(100);
                        slides.setCoefs(0.003 ,0 ,0);
                        slides.goToPosition(hardwareClass.IN_ROBOT);
                        //pivotType = 1;
                        Style = "Intake";
                    }

                    //Place in basket and return
                    if(gamepad1.left_bumper){
                        servos.outtake();
                        delay(50);
                        slides.setCoefs(0.01,0 ,0);
                        slides.goToPosition(hardwareClass.HIGH_BASKET);
                    }

                    //Prep for high basket
                    if(gamepad1.left_trigger > 0.5){
                        servos.placeInBasket();
                        delay(100);
                        slides.setCoefs(0.003 ,0 ,0);
                        slides.goToPosition(hardwareClass.IN_ROBOT);
                    }

                    if(gamepad1.x){
                        extendo.goToPosition(-100);
                        extendo.resetMotor();
                        hardwareClass.IN = 260;
                    }

                    if(gamepad1.dpad_up){
                        servos.outtake();
                        delay(50);
                        slides.setCoefs(0.01,0 ,0);
                        slides.goToPosition(hardwareClass.LOW_BASKET);
                    }

                    break;
                }
                case "Intake" : {
                    if(gamepad1.right_trigger > 0.5 && ExtendoPosition < hardwareClass.MAX){
                        ExtendoPosition += 30;
                    }else if(gamepad1.right_bumper && ExtendoPosition > hardwareClass.CLOSE){
                        ExtendoPosition -= 10;
                    }
                    extendo.goToPosition(ExtendoPosition);

                    if(gamepad1.left_trigger > 0.5){
                        servos.intake();
                    }
                    else{
                        servos.intakePrep();
                    }

                    if(gamepad1.a){
                        servos.mobyDickIn();
                        Style = "Drive";
                        holonomic.accel();
                        servos.transfer();
                        extendo.goToPosition(hardwareClass.IN);
                    }
                    break;
                }
                default: break;
            }
        }
    }

    private void delay(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }
}