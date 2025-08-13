package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareClass;
import org.firstinspires.ftc.teamcode.Threads.Extendo;
import org.firstinspires.ftc.teamcode.Threads.Servos;
import org.firstinspires.ftc.teamcode.Threads.Slides;
import org.firstinspires.ftc.teamcode.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedro.constants.LConstants;

import java.util.List;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "6_LEFT_BLUE_HATZ", group = "Examples")
public class LEFT_5PLUS_SMECHERIE extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private Limelight3A limelight;

    Servos servos = null;
    Slides slides = null;
    Extendo extendo = null;
    HardwareClass hardwareClass = null;

    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;


    public static int smallDelay = 200, bigDealy = 500;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    double x = 0 , cos = 0 , sin = 0 , ClampPosition = 0 , ClampPositionLeft = 0, Close = -1;
    double y = 0 , max=0 , FLPower = 0 , FRPower = 0, BLPower = 0 , BRPower = 0;
    double turn = 0 , theta = 0 , power = 0 , Trig_Forc = 0;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(37.5, 11.01, Math.toRadians(90));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(17, 18, Math.toRadians(45));

    private final Pose scorePose1 = new Pose(17, 20, Math.toRadians(45));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(17, 22.5, Math.toRadians(72));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(16.4, 22.5, Math.toRadians(95));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(16, 24.5, Math.toRadians(118));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(47, 68, Math.toRadians(0));

    private final Pose parkPose2 = new Pose(30, 60, Math.toRadians(0));

    private final Pose parkPose3 = new Pose(47, 68, Math.toRadians(180));

    private final Pose parkPose4 = new Pose(30, 62, Math.toRadians(180));

    Pose takeSample;


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, park2;

    private PathChain grab1 , grab2 , grab3 , parkPath , placeFOUND , takeFisrt, placeP, place1 , place2,place3 , park;
    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        placeP = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grab1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup1Pose.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.placeInBasket();
                    slidesBackInRobot();
                    extendAndPivot(850 , (float) 0.16);
                })
                .build();

        place1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1 , () -> {
                    OuttakeSample();
                })
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2Pose.getHeading())
                .addParametricCallback(0 , () -> {
                    extendAndPivot(900 , (float) 0.16);
                })
                .addParametricCallback(1 , () -> {
                    servos.placeInBasket();
                    slidesBackInRobot();
                })
                .build();

        place2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1 , () -> {
                    OuttakeSample();
                })
                .build();

        grab3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.placeInBasket();
                    slidesBackInRobot();
                    extendAndPivot(900 , (float) 0.16);
                })
                .build();

        place3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(1 , () -> {
                    OuttakeSample();
                })
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(parkPose3)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose3.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.outtakePut();
                })

                .addPath(new BezierLine(new Point(parkPose3), new Point(parkPose4)))
                .setLinearHeadingInterpolation(parkPose3.getHeading(), parkPose4.getHeading())
                .build();


        placeFOUND = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(parkPose2)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), parkPose2.getHeading())

                .addParametricCallback(0 , () -> {

                })

                .addPath(new BezierLine(new Point(parkPose2), new Point(scorePose)))
                .setLinearHeadingInterpolation(parkPose2.getHeading(), scorePose.getHeading())

                .addParametricCallback(0.1 , () -> {

                })

                .addParametricCallback(0.3 , () -> {
                    OuttakeSample();
                })
                .build();

        takeFisrt = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(parkPose2)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose2.getHeading())
                .addParametricCallback(0 , () -> {
                    servos.placeInBasket();
                    slidesBackInRobot();
                })

                .addPath(new BezierLine(new Point(parkPose2), new Point(parkPose)))
                .setLinearHeadingInterpolation(parkPose2.getHeading(), parkPose.getHeading())

                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                OuttakeSample();
                follower.followPath(grab2,false);
                setPathState(2);
                break;
            case 1:
               if(!follower.isBusy()) {
                   servos.transfer();
                   delay(200);
                   servos.invert();
                   extendo.goToPosition(hardwareClass.IN);
                   follower.followPath(place1,false);
                   setPathState(2);
                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    follower.followPath(grab1,false);
                    setPathState(3);
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    servos.transfer();
                    delay(200);
                    servos.invert();
                    extendo.goToPosition(hardwareClass.IN);
                    follower.followPath(place2,false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    follower.followPath(grab3,true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()) {
                    servos.transfer();
                    delay(200);
                    servos.invert();
                    extendo.goToPosition(hardwareClass.IN);
                    follower.followPath(place3,false);
                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(takeFisrt,true);
                    setPathState(-1);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(takeFisrt,true);
                    setPathState(7);
                }
                break;
            case 8:
                if(!follower.isBusy()) {
                    find();
                    setPathState(9);
                }
                break;
            case 9:
                if(!follower.isBusy()) {
                    servos.camIn();
                    extendo.goToPosition(hardwareClass.IN);
                    follower.followPath(placeFOUND);
                    int i = 10;
                    if(opmodeTimer.getElapsedTimeSeconds() > 24){
                        i = 14;
                    }
                    setPathState(i);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    servos.placeInBasket();
                    slidesBackInRobot();
                    follower.followPath(takeFisrt,true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    find();
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    servos.camIn();
                    extendo.goToPosition(hardwareClass.IN);
                    follower.followPath(placeFOUND);
                    setPathState(13);
                }
                break;
            case 13:
                if(!follower.isBusy()) {
                    servos.placeInBasket();
                    slidesBackInRobot();
                    setPathState(14);
                }
                break;
            case 14:
                if(!follower.isBusy()) {
                    servos.placeInBasket();
                    slidesBackInRobot();
                    follower.followPath(park);
                    setPathState(-1);
                }
                break;
            default:
                setPathState(-1);
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        servos = Servos.getInstance(hardwareMap,telemetry);
        slides = Slides.getInstance(hardwareMap,telemetry);
        extendo = Extendo.getInstance(hardwareMap,telemetry);
        hardwareClass = HardwareClass.getInstance(hardwareMap);

        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);

        slides.setup();
        servos.setup();
        extendo.setup();


        servos.stop();

        hardwareClass.LS.setDirection(DcMotor.Direction.REVERSE);

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        //limelight.pipelineSwitch(0);

        //limelight.start();

        //telemetry.addLine("Limelight :" + limelight.getConnectionInfo());
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void giveSpecimen(){
        servos.wait(50);

        servos.wait(300);
        OuttakeSample();
    }

    private void delay(int delay){
        try {
            Thread.sleep(delay);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void nuAmChefAzi(){
        LLResult result = limelight.getLatestResult();
        servos.intakePrep();
        extendo.goToPosition(200);

        int extendoPose;
        extendoPose = convertToNewRange(result.getTy() , -16 , 16 , 300 , 1500);
        int i = 0;

        int rotation;
        rotation = convertToNewRange(result.getTx() , -23 , 23 , -60 , 60);

        takeSample = new Pose(follower.getPose().getX() + 0.1, follower.getPose().getY() + 0.1, Math.toRadians(rotation));

        placeFOUND = follower.pathBuilder()
                .addPath(new BezierLine(new Point(follower.getPose()), new Point(takeSample)))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), takeSample.getHeading())

                .addParametricCallback(0 , () -> {
                    extendAndPivot(extendoPose,(float)0.16);
                })
                .build();

        servos.transfer();
        delay(200);
        servos.invert();
        extendo.goToPosition(hardwareClass.IN);
    }

    public void find(){
        int v = 1;
        int j = 0;
        while (v == 1){
            j++;
            LLResult result = limelight.getLatestResult();
            servos.intakePrep();
            extendo.goToPosition(200);
            servos.wait(100);
            if (result.isValid()){
                int extendoPose;
                extendoPose = convertToNewRange(result.getTx() , -23 , 23 , 300 , 1500);
                int i = 0;

                Pose takeSample = new Pose(follower.getPose().getX() + 0.1, follower.getPose().getY() + 0.1, Math.toRadians(0));

                placeFOUND = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(follower.getPose()), new Point(takeSample)))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), takeSample.getHeading())

                        .addParametricCallback(0 , () -> {
                            extendAndPivot(extendoPose,(float)0.16);
                        })
                        .build();

                servos.transfer();
                delay(200);
                servos.invert();
                extendo.goToPosition(hardwareClass.IN);
            }

            if(j >= 15){
                setPathState(-1);
                break;
            }
        }
    }

    public void angle(LLResult result){
        double width = 0 , height = 0;
        double Mx = -999 , mx = 999, My = -999, my = 999;
        double i = 0, Iy = 0;

        List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("corners: ",  cr.getTargetCorners());
            for(List<Double> point : cr.getTargetCorners()){
                double x = point.get(0);
                double y = point.get(1);
                telemetry.addLine("Corner: " + "X:" + x +"; Y:" + y + "; Index: " + i);

                if(Mx < x){ Mx = x;}
                if(My < y){ My = y; Iy = i;}
                if(mx > x) mx = x;
                if(my > y) my = y;
                i++;
            }
        }

        width = Math.abs(Mx - mx);
        height = Math.abs(My - my);


    }

    public void Drive(double power){
        FLPower = -power;
        FRPower = -power;
        BLPower = power;
        BRPower = power;

        FL.setPower(FLPower);
        FR.setPower(FRPower);
        BL.setPower(BLPower);
        BR.setPower(BRPower);
    }

    public int convertToNewRange(double value, double oldMin, double oldMax, double newMin, double newMax){
        return (int)(newMin + (value - oldMin) * (newMax - newMin) / (oldMax - oldMin));
    }

    private void OuttakeSample(){
        servos.outtake();
        slides.setCoefs(0.01,0,0);
        slides.goToPosition(hardwareClass.HIGH_BASKET);
    }

    private void OuttakeSampleSpec(){
        servos.outtakeSpec();
        slides.setCoefs(0.01,0,0);
        slides.goToPosition(hardwareClass.HIGH_BASKET);
    }

    private void slidesBackInRobot(){
        slides.setCoefs(0.002,0,0);
        slides.goToPosition(hardwareClass.IN_ROBOT);
    }

    private void extendAndPivot(int distance , float angle){
        extendo.goToPosition(distance);
        servos.intakePrep();
        servos.rotPiv(angle);
    }
}

