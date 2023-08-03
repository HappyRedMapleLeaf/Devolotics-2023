package org.firstinspires.ftc.teamcode.drive.opmode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;

import static java.lang.Enum.valueOf;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(group = "drive")
public class Quals extends LinearOpMode {
    private DcMotorEx horizontalPitch = null;
    private DcMotorEx horizontalExtend = null;

    private Servo horizontalClaw = null;
    private Servo horizontalHeading = null;
    private Servo verticalClaw = null;
    private Servo verticalPitch = null;

    public static double hclawheading = 0;
    public static double vclawpitch = 0;
    public static int hextendtransferpos= 200;
    public static int hextendtarget = 0;

    //motor positions/constants
    public static int hpitchhome = 0;        //arm parallel to the ground
    public static int hpitchtransfer = 200;  //arm ready to cone transfer

    //servo positions/constants
    public static double vclawopen = 0;           //open claw
    public static double vclawclose = 0.3;            //close claw
    public static double hclawclose = 0.3;         //close claw
    public static double hclawopen = 0;           //open claw
    public static double hheadinghome = 1;          //horizontal heading that lines up for cones transfer
    public static double hheadingscore1 = 0.66;     //horizontal heading to score
    public static double vpitchhome = 0;            //sets pitch of vertical claw for cone transfer
    public static double vpitchscore = 0.6;         //sets pitch of vertical claw to score on junction

    //Manual and automated modes for gamepad 2
    public static double  modes = 0;            //the mode for gamepad 2
    public static double  extensionmode = 0;


    //PIDF STUFF
    double horizontalHeadingTarget = 0.0, verticalPitchTarget = 0.0, horizontalClawTarget = 0.0, verticalClawTarget = 0.0;

    public PIDController frontController;
    public PIDController backController;

    public static double Pf = 0.008, If = 0, Df = 0, Ff = 0.007;
    public static double Pb = 0.005, Ib = 0, Db = 0, Fb = 0.15;

    private int frontTarget = 0;
    private int backTarget = 0;
    final int HIGHJUNC = 625;
    final int LOWERED = 0;

    private final double ticks_per_degree_tetrix = 3.844444444444;
    private final double ticks_per_degree_rev = 2.22222222222222222222222222222222222222222222222222222222222;

    private DcMotorEx armLeft;
    private DcMotorEx armRight;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armLeft  = hardwareMap.get(DcMotorEx.class, "armLeft");
        armRight = hardwareMap.get(DcMotorEx.class, "armRight");
        armLeft.setDirection(DcMotorEx.Direction.FORWARD);
        armRight.setDirection(DcMotorEx.Direction.REVERSE);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalExtend  = hardwareMap.get(DcMotorEx.class, "horizontalExtend");
        horizontalPitch = hardwareMap.get(DcMotorEx.class, "horizontalPitch");

        armLeft.setZeroPowerBehavior(BRAKE);
        armRight.setZeroPowerBehavior(BRAKE);
        horizontalExtend.setZeroPowerBehavior(FLOAT);
        horizontalPitch.setZeroPowerBehavior(BRAKE);

        horizontalHeading  = hardwareMap.get(Servo.class, "horizontalHeading");
        horizontalClaw  = hardwareMap.get(Servo.class, "horizontalClaw");
        verticalClaw  = hardwareMap.get(Servo.class, "verticalClaw");
        verticalPitch  = hardwareMap.get(Servo.class, "verticalPitch");

        horizontalExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalPitch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        backController = new PIDController(Pb, Ib, Db);
        frontController = new PIDController(Pf, If, Df);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        FtcDashboard dashboard = FtcDashboard.getInstance();

        int backPosition = armLeft.getCurrentPosition();
        int frontPosition = horizontalPitch.getCurrentPosition();


//Roadrunner to go to specified location on the field after autonomous
//        Trajectory forwards = drive.trajectoryBuilder(new Pose2d())
//                .forward(2)
//                .build();
//        Trajectory backwards = drive.trajectoryBuilder(new Pose2d())
//                .back(2)
//                .build();
//        Trajectory strafeLeft = drive.trajectoryBuilder(new Pose2d())
//                .strafeLeft(2)
//                .build();
//        Trajectory strafeRight = drive.trajectoryBuilder(new Pose2d())
//                .strafeRight(2)
//                .build();

        waitForStart();

//MECANUM DRIVE
        while (!isStopRequested()) {
            if (extensionmode % 2 > 0) {
                drive.setWeightedDrivePower (
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }

            //SPECIFIC MOVEMENTS IN TELEOPERATED MODE for gamepad 1
//            if (gamepad1.dpad_up && extensionmode%2 > 0){
//                drive.followTrajectory(forwards);
//                sleep(10);
//            }
//            if (gamepad1.dpad_down && extensionmode%2 > 0){
//                drive.followTrajectory(backwards);
//                sleep(10);
//            }
//            if (gamepad1.dpad_left && extensionmode%2 > 0){
//                drive.followTrajectory(strafeLeft);
//                sleep(10);
//            }
//            if (gamepad1.dpad_right && extensionmode%2 > 0){
//                drive.followTrajectory(strafeRight);
//                sleep(10);
//            }

            //stop and reset horizontal extend encoder

            if (gamepad1.y) {
                extensionmode += 1;
            }

            //if y mode is on then the horizontal extension's encoder will get stopped and reset continually if y mode is off
            //the then the horizontal extension go back to be normal
            if (extensionmode%2 > 0 && horizontalExtend.getCurrentPosition() < 200) {
                horizontalExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //brings the horizontal slides back
            if (gamepad1.b){
                hextendtarget = 0;
                //setting the target
                horizontalExtend.setTargetPosition(hextendtarget);

                //going to target
                horizontalExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //velocity of horizontal extend
                horizontalExtend.setVelocity(1);
            }

            if (horizontalExtend.getCurrentPosition() < 150 && hextendtarget < 200){
                horizontalExtend.setPower(-0.01);
            }

//Roadrunner movement to get to position


//ARM LIFTING
            //ARM LEFT AND RIGHT LIFTING
//            backController.setPID(Pf, If, Df);
//            int armPos = armLeft.getCurrentPosition();
//            double pid = backController.calculate(armPos, backTarget);
//            double ff = Math.cos(Math.toRadians(backTarget / ticks_per_degree_tetrix) * Fb);
//
//            double power = pid * ff;
//
//            armLeft.setPower(power);
//            armRight.setPower(power);

            //New PID
            double frontPower = 0, backPower = 0;
            backController.setPID(Pb, Ib, Db);
            frontController.setPID(Pf, If, Df);

            double backPID = backController.calculate(backPosition, backTarget);
            double frontPID = frontController.calculate(frontPosition, frontTarget);

            double backFeed = Math.cos(Math.toRadians((backTarget - 315) / ticks_per_degree_tetrix)) * Fb;
            double frontFeed = Math.cos(Math.toRadians((frontTarget + 430) / ticks_per_degree_rev)) * Ff;

            backPower = backPID + backFeed;
            frontPower = frontPID + frontFeed;

            armLeft.setPower(backPower);
            armRight.setPower(backPower);
            horizontalPitch.setPower(frontPower);

            //STEPS TO TUNE
//            1. increase the feedforward component (f) until the arm can be held in place without the help of gravtiy)
//            2. increase or decrease the proportional(p) component to match the target and pos values
//            3. increase the dampening(d) component to dampen oscillations (should be really low eg. 0.0001)

// Game pad 2 controls mode 1

            if (gamepad2.left_stick_button){
                modes += 1;
                sleep(10);
            }
            //use "modes%2 == 0" or "modes%2 > 0" depending on which setting it needs to be in

            //HORIZONTAL EXTENSION
            if (horizontalExtend.getCurrentPosition() < 400 && modes%2 == 0){    //if to far motor stops
                horizontalExtend.setPower(gamepad2.right_trigger);   //extend out
            }
            else if (horizontalExtend.getCurrentPosition() > 0 && modes%2 == 0){  //if fully collapsed motor stops
                horizontalExtend.setPower(-gamepad2.left_trigger);  //retract in
            }
            else {
                horizontalExtend.setPower(0);
            }

            //HORIZONTAL PITCH
            if (horizontalPitch.getCurrentPosition() < hpitchtransfer && horizontalPitch.getCurrentPosition() > 0 && modes%2 == 0) {
                if (gamepad2.left_stick_y > 0) {
                    frontTarget += 10;
                    sleep(50);
                }
                else if (gamepad2.left_stick_y < 0){
                    frontTarget -= 10;
                    sleep(50);
                }
            }

            //HORIZONTAL HEADING
            if (gamepad2.left_stick_x < 0 && modes%2 == 0){
                hclawheading += 0.1;    //decreases the position that servo goes to
                sleep(100);
                horizontalHeading.setPosition(hclawheading);
            }
            if (gamepad2.left_stick_x > 0 && modes%2 == 0){
                hclawheading -= 0.1;    //increases the position that servo goes to
                sleep(100);
                horizontalHeading.setPosition(hclawheading);
            }

            //HORIZONTAL CLAW
            if (gamepad2.x){
                horizontalClaw.setPosition(hclawopen);  //open claw
            }
            else if (gamepad2.y){
                horizontalClaw.setPosition(hclawclose); //close claw
            }

//ARM LIFTING
            if (gamepad2.right_bumper && modes%2 == 0) {
                backTarget += 2;
                sleep(50);
            }
            else if (gamepad2.left_bumper && modes%2 == 0) {
                backTarget -= 2;
                sleep(50);
            }

//VERTICAL SECTION
            //VERTICAL PITCH
            if (gamepad2.right_stick_y > 0 && modes%2 == 0){
                vclawpitch -= 0.1;
                sleep(100);
                verticalPitch.setPosition(vclawpitch);

            }
            if (gamepad2.right_stick_y < 0 && modes%2 == 0){
                vclawpitch += 0.1;
                sleep(100);
                verticalPitch.setPosition(vclawpitch);
            }

            //VERTICAL CLAW
            if (gamepad2.a){
                verticalClaw.setPosition(vclawopen);  //open claw
            }

            else if (gamepad2.b){
                verticalClaw.setPosition(vclawclose); //close claw
            }

//gamepad 2 Mode 2

            //EXTENSION
            if (gamepad2.right_trigger > 0 && modes%2 > 0){
                hextendtarget = 500;
                //setting the target
                horizontalExtend.setTargetPosition(hextendtarget);

                //going to target
                horizontalExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //velocity of horizontal extend
                horizontalExtend.setVelocity(1);           }

            else if (gamepad2.left_trigger > 0 && modes%2 > 0){
                hextendtarget = hextendtransferpos;
                //setting the target
                horizontalExtend.setTargetPosition(hextendtarget);

                //going to target
                horizontalExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //velocity of horizontal extend
                horizontalExtend.setVelocity(1);
            }

            //HORIZONTAL PITCH
            if (gamepad2.left_stick_y > 0 && modes%2 > 0){
                frontTarget = hpitchtransfer;
            }
            else if (gamepad2.left_stick_y < 0 && modes%2 > 0){
                frontTarget = hpitchhome;
            }

            //HORIZONTAL HEADING
            if (gamepad2.left_stick_x < 0 && modes%2 > 0){
                horizontalHeading.setPosition(hheadinghome);
            }
            else if (gamepad2.left_stick_x > 0 && modes%2 > 0){
                horizontalHeading.setPosition(hheadingscore1);
            }

            //ARM TARGET
            if (gamepad2.right_bumper && modes%2 > 0){
                backTarget = HIGHJUNC;
            }
            else if (gamepad2.left_bumper && modes%2 > 0){
                if (backTarget > 330){
                    backTarget = 330;
                    sleep(100);
                }
                backTarget = LOWERED;
            }

            if (gamepad2.right_stick_y > 0 && modes%2 > 0){
                verticalPitch.setPosition(vpitchscore);
            }
            else if (gamepad2.right_stick_y < 0 && modes%2 > 0){
                verticalPitch.setPosition(vpitchhome);
            }

//gamepad 1 controls

            //Extends the horizontal section out, and brings the horizontal arm down
            if (gamepad1.a){
                //target values
                hextendtarget = 500;   //Horizontal extend ticks

                //opens claw
                horizontalClaw.setPosition(hclawopen);

                //pitch of claw is set to 0
                frontTarget = hpitchhome;
                sleep(500);

                //heading of claw
                horizontalHeading.setPosition(hheadingscore1);   //position to grab cone

                //setting the target
                horizontalExtend.setTargetPosition(hextendtarget);

                //going to target
                horizontalExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //velocity of horizontal extend
                horizontalExtend.setVelocity(1);

                //close claw
                horizontalClaw.setPosition(hclawclose);  //close claw
            }

            //brings the horizontal extension back, sets the heading of the claw back to normal, closes the claw,
            //lifts arm to transfer, opens claw, vertical claw closes to grasp the cone transfer, arm's lift to high
            //junction, vertical pitch goes to high junction, claw opens, horizontal extends out again, and
            //horizontal heading and pitch get ready for next cycle
            if (gamepad1.x) {//sets the vertical claw to correct pitch
                verticalPitch.setPosition(vpitchhome);   //sets to correct pitch
                sleep(500); //has time to settle
                //picking up cone
                //target values
                frontTarget = hpitchtransfer;      //pitch of horizontal arm
                hextendtarget = hextendtransferpos;                //extension of the horizontal arm
                //setting the target
                horizontalExtend.setTargetPosition(hextendtarget);
                //going to target
                horizontalExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //velocity of motor
                horizontalExtend.setVelocity(0.5);

                //heading of claw
                horizontalHeading.setPosition(hheadinghome);   //position to cone transfer

                sleep(500);    //waits 2 seconds to complete

                //transfers cone to vertical
                horizontalClaw.setPosition(hclawopen);  //open claw to cone transfer
                sleep(500); //has time to drop

                verticalClaw.setPosition(vclawclose);    //closes claw
                sleep(500); //has time to settle

                //Moves arm up and gets ready to place on junction
                backTarget = 330;
                sleep(100);
                backTarget = HIGHJUNC;

                //sets the vertical claw to correct pitch to score the cone
                verticalPitch.setPosition(vpitchscore);   //sets to correct pitch to score
                sleep(500); //has time to settle
                verticalClaw.setPosition(vclawopen);    //opens claw and drops cone onto the junction
                sleep(1000);    //has time to settle after scoring

                //New: Horizontal claw getting ready for next cycle
                horizontalHeading.setPosition(hheadingscore1);

                //New: Extend out again
                frontTarget = hpitchhome;
                hextendtarget = 500;

                //setting the target
                horizontalExtend.setTargetPosition(hextendtarget);

                //going to target
                horizontalExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //velocity of motor
                horizontalExtend.setVelocity(0.5);
                sleep(500);

                backTarget = 330;
                sleep(100);
                backTarget = LOWERED;

                //resets the pitch of the claw to cycle again
                verticalPitch.setPosition(vpitchhome);   //sets to correct pitch to cycle
                verticalClaw.setPosition(vclawopen);    //opens claw to get ready for new cone

                //picks up the cone
                horizontalClaw.setPosition(hclawclose);  //closes claw to pick up cone
            }

            vclawpitch = Math.max(0, Math.min(0.6, vclawpitch));
            hclawheading = Math.max(0, Math.min(1, hclawheading));
            frontTarget=  Math.max(0, Math.min(-430, frontTarget));

            //telemetry information
            int modetelem = 0;
            if (modes % 2 == 0){
                modetelem = 0;
            }
            else if (modes % 2 > 0){
                modetelem = 1;
            }
            //horizontal stop and reset encoder on and off
            int Modeextension = 0;
            if (extensionmode % 2 == 0){
                Modeextension = 0;
            }
            else if (extensionmode % 2 > 0) {
                Modeextension = 1;
            }

            //ALL THE TELEMETRY
            telemetry.addData("Gamepad 2 mode", modetelem);
            telemetry.addData("Reset encoder on/off", Modeextension);
            telemetry.addData("pos", armLeft.getCurrentPosition());
            telemetry.addData("horizontal heading", hclawheading);
            telemetry.addData("vertical pitch", vclawpitch);
            telemetry.addData("Back target", backTarget);
            telemetry.addData("Front target", frontTarget);
            telemetry.addData("Horizontal Arm Pitch", horizontalPitch.getCurrentPosition());
            telemetry.addData("Horizontal Extension", horizontalExtend.getCurrentPosition());
            telemetry.update();
        }
    }
}