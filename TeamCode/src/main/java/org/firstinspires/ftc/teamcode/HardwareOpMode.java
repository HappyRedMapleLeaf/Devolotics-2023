package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/**
 * Basis OpMode that contains and initializes robot hardware: motors, servos, drivetrain, and distance sensor
 * Does not initialize the webcam
 * Must call super.init() in init()!!!
 */
public class HardwareOpMode extends OpMode {
    public DcMotorEx vArmLeft, vArmRight, extend, extend2;
    public Servo hClaw, vClaw, vPitch, align, hClaw2, clawPitch, lPitch, rPitch;
    public SampleMecanumDrive drive;
    public DistanceSensor distanceSensor;

    @Override
    public void init() {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");

        //HARDWARE INITIALIZATION
        vArmLeft = hardwareMap.get(DcMotorEx.class, "12");
        vArmRight = hardwareMap.get(DcMotorEx.class, "3");
        vArmRight.setDirection(DcMotorEx.Direction.REVERSE);
        vArmLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vArmLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vArmLeft.setZeroPowerBehavior(BRAKE);
        vArmRight.setZeroPowerBehavior(BRAKE);

        extend = hardwareMap.get(DcMotorEx.class, "13");
        extend.setDirection(DcMotorEx.Direction.REVERSE);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extend.setZeroPowerBehavior(BRAKE);

        extend2 = hardwareMap.get(DcMotorEx.class, "0");
        extend2.setZeroPowerBehavior(BRAKE);

        align = hardwareMap.get(Servo.class, "0");
        rPitch = hardwareMap.get(Servo.class, "2");
        clawPitch = hardwareMap.get(Servo.class, "3");
        vClaw = hardwareMap.get(Servo.class, "11");
        hClaw = hardwareMap.get(Servo.class, "12");
        hClaw2 = hardwareMap.get(Servo.class, "5");
        lPitch = hardwareMap.get(Servo.class, "13");
        vPitch = hardwareMap.get(Servo.class, "15");
        rPitch.setDirection(Servo.Direction.REVERSE);
        align.setDirection(Servo.Direction.REVERSE);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {}
}
