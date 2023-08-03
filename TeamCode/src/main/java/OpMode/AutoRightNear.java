package OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Auto;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

/**
 * Right side start, scores on near (aka contested) high pole
 */
@Autonomous
public class AutoRightNear extends Auto
{
    //PIDF CONSTANTS
    public PIDController vController;
    public PIDController eController;
    double maxExtendSpeed = 1.0;
    public int vTarget = 0;
    public int eTarget = 0;

    //OTHER
    private ElapsedTime runtime = new ElapsedTime();
    public int prevLine = -1;
    public int line = 0;
    public int vTargetTarget = 0;
    public int eTargetTarget = 0;
    public int vSpeed = 0;
    public int eSpeed = 0;

    public int clawPitchTargetTarget = 150;
    public int clawPitchPos = 150;
    public int clawPitchSpeed = 0;
    public int hPitchTargetTarget = 700;
    public int hPitchPos = 700;
    public int hPitchSpeed = 0;

    public int vPitchPos = vPitchIn;
    public int vClawPos = vClawOpen;
    public int hClawPos = hClawOpen;
    public int hClawPos2 = hClawOpen2;
    public int alignPos = alignIn;

    //Trajectories
    Pose2d startPose = new Pose2d(-37.5, 60.2, Math.toRadians(90));
    Trajectory traj1 = null;
    Trajectory traj4 = null;
    Trajectory traj5 = null;
    Trajectory traj6 = null;

    //AUTONOMOUS PROGRAM
    public List<int[]> program = new ArrayList<>();

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    int LEFT = 173;
    int MIDDLE = 319;
    int RIGHT = 576;
    int finalTag = 0;
    AprilTagDetection tagOfInterest = null;

    int[] hPitchPositions = {470, 622, 417, 568, 353, 496, 290, 457, 245, 245};
    int[] clawPitchPositions = {440, 436, 404, 380, 351, 352, 276, 333, 255, 255};
    int[] extendPositions = {450, 440, 440, 435, 430};

    @Override
    public void init() {
        super.init();
        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        double voltage = chub.getInputVoltage(VoltageUnit.VOLTS);
        maxExtendSpeed = -0.06 * voltage + 1.69;
        maxExtendSpeed = Math.min(1.0, Math.max(0.85, maxExtendSpeed));
        Pe = -0.0008 * voltage + 0.0152;
        Pe = Math.min(0.006, Math.max(0.004, Pe));
        Pv = -0.0008 * voltage + 0.0142;
        Pv = Math.min(0.005, Math.max(0.003, Pv));

        telemetry.addData("voltage", voltage);
        telemetry.update();

        vController = new PIDController(Pv, Iv, Dv);
        eController = new PIDController(Pe, Ie, De);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(0.166, 578.272, 578.272, 402.145, 221.506);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() { camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); }
            @Override
            public void onError(int errorCode) { }
        });

        telemetry.setMsTransmissionInterval(50);
        vClaw.setPosition(vClawOpen / 1000.0);
        vPitch.setPosition(vPitchIn / 1000.0);
        lPitch.setPosition(730 / 1000.0);
        rPitch.setPosition((730 / 1000.0) + 0.02);
        clawPitch.setPosition(150 / 1000.0);
    }

    @Override
    public void init_loop() {
        if (runtime.milliseconds() > 20) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine(Integer.toString(tagOfInterest.id));
                }
            }

            telemetry.update();
            runtime.reset();
        }
    }

    @Override
    public void start() {
        camera.closeCameraDeviceAsync(() -> {});

        if (tagOfInterest == null) {
            finalTag = 0;
        } else if (tagOfInterest.id == LEFT) {
            finalTag = 1;
        } else if (tagOfInterest.id == RIGHT) {
            finalTag = 3;
        } else if (tagOfInterest.id == MIDDLE){
            finalTag = 2;
        }

        drive.setPoseEstimate(startPose);
        traj1 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-37, 55))
                .splineToSplineHeading(new Pose2d(-30.2, -8, Math.toRadians(-17.6)), Math.toRadians(270))
                .build();
        traj4 = drive.trajectoryBuilder(traj1.end())
                .splineToLinearHeading(new Pose2d(-28, 2.4, Math.toRadians(0)), Math.toRadians(-17.6))
                .build();
        traj5 = drive.trajectoryBuilder(traj4.end())
                .forward(25)
                .build();
        traj6 = drive.trajectoryBuilder(traj4.end())
                .back(26)
                .build();
        buildProgram();
    }

    @Override
    public void loop() {

        //MOTOR POSITION VARIABLES
        int vPosition = vArmLeft.getCurrentPosition();
        int ePosition = extend.getCurrentPosition();

        //TARGET SETTING
        if (vTarget != vTargetTarget) { vTarget += vSpeed; }
        if (Math.abs(vTarget - vTargetTarget) < Math.abs(vSpeed)) { vTarget = vTargetTarget; vSpeed = 0; }
        if (eTarget != eTargetTarget) { eTarget += eSpeed;}
        if (Math.abs(eTarget - eTargetTarget) < Math.abs(eSpeed)) { eTarget = eTargetTarget; eSpeed = 0; }

        if ((eTarget < eTargetTarget && eSpeed < 0) || (eTarget > eTargetTarget && eSpeed > 0)) {
            eSpeed = 0;
            eTarget = eTargetTarget;
        }
        if ((vTarget < vTargetTarget && vSpeed < 0) || (vTarget > vTargetTarget && vSpeed > 0)) {
            vSpeed = 0;
            vTarget = vTargetTarget;
        }

        if (clawPitchPos != clawPitchTargetTarget) { clawPitchPos += clawPitchSpeed; }
        if (Math.abs(clawPitchPos - clawPitchTargetTarget) < Math.abs(clawPitchSpeed)) { clawPitchPos = clawPitchTargetTarget; clawPitchSpeed = 0; }
        if (hPitchPos != hPitchTargetTarget) { hPitchPos += hPitchSpeed;}
        if (Math.abs(hPitchPos - hPitchTargetTarget) < Math.abs(hPitchSpeed)) { hPitchPos = hPitchTargetTarget; hPitchSpeed = 0; }

        if ((hPitchPos < hPitchTargetTarget && hPitchSpeed < 0) || (hPitchPos > hPitchTargetTarget && hPitchSpeed > 0)) {
            hPitchSpeed = 0;
            hPitchPos = hPitchTargetTarget;
        }
        if ((clawPitchPos < clawPitchTargetTarget && clawPitchSpeed < 0) || (clawPitchPos > clawPitchTargetTarget && clawPitchSpeed > 0)) {
            clawPitchSpeed = 0;
            clawPitchPos = clawPitchTargetTarget;
        }

        //region PID UPDATING
        double vPID = vController.calculate(vPosition, vTarget);
        double ePID = eController.calculate(ePosition, eTarget) * maxExtendSpeed;

        double ticks_per_degree_tetrix = 3.84444444444444444444444444444;
        double vFeed = Math.cos(Math.toRadians((vTarget - 240) / ticks_per_degree_tetrix)) * Fv;

        vArmLeft.setPower(vPID + vFeed);
        vArmRight.setPower(vPID + vFeed);
        extend2.setPower(ePID);
        extend.setPower(ePID);
        //endregion PID UPDATING


        vPitch.setPosition((double)vPitchPos / 1000);
        vClaw.setPosition((double)vClawPos / 1000);

        hClaw.setPosition((double)hClawPos / 1000);
        hClaw2.setPosition((double)hClawPos2 / 1000);

        align.setPosition((double)alignPos / 1000);
        lPitch.setPosition((double)hPitchPos / 1000);
        rPitch.setPosition(((double)hPitchPos / 1000) + 0.02);
        clawPitch.setPosition((double)clawPitchPos / 1000);
        //endregion PID updating

        if (line < program.size()) {
            int func = program.get(line)[0];
            int arg1 = program.get(line)[1];
            int arg2 = program.get(line)[2];
            int arg3 = program.get(line)[3];

            boolean changeLine = false;

            switch (func) {
                case 1:
                    //setServoPos(int servo, int target1k)
                    switch (arg1) {
                        case 0:
                            vPitchPos = arg2;
                            break;
                        case 1:
                            vClawPos = arg2;
                            break;
                        case HCLAW:
                            if (arg2 == hClawOpen) {
                                hClawPos2 = hClawOpen2;
                            } else if (arg2 == hClawClose) {
                                hClawPos2 = hClawClose2;
                            }
                            hClawPos = arg2;
                            break;
                        case 3:
                            alignPos = arg2;
                            break;
                        case 4:
                            hPitchPos = arg2;
                            hPitchTargetTarget = arg2;
                            hPitchSpeed = 0;
                            break;
                        case 5:
                            clawPitchPos = arg2;
                            clawPitchTargetTarget = arg2;
                            clawPitchSpeed = 0;
                            break;
                    }
                    changeLine = true;
                    break;
                case 2:
                    //setMotorPos(int motor, int target, int speed)
                    switch (arg1) {
                        case 0:
                            vTargetTarget = arg2;
                            vSpeed = arg3;
                            break;
                        case 2:
                            eTargetTarget = arg2;
                            eSpeed = arg3;
                            break;
                    }
                    changeLine = true;
                    break;
                case 99:
                    //setServoSpeed(int servo, int target, int speed)
                    switch (arg1) {
                        case HPITCH:
                            hPitchTargetTarget = arg2;
                            hPitchSpeed = arg3;
                            break;
                        case CLAWPITCH:
                            clawPitchTargetTarget = arg2;
                            clawPitchSpeed = arg3;
                            break;
                    }
                    changeLine = true;
                    break;
                case 3:
                    //followTraj(trajno)
                    switch (arg1) {
                        case 1:
                            drive.followTrajectoryAsync(traj1);
                            break;
                        case 4:
                            drive.followTrajectoryAsync(traj4);
                            break;
                        case 5:
                            drive.followTrajectoryAsync(traj5);
                            break;
                        case 6:
                            drive.followTrajectoryAsync(traj6);
                            break;
                    }
                    changeLine = true;
                    break;
                case 4:
                    //waitMotorTarget(int motor, int waitUntilPosition, int sign)
                    switch (arg1) {
                        case 0:
                            if ((vTarget >= arg2 && arg3 > 0) || (vTarget <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                        case 2:
                            if ((eTarget >= arg2 && arg3 > 0) || (eTarget <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                    }
                    break;
                case 5:
                    //waitTime(int ms)
                    if (prevLine != line) {
                        runtime.reset();
                    }
                    if (runtime.milliseconds() > arg1) {
                        changeLine = true;
                    }
                    break;
                case 6:
                    //waitMotorTick(int motor, int waitUntilPosition, int sign)
                    switch (arg1) {
                        case 0:
                            if ((vTarget >= arg2 && arg3 > 0) || (vPosition <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                        case 2:
                            if ((eTarget >= arg2 && arg3 > 0) || (ePosition <= arg2 && arg3 < 0)) {
                                changeLine = true;
                            }
                            break;
                    }
                    break;
                case 7:
                    //waitTrajDone()
                    if (!drive.isBusy()) {
                        changeLine = true;
                    }
                    break;
                case 20:
                    //reset extend encoder
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    changeLine = true;
                    break;
                case 21:
                    switch (arg1) {
                        case 0:
                            vTargetTarget = arg2;
                            vTarget = arg2;
                            vSpeed = 0;
                            break;
                        case 2:
                            eTargetTarget = arg2;
                            eTarget = arg2;
                            eSpeed = 0;
                            break;
                    }
                    changeLine = true;
                    break;
                case 22:
                    //intakesensor1
                    eTargetTarget = extendPositions[arg1] + 50 + 110;
                    eTarget += 110;
                    changeLine = true;
                    eSpeed = 3;
                    break;
                case 23:
                    //intakesensor2
                    if (distanceSensor.getDistance(DistanceUnit.MM) < 25 || eSpeed == 0) {
                        changeLine = true;
                        eSpeed = 0;
                    }
                    break;
            }
            prevLine = line;
            if (changeLine) {
                line += 1;
            }
        }


        drive.update();

        //TELEMETRY
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.MM));
        telemetry.addData("line", line);
        telemetry.addData("ePosition", ePosition);
        telemetry.addData("eTarget", eTarget);
        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.update();

    }

    //PROGRAM BUILDING FUNCTIONS
    public void setServoPos(int servo, int target1k) {
        program.add(new int[] {1, servo, target1k, 0});
    }
    public void setMotorPos(int motor, int target, int speed) {
        program.add(new int[] {2, motor, target, speed});
    }

    public void setServoSpeed(int servo, int target, int speed) {
        program.add(new int[] {99, servo, target, speed});
    }

    public void followTraj(int trajno) {
        program.add(new int[] {3, trajno, 0, 0});
    }
    public void waitMotorTarget(int motor, int waitUntilPosition, int sign) {
        program.add(new int[] {4, motor, waitUntilPosition, sign});
    }
    public void waitTime(int ms) {
        program.add(new int[] {5, ms, 0, 0});
    }
    public void waitMotorTick(int motor, int waitUntilPosition, int sign) {
        program.add(new int[] {6, motor, waitUntilPosition, sign});
    }
    public void waitTrajDone() {
        program.add(new int[] {7, 0, 0, 0});
    }
    public void resetExtend() {
        program.add(new int[] {20, 0, 0, 0});
    }
    public void setMotorTarget(int motor, int position) {
        program.add(new int[] {21, motor, position, 0});
    }

    public void intakeSensor(int i) {
        program.add(new int[] {22, i, 0, 0});
    }
    public void intakeSensor2(int i) {
        program.add(new int[] {23, i, 0, 0});
    }

    public void buildProgram() {
        followTraj(1);
        setServoPos(VCLAW, vClawClose);
        setServoPos(VPITCH, vPitchMiddle);
        setServoPos(HCLAW, hClawOpen);
        waitTrajDone();
        for (int i = 0; i < 5; i++) {
            setServoSpeed(CLAWPITCH, clawPitchPositions[2 * i], 15);
            setServoSpeed(HPITCH, hPitchPositions[2 * i], -15);
            setServoPos(VPITCH, vPitchMiddle);
            setServoPos(ALIGN, alignOut);
            setMotorPos(EXTEND, extendPositions[i] - 160, 40);
            waitTime(150);
            setServoPos(VCLAW, vClawClose);
            setMotorPos(VARM, vHigh, 40);
            waitTime(200);
            waitMotorTick(VARM, vHigh - 150, 1);
            setServoPos(VPITCH, vPitchOut);
            intakeSensor(i);
            intakeSensor2(i);
            setServoPos(VCLAW, vClawOpen);
            setServoPos(ALIGN, alignIn);
            waitTime(50);
            setServoPos(HCLAW, hClawClose);
            setServoPos(VPITCH, vPitchMiddle);
            waitTime(150);
            setMotorPos(VARM, -15, -20);
            setServoSpeed(HPITCH, hPitchPositions[2 * i + 1], 15);
            setServoSpeed(CLAWPITCH, clawPitchPositions[2 * i + 1], 15);
            waitTime(130);
            setMotorPos(EXTEND, -15, -40);
            waitMotorTick(VARM, 160, -1);
            setServoPos(VPITCH, vPitchIn);
            setServoSpeed(HPITCH, hPitchTransfer - 100, 15);
            setServoSpeed(CLAWPITCH, clawPitchTransfer, -15);
            waitMotorTick(EXTEND, 30, -1);
            waitTime(100);
            resetExtend();
            setMotorTarget(EXTEND, -30);
            setServoPos(HPITCH, hPitchTransfer);
            waitTime(200);
            setServoPos(HCLAW, hClawOpen);
            waitTime(130);
        }
        setMotorTarget(EXTEND, -40);
        waitTime(130);
        setServoPos(HPITCH, hPitchTransfer - 50);
        setServoPos(CLAWPITCH, clawPitchTransfer + 100);
        setServoPos(VPITCH, vPitchMiddle);
        setServoPos(VCLAW, vClawClose);
        setServoPos(ALIGN, alignOut);
        waitTime(150);
        setMotorPos(VARM, vHigh, 20);
        waitMotorTick(VARM, vHigh - 120, 1);
        setServoPos(VPITCH, vPitchOut);
        waitTime(400);
        setServoPos(ALIGN, alignIn);
        setServoPos(VCLAW, vClawOpen);
        setServoPos(VPITCH, vPitchMiddle);
        followTraj(4);
        waitTime(150);
        setMotorPos(VARM, -15, -20);
        waitTrajDone();

        switch (finalTag) {
            case 1:
                followTraj(5);
                break;
            case 3:
                followTraj(6);
                break;
        }
    }
}