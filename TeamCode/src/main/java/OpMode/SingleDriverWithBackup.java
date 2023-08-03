package OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Driver;

/**
 * Driver 1 can solo drive.
 * Driver 2 is the 'backup' driver, responsible for:
 *      grabbing cones on stacks between 2-5 cones high
 *      manual adjustment of every motor and servo with the exception of extend
 */
@TeleOp
public class SingleDriverWithBackup extends Driver
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();

    final double driveSpeed = 1;
    final double turnSpeed = 0.8;

    //region OTHER
    int resetState = 0;
    boolean pReset = false;
    double resetStartTime = 0;

    private String autoProcess = "none";
    int autoState = 0;
    boolean pHome = false;
    boolean pState = false;
    boolean pHigh = false;
    boolean pMed = false;
    boolean pLow = false;
    boolean pGrabclose = false;
    boolean pGrabfar = false;
    boolean pGround = false;
    boolean pnewground = false;
    boolean pHighCycle = false;
    boolean pTransfer = false;

    double vPitchPos = vPitchMiddle;
    double alignPos = alignIn;
    double clawPitchPos = clawPitchHome;
    double hPitchPos = hPitchTransfer - 0.04;

    boolean prevVClaw = false, prevHClaw = false;
    boolean hClawIsOpen = true, vClawIsOpen = true;
    //endregion OTHER

    boolean pStack = false;
    boolean pStackGrab = false;
    int stackCount = 1;

    //region PIDS
    public PIDController vController;
    public PIDController eController;

    public double Pv = 0.005, Iv = 0, Dv = 0, Fv = 0.2;
    public double Pe = 0.006, Ie = 0, De = 0;
    double maxExtendSpeed = 1.0;

    public int vTarget = 0;
    public int eTarget = 0;
    //endregion PIDS

    double[] hPitchPositions = {0, 0.480, 0.417, 0.353, 0.300};
    double[] clawPitchPositions = {0, 0.440, 0.404, 0.351, 0.276};

    @Override
    public void init() {
        super.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LynxModule chub = hardwareMap.getAll(LynxModule.class).get(0);
        double voltage = chub.getInputVoltage(VoltageUnit.VOLTS);
        maxExtendSpeed = -0.04 * voltage + 1.46;
        maxExtendSpeed = Math.min(1.0, Math.max(0.9, maxExtendSpeed));
        Pe = -0.0004 * voltage + 0.0106;
        Pe = Math.min(0.006, Math.max(0.005, Pe));

        telemetry.addData("voltage", voltage);
        telemetry.update();

        vController = new PIDController(Pv, Iv, Dv);
        eController = new PIDController(Pe, Ie, De);
    }

    public void start() {
        totalTime.reset();
    }

    @Override
    public void loop() {

        if (totalTime.seconds() > 90 && totalTime.seconds() < 91) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        if (totalTime.seconds() > 115 && totalTime.seconds() < 116) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        Pose2d poseEstimate = drive.getPoseEstimate();
        int vPosition = vArmLeft.getCurrentPosition();
        int ePosition = extend.getCurrentPosition();

        vController.setPID(Pv, Iv, Dv);
        eController.setPID(Pe, Ie, De);

        //region PID UPDATING
        double vPID = vController.calculate(vPosition, vTarget);
        double ePID = eController.calculate(ePosition, eTarget) * maxExtendSpeed;
        if (Math.abs(ePID) < 0.05) {
            ePID = 0;
        }

        double ticks_per_degree_tetrix = 3.84444444444444444444444444444;
        double vFeed = Math.cos(Math.toRadians((vTarget - 240) / ticks_per_degree_tetrix)) * Fv;

        if (vTarget < 0) {
            vFeed = -vFeed;
        }

        vArmLeft.setPower(vPID + vFeed);
        vArmRight.setPower(vPID + vFeed);
        extend2.setPower(ePID);
        extend.setPower(ePID);
        //endregion PID UPDATING

        drive.setWeightedDrivePower(
                new Pose2d(
                        driveSpeed * -gamepad1.left_stick_y, driveSpeed * -gamepad1.left_stick_x,
                        -turnSpeed * gamepad1.right_stick_x
                )
        );
        drive.update();

        //region CLAWS
        boolean vClawButton = gamepad2.x;
        boolean hClawButton = gamepad2.b;

        //Vertical Claw
        if (vClawButton && !prevVClaw) {
            vClawIsOpen = !vClawIsOpen;
        }
        if (vClawIsOpen) {
            vClaw.setPosition(vClawOpen);
        } else {
            vClaw.setPosition(vClawClose);
        }
        prevVClaw = vClawButton;

        //Horizontal Claw
        if (hClawButton && !prevHClaw) {
            hClawIsOpen = !hClawIsOpen;
        }
        if (hClawIsOpen) {
            hClaw.setPosition(hClawOpen);
            hClaw2.setPosition(hClawOpen2);
        } else {
            hClaw.setPosition(hClawClose);
            hClaw2.setPosition(hClawClose2);
        }
        prevHClaw = hClawButton;
        //endregion CLAWS

        //region MANUAL MOVEMENT
        if (gamepad2.left_bumper) { alignPos -= 0.02; }
        if (gamepad2.right_bumper) { alignPos += 0.02; }
        if (gamepad2.a) { vPitchPos += 0.02; }
        if (gamepad2.y) { vPitchPos -= 0.02; }
        hPitchPos -= 0.02 * gamepad2.left_stick_y;
        clawPitchPos += 0.03 * gamepad2.right_stick_y;

        alignPos = Math.min(alignOut, Math.max(alignIn, alignPos));
        vPitchPos = Math.min(vPitchOut, Math.max(vPitchIn, vPitchPos));
        hPitchPos = Math.min(hPitchTransfer, Math.max(hPitchIntake, hPitchPos));
        clawPitchPos = Math.min(0.6, Math.max(0, clawPitchPos));

        vTarget -= Math.round(15.0 * gamepad2.left_trigger);
        vTarget += Math.round(25.0 * gamepad2.right_trigger);
        vTarget = Math.min(vMax, Math.max(-20, vTarget));

        if (eTarget != -100) {
            eTarget += (int)Math.round(30.0 * gamepad1.right_trigger);
            if(!(gamepad1.left_stick_button && gamepad1.right_bumper)) {eTarget = Math.min(1020, Math.max(-15, eTarget));}
            if (gamepad1.right_bumper) {
                eTarget -= 30;
                if(!(gamepad1.left_stick_button && gamepad1.right_bumper)) {eTarget = Math.min(1020, Math.max(-15, eTarget));}
            }
        }

        switch (resetState) {
            case 0:
                if ((gamepad1.left_stick_button || gamepad2.left_stick_button) && !pReset) {
                    resetState += 1;
                }
                break;
            case 1:
                resetState += 1;
                resetStartTime = runtime.milliseconds();
                eTarget = -150;
                break;
            case 2:
                if (runtime.milliseconds() > resetStartTime + 500) {
                    eTarget = -15;
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    resetState = 0;
                }
        }
        pReset = gamepad1.left_stick_button || gamepad2.left_stick_button;


        //endregion MANUAL MOVEMENT

        //region controls
        boolean home = gamepad1.b || gamepad2.right_stick_button;
        boolean grabclose = gamepad1.left_bumper;
        boolean grabfar = gamepad1.left_trigger > 0.5;
        boolean transfer = gamepad2.start;
        boolean low = gamepad1.a;
        boolean med = gamepad1.x;
        boolean high = gamepad1.y;
        boolean HighCycle = gamepad1.share;
        boolean newground = gamepad1.dpad_right;
        boolean ground = gamepad1.dpad_left;
        boolean state = gamepad1.right_stick_button;

        if (gamepad2.dpad_left && !pStack) {
            stackCount += 1;
            if (stackCount > 4) {
                stackCount = 1;
            }
        }
        pStack = gamepad2.dpad_left;

        boolean stackGrab = gamepad2.dpad_right;
        //endregion controls

        if (home && !pHome) {
            autoState = 0;
            autoProcess = "home";
            eTarget = extend.getCurrentPosition();
            vTarget = vArmLeft.getCurrentPosition();
        }

        if (state && !pState) {
            autoState = 0;
            autoProcess = "none";
        }

        switch (autoProcess) {
            case "none":
                if (grabclose && !pGrabclose) { autoProcess = "grabclose"; }
                if (grabfar && !pGrabfar) { autoProcess = "grabfar"; }
                if (ground && !pGround) { autoProcess = "ground"; }
                if (newground && !pnewground) { autoProcess = "newground"; }
                if (HighCycle && !pHighCycle) { autoProcess = "HighCycle"; }
                if (transfer && !pTransfer) {
                    if (vPitchPos != vPitchIn) {
                        vPitchPos = vPitchIn;
                    } else {
                        autoProcess = "transfer";
                    }
                }
                if (low && !pLow) { autoProcess = "low"; }
                if (med && !pMed) { autoProcess = "med"; }
                if (high && !pHigh) { autoProcess = "high"; }
                if (stackGrab && !pStackGrab) { autoProcess = "stackGrab"; }
                break;
            case "home":
                switch (autoState) {
                    case 0:
                        autoState += 1;
                        hClawIsOpen = false;
                        alignPos = alignIn;
                        hPitchPos = hPitchTransfer - 0.04;
                        clawPitchPos = clawPitchHome;
                        vPitchPos = vPitchMiddle;
                        eTarget = 0;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 200) {
                            vClawIsOpen = false;
                            if (vTarget > -30) {
                                vTarget -= 15;
                            }
                            if (ePosition < 20) {
                                autoState += 1;
                                resetState = 1;
                            }
                        }
                        break;
                    case 2:
                        if (vTarget > -20) {
                            vTarget -= 30;
                        } else {
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "grabclose":
                switch (autoState) {
                    case 0:
                        clawPitchPos = clawPitchIntake;
                        hClawIsOpen = true;
                        runtime.reset();
                        autoState += 1;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 200) {
                            autoState += 1;
                            hPitchPos = hPitchIntake;
                            vPitchPos = vPitchIn;
                        }
                        break;
                    case 2:
                        if (grabclose && !pGrabclose && !gamepad1.options) {
                            eTarget += 150;
                            autoState += 1;
                            hClawIsOpen = false;
                            runtime.reset();
                        } else if (gamepad1.options && grabclose && !pGrabclose) {
                            hClawIsOpen = false;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 3:
                        vClawIsOpen = true;
                        hClawIsOpen = false;
                        runtime.reset();
                        vPitchPos = vPitchIn;
                        autoState = 69;
                        break;
                    case 69:
                        if (runtime.milliseconds() > 600) {
                            clawPitchPos = clawPitchHome;
                            autoState = 4;
                            runtime.reset();
                            eTarget = -100;
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 200 && ePosition < 200) {
                            hPitchPos = hPitchTransfer;
                            eTarget = -20;
                            if (runtime.milliseconds() > 400) {
                                clawPitchPos = clawPitchTransfer;
                                autoState += 1;
                            }
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 500 && grabclose && !pGrabclose) {
                            hClawIsOpen = true;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 6:
                        if (runtime.milliseconds() > 300) {
                            autoState += 1;
                        }
                        break;
                    case 7:
                        autoState = 0;
                        autoProcess = "none";
                        hPitchPos = hPitchTransfer - 0.04;
                        clawPitchPos = clawPitchHome;
                        break;
                }
                break;
            case "stackGrab":
                switch (autoState) {
                    case 0:
                        clawPitchPos = clawPitchPositions[stackCount];
                        hClawIsOpen = true;
                        runtime.reset();
                        autoState += 1;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 200) {
                            autoState += 1;
                            hPitchPos = hPitchPositions[stackCount];
                            vPitchPos = vPitchIn;
                        }
                        break;
                    case 2:
                        if (grabclose && !pGrabclose) {
                            hClawIsOpen = false;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 3:
                        if (runtime.milliseconds() > 100) {
                            vClawIsOpen = true;
                            hClawIsOpen = false;
                            runtime.reset();
                            vPitchPos = vPitchIn;
                            autoState = 69;
                        }
                        break;
                    case 69:
                        if (runtime.milliseconds() > 200) {
                            hPitchPos = hPitchTransfer;
                            clawPitchPos = clawPitchTransfer;
                            autoState = 70;
                            runtime.reset();
                        }
                        break;
                    case 70:
                        if (runtime.milliseconds() > 200) {
                            if (eTarget > eTooFar) {
                                eTarget = eSlam;
                            } else {
                                eTarget = -100;
                            }
                            autoState = 4;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 200 && ePosition < 200) {
                            eTarget = -50;
                            if (runtime.milliseconds() > 400) {
                                autoState += 1;
                            }
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 500 && grabclose && !pGrabclose) {
                            hClawIsOpen = true;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 6:
                        if (runtime.milliseconds() > 350) {
                            autoState = 0;
                            autoProcess = "none";
                            hPitchPos = hPitchTransfer - 0.04;
                            clawPitchPos = clawPitchHome;
                            stackCount = 1;
                        }
                        break;
                }
                break;
            case "grabfar":
                switch (autoState) {
                    case 0:
                        autoState += 1;
                        runtime.reset();
                        clawPitchPos = clawPitchIntake;
                        hClawIsOpen = true;
                        eTarget = 650;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 50) {
                            autoState += 1;
                            hPitchPos = hPitchIntake;
                            vPitchPos = vPitchIn;
                        }
                        break;
                    case 2:
                        if (ePosition > 400) {
                            autoState += 1;
                        }
                        break;
                    case 3:
                        if (grabfar && !pGrabfar && !gamepad1.options) {
                            eTarget += 250;
                            autoState += 1;
                            runtime.reset();
                        } else if (gamepad1.options && grabfar && !pGrabfar) {
                            hClawIsOpen = false;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 5) {
                            vClawIsOpen = true;
                            hClawIsOpen = false;
                            runtime.reset();
                            vPitchPos = vPitchIn;
                            autoState = 420;
                        }
                        break;
                    case 420:
                        if (runtime.milliseconds() > 350) {
                            eTarget = eSlam + 200;
                            hPitchPos = hPitchTransfer;
                            clawPitchPos = clawPitchHome;
                            autoState = 5;
                            runtime.reset();
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 500) {
                            eTarget = -120;
                            if (runtime.milliseconds() > 900) {
                                clawPitchPos = clawPitchTransfer;
                                autoState += 1;
                            }
                        }
                        break;
                    case 6:
                        if (runtime.milliseconds() > 500 && grabfar && !pGrabfar) {
                            hClawIsOpen = true;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 7:
                        if (runtime.milliseconds() > 350) {
                            autoState += 1;
                        }
                        break;
                    case 8:
                        autoState = 0;
                        autoProcess = "none";
                        hPitchPos = hPitchTransfer - 0.04;
                        clawPitchPos = clawPitchHome;
                        break;
                }
                break;
            case "transfer":
                switch (autoState) {
                    case 0:
                        eTarget = eSlam;
                        vClawIsOpen = true;
                        hClawIsOpen = false;
                        hPitchPos = hPitchTransfer;
                        clawPitchPos = clawPitchHome;
                        runtime.reset();
                        vPitchPos = vPitchIn;
                        autoState += 1;
                        break;
                    case 1:
                        if (ePosition < 100) {
                            eTarget = -20;
                            if (runtime.milliseconds() > 150) {
                                clawPitchPos = clawPitchTransfer;
                                autoState += 1;
                            }
                        }
                        break;
                    case 2:
                        if (runtime.milliseconds() > 500 && transfer && !pTransfer) {
                            hClawIsOpen = true;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 3:
                        if (runtime.milliseconds() > 350) {
                            hPitchPos = hPitchTransfer - 0.04;
                            clawPitchPos = clawPitchHome;
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "low":
                switch (autoState) {
                    case 0:
                        autoState += 1;
                        vPitchPos = 0.87;
                        runtime.reset();
                        break;
                    case 1:
                        if (runtime.milliseconds() > 100) {
                            autoState += 1;
                            vClawIsOpen = false;
                        }
                        break;
                    case 2:
                        if (low && !pLow) {
                            vClawIsOpen = true;
                            alignPos = alignIn;
                            runtime.reset();
                            autoState += 1;
                        }
                        break;
                    case 3:
                        if (runtime.milliseconds() > 200) {
                            vPitchPos = vPitchIn;
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "ground":
                switch (autoState) {
                    case 0:
                        clawPitchPos = clawPitchIntake;
                        hClawIsOpen = true;
                        runtime.reset();
                        autoState += 1;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 200) {
                            autoState += 1;
                            hPitchPos = hPitchIntake;
                            vPitchPos = vPitchIn;
                        }
                        break;
                    case 2:
                        if (ground && !pGround && !gamepad1.options) {
                            eTarget += 230;
                            autoState += 1;
                            runtime.reset();
                        } else if (gamepad1.options && ground && !pGround) {
                            hClawIsOpen = false;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 3:
                        if (runtime.milliseconds() > 250) {
                            hClawIsOpen = false;
                            runtime.reset();
                            vPitchPos = vPitchIn;
                            autoState = 69;
                        }
                        break;
                    case 69:
                        if (runtime.milliseconds() > 350) {
                            eTarget = eSlam;
                            hPitchPos = hPitchIntake + 0.05;
                            clawPitchPos = clawPitchIntake;
                            autoState = 4;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 300) {
                            eTarget = -30;
                            autoState += 1;
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 500 && ground && !pGround) {
                            hClawIsOpen = true;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 6:
                        if (runtime.milliseconds() > 100) {
                            autoState += 1;
                        }
                        break;
                    case 7:
                        autoState = 0;
                        autoProcess = "none";
                        hPitchPos = hPitchTransfer - 0.04;
                        clawPitchPos = clawPitchHome;
                        break;
                }
                break;
            case "high":
                switch (autoState) {
                    case 0:
                        autoState += 1;
                        vPitchPos = vPitchMiddle;
                        runtime.reset();
                        break;
                    case 1:
                        if (runtime.milliseconds() > 100) {
                            autoState += 1;
                            vClawIsOpen = false;
                            alignPos = 0.359;
                        }
                        break;
                    case 2:
                        if (vTarget < vHigh) {
                            vTarget += 25;
                        }
                        if (vTarget >= vHigh) {
                            vTarget = vHigh;
                            vPitchPos = 0.8;
                            alignPos = 0.359;
                            autoState += 1;
                        }
                        break;
                    case 3:
                        if (high && !pHigh) {
                            autoState += 1;
                            vClawIsOpen = true;
                            alignPos = alignIn;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 150) {
                            autoState = 5;
                            runtime.reset();
                            vPitchPos = vPitchMiddle;
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 300) {
                            autoState = 6;
                            runtime.reset();
                        }
                        break;
                    case 6:
                        if (vTarget > -30) {
                            vTarget -= 25;
                        }
                        if (vTarget <= 100) {
                            vClawIsOpen = true;
                            vPitchPos = vPitchIn;
                        }
                        if (vTarget <= 0) {
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "med":
                switch (autoState) {
                    case 0:
                        autoState += 1;
                        vPitchPos = vPitchMiddle;
                        runtime.reset();
                        break;
                    case 1:
                        if (runtime.milliseconds() > 100) {
                            autoState += 1;
                            alignPos = 0.312;
                            vClawIsOpen = false;
                        }
                        break;
                    case 2:
                        if (vTarget < vMed) {
                            vTarget += 25;
                        }
                        if (vTarget >= vMed) {
                            vTarget = vMed;
                            vPitchPos = 0.82;
                            alignPos = 0.312;
                            autoState += 1;
                        }
                        break;
                    case 3:
                        if (med && !pMed) {
                            autoState += 1;
                            vClawIsOpen = true;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 100) {
                            vPitchPos = vPitchMiddle;
                            alignPos = alignIn;
                            autoState = 5;
                            runtime.reset();
                        }
                        break;
                    case 5:
                        if (vTarget > -30) {
                            vTarget -= 15;
                        }
                        if (vTarget <= 100) {
                            vClawIsOpen = true;
                            vPitchPos = vPitchIn;
                        }
                        if (vTarget <= 0) {
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "newground":
                switch (autoState) {
                    case 0:
                        autoState += 1;
                        hPitchPos = hPitchIntake + 0.05;
                        clawPitchPos = clawPitchIntake;
                        eTarget = 0;
                        break;
                    case 1:
                        if (newground && !pnewground) {
                            hClawIsOpen = true;
                            runtime.reset();
                            autoState += 1;

                        }
                        break;
                    case 2:
                        if (runtime.milliseconds() > 100) {
                            autoState = 0;
                            hPitchPos = hPitchTransfer;
                            clawPitchPos = clawPitchHome;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "HighCycle":
                switch (autoState) {
                    case 0:
                        vClawIsOpen = true;
                        hClawIsOpen = true;
                        vPitchPos = vPitchIn;
                        alignPos = alignIn;
                        clawPitchPos = clawPitchIntake;
                        eTarget = 190;
                        autoState += 1;
                        runtime.reset();
                        break;
                    case 1:
                        if (runtime.milliseconds() > 50) {
                            hPitchPos = hPitchIntake;
                            autoState = 3;
                        }
                        break;
                    case 3:
                        if (vTarget > -30) {
                            vTarget -= 25;
                        }
                        if (vPosition <= 100){
                            vClawIsOpen = true;
                            vPitchPos = vPitchIn;
                            runtime.reset();
                            autoState = 69;
                        }
                        break;
                    case 69:
                        if (vTarget > -30) {
                            vTarget -= 25;
                        }
                        if (HighCycle && !pHighCycle && !gamepad1.options) {
                            eTarget += 300;
                            autoState = 4;
                            runtime.reset();
                        } else if (gamepad1.options && HighCycle && !pHighCycle) {
                            hClawIsOpen = false;
                            autoState = 4;
                            runtime.reset();
                        }
                        break;
                    case 4:
                        hClawIsOpen = false;
                        if (vTarget > -30) {
                            vTarget -= 25;
                        }
                        if (vPosition <= 100){
                            vClawIsOpen = true;
                            vPitchPos = vPitchIn;
                        }
                        if (runtime.milliseconds() > 300) {
                            hPitchPos = hPitchTransfer;
                            if (eTarget > eTooFar) {
                                eTarget = eSlam;
                            } else {
                                eTarget = -30;
                            }
                            autoState = 6;
                            runtime.reset();
                        }
                        break;
                    case 6:
                        if (runtime.milliseconds() > 50) {
                            eTarget = - 30;
                            clawPitchPos = clawPitchTransfer;
                        }
                        if (runtime.milliseconds() > 200 && HighCycle && !pHighCycle) {
                            hClawIsOpen = true;
                            autoState += 1;
                            runtime.reset();
                        }
                        break;
                    case 7:
                        if (runtime.milliseconds() > 300) {
                            hPitchPos = hPitchIntake;
                            autoState += 1;
                            runtime.reset();
                            eTarget = 200;
                        }
                        break;
                    case 8:
                        if (runtime.milliseconds() > 100) {
                            clawPitchPos = clawPitchIntake;
                            vPitchPos = vPitchMiddle;
                            autoState += 1;
                        }
                        break;
                    case 9:
                        if (runtime.milliseconds() > 300) {
                            hClawIsOpen = true;
                            vClawIsOpen = false;
                            autoState += 1;
                            runtime.reset();
                            alignPos = 0.352;
                        }
                        break;
                    case 10:
                        if (vTarget < vHigh) {
                            vTarget += 25;
                        } else if (vTarget > vHigh) {
                            vTarget = vHigh;
                        } else {
                            vPitchPos = 0.79;
                            alignPos = 0.280;
                            autoState += 1;
                        }
                        break;
                    case 11:
                        if (HighCycle && !pHighCycle) {
                            autoState += 1;
                            vClawIsOpen = true;
                            alignPos = alignIn;
                            runtime.reset();
                        }
                        break;
                    case 12:
                        if (runtime.milliseconds() > 400){
                            vPitchPos = vPitchMiddle;
                            autoState = 69;
                        }
                        break;
                }
                break;
        }

        pHome = home;
        pState = state;
        pGrabclose = grabclose;
        pGrabfar = grabfar;
        pLow = low;
        pMed = med;
        pHigh = high;
        pTransfer = transfer;
        pGround = ground;
        pnewground = newground;
        pHighCycle = HighCycle;
        pStackGrab = stackGrab;
        vPitch.setPosition(vPitchPos);
        align.setPosition(alignPos);
        clawPitch.setPosition(clawPitchPos);
        setHPitch(hPitchPos);

        telemetry.addData("stackCount", stackCount);
        telemetry.addData("totalTime", totalTime.seconds());
        telemetry.addData("clawPitch", clawPitchPos);
        telemetry.addData("hPitch", hPitchPos);
        telemetry.addData("ePosition", ePosition);
        telemetry.addData("eTarget", eTarget);
        telemetry.addData("vPosition", vPosition);
        telemetry.addData("vTarget", vTarget);
        telemetry.addData("autoProcess", autoProcess);
        telemetry.addData("autoState", autoState);
        telemetry.addData("resetState", resetState);
        telemetry.update();
    }

    public void setHPitch(double pos) {
        lPitch.setPosition(pos);
        rPitch.setPosition(pos + 0.02);
    }
}