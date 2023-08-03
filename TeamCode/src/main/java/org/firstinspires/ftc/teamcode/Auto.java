package org.firstinspires.ftc.teamcode;

/**
 * Stores some constants common to all of the autonomous OpModes
 * Includes PID values, servo and motor position presets, and servo and motor ID's
 */
public class Auto extends HardwareOpMode {
    public final int hClawOpen = 580;
    public final int hClawClose = 390;
    public final int hClawOpen2 = 210;
    public final int hClawClose2 = 400;
    public final int vClawOpen = 570;
    public final int vClawClose = 515;
    public final int vPitchIn = 170;
    public final int vPitchMiddle = 500;
    public final int vPitchOut = 840;
    public final int alignIn = 10;
    public final int alignOut = 307;
    public final int alignRetract = 469;
    public final int hPitchTransfer = 730;
    public final int clawPitchTransfer = 40;
    public final int clawPitchHome = 600;
    public final int vHigh = 600;

    public final int VPITCH = 0, VARM = 0, VCLAW = 1, HCLAW = 2, EXTEND = 2, ALIGN = 3, HPITCH = 4, CLAWPITCH = 5;

    public double Pv = 0.005, Iv = 0, Dv = 0, Fv = 0.2;
    public double Pe = 0.006, Ie = 0, De = 0;
}
