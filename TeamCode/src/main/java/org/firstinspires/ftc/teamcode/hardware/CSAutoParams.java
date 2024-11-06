package org.firstinspires.ftc.teamcode.hardware;

public class CSAutoParams {
    /*

    PARAMETER MAP - PLEASE READ BEFORE EDITING

    This file contains all the parameters for RRAuto.
    This file also contains all Lift encoder values for cone retrieval from the stack.

    If making a new auto, include "AutoParams params = new AutoParams();" within the class.

    All headings are converted to radians here, so there is no need to convert them in the main autonomous programs.

    All values are based on BlueTerminalAuto and are negated as needed in RedTerminalAuto
     */

    //lift PID values
    public final double P=0.0035;
    public final double I=0;
    public final double D=0.000001;
    public final double F=0.05;

    //tflite file name
    public final String tfliteFileName = "PP_Generic_SS.tflite";

    //constants
    public final boolean fieldCentric=true;

    //lift PID constants
    public final double kF=-0.2;
    public final double ticks_in_degrees = 384.5/360;

    //drive constants
    final public double TURN_MULTIPLIER = 0.75;
    public final int USD_COUNTS_PER_INCH = 23;
    final public int DRIVE_TICKS_PER_INCH = 23;        // needs to be set
    public final double DRIVE_TICKS_PER_ROTATION = 145.6;


    //arm positions
    public final double ARM_COLLAPSED_INTO_ROBOT;
    public final double ARM_COLLECT               = 0;
    public final int    ARM_CLEAR_BARRIER         = 275;
    //    public final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    public final int    ARM_SCORE_SPECIMEN        = 300;
    //    public final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    public final double ARM_SCORE_SAMPLE_IN_LOW   = 100;
    public final double ARM_ATTACH_HANGING_HOOK   = 150;
    public final double ARM_WINCH_ROBOT           = 0;
    public final int    ARM_HIGH_SCORE            = 600;
    public final double ARM_EXTENSION_ANGLE       = 400;




    //Distance Sensor
    final public double M = 0.000448004;
    final public double B = 0.138705;

    public CSAutoParams() {
        ARM_COLLAPSED_INTO_ROBOT = 100;
    }
    //Distance calculation (x) in inches
    // [(y-b)/m]/25.4mm  where y = sensor voltage reading

}
