package org.firstinspires.ftc.teamcode.libraries;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.RRHWProfile;

public class IntakeControlThread implements Runnable{

    private final double STALL_CURRENT = 10;              // Value to check for stalled motor
    private final double EJECT_TIMER = 0.450;            // time to allow to auto-eject pixels
    private final double AUTO_SHUTOFF_TIMER = 0.75;         // time to wait before shutting down intake motor after determining
    private final double BUCKET_BEAM_TIMER = 0.250;
    private RRHWProfile robot = null;
    private CSAutoParams params = null;
    private boolean threadIsRunning = true;              // controls when thread is killed
    private boolean intakeReverse = false;               //
    private boolean powerOn = false;
    private boolean kickDelaySet = false;
    private boolean timeOutSet = false;
    private boolean isDormant = false;
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime kickTime = new ElapsedTime();
    private ElapsedTime timeoutTimer = new ElapsedTime();
    private double kickDelay = 0.5;
    private boolean bottomBeamFlag = false;
    private boolean topBeamFlag = false;
    private ElapsedTime topBeamTime = new ElapsedTime();
    private boolean bucketIsFull = false;
    private PIDController controller=null;


    public int target=0, offset=0;

    public IntakeControlThread(RRHWProfile myRobot){
        this.robot = myRobot;
        this.params = new CSAutoParams();

        controller=new PIDController(params.P,params.I,params.D);
    }

    public void setLiftTarget(int ticks, int bottomOffset){
        target=ticks;
        offset=bottomOffset;
    }

    private void teleop_runTo(int target, int offset){
        controller.setPID(params.P,params.I,params.D);
        double pid = controller.calculate((robot.motorLiftFront.getCurrentPosition()-offset), target);
        double ff = Math.cos(Math.toRadians((target/params.ticks_in_degrees)))*params.F;
        robot.lift.set(Range.clip(pid + ff, -1, 1));
    }

    private void auto_runTo(int target){
        robot.auto_MotorLiftFront.setPower(1);
        robot.auto_MotorLiftFront.setTargetPosition(target);
    }

    private void intakeForward() {
        robot.motorIntake.setPower(params.INTAKE_IN_PWR);
    }

    private void intakeReverse() {
        robot.motorIntake.setPower(params.INTAKE_OUT_PWR);
    }

    private void intakeOff() {
        robot.motorIntake.setPower(0);
    }

    public void setIntakeOn(){
        kickTime.reset();
        powerOn = true;
    }

    public void setIntakeOff(){
        powerOn = false;
    }

    public void setKickDelay(double kickDelay){
        if(powerOn) {
            this.kickDelay = kickDelay;
            this.kickDelaySet = true;
            kickTime.reset();
        }
    }

    private void ejectPixels() {
        robot.motorIntake.setPower(-1);
        kickTime.reset();
    }

    public void setIntakeReverse(){
        intakeReverse = true;
        intakeReverse();
    }

    public double getIntakeAmps() {
        return robot.motorIntake.getCurrent(CurrentUnit.AMPS);
    }

    public boolean checkStallCondition(){

        return (robot.motorIntake.getCurrent(CurrentUnit.AMPS) > STALL_CURRENT);
    }

    private boolean bottomBeamMonitor(){
        if(!robot.beamBucketBottom.getState()){
            if(!bottomBeamFlag){
                bottomBeamFlag = true;
            }
        } else {
            bottomBeamFlag = false;
        }
        return bottomBeamFlag;
    }

    private boolean topBeamMonitor(){
        if(!robot.beamBucketTop.getState()){
            if(!topBeamFlag){
                topBeamFlag = true;
                topBeamTime.reset();
            }
        } else {
            topBeamFlag = false;
        }
        return topBeamFlag;
    }

    private boolean bucketIsFull(){
        // monitor the beam-break sensors in bucket and shut down the intake when two pixels are present
        if(bottomBeamMonitor() && topBeamMonitor()){
            if(topBeamTime.time() >= BUCKET_BEAM_TIMER){
                bucketIsFull = true;
            }
        } else bucketIsFull = false;

        return bucketIsFull;       //TODO: finish this method when the hardware is ready
    }

    private void turnFrontLEDGreen(){
        robot.ledRightFrontGreen.setState(true);
        robot.ledRightFrontRed.setState(false);
        robot.ledLeftFrontGreen.setState(true);
        robot.ledLeftFrontRed.setState(false);
    }

    private void turnFrontLEDRed(){
        robot.ledRightFrontGreen.setState(false);
        robot.ledRightFrontRed.setState(true);
        robot.ledLeftFrontGreen.setState(false);
        robot.ledLeftFrontRed.setState(true);
    }

    private void turnRearLEDGreen(){
        robot.ledRightRearGreen.setState(true);
        robot.ledRightRearRed.setState(false);
        robot.ledLeftRearGreen.setState(true);
        robot.ledLeftRearRed.setState(false);
    }

    private void turnRearLEDRed(){
        robot.ledRightRearGreen.setState(false);
        robot.ledRightRearRed.setState(true);
        robot.ledLeftRearGreen.setState(false);
        robot.ledLeftRearRed.setState(true);
    }

    public void setDormant (boolean dormancy){
        this.isDormant = isDormant;
    }

    private void LEDControl(){
        if(bottomBeamMonitor()){
            turnFrontLEDGreen();
        } else {
            turnFrontLEDRed();
        }

        if(topBeamMonitor()){
            turnRearLEDGreen();
        } else {
            turnRearLEDRed();
        }
    }

    private void controlMechs(){
        if(robot.opModeTeleop) {
            teleop_runTo(target, offset);
        } else if(!robot.opModeTeleop){
            auto_runTo(target);
        }

        // check for normal conditions
        if(!intakeReverse && powerOn && (runTime.time() > EJECT_TIMER)) {
            intakeForward();
        }else if(intakeReverse && powerOn){
            intakeReverse();
        }

        if (kickDelaySet && (kickTime.time() > kickDelay)){
            intakeReverse = true;
            ejectPixels();
            kickDelaySet = false;        // flag to monitor when to turn on reverse intake to kick out extra pixels
            timeoutTimer.reset();        // start clock to time out the intake
            timeOutSet = true;           // tells process to turn off when the timeout has been reached
        }

        if(powerOn && timeOutSet && (timeoutTimer.time() > AUTO_SHUTOFF_TIMER)) {
            // turn off intake and reset all values
            setIntakeOff();
            intakeReverse = false;
            kickDelaySet = false;
            timeOutSet = false;
        }
        if(!powerOn) {
            intakeOff();
            intakeReverse = false;
            timeOutSet = false;
            kickDelaySet = false;
        }

        // check for intake motor stall condition
        // if motor is stalled, reverse spin it to eject the pixel causing the stall
        if(checkStallCondition()){
            runTime.reset();        // measure time for reverse before going forward again
            ejectPixels();        // kick out the pixels causing stall
            kickTime.reset();       // restart kicktime to provide more time for proper intake of pixels
        }


        // monitor pixel count
        // Kickout extra pixels if there are more than 1 in the bucket
        if(!intakeReverse && bucketIsFull() && powerOn){
            intakeReverse = true;
            ejectPixels();
            timeoutTimer.reset();
            timeOutSet = true;
        }

        LEDControl();           // set the color of the LEDs based on Pixel loading
    }

    public void stopThread(){
        threadIsRunning = false;
    }

    @Override
    public void run() {
        while (threadIsRunning){
            if (!isDormant) {
                controlMechs();
            }
            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }   // end of try - catch
        }   // end of while(isRunning)
    }
}
