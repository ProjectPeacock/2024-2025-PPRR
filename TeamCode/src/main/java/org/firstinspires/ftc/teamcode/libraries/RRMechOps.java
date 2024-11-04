package org.firstinspires.ftc.teamcode.libraries;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RRMechOps {

    public HWProfile robot;
    public LinearOpMode opMode;
    public CSAutoParams params;

    /*
     * Constructor method
     */
    public RRMechOps(HWProfile myRobot, LinearOpMode myOpMode, CSAutoParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close DriveMecanum constructor Method

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */


//    public void intakePower(double power){robot.motorIntake.setPower(power);}

    public void clawOpen(){
        robot.claw.setPosition(params.INTAKE_DOWN_RIGHT);
    }

    public void clawClose(){
        robot.claw.setPosition(params.INTAKE_MID_RIGHT);    // TODO: create target position constant
    }

    public void wristRotate(){
        robot.wrist.setPosition(params.INTAKE_UP_RIGHT);    // TODO: create target position constant
    }

    public void wristOut(){
        robot.wrist.setPosition(params.INTAKE_UP_RIGHT);    // TODO: create target position constant
    }

    private void armUp(){

        robot.hangMotor.setPower(1);      // TODO: create power constant
        robot.hangMotor.setTargetPosition(1); // TODO: create target position constant
    }

    private void armDown(){

        robot.hangMotor.setPower(1);        // TODO: create power constant
        robot.hangMotor.setTargetPosition(1);   // TODO: create target position constant
    }



    /***
     * This method provides a delay that will cancel if the program is stopped
     * @param time
     */
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();

        // wait while a stop is not requested
        while (!opMode.isStopRequested() && timer.time() < time) {
        }
    }


}   // close the RRMechOps class