package org.firstinspires.ftc.teamcode.libraries;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;

public class RRMechOps2 {

    public HWProfile robot;
    public LinearOpMode opMode;
    public CSAutoParams params;

    /*
     * Constructor
     */
    public RRMechOps2(HWProfile myRobot, LinearOpMode myOpMode, CSAutoParams autoParams){
        robot = myRobot;
        opMode = myOpMode;
        params = autoParams;

    }   // close RRMechOps constructor

    /*
     * ###########################################################################################
     * ###########################################################################################
     * #######################      SYSTEM CONTROLS      #########################################
     * ###########################################################################################
     * ###########################################################################################
     */

    public void clawOpen(){

        robot.claw.setPosition(params.ARM_ATTACH_HANGING_HOOK);
    }

    public void rotateClaw(double targetClawPosition){
        robot.wrist.setPosition(targetClawPosition);
    }

    public void clawClose(){
       // robot.claw.setPosition(params.);    // TODO: create target position constant
    }

    public void scoreHighBucket(){
        // set arm position to up
        robot.armMotor.setPower(1);

        // extend arm to full reach
        extendArm(0);                   // TODO: create parameter for target position
        //robot.armMotor.setPosition(params);

    }

    public void scoreMidBucket(){
        // set arm position to up
        robot.armMotor.setPower(1);

        // extend arm to full reach
        robot.extendMotor.setPower(1);
        //robot.armMotor.setPosition(params);
    }

    public void extendArm(int targetPosition){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(targetPosition);
    }

    public void resetArm(){
        // retract the arm
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(0);     // TODO: replace 0 with parameter constant

        // give arm time to retract before lowering to avoid tipping the bot
        opMode.sleep(1000);

        // reset arm position to down
        robot.armMotor.setPower(0.25);
        robot.armMotor.setTargetPosition(0);        // TODO: replace 0 with parameter constant

        // set claw to reset position
        robot.claw.setPosition(0);                  // TODO: set claw to target position
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