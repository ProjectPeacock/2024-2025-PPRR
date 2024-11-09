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


//
public void clawOpen(){

    robot.claw.setPosition(robot.CLAW_OPEN);
}

    public void rotateClaw(double targetClawPosition){
        robot.wrist.setPosition(targetClawPosition);
    }

    public void clawClose(){
        robot.claw.setPosition(robot.CLAW_CLOSED);    // TODO: create target position constant
    }

    public void scoreHighBasket(){
        // set arm position to up
        robot.armMotor.setPower(1);

        // extend arm to full reach
        extendArm(robot.ARM_HIGH_SCORE);
        //robot.armMotor.setPosition(params);

    }

    public void scoreLowBasket(){
        // set arm position to up
        robot.armMotor.setPower(1);

        // extend arm to full reach
        robot.extendMotor.setPower(robot.LIFT_SCORING_IN_LOW_BASKET);
        //robot.armMotor.setPosition(params);
    }

    public void extendArm(int targetPosition){
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(targetPosition);
    }

    public void resetArm(){
        // retract the arm
        robot.extendMotor.setPower(1);
        robot.extendMotor.setTargetPosition(robot.ARM_RESET);     // TODO: replace 0 with parameter constant

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