package org.firstinspires.ftc.teamcode.opmodes;

/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.hardware.CSAutoParams;
import org.firstinspires.ftc.teamcode.hardware.HWProfile;
import org.firstinspires.ftc.teamcode.libraries.IntakeControlThread;
import org.firstinspires.ftc.teamcode.libraries.RRMechOps;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Disabled
@Autonomous(name = "Auto Samples", group = "Competition", preselectTeleOp = "TeleOp")
public class RRAutoBase extends LinearOpMode{

    public static String TEAM_NAME = "Project Peacock";
    public static int TEAM_NUMBER = 10355;

    //Define and declare Robot Starting Locations
    public enum START_POSITION {
        BLUE_SAMPLES,
        BLUE_SPECIMENS,
        RED_SAMPLES,
        RED_SPECIMENS
    }

    public static START_POSITION startPosition;

    public final static HWProfile robot = new HWProfile();
    public LinearOpMode opMode = this;
    public CSAutoParams params = new CSAutoParams();
    public RRMechOps mechOps = new RRMechOps(robot, opMode, params);

    public double startDelay = 0;
    public int delayPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //TODO: Initialize hardware
        robot.init(hardwareMap, false);

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        setStartingDelay();

        // Wait for the DS start button to be touched.
        telemetry.addData("Selected Starting Position", startPosition);
        telemetry.addData("Selected Delay Position", delayPosition);
        telemetry.addData("Selected Delay Position", startDelay);
        telemetry.addLine("Open CV Vision for Red/Blue Team Element Detection");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addLine("The starting point of the robot is assumed to be on the starting tile, " +
                "and along the edge farther from the truss legs. ");
        telemetry.addLine("You should also have a webcam connected and positioned in a way to see " +
                "the middle spike mark and the spike mark away from the truss (and ideally nothing else). " +
                "We assumed the camera to be in the center of the robot. ");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Selected Delay Position", delayPosition);
            telemetry.addData("Selected Delay Position", startDelay);

        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            runAutonoumousMode();
        }
    }

    //end runOpMode();

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d specimenScoringPosition = new Pose2d(0, 0, 0);
        Pose2d sampleScoringPosition = new Pose2d(0, 0, 0);
        Pose2d coloredSample1Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample2Position = new Pose2d(0, 0, 0);
        Pose2d coloredSample3Position = new Pose2d(0, 0, 0);
        Pose2d grabSpecimenPosition = new Pose2d(0, 0, 0);
        Pose2d yellowSample1Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample2Position = new Pose2d(0, 0, 0);
        Pose2d yellowSample3Position = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        switch (startPosition) {
            case BLUE_SAMPLES:
                drive = new MecanumDrive(hardwareMap, initPose);
                sampleScoringPosition = new Pose2d(18, 25, Math.toRadians(-90));
                yellowSample1Position = new Pose2d(30, -78, Math.toRadians(-90));
                yellowSample2Position = new Pose2d(40, -75, Math.toRadians(-45));
                yellowSample3Position = new Pose2d(4, -65, Math.toRadians(-90));
                midwayPose1 = new Pose2d(8, -65, Math.toRadians(-90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                break;

            case RED_SAMPLES:
                drive = new MecanumDrive(hardwareMap, initPose);
                sampleScoringPosition = new Pose2d(18, 25, Math.toRadians(-90));
                yellowSample1Position = new Pose2d(30, -78, Math.toRadians(-90));
                yellowSample2Position = new Pose2d(40, -75, Math.toRadians(-45));
                yellowSample3Position = new Pose2d(4, -65, Math.toRadians(-90));
                midwayPose1 = new Pose2d(8, -65, Math.toRadians(-90));
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                break;

            case BLUE_SPECIMENS:
                drive = new MecanumDrive(hardwareMap, initPose);
                grabSpecimenPosition = new Pose2d(7, 57, Math.toRadians(-90));
                specimenScoringPosition = new Pose2d(0, 0, 0);
                sampleScoringPosition = new Pose2d(0, 0, 0);
                grabSpecimenPosition = new Pose2d(0, 0, 0);
                midwayPose1 = new Pose2d(0, 0, 0);
                parkPose = new Pose2d(0, 0, 0);
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board
                break;

            case RED_SPECIMENS:
                drive = new MecanumDrive(hardwareMap, initPose);
                grabSpecimenPosition = new Pose2d(7, 57, Math.toRadians(-90));
                specimenScoringPosition = new Pose2d(0, 0, 0);
                sampleScoringPosition = new Pose2d(0, 0, 0);
                grabSpecimenPosition = new Pose2d(0, 0, 0);
                midwayPose1 = new Pose2d(0, 0, 0);
                parkPose = new Pose2d(0, 0, 0);
                waitSecondsBeforeDrop = 0; //TODO: Adjust time to wait for alliance partner to move from board

                break;
        }

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(specimenScoringPosition.position, specimenScoringPosition.heading)
                        .strafeToLinearHeading(coloredSample1Position.position, coloredSample1Position.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        if (delayPosition == 2) safeWaitSeconds(startDelay);

        //Move robot to grabSpecimenPosition
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .build());

        if (delayPosition == 3) safeWaitSeconds(startDelay);

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_SAMPLES ||
                startPosition == START_POSITION.RED_SAMPLES) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                            .strafeToLinearHeading(coloredSample2Position.position, coloredSample2Position.heading)
                            .build());


            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                            .build());
        }

        //Move robot to yellowSample3Position and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
//                        .splineToLinearHeading(dropYellowPixelPose, 0)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .strafeToLinearHeading(yellowSample3Position.position, yellowSample3Position.heading)
                        .build());

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(yellowSample2Position.position, yellowSample2Position.heading)
                        .strafeToLinearHeading(coloredSample2Position.position, coloredSample2Position.heading)
                        .strafeToLinearHeading(coloredSample3Position.position, coloredSample3Position.heading)
                        .strafeToLinearHeading(yellowSample1Position.position, yellowSample1Position.heading)
                        .build());

        //Move robot to yellowSample3Position and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .strafeToLinearHeading(sampleScoringPosition.position, sampleScoringPosition.heading)
                        .build());

        safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to yellowSample3Position and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(grabSpecimenPosition.position, grabSpecimenPosition.heading)
                        .setReversed(true)
//                        .splineToLinearHeading(dropWhitePixelPose, 0)
                        .build());

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
    }
    /**
     *
     */
    public void setStartingDelay(){
        double delay = 0;
        boolean exit = false;
        ElapsedTime bPressed = new ElapsedTime();

        bPressed.reset();
        while(!exit){
            telemetry.addData("Delay Position = ", delayPosition);
            telemetry.addData("Current Delay = ", delay);
            telemetry.addData("to increase delay =>", "DPAD_UP");
            telemetry.addData("to decrease delay =>", "DPAD_DOWN");
            telemetry.addData("to delay in start position =>", "Y/Δ");
            telemetry.addData("to cancel the delay =>", "B/O");
            telemetry.addData("to delay at purple pixel =>", "A/X");
            telemetry.addData("to delay at backdrop approach =>", "X/▢");
            telemetry.addData("to exit =>", "right_bumper");
            telemetry.update();

            if(gamepad1.dpad_up && bPressed.time() > 0.3){
                delay = delay + 0.5;
                bPressed.reset();
            }

            if(gamepad1.dpad_down && bPressed.time() > 0.3){
                delay = delay - 0.5;
                bPressed.reset();
            }

            if(delay < 0) {
                delay = 0;
            } else if(delay > 15) {
                delay = 15;
            }


            if(gamepad1.b && bPressed.time() > 0.3) {
                bPressed.reset();
                delayPosition=0;
            }

            if(gamepad1.y && bPressed.time() > 0.3) {
                bPressed.reset();
                delayPosition=1;
            }

            if(gamepad1.a && bPressed.time() > 0.3) {
                bPressed.reset();
                delayPosition=2;
            }

            if(gamepad1.x && bPressed.time() > 0.3) {
                bPressed.reset();
                delayPosition=3;
            }

            if(gamepad1.right_bumper) exit = true;
        }
        startDelay=delay;
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        State setupConfig = State.START_POSITION;
        Boolean menuActive = true;

        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested() && menuActive){
            switch(setupConfig){
                case START_POSITION:
                    telemetry.addData("Initializing Autonomous:",
                            TEAM_NAME, " ", TEAM_NUMBER);
                    telemetry.addData("---------------------------------------","");
                    telemetry.addLine("This Auto program uses Open CV Vision Processor for Team Element detection");
                    telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstayion) on gamepad 1:","");
                    telemetry.addData("    Blue Left   ", "(X / ▢)");
                    telemetry.addData("    Blue Middle ", "(DPAD_LEFT)");
                    telemetry.addData("    Blue Right ", "(Y / Δ)");
                    telemetry.addData("    Red Left    ", "(B / O)");
                    telemetry.addData("    Red Middle    ", "(DPAD_RIGHT)");
                    telemetry.addData("    Red Right  ", "(A / X)");
                    if(gamepad1.x){
                        startPosition = START_POSITION.BLUE_SAMPLES;
                        setupConfig = State.PARK_POSITION;
                    }
                    if(gamepad1.dpad_left){
                        startPosition = START_POSITION.BLUE_SPECIMENS;
                        setupConfig = State.PARK_POSITION;
                    }
                    if(gamepad1.b){
                        startPosition = START_POSITION.RED_SAMPLES;
                        setupConfig = State.PARK_POSITION;
                    }
                    if(gamepad1.dpad_right){
                        startPosition = START_POSITION.RED_SPECIMENS;
                        setupConfig = State.PARK_POSITION;
                    }
                    telemetry.update();
                    break;
                case PARK_POSITION:
                    telemetry.addData("Select PARK Position using DPAD_UP & DPAD_DOWN on gamepad 1:","");
                    telemetry.addData("    Park by WALL   ", "DPAD_DOWN");
                    telemetry.addData("    Park on Field Side ", "DPAD_UP");

                    if(gamepad1.dpad_down){
                        menuActive = false;
                        break;
                    }
                    if(gamepad1.dpad_up){
                        menuActive = false;
                        break;
                    }
                    break;

            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    public enum State {
        START_POSITION,
        PARK_POSITION
    }

}   // end class
