package org.firstinspires.ftc.teamcode.teamcode2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.robot.StartPosition;
import org.firstinspires.ftc.teamcode.game.robot.TeamColor;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "Autonomous", group = "red")
//originally had it as TeleOp b/c Autonomous wasn't working, but changed back over
//if it still doesn't work just change to annotate to TeleOp again -- pretty sure there aren't regulations about this
public class Auto extends LinearOpMode {
    private Robot2017 robot;
    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() throws InterruptedException {
        robot = new Robot2017(TeamColor.red, StartPosition.left);
        robot.init(hardwareMap);
        robot.setTelemetry(telemetry);
        robot.setTime(runtime);

        inputGameConfig();

        //Wait for the match to begin, presses start button
        waitForStart();
        while (opModeIsActive()) {
            int RE_ADJUST = 0;
            int POINT_TOWARDS_CRATER = 0;
            int ANGLE_PARALLEL_CRATER = 0;
            String glyphPosition = "left";

            // Get Down
            robot.lift.setPower(.5);
            wait1(500);
            robot.lift.setPower(0);
            robot.drive.veritical(.1);
            robot.lift.setPower(-.5);
            wait1(500);
            robot.lift.setPower(0);

            // If Pointed at Square ->
            if(robot.startPosition == StartPosition.left && robot.teamColor == TeamColor.red){
                // Scan Glyphs

                // Move Gold

                // Set Marker
                robot.drive.turn(RE_ADJUST);
                robot.drive.vertical(10);
                deployMarker();
                // Park in Crater
                robot.drive.turn(POINT_TOWARDS_CRATER);
                robot.drive.vertical(10); // CAN USE HORIZONTAL?
            } else if (robot.startPosition == StartPosition.left && robot.teamColor == TeamColor.blue){
                // Scan Glyphs

                // Move Gold

                // Set Marker
                robot.drive.turn(RE_ADJUST);
                robot.drive.vertical(10);
                deployMarker();
                // Park in Crater
                robot.drive.turn(POINT_TOWARDS_CRATER);
                robot.drive.vertical(10); // CAN USE HORIZONTAL?

            // If Pointed at Crater
            } else if (robot.startPosition == StartPosition.right && robot.teamColor == TeamColor.red){
                //Scan Glyphs

                // Set Marker
                robot.drive.turn(45);
                robot.drive.vertical(10);
                robot.drive.turn(90);
                robot.drive.vertical(12);
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.drive.turn(180);
                robot.drive.vertical(11);
                robot.drive.turn(ANGLE_PARALLEL_CRATER/GLYPHS);
                if(glyphPosition == "left"){
                    robot.drive.vertical(5);
                } else if (glyphPosition == "middle"){
                    robot.drive.vertical(4);
                } else {
                    robot.drive.vertical(3);
                }
                robot.drive.horizontal(5);
            } else if (robot.startPosition == StartPosition.righ && robot.teamColor == TeamColor.blue){
                //Scan Glyphs

                // Set Marker
                robot.drive.turn(45);
                robot.drive.vertical(10);
                robot.drive.turn(90);
                robot.drive.vertical(12);
                deployMarker();
                // Park in Crater, While Moving Gold
                robot.drive.turn(180);
                robot.drive.vertical(11);
                robot.drive.turn(ANGLE_PARALLEL_CRATER);
                if(glyphPosition == "left"){
                    robot.drive.vertical(5);
                } else if (glyphPosition == "middle"){
                    robot.drive.vertical(4);
                } else {
                    robot.drive.vertical(3);
                }
                robot.drive.horizontal(5);
            }
        }
    }

    public void deployMarker() throws InterruptedException{
        robot.markerServo.setPosition(.90f);
        wait1(1000);
        robot.markerServo.setPosition(.20f);
        wait1(1000);
    }

    private void inputGameConfig() throws InterruptedException {
        telemetry.addData("Input team color", "Red (press b) or Blue (press x)");
        telemetry.update();
        while (!gamepad1.b && !gamepad1.x) {
        }

        if (gamepad1.b == true) {
            robot.teamColor = TeamColor.red;
        } else {
            robot.teamColor = TeamColor.blue;
        }
        telemetry.addData("Chosen team color", robot.teamColor);

        telemetry.addData("Input which side", "Left (Square) or right (Crater) (use triggers)");
        telemetry.update();
        while (gamepad1.left_trigger < 0.5 && gamepad1.right_trigger < 0.5) {
        }

        if (gamepad1.left_trigger >= 0.5) {
            robot.startPosition = StartPosition.left;
        } else {
            robot.startPosition = StartPosition.right;
        }
        telemetry.addData("Chosen start postion", robot.startPosition);
        telemetry.update();
    }

    public void wait1(int t) throws InterruptedException {
        TimeUnit.MILLISECONDS.sleep(t);
    }

    public android.hardware.Camera initVision() {
        android.hardware.Camera camera = android.hardware.Camera.open(0);

        return camera;
        //make sure to camera.release() after using
    }

}
