package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Auton", group = "Iterative Opmode")
public class AutonOp extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        waitForStart();

        sleep(500);

        robot.MoveDirection(0, 0, .5, 1); // Direction (-180 ≤ angle ≤ 180), Turn (-1 ≤ turn ≤ 1), Throttle (1 is max speed possible), time is in seconds


        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
