package org.firstinspires.ftc.teamcode.Main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem;

/**
 * Fc = field centric, Rc = robot centric
 */
@TeleOp(name = "Fc-drive", group = "Drive")
public class Robot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DriveSubsystem drive = new DriveSubsystem(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {

            // Define joystick values
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            // Drive
            drive.drive(y, x, rx);

            if (gamepad1.a) {drive.resetIMU();}

            // Telemetry
            telemetry.addData("Status", "Run Time: " + getRuntime());
            telemetry.addData("heading", drive.getHeading());
            telemetry.addData("Speed: ", drive.getSpeed() + "mph"); // This will be removed
            telemetry.update();
        }
    }
}
