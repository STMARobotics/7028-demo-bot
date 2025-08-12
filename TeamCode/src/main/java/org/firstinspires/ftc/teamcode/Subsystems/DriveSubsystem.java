package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Main.Robot;
import org.firstinspires.ftc.teamcode.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DriveSubsystem extends SubsystemBase {

    private final DcMotorEx left1;
    private final DcMotorEx left2;
    private final DcMotorEx right1;
    private final DcMotorEx right2;
    private final IMU imu;
    
    public DriveSubsystem(Robot hardwareMap) {
        /**
         * Initialize the motors
         */
        left1 = hardwareMap.hardwareMap.get(DcMotorEx.class, "left1"); // Cast to DcMotorEx
        left2 = hardwareMap.hardwareMap.get(DcMotorEx.class, "left2"); // Cast to DcMotorEx
        right1 = hardwareMap.hardwareMap.get(DcMotorEx.class, "right1"); // Cast to DcMotorEx
        right2 = hardwareMap.hardwareMap.get(DcMotorEx.class, "right2"); // Cast to DcMotorEx

        /**
         * Initialize the IMU
         */
        imu = hardwareMap.hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        /**
         * Set the direction of the motors
         * For mecanum, front left and back left motors are typically REVERSE
         * front right and back right motors are typically FORWARD
         */
        left1.setDirection(DcMotor.Direction.REVERSE); // Front Left
        left2.setDirection(DcMotor.Direction.REVERSE); // Back Left
        right1.setDirection(DcMotor.Direction.FORWARD); // Front Right
        right2.setDirection(DcMotor.Direction.FORWARD); // Back Right

        /**
         * Set the zero power behavior to BRAKE
         */
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /**
         * Set motor mode to RUN_USING_ENCODER for velocity readings
         */
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Drives the robot field-centrically.
     * @param ySpeed The desired forward/backward speed on the field (usually from left stick Y).
     * @param xSpeed The desired strafing speed on the field (usually from left stick X).
     * @param turnSpeed The desired rotational speed of the robot (usually from right stick X).
     */
    public void drive(double ySpeed, double xSpeed, double turnSpeed) {
        // Get the heading in degrees
        double botHeadingDegrees = getHeading();

        double botHeadingRadians = degreesToRadians(botHeadingDegrees);

        // Edit the movement to match the heading
        double rotX = xSpeed * Math.cos(-botHeadingRadians) - ySpeed * Math.sin(-botHeadingRadians);
        double rotY = xSpeed * Math.sin(-botHeadingRadians) + ySpeed * Math.cos(-botHeadingRadians);

        // Calculate the power to each motor
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turnSpeed), 1.0);
        double frontLeftPower = (rotY + rotX + turnSpeed) / denominator;
        double backLeftPower = (rotY - rotX + turnSpeed) / denominator;
        double frontRightPower = (rotY - rotX - turnSpeed) / denominator;
        double backRightPower = (rotY + rotX - turnSpeed) / denominator;

        // Set the power to the motors
        left1.setPower(frontLeftPower);
        left2.setPower(backLeftPower);
        right1.setPower(frontRightPower);
        right2.setPower(backRightPower);
    }

    /**
     * Resets the IMU's yaw angle to 0.
     */
    public void resetIMU() {
        imu.resetYaw();
    }

    /**
     * Gets the robot's current heading in degrees.
     * @return The current heading in degrees.
     */
    public double getHeading() {
        // Changed to DEGREES
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * Stops the robot.
     */
    public void stop() {
        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
    }

    /**
     * Converts radians to degrees.
     * @param radians The angle in radians.
     * @return The angle in degrees.
     */
    public double radiansToDegrees(double radians) {
        return radians * 180 / Math.PI;
    }

    /**
     * Converts degrees to radians.
     * @param degrees The angle in degrees.
     * @return The angle in radians.
     */
    public double degreesToRadians(double degrees) {
        return degrees * Math.PI / 180;
    }

    /**
     * Snaps the robot to a target heading.
     * This is just a placeholder and still needs some work but we only need it to drive for now.
     * @param targetHeadingDegrees The desired heading in degrees.
     * @param turnPowerMagnitude The maximum power to apply for turning.
     */
    public void snapToHeading(double targetHeadingDegrees, double turnPowerMagnitude) {
        // Placeholder for P-controller or other turning logic
        // Example P-controller logic:
        // double currentHeadingDegrees = getHeading(); // This will now correctly be in degrees
        // double error = targetHeadingDegrees - currentHeadingDegrees;
        // // Normalize error to be within -180 to 180
        // while (error > 180) error -= 360;
        // while (error < -180) error += 360;
        //
        // double kP = 0.01; // Proportional gain, needs tuning
        // double turnSpeed = error * kP;
        //
        // // Clamp turn speed to the allowed magnitude
        // turnSpeed = Math.max(-turnPowerMagnitude, Math.min(turnSpeed, turnPowerMagnitude));
        //
        // drive(0, 0, turnSpeed); // Apply only turning
    }

    /**
     * Calculates the robot's approximate speed in miles per hour (This is not going to stay here
     * its just for fun).
     * @return Approximate robot speed in MPH.
     */
    public double getSpeed() {
        double left1VelocityTicksPerSec = left1.getVelocity();
        double left2VelocityTicksPerSec = left2.getVelocity();
        double right1VelocityTicksPerSec = right1.getVelocity();
        double right2VelocityTicksPerSec = right2.getVelocity();

        // Calculate velocity
        double avgAbsoluteTicksPerSecond = (Math.abs(left1VelocityTicksPerSec) +
                                            Math.abs(left2VelocityTicksPerSec) +
                                            Math.abs(right1VelocityTicksPerSec) +
                                            Math.abs(right2VelocityTicksPerSec)) / 4.0;

        // Convert tps to rps
        double avgWheelRps = avgAbsoluteTicksPerSecond / Constants.DriveConstants.TICKS_PER_REVOLUTION;

        // Calculate diameter
        double wheelCircumferenceMm = Math.PI * Constants.DriveConstants.WHEEL_DIAMETER_MM;

        // Calculate speed in millimeters per second
        double speedMmPerSecond = avgWheelRps * wheelCircumferenceMm;

        // Convert to meters per second
        double speedMetersPerSecond = speedMmPerSecond * Constants.DriveConstants.MM_TO_METERS;

        // Convert to miles per hour
        double speedMph = speedMetersPerSecond * Constants.DriveConstants.METERS_TO_MILES / Constants.DriveConstants.SECONDS_TO_HOURS;
        
        return speedMph;
    }
}
