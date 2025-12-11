package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousOpMode", group="Bot")
public class AutonomousOpMode extends OpMode {

    // All length/distance values are in inches

    // Parameters
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private enum AutonomousState {
        TARGETING,
        DRIVING,
        LAUNCHING,
        DONE
    }

    private final ElapsedTime driveTimer = new ElapsedTime();

    private boolean drive(double speed, double dx, double dy, double timeout) {
        int newFrontLeftTarget;
        int newBackLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;

        int dxIn = (int)(dx * COUNTS_PER_INCH / Math.cos(45));
        int dyIn = (int)(dy * COUNTS_PER_INCH);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + dyIn + dxIn;
        newFrontRightTarget = frontRight.getCurrentPosition() + dyIn - dxIn;
        newBackLeftTarget = backLeft.getCurrentPosition() + dyIn - dxIn;
        newBackRightTarget = backRight.getCurrentPosition() + dyIn + dxIn;

        frontLeft.setTargetPosition(newFrontLeftTarget);
        frontRight.setTargetPosition(newFrontRightTarget);
        backLeft.setTargetPosition(newBackLeftTarget);
        backRight.setTargetPosition(newBackRightTarget);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        driveTimer.reset();
        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        if((driveTimer.seconds() > timeout) ||
                !(frontLeft.isBusy() && frontRight.isBusy()
                        && backLeft.isBusy() && backRight.isBusy())) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return true;
        }

        return false;
    }

    double posX, posY;

    private AutonomousState state;

    @Override
    public void init() {
        posX = 0;
        posY = 0;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void test(double d) {
        drive(DRIVE_SPEED, -d, d, 1);
        drive(DRIVE_SPEED, d, 0, 1);
        drive(DRIVE_SPEED, 0, -d, 1);
        drive(DRIVE_SPEED, -d, 0, 1);
        drive(DRIVE_SPEED, -d, d, 1);
    }

    @Override
    public void start() {
        test(4);
        state = AutonomousState.TARGETING;
    }

    double targetX = 0, targetY = 0;
    @Override
    public void loop() {
        switch(state) {
            case TARGETING:
                // TODO: targeting
                targetX = 5;
                targetY = 5;
                state = AutonomousState.DRIVING;
                break;
            case DRIVING:
                if(drive(DRIVE_SPEED, posX - targetX, posY - targetY, 5)) {
                    posX = targetX;
                    posY = targetY;
                    state = AutonomousState.LAUNCHING;
                }
                break;
            case LAUNCHING:
                // TODO: launching
                state = AutonomousState.DONE;
                break;
        }
    }
}
