package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Value;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(DriveConstants.left_front_motor_id);
    private final WPI_TalonSRX leftBack = new WPI_TalonSRX(DriveConstants.left_back_motor_id);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(DriveConstants.right_front_motor_id);
    private final WPI_TalonSRX rightBack = new WPI_TalonSRX(DriveConstants.right_back_motor_id);

    public static enum Side {
        LEFT,
        RIGHT
    }

    public DriveSubsystem() {
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        leftFront.setNeutralMode(NeutralMode.Coast);
        rightFront.setNeutralMode(NeutralMode.Coast);
        leftBack.setNeutralMode(NeutralMode.Brake);
        rightBack.setNeutralMode(NeutralMode.Brake);

        leftFront.setInverted(true);
    }

    private WPI_TalonSRX getMotorObject(Side whichMotor) {
        if (whichMotor == Side.LEFT) {
            return leftFront;
        } else {
            return rightFront;
        }
    }

    public void setMotorSpeed(Dimensionless speed, Side whichMotor) {
        getMotorObject(whichMotor).set(speed.in(Value));
    }

    public Dimensionless getMotorSpeed(Side whichMotor) {
        return Value.of(getMotorObject(whichMotor).get());
    }
}
