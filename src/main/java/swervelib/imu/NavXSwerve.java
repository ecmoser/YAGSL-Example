package swervelib.imu;

import java.util.Optional;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Communicates with the NavX({@link AHRS}) as the IMU.
 */
public class NavXSwerve extends SwerveIMU
{

  /**
   * Mutable {@link MutAngularVelocity} for readings.
   */
  private final MutAngularVelocity yawVel      = new MutAngularVelocity(0, 0, DegreesPerSecond);
  /**
   * NavX IMU.
   */
  private       AHRS               imu;
  /**
   * Offset for the NavX.
   */
  private       Rotation3d         offset      = new Rotation3d();
  /**
   * Inversion for the gyro
   */
  private       boolean            invertedIMU = false;
  /**
   * An {@link Alert} for if there is an error instantiating the NavX.
   */
  private       Alert              navXError;

  /**
   * Constructor for the NavX({@link AHRS}) swerve.
   *
   * @param port Serial Port to connect to.
   */
  public NavXSwerve(NavXComType port)
  {
    navXError = new Alert("IMU", "Error instantiating NavX.", AlertType.kError);
    try
    {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      imu = new AHRS(port);
      factoryDefault();
    } catch (RuntimeException ex)
    {
      navXError.setText("Error instantiating NavX: " + ex.getMessage());
      navXError.set(true);
    }
  }

  @Override
  public void close() {
    imu.close();
  }

  /**
   * Reset offset to current gyro reading. Does not call NavX({@link AHRS#reset()}) because it has been reported to be
   * too slow.
   */
  @Override
  public void factoryDefault()
  {
    // gyro.reset(); // Reported to be slow
    offset = imu.getRotation3d();
  }

  /**
   * Clear sticky faults on IMU.
   */
  @Override
  public void clearStickyFaults()
  {
  }

  /**
   * Set the gyro offset.
   *
   * @param offset gyro offset as a {@link Rotation3d}.
   */
  public void setOffset(Rotation3d offset)
  {
    this.offset = offset;
  }

  /**
   * Set the gyro to invert its default direction
   *
   * @param invertIMU invert gyro direction
   */
  public void setInverted(boolean invertIMU)
  {
    invertedIMU = invertIMU;
    setOffset(getRawRotation3d());
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU without any zeroing. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRawRotation3d()
  {
    SmartDashboard.putBoolean("invertedIMU", invertedIMU);
    SmartDashboard.putNumber("Raw Gyro Reading", Units.radiansToDegrees(imu.getRotation3d().getZ()));
    SmartDashboard.putNumber("unaryMinus Gyro Reading", Units.radiansToDegrees((invertedIMU ? imu.getRotation3d().unaryMinus().getZ() : imu.getRotation3d().getZ())));
    SmartDashboard.putNumber("negative Gyro Reading", Units.radiansToDegrees((invertedIMU ? imu.getRotation3d().times(-1).getZ() : imu.getRotation3d().getZ())));
    return invertedIMU ? imu.getRotation3d() : imu.getRotation3d();
  }

  /**
   * Fetch the {@link Rotation3d} from the IMU. Robot relative.
   *
   * @return {@link Rotation3d} from the IMU.
   */
  @Override
  public Rotation3d getRotation3d()
  {
    return getRawRotation3d().minus(offset);
  }

  /**
   * Fetch the acceleration [x, y, z] from the IMU in meters per second squared. If acceleration isn't supported returns
   * empty.
   *
   * @return {@link Translation3d} of the acceleration as an {@link Optional}.
   */
  @Override
  public Optional<Translation3d> getAccel()
  {
    return Optional.of(
        new Translation3d(
            imu.getWorldLinearAccelX(),
            imu.getWorldLinearAccelY(),
            imu.getWorldLinearAccelZ())
            .times(9.81));
  }

  @Override
  public MutAngularVelocity getYawAngularVelocity()
  {
    return yawVel.mut_setMagnitude(imu.getRate());
  }

  /**
   * Get the instantiated NavX({@link AHRS}) IMU object.
   *
   * @return IMU object.
   */
  @Override
  public Object getIMU()
  {
    return imu;
  }
}
