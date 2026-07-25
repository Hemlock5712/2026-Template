package frc.robot.utils;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * TalonFX helpers. Phoenix's {@code apply(...)} does NOT retry on its own; {@link
 * #applyConfigWithRetries} retries a few times to ride out CAN hiccups at boot.
 */
public final class TalonFXUtil {

  private TalonFXUtil() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Applies a config to a TalonFX, retrying on failure.
   *
   * @return true if the config applied, false if every retry failed
   */
  public static boolean applyConfigWithRetries(
      TalonFX motor, TalonFXConfiguration config, int maxRetries) {
    StatusCode status = StatusCode.OK;
    for (int i = 0; i < maxRetries; i++) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return true;
      }
    }
    // Report loudly so a misconfigured motor isn't silent.
    DriverStationErrors.reportError(
        "TalonFX "
            + motor.getDeviceID()
            + " failed to configure after "
            + maxRetries
            + " attempts ("
            + status
            + "). Check CAN wiring and device ID.",
        false);
    return false;
  }

  /** Applies a config with the default retry count (5). */
  public static boolean applyConfigWithRetries(TalonFX motor, TalonFXConfiguration config) {
    return applyConfigWithRetries(motor, config, 5);
  }
}
