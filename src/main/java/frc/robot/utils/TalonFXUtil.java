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

  /** Applies a config to a TalonFX, retrying five times before giving up. */
  public static void applyConfigWithRetries(TalonFX motor, TalonFXConfiguration config) {
    StatusCode status = StatusCode.OK;
    for (int i = 0; i < 5; i++) {
      status = motor.getConfigurator().apply(config);
      if (status.isOK()) {
        return;
      }
    }
    // Report loudly so a misconfigured motor isn't silent.
    DriverStationErrors.reportError(
        "TalonFX "
            + motor.getDeviceID()
            + " failed to configure after 5 attempts ("
            + status
            + "). Check CAN wiring and device ID.",
        false);
  }
}
