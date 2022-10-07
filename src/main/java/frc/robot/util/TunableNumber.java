//AAAAAaaaAaaa
package frc.robot.util;
//AaaAaAa

//aAAAAaaaAaaAAAAAaa
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//AAAAAaaA
import frc.robot.Constants;
//aAaaAaa

//Aaaaaa
/**
//AaAAaAaaaaaaA
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
//AAaaAAaAAaaA
 * value not in dashboard.
//aAAaaaA
 */
//AaaAa
public class TunableNumber {
//aAAAaAA
  private static final String tableKey = "TunableNumbers";
//aaaAaaAAAAaaAaAAAA

//aAAaaAAA
  private String key;
//aAAAaAAa
  private double defaultValue;
//aaAAaaAAaa
  private double lastHasChangedValue = defaultValue;
//aaaaAAAAaAaAAAaA

//aaaaAAAAaaAAa
  /**
//aaAAAAaaAaaA
   * Create a new TunableNumber
//aaAaaa
   * 
//AaAaAaaA
   * @param dashboardKey Key on dashboard
//aaAaAAAAa
   */
//AaaAaAAaAaaaAA
  public TunableNumber(String dashboardKey) {
//aAAaAaAaaAAAaaaAAA
    this.key = tableKey + "/" + dashboardKey;
//aAaAaAA
  }
//aaAaa

//AaAaAaaAaAAAaAaA
  /**
//aAaAaAAAaAA
   * Get the default value for the number that has been set
//AAAaaAaaAaAAaa
   * 
//aAAaaAAaaA
   * @return The default value
//aaaaAAAaaAaaaAaaA
   */
//AAaaaaaaaAAAaAaAa
  public double getDefault() {
//AAaAAaAAaaAAaaaA
    return defaultValue;
//aaaAaAAAaa
  }
//aaAAAaaAAa

//AAaAAaaaaaaAaAaaAa
  /**
//AAaaAaaaaaaaaaaaa
   * Set the default value of the number
//aaAAAaAaAaAAa
   * 
//aAaaaaAAaAAaAaa
   * @param defaultValue The default value
//AAaAaaaAaAaA
   */
//AaAAAaaAaAaAAAaa
  public void setDefault(double defaultValue) {
//AAAaAAAaAaaAAAAA
    this.defaultValue = defaultValue;
//aAAaAaaaAaAaaAa
    if (Constants.tuningMode) {
//AaAAaAaAaAaaa
      // This makes sure the data is on NetworkTables but will not change it
//AAaAaaAaAaAAaaAAA
      SmartDashboard.putNumber(key,
//aaAAaaAAAaa
          SmartDashboard.getNumber(key, defaultValue));
//aaAAaAaa
    } else {
//aAaAaaaAaaaAaaaAa
      SmartDashboard.delete(key);
//aAAaAaAaaAaAAAa
    }
//AAaAaaaaAaAAa
  }
//aAaAaAAAaAAAA

//aAAAaAaaaAAaaaaaAa
  /**
//AAAaaAaAAaaA
   * Get the current value, from dashboard if available and in tuning mode
//AaaAa
   * 
//AAaaaAAAaaAaAAAAaAA
   * @return The current value
//aaAAAaAAaAa
   */
//aAaAaAA
  public double get() {
//aAAAAaAaaAAA
    return Constants.tuningMode ? SmartDashboard.getNumber(key, defaultValue)
//AaaaAAaaaAAAaaAAAA
        : defaultValue;
//AAaaAAaAaAAaaAAaAA
  }
//aAAAaaAaAAAaaAA

//AAaaaAAAAa
  /**
//aaaaaa
   * Checks whether the number has changed since our last check
//AaaAAAAa
   * 
//AAaAAAa
   * @return True if the number has changed since the last time this method was called, false
//AAAaAaaaaAAA
   *         otherwise
//aAaAaaAAAaaAaAAAAa
   */
//AaAAaAAAaaaaAAaAaa
  public boolean hasChanged() {
//aAaaaaaaAAaAaA
    double currentValue = get();
//AAaaaaAaaaaaaAaA
    if (currentValue != lastHasChangedValue) {
//aAAaaaaAaaA
      lastHasChangedValue = currentValue;
//aaaAAAA
      return true;
//AAAaaA
    }
//aAAaAAaAaaa

//aAaaaaaaaa
    return false;
//AaaaAaAAaAaAaAa
  }
//aAAAaAaAaAaAAaaa
}