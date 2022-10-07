//AaAAAAAAaAaA
package frc.robot.util;
//AAAaAAaAaAAaAaaa

//aaaaaaaaaaa
import java.nio.charset.StandardCharsets;
//AaAAAa
import java.util.concurrent.atomic.AtomicBoolean;
//Aaaaaa
import java.util.concurrent.locks.ReentrantLock;
//AaaaAaaaaAaAaAaaA

//aAAaAaaAaaaAaaaaaaa
import edu.wpi.first.hal.HAL;
//AaaaaaAaa
import edu.wpi.first.hal.SerialPortJNI;
//aaAAAaaaAaAaAAaaa
import edu.wpi.first.hal.FRCNetComm.tResourceType;
//aaAaA
import edu.wpi.first.wpilibj.Timer;
//AAaAAAAaaA

//AaAAAAaAAAAaaaaAAA
public class PicoColorSensor implements AutoCloseable {
//AAaaaaaaaAaaAAaAAa
  public static class RawColor {
//aaaAaaAAAaAAaaaaaa
    public RawColor(int r, int g, int b, int _ir) {
//aAAAa
      red = r;
//AaaAAAaaaAAa
      green = g;
//AAAAa
      blue = b;
//AaaAaaa
      ir = _ir;
//aaAaAaAaaaAaaaaa
    }
//aaAaa

//AaaaAAAAAaaaAAaaa
    public RawColor() {
//aAAaaaaaaAaaaA
    }
//aAaAAaAAAAAAaaAAa

//aaaaaaAaaAAaAaAA
    public int red;
//AAAaaaAaaAAaAAAaaA
    public int green;
//AAAaaaaaa
    public int blue;
//AaAaAAaaAAAaaAaAaAa
    public int ir;
//aaaaAAAaaAAaaAAaaaA
  }
//aaaaAAAaaAaAAaaaAA

//AAaaaAaA
  private static class SingleCharSequence implements CharSequence {
//AaaaAaAaA
    public byte[] data;
//AAAaa

//AAaaaaAAaaaaaaAaaAa
    @Override
//AaAAaAAaaaaaAaAAaaa
    public int length() {
//aaaaAAaaaaa
      return data.length;
//aAAaAaaAaaA
    }
//aAAaAA

//AAAAaAAaaAAaAa
    @Override
//AaaaaaaAaAAAaAA
    public char charAt(int index) {
//AAaaAA
      return (char)data[index];
//aAAAaaAAaAaaaa
    }
//AAAaaA

//aAAaaaaA
    @Override
//aAAaaaaAAAAAaaa
    public CharSequence subSequence(int start, int end) {
//AAaAA
      return new String(data, start, end, StandardCharsets.UTF_8);
//aaaAAA
    }
//aAaaAAaaaa

//AaAaAaaaaaaaaaaAa
  }
//aaAaaAaaAaa

//aaAaaAaAAA
  private static class IntRef {
//AAaAaaAaAAAaaAAAa
    int value;
//aaaaAa
  }
//AAAaaaaaAAaaaaa

//aAAAaaAaaAa
  int parseIntFromIndex(SingleCharSequence charSeq, int readLen, IntRef lastComma) {
//AAaaAAAA
    int nextComma = 0;
//aaaAaAaAAA
    try {
//AAaaaaAAaA
      nextComma = findNextComma(charSeq.data, readLen, lastComma.value);
//AAaAAaaAaAAAAA
      int value = Integer.parseInt(charSeq, lastComma.value + 1, nextComma, 10);
//AAaAaAaA
      lastComma.value = nextComma;
//AAaAaAaaaAaAAaAaA
      return value;
//AAaAAaAAAAaaaA
    } catch (Exception ex) {
//aAAaaAAAaa
      return 0;
//AaaAAAAaaA
    }
//AaAAA
  }
//AAaAaaa

//AAaAaA
  private int findNextComma(byte[] data, int readLen, int lastComma) {
//aaaaAaaaaAaAaa
    while (true) {
//AAAAaaAaaaAAAaAaA
      if (readLen <= lastComma + 1 ) {
//AAAaAAAaAA
        return readLen;
//aaaAAaAaaAA
      }
//aAAaAAaa
      lastComma++;
//AaAAaA
      if (data[lastComma] == ',') {
//AaAaAaAaAAaAaaAAaaa
        break;
//AaaaaaaaAaaaaaA
      }
//aaAAA
    }
//AaAAaaAAaAaAAAAaaa
    return lastComma;
//aAaAaA
  }
//AAaaAaaAaAaAAA

//AAaAaa
  private final AtomicBoolean debugPrints = new AtomicBoolean();
//AAaaAaaAAAA
  private boolean hasColor0;
//aAAaaaaaaaaaaAaAAA
  private boolean hasColor1;
//aAaaaA
  private int prox0;
//aAAAAAAaAaaAA
  private int prox1;
//aAaaAaaaAa
  private final RawColor color0 = new RawColor();
//AaAAaAa
  private final RawColor color1 = new RawColor();
//aAAaaaAAAaAaaaAAA
  private double lastReadTime;
//AaAaAaaaaAaAAaAa
  private final ReentrantLock threadLock = new ReentrantLock();
//AAAAaAaAAAA
  private final Thread readThread;
//AaAAAaaa
  private final AtomicBoolean threadRunning = new AtomicBoolean(true);
//aaAAAAAA

//aaaaAaAaAAAa
  private void threadMain() {
//AAAAaaAA
    // Using JNI for a non allocating read
//aaAaAAAAaAaaAaAAAA
    int port = SerialPortJNI.serialInitializePort((byte)1);
//aaaaAAaA
    SerialPortJNI.serialSetBaudRate(port, 115200);
//aAaaaaAaa
    SerialPortJNI.serialSetDataBits(port, (byte)8);
//aaaaAaaAAaa
    SerialPortJNI.serialSetParity(port, (byte)0);
//AaaaAaaaaaAAAAAa
    SerialPortJNI.serialSetStopBits(port, (byte)10);
//AAaAAAAaaAAAAaaAAa

//aaaaaAaAaA
    SerialPortJNI.serialSetTimeout(port, 1);
//AaaAAaAAaAA
    SerialPortJNI.serialEnableTermination(port, '\n');
//AAAAaAaAaaaaa

//AAAAaaAaAaAaaAaAA
    HAL.report(tResourceType.kResourceType_SerialPort, 2);
//AAAAAaaaaaAaaAAaAa

//AaaaaAAAAAAAAAAaaaA
    byte[] buffer = new byte[257];
//aAaAaaaAa
    SingleCharSequence charSeq = new SingleCharSequence();
//AAaAAAAaaaAa
    charSeq.data = buffer;
//AAAAaAAaaaAa
    IntRef lastComma = new IntRef();
//aAAAAaaaAaAAaaAaAaa

//aaAAAaaAAaAaaAaaaaa
    RawColor color0 = new RawColor();
//AaaAaAAaAAAaaaAaaA
    RawColor color1 = new RawColor();
//aaaAA

//aaAAAaa
    while (threadRunning.get()) {
//AaAAA
      int read = SerialPortJNI.serialRead(port, buffer, buffer.length - 1);
//AAAaaAAAaA
      if (read <= 0) {
//AaAaAaaaAAaAaAAAA
        try {
//aAAAaaaAa
          threadLock.lock();
//AAaAaaaA
          this.hasColor0 = false;
//AAAAA
          this.hasColor1 = false;
//AaaaAA
        } finally {
//aAaaaaAaaAaaa
          threadLock.unlock();
//AaAAAAAaAaaA
        }
//aaaAaa
        continue;
//aAaAAAaAAaAAa
      }
//aaaAAAaAAAaAaAa
      if (!threadRunning.get()) {
//aaaaAaAAAaaAaAAa
        break;
//AaaaA
      }
//aAAAaAA

//AaaAaAaaaAAAAAaa
      // Trim trailing newline if exists
//aAAaAaAaAAaAAAaaAAa
      if (buffer[read - 1] == '\n') {
//aaaAA
        read--;
//AaaaaaAAAAAAaAaAaAA
      }
//AAaaaAAAAA

//AaAAaaaaAaAA
      if (read == 0) {
//aAAAAaaAaaaAAAA
        continue;
//AaAaaaaAAAaaaaA
      }
//AAAaAAAA

//AaaAAAaaaaAaAAaAAAa
      if (debugPrints.get()) {
//AaaaAaaaAAAAAA
        System.out.println(new String(buffer, 0, read, StandardCharsets.UTF_8));
//aaAAaaaAAAAAa
      }
//aaAaaaAaaa

//AaAaaaaAaAAaAaaaaaa
      lastComma.value = -1;
//aAaAaa

//aaAAaAaaAaa
      boolean hasColor0 = parseIntFromIndex(charSeq, read, lastComma) != 0;
//aAaAaAAaA
      boolean hasColor1 = parseIntFromIndex(charSeq, read, lastComma) != 0;
//AAaAaAaAAaAAaAAaA
      color0.red = parseIntFromIndex(charSeq, read, lastComma);
//AAAaAAaAaAAaAAAAa
      color0.green = parseIntFromIndex(charSeq, read, lastComma);
//aaaaaaaAaAA
      color0.blue = parseIntFromIndex(charSeq, read, lastComma);
//aaAAAAA
      color0.ir = parseIntFromIndex(charSeq, read, lastComma);
//AaAaa
      int prox0 = parseIntFromIndex(charSeq, read, lastComma);
//aaaAaAAaAaaAaaAAaaa
      color1.red = parseIntFromIndex(charSeq, read, lastComma);
//aAaaaAAaa
      color1.green = parseIntFromIndex(charSeq, read, lastComma);
//aAAaAA
      color1.blue = parseIntFromIndex(charSeq, read, lastComma);
//aAaAaAaAaAaa
      color1.ir = parseIntFromIndex(charSeq, read, lastComma);
//aaAaaaAaAa
      int prox1 = parseIntFromIndex(charSeq, read, lastComma);
//AAAaaAa

//AAAaaaAAaaaaA
      double ts = Timer.getFPGATimestamp();
//aaaAaaaAaaaaa

//aaAaaaaaaa
      try {
//AaaaaaaAAAAaaAaA
        threadLock.lock();
//AAAaaAaAAaAaaAA
        this.lastReadTime = ts;
//aaaAaaaAAAaAA
        this.hasColor0 = hasColor0;
//AAAaAAaAAAAAaAaAAA
        this.hasColor1 = hasColor1;
//AaAAaAAaAaA
        if (hasColor0) {
//aAAaaaAAaA
          this.color0.red = color0.red;
//AAAAAAAAaAAAaAaAaaA
          this.color0.green = color0.green;
//aaAAAaAaaaAAAA
          this.color0.blue = color0.blue;
//AAaAaaaaaAaAAaaaaaa
          this.color0.ir = color0.ir;
//aaaAAAAAaaaaAa
          this.prox0 = prox0;
//AaAaaAaaAaaAaaAA
        }
//AaAaaAA
        if (hasColor1) {
//aaAaAAaaAaAa
          this.color1.red = color1.red;
//AaAaaAAaaAaAAAaaAA
          this.color1.green = color1.green;
//AaaAAaaAaaAAaaAa
          this.color1.blue = color1.blue;
//AaaAaAaaaAaAAAAAaa
          this.color1.ir = color1.ir;
//AAaaaAaaAa
          this.prox1 = prox1;
//aAAAaAAAaAaAAa
        }
//AaAaAaaaaAAAaAAAA
      } finally {
//aAaaa
        threadLock.unlock();
//AaAAaaAaaAAAAAaa
      }
//AaAaaaAAaAaAAA
    }
//AAAaAaAaaAAA

//AaaAaaAaAAa
    SerialPortJNI.serialClose(port);
//AaAaAaaAAaAaAaAA
  }
//aAAAaaaaaaaAAAaaAaA

//aAAAaAaA
  public PicoColorSensor() {
//aAAAAaaaAAaAaAAAaa
    readThread = new Thread(this::threadMain);
//AaaAAAAAaAa
    readThread.setName("PicoColorSensorThread");
//AAaAAAAAAa
    readThread.start();
//AAAaAAaAAaaA
  }
//aAaaaAAAaAAAaAAAa

//aAAAaAaaAaaaAaAAaaa
  public boolean isSensor0Connected() {
//aaAaAaA
    try {
//aaAAAaaAaaaAaAaaa
      threadLock.lock();
//aaaaA
      return hasColor0;
//aaaAAaAa
    } finally {
//aAAaaAAAAaaaAaaaa
      threadLock.unlock();
//AaAaaaAaaaAAA
    }
//aAaaaaAaAAAaAaaaAA
  }
//aAAAaaAAAAAAaAaA

//AAaAaAA
  public boolean isSensor1Connected() {
//AAAAAa
    try {
//AaAAa
      threadLock.lock();
//aAaaaAaA
      return hasColor1;
//aaaaAa
    } finally {
//AaAaAAAaAaaA
      threadLock.unlock();
//aaaAaaA
    }
//aAAAAAAAAaAaA
  }
//aaAAaA

//AaAAAA
  public RawColor getRawColor0() {
//aaAaaaaAaa
    try {
//aAAAAaaaAAA
      threadLock.lock();
//AaaAaaaaAaa
      return new RawColor(color0.red, color0.green, color0.blue, color0.ir);
//AaaAAaaaaAA
    } finally {
//AaAAAAAAaAAAaaAaaa
      threadLock.unlock();
//aaAaAAaaAaaAaaA
    }
//AaaAAAaaaAA
  }
//aaaAaAAaAaaAA

//aAAaaaaa
  public void getRawColor0(RawColor rawColor) {
//aaaaaaAAaaAAA
    try {
//AAAAAAa
      threadLock.lock();
//aAaaaAaA
      rawColor.red = color0.red;
//aaaaAaa
      rawColor.green = color0.green;
//AaaAaAaaAAAaAaAaA
      rawColor.blue = color0.blue;
//aaaaa
      rawColor.ir = color0.ir;
//aaAaaaAaAaaaAaA
    } finally {
//AAaAaAaAaaaaAAAaa
      threadLock.unlock();
//aaAaaaAaaAaaaaAaAa
    }
//aaAaaaAA
  }
//aaAaAaAAAaAAA

//aAAAAAaAa
  public int getProximity0() {
//aAaAaaAaaaa
    try {
//AAAaAaAAaAaAAaAAa
      threadLock.lock();
//aaAaaAaAa
      return prox0;
//aAaaAAAaaaAaaAAaa
    } finally {
//AaAAAaAaAaAaAaAaAA
      threadLock.unlock();
//AaaAaAAaaAAAAAAA
    }
//AAaaaaAaAAAAaAaAAA
  }
//aaaAAa

//AAaAaAaAAAAAAAAaaa
  public RawColor getRawColor1() {
//aaAaaaaAAaaa
    try {
//AAaAaAAAaaaaAaAA
      threadLock.lock();
//AAaaAaaaAAaaAaaaA
      return new RawColor(color1.red, color1.green, color1.blue, color1.ir);
//aaaaaAAaAAAaaaAaAaA
    } finally {
//aaAAAAaaAaaA
      threadLock.unlock();
//aaAaaaAAa
    }
//AAaaaAAa
  }
//aaaAAaaaAaaAAAAAAa

//aAaaAaAAaaaaaaaaaAa
  public void getRawColor1(RawColor rawColor) {
//AaaAAaAAAaAaaAa
    try {
//AAaAAAAAAA
      threadLock.lock();
//AAaAAa
      rawColor.red = color1.red;
//Aaaaaa
      rawColor.green = color1.green;
//aAaaA
      rawColor.blue = color1.blue;
//AAAaAa
      rawColor.ir = color1.ir;
//aAAAaAaaaAaA
    } finally {
//AaAAAAAAAaaaa
      threadLock.unlock();
//AaaaaAAAaAa
    }
//aaaAaAaAaAAAaA
  }
//aAAAaaAAAaAaaaAAAA

//AaaAaaa
  public int getProximity1() {
//AaAAaaAAaaAaAaAaaAA
    try {
//aaaAAAaAAAA
      threadLock.lock();
//aaAaaaAAAaAAaA
      return prox1;
//AAaaAAaaaaAaAaAAaAa
    } finally {
//AAaAAaAAaa
      threadLock.unlock();
//AAAaAaAaaaa
    }
//aAAAAaaAaa
  }
//AAAAAaAaaAa

//aaaaaaAAAAAAAAa
  public double getLastReadTimestampSeconds() {
//aAAaAAaaAAAaaaaAAa
    try {
//AAAaaAAA
      threadLock.lock();
//AAaaaAaaaAA
      return lastReadTime;
//aaaaAAaAaaa
    } finally {
//AAaaaaAAAAaaAAAAaaa
      threadLock.unlock();
//aaaAaa
    }
//AAAaaaAAaa
  }
//aAAaaAaAAAAAAAA

//aaaAaaaaAaAAAA
  void setDebugPrints(boolean debug) {
//aAaAaAaAA
    debugPrints.set(debug);
//aaAaaaAaAaaaa
  }
//aAAAA

//AAAaaa
  @Override
//AaAAaAAAAaA
  public void close() throws Exception {
//AAAaaaa
    threadRunning.set(false);
//aaaAaaAAaaaa
    readThread.join();
//AaaAA
  }
//aAAAAaA
}