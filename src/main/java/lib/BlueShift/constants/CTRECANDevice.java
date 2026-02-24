package lib.BlueShift.constants;

import com.ctre.phoenix6.CANBus;

public class CTRECANDevice {
    int deviceID;
    CANBus canbus;

    public CTRECANDevice(int deviceID) {
        this(deviceID, new CANBus());
    }

    public CTRECANDevice(int deviceID, CANBus canbus) {
        this.deviceID = deviceID;
        this.canbus = canbus;
    }

    public int getDeviceID() {
        return deviceID;
    }

    public CANBus getCanbus() {
        return canbus;
    }
}
