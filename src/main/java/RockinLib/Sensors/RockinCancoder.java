package RockinLib.Sensors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;

public class RockinCancoder extends CANcoder {
    public RockinCancoder(int canId, String canBus){
        super(canId,canBus);
    }
    public double getDegrees(){
        return this.getAbsolutePosition().getValueAsDouble();
    }
}
