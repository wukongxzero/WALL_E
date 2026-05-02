using System;
using System.Runtime.InteropServices;

// Matches the 'struct TankStatus' from C
[StructLayout(LayoutKind.Sequential)]
public struct TankStatus
{
    public byte driveLeft;
    public byte driveRight;
    public short eulerX;
    public short eulerY;
    public short eulerZ;
    public byte changeFlag;
}

public static class TankStatusNative
{
    public const int PacketLength = 16;

    public static void constructTankStatus(ref TankStatus self)
    {
        self.driveLeft = 0;
        self.driveRight = 0;
        self.eulerX = 0;
        self.eulerY = 0;
        self.eulerZ = 90;
        self.changeFlag = 0;
    }

    public static void makeByteTankStatus(byte[] buffer, int byteLength, ref TankStatus ts)
    {
        if (byteLength < PacketLength || buffer == null || buffer.Length < PacketLength) return;

        buffer[0] = ts.driveLeft;
        buffer[1] = ts.driveRight;
        
        // Offset 2 and 3 are padding, skip them
        
        byte[] ex = BitConverter.GetBytes(ts.eulerX);
        buffer[4] = ex[0];
        buffer[5] = ex[1];

        byte[] ey = BitConverter.GetBytes(ts.eulerY);
        buffer[6] = ey[0];
        buffer[7] = ey[1];

        byte[] ez = BitConverter.GetBytes(ts.eulerZ);
        buffer[8] = ez[0];
        buffer[9] = ez[1];
    }

    public static void readByteTankStatus(byte[] buffer, int byteLength, ref TankStatus ts)
    {
        if (byteLength < PacketLength || buffer == null || buffer.Length < PacketLength) return;

        ts.driveLeft = buffer[0];
        ts.driveRight = buffer[1];
        
        // Offset 2 and 3 are padding, skip them
        
        ts.eulerX = BitConverter.ToInt16(buffer, 4);
        ts.eulerY = BitConverter.ToInt16(buffer, 6);
        ts.eulerZ = BitConverter.ToInt16(buffer, 8);
    }

    public static int get_tankstatus_packet_length()
    {
        return PacketLength;
    }

    public static float getDecodedShortToFloat(short data)
    {
        return (float)data / 256.0f;
    }

    public static short getEncodedFloatToShort(float reading)
    {
        return (short)(reading * 256.0f);
    }
}
