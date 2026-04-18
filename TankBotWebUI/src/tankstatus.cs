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
    // Define the library name once so you don't have to type it everywhere
    private const string LibName = "libtankstatus_lib.so";

    // void constructTankStatus(struct TankStatus *self);
    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void constructTankStatus(ref TankStatus self);

    // void makeByteTankStatus(unsigned char *buffer, int byteLength, struct TankStatus *ts);
    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void makeByteTankStatus(byte[] buffer, int byteLength, ref TankStatus ts);

    // void readByteTankStatus(unsigned char *buffer, int byteLength, struct TankStatus *ts);
    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern void readByteTankStatus(byte[] buffer, int byteLength, ref TankStatus ts);

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern int get_tankstatus_packet_length();

    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern float getDecodedShortToFloat(short data);

    // Note: The C function name says "ShortToFloat" but it takes a float and returns a short.
    // You might want to rename this in C to "getEncodedFloatToShort"!
    [DllImport(LibName, CallingConvention = CallingConvention.Cdecl)]
    public static extern short getEncodedFloatToShort(float reading);
}
