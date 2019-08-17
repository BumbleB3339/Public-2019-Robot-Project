package frc.bumblelib.util.hardware.pixy;

/**
 * This class storage all the Pixy data
 */
public class PixyPacket implements Comparable<PixyPacket> {
	public int X;
	public int Y;
	public int Width;
	public int Height;
	public int checksumError;

	@Override
	public int compareTo(PixyPacket arg0) {
		return (int) Math.signum(X - arg0.X);
	}
}