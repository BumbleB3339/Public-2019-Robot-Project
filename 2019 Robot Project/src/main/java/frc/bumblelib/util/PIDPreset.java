package frc.bumblelib.util;

public class PIDPreset {
	private double kP = 0;
	private double kI = 0;
	private double kD = 0;
	private double kF = 0;
	private int kA = 0;
	private int kV = 0;

	/**
	 * Constructor for class PIDPreset
	 * 
	 * @param kP Value of the P constant
	 * @param kI Value of the I constant
	 * @param kD Value of the D constant
	 */
	public PIDPreset(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	}

	/**
	 * Constructor for class PIDPreset with values to support Motion Magic
	 * 
	 * @param kP Value of the P constant
	 * @param kI Value of the I constant
	 * @param kD Value of the D constant
	 * @param kF Value of the F constant
	 * @param kA Value of the ACCELERATION constant
	 * @param kV Value of the VELOCITY constant
	 */
	public PIDPreset(double kP, double kI, double kD, double kF, int kA,
			int kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
		this.kA = kA;
		this.kV = kV;
	}
	
	
	/**
	 * Getter for P constant
	 * 
	 * @return P constant
	 */
	public double getKp() {
		return kP;
	}

	/**
	 * Setter for P constant
	 * 
	 * @param Kp P constant
	 */
	public void setKp(double Kp) {
		this.kP = Kp;
	}

	
	/**
	 * Getter for I constant
	 * 
	 * @return I constant
	 */
	public double getKi() {
		return kI;
	}

	/**
	 * Setter for I constant
	 * 
	 * @param Ki I constant
	 */
	public void setKi(double Ki) {
		this.kI = Ki;
	}

	
	/**
	 * Getter for D constant
	 * 
	 * @return D constant
	 */
	public double getKd() {
		return kD;
	}

	/**
	 * Setter for D constant
	 * 
	 * @param Kd D constant
	 */
	public void setKd(double Kd) {
		this.kD = Kd;
	}

	
	/**
	 * Getter for F constant
	 * 
	 * @return F constant
	 */
	public double getKf() {
		return kF;
	}

	/**
	 * Setter for F constant
	 * 
	 * @param Kf F constant
	 */
	public void setKf(double Kf) {
		this.kF = Kf;
	}

	
	/**
	 * Getter for ACCELERATION constant
	 * 
	 * @return ACCELERATION constant
	 */
	public int getKa() {
		return kA;
	}

	/**
	 * Setter for ACCELERATION constant
	 * 
	 * @param acceleration ACCELERATION constant
	 */
	public void setKa(int acceleration) {
		this.kA = acceleration;
	}
	
	
	/**
	 * Getter for VELOCITY constant
	 * 
	 * @return VELOCITY constant
	 */
	public int getKv() {
		return kV;
	}

	/**
	 * Setter for VELOCITY constant
	 * 
	 * @param velocity VELOCITY constant
	 */
	public void setKv(int velocity) {
		this.kV = velocity;
	}

	@Override
	public String toString() {
		return "{Kp: " + this.kI + ", Ki: " + this.kI + ", Kd: " + this.kD + ", Kf: " + this.kF + 
				", Kv: " + this.kV + ", Ka: " + this.kA + "}";
	}
}