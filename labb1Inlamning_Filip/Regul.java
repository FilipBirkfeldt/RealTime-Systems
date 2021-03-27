public class Regul extends Thread {

	private PI inner = new PI();
	private PID outer = new PID();
	private BallBeamAnimator ballBeam;
	private ReferenceGenerator refGen;
	private OpCom opCom;
	private int priority;
	private boolean shouldRun = true;
	private long starttime;
	private ModeMonitor modeMon;

	public Regul(int pri, ModeMonitor modeMon) {
		priority = pri;
		setPriority(priority);
		ballBeam = new BallBeamAnimator(modeMon);
		this.modeMon = modeMon;
	}

	/** Sets OpCom (called from main) */
	public void setOpCom(OpCom opCom) {
		/** Written by you */
		this.opCom = opCom;
	}

	/** Sets ReferenceGenerator (called from main) */
	public void setRefGen(ReferenceGenerator refGen) {
		/** Written by you */
		this.refGen = refGen;
	}

	// Called in every sample in order to send plot data to OpCom
	private void sendDataToOpCom(double yRef, double y, double u) {
		double x = (double) (System.currentTimeMillis() - starttime) / 1000.0;
		opCom.putControlData(x, u);
		opCom.putMeasurementData(x, yRef, y);
	}

	// Sets the inner controller's parameters
	public void setInnerParameters(PIParameters p) {
		/** Written by you */
		inner.setParameters(p);
	}

	// Gets the inner controller's parameters
	public PIParameters getInnerParameters() {
		/** Written by you */
		return inner.getParameters();
	}

	// Sets the outer controller's parameters
	public void setOuterParameters(PIDParameters p) {
		/** Written by you */
		outer.setParameters(p);
	}

	// Gets the outer controller's parameters
	public PIDParameters getOuterParameters() {
		/** Written by you */
		return outer.getParameters();
	}

	// Called from OpCom when shutting down
	public void shutDown() {
		shouldRun = false;
	}

	// Saturation function
	private double limit(double v) {
		return limit(v, -10, 10);
	}

	// Saturation function
	private double limit(double v, double min, double max) {
		if (v < min)
			v = min;
		else if (v > max)
			v = max;
		return v;
	}

	public void run() {

		long duration;
		long t = System.currentTimeMillis();
		starttime = t;

		double yRef = 0;
		double y = 0;
		double u = 0;

		double PIref = 0;
		double PIy = 0;
		double PIu = 0;

		double PIDref = 0;
		double PIDy = 0;
		double PIDu = 0;

		while (shouldRun) {
			/** Written by you */

			PIDy = ballBeam.getBallPos();

			PIDref = refGen.getRef();
			yRef = refGen.getRef();

			switch (modeMon.getMode()) {
			case OFF: {
				y = u = yRef = 0;

				inner.reset();
				outer.reset();

				/** Written by you */
				break;
			}
			case BEAM: {
				/** Written by you */
				synchronized (inner) {
					PIy = ballBeam.getBeamAngle();
					PIu = inner.calculateOutput(PIy, yRef);

					PIu = PIu + refGen.getUff();
					PIu = this.limit(PIu);
					// Set output

					inner.updateState(PIu - refGen.getUff());
					ballBeam.setControlSignal(PIu);

				}
				// För nästa loop nu så ändras y,u 
				// skickas till PID-regulatorn istället
				y = PIy;
				u = PIu;

				break;
			}
			case BALL: {
				/** Written by you */
				// yRef = PIDref;

				double phiff = refGen.getPhiff();
				PIy = ballBeam.getBeamAngle();
				double uff = refGen.getUff();

				synchronized (outer) {
					PIDu = outer.calculateOutput(PIDy, PIDref);
					PIDu += phiff;

					PIref = PIDu;

					if (PIu >= 10 || PIu <= -10) {
						PIDu = ballBeam.getBeamAngle() - refGen.getPhiff();
					} else {
						PIDu = PIDu - refGen.getPhiff();
					}
					PIDu = this.limit(PIDu);
					outer.updateState(PIDu);

					synchronized (inner) {
						PIu = inner.calculateOutput(PIy, PIref);
						PIu = PIu + uff;
						PIu = this.limit(PIu);
						inner.updateState(limit(PIu - refGen.getUff()));
						ballBeam.setControlSignal(PIu);
					}
				}
				y = PIDy;
				u = PIu;
				break;
			}
			default: {
				System.out.println("Error: Illegal mode.");
				break;
			}
			}

			sendDataToOpCom(yRef, y, u);

			// sleep
			t = t + inner.getHMillis();
			duration = t - System.currentTimeMillis();
			if (duration > 0) {
				try {
					sleep(duration);
				} catch (InterruptedException x) {
				}
			} else {
				System.out.println("Lagging behind...");
			}
		}
		ballBeam.setControlSignal(0.0);
	}
}