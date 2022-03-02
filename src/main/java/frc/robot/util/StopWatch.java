// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class StopWatch {
    private long startTime = 0;

    private long accumlatedTime = 0;
	
	/**
	 * Start the stopwatch
	 */
	public void start()
	{
        // set startTime if it has not been set.
        if (startTime == 0) {
		    startTime = getTimeNow();
        }
	}

    public void pause() {
        if (startTime > 0) {
            accumlatedTime += getTimeNow() - startTime;
            reset();
        }
    }

	public boolean isRunning() {
        return startTime > 0;
    }

    public void restart()
	{
		startTime = getTimeNow();
	}

	public void reset()
	{
		startTime = 0;
	}
	
	/**
	 * @return Current time elapsed since start in s
	 */
	public double getDuration()
	{
		return (double)getDurationMs() / 1000;
	}

	/**
	 * @return Current time elapsed since start in ms
	 */
	public int getDurationMs()
	{
        if (startTime == 0) {
            return (int)accumlatedTime;
        }

		var now = getTimeNow();
		long retval = accumlatedTime + now - startTime;

		if(retval < 0) {
			retval = 0;
        }

		return (int)retval;
	}

    private long getTimeNow() {
        return System.currentTimeMillis();
    }

}
