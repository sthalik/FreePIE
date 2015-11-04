package com.freepie.android.imu;// this code is based on the document:

//Implementing a Tilt-Compensated
//        eCompass using Accelerometer and
//        Magnetometer Sensors
//        by: Talat Ozyagcilar
//        Applications Engineer

// a copy is located at http://cache.freescale.com/files/sensors/doc/app_note/AN4248.pdf

public class MagAccEstimator {
    private MagAccEstimator() {
    }

    public static class result {
        public short angles[] = new short[3];
        public short iBfx, iBfy, iBfz;
        public short filt_acc[] = new short[3];
        public short filt_mag[] = new short[3];
    }

    // note java has no unsigned integers
    static short iTrig(short ix, short iy) {
        final short MINDELTATRIG = 1;

        int itmp; /* scratch */
        int ixsq; /* ix * ix */
        short isignx; /* storage for sign of x. algorithm assumes x >= 0 then corrects later */
        int ihypsq; /* (ix * ix) + (iy * iy) */
        short ir; /* result = ix / sqrt(ix*ix+iy*iy) range -1, 1 returned as signed Int16 */
        short idelta; /* delta on candidate result dividing each stage by factor of 2 */
/* stack variables */
/* ix, iy: signed 16 bit integers representing sensor reading in range -32768 to 32767 */
/* function returns signed Int16 as signed fraction (ie +32767=0.99997, -32768=-1.0000) */
/* algorithm solves for ir*ir*(ix*ix+iy*iy)=ix*ix */
/* correct for pathological case: ix==iy==0 */
        if ((ix == 0) && (iy == 0)) ix = iy = 1;
/* check for -32768 which is not handled correctly */
        if (ix == -32768) ix = -32767;
        if (iy == -32768) iy = -32767;
/* store the sign for later use. algorithm assumes x is positive for convenience */
        isignx = 1;
        if (ix < 0) {
            ix = (short) -ix;
            isignx = -1;
        }
/* for convenience in the boosting set iy to be positive as well as ix */
        iy = (short) Math.abs(iy);
/* to reduce quantization effects, boost ix and iy but keep below maximum signed 16 bit */
        while ((ix < 16384) && (iy < 16384)) {
            ix = (short) (ix + ix);
            iy = (short) (iy + iy);
        }
/* calculate ix*ix and the hypotenuse squared */
        ixsq = (int) (ix * ix); /* ixsq=ix*ix: 0 to 32767^2 = 1073676289 */
        ihypsq = (int) (ixsq + iy * iy); /* ihypsq=(ix*ix+iy*iy) 0 to 2*32767*32767=2147352578 */
/* set result r to zero and binary search step to 16384 = 0.5 */
        ir = 0;
        idelta = 16384; /* set as 2^14 = 0.5 */
/* loop over binary sub-division algorithm */
        do {
/* generate new candidate solution for ir and test if we are too high or too low */
/* itmp=(ir+delta)^2, range 0 to 32767*32767 = 2^30 = 1073676289 */
            itmp = (int) ((ir + idelta) * (ir + idelta));
/* itmp=(ir+delta)^2*(ix*ix+iy*iy), range 0 to 2^31 = 2147221516 */
            itmp = (itmp >> 15) * (ihypsq >> 15);
            if (itmp <= ixsq) ir += idelta;
            idelta = (short) (idelta >> 1); /* divide by 2 using right shift one bit */
        } while (idelta >= MINDELTATRIG); /* last loop is performed for idelta=MINDELTATRIG */
/* correct the sign before returning */
        return (short) (ir * isignx);
    }

    /* calculates 100*atan2(iy/ix)=100*atan2(iy,ix) in deg for ix, iy in range -32768 to 32767 */
    static short iHundredAtan2Deg(short iy, short ix) {
        short iResult; /* angle in degrees times 100 */
/* check for -32768 which is not handled correctly */
        if (ix == -32768) ix = -32767;
        if (iy == -32768) iy = -32767;
/* check for quadrants */
        if ((ix >= 0) && (iy >= 0)) /* range 0 to 90 degrees */
            iResult = iHundredAtanDeg(iy, ix);
        else if ((ix <= 0) && (iy >= 0)) /* range 90 to 180 degrees */
            iResult = (short) (18000 - (short) iHundredAtanDeg(iy, (short) -ix));
        else if ((ix <= 0) && (iy <= 0)) /* range -180 to -90 degrees */
            iResult = (short) ((short) -18000 + iHundredAtanDeg((short) -iy, (short) -ix));
        else /* ix >=0 and iy <= 0 giving range -90 to 0 degrees */
            iResult = (short) (-iHundredAtanDeg((short) -iy, ix));
        return iResult;
    }

    /* fifth order of polynomial approximation giving 0.05 deg max error */
    static final short K1 = 5701;
    static final short K2 = -1645;
    static final short K3 = 446;

    /* calculates 100*atan(iy/ix) range 0 to 9000 for all ix, iy positive in range 0 to 32767 */
    static short iHundredAtanDeg(short iy, short ix) {
        int iAngle; /* angle in degrees times 100 */
        short iRatio; /* ratio of iy / ix or vice versa */
        int iTmp; /* temporary variable */
/* check for pathological cases */
        if ((ix == 0) && (iy == 0)) return 0;
        if ((ix == 0) && (iy != 0)) return 9000;
/* check for non-pathological cases */
        if (iy <= ix)
            iRatio = iDivide(iy, ix); /* return a fraction in range 0. to 32767 = 0. to 1. */
        else
            iRatio = iDivide(ix, iy); /* return a fraction in range 0. to 32767 = 0. to 1. */
/* first, third and fifth order polynomial approximation */
        iAngle = (int) K1 * (int) iRatio;
        iTmp = ((int) iRatio >> 5) * ((int) iRatio >> 5) * ((int) iRatio >> 5);
        iAngle += (iTmp >> 15) * (int) K2;
        iTmp = (iTmp >> 20) * ((int) iRatio >> 5) * ((int) iRatio >> 5);
        iAngle += (iTmp >> 15) * (int) K3;
        iAngle = iAngle >> 15;
/* check if above 45 degrees */
        if (iy > ix) iAngle = (short) (9000 - iAngle);
/* for tidiness, limit result to range 0 to 9000 equals 0.0 to 90.0 degrees */
        if (iAngle < 0) iAngle = 0;
        if (iAngle > 9000) iAngle = 9000;
        return ((short) iAngle);
    }

    static final short MINDELTADIV = 1; /* final step size for iDivide */

    /* function to calculate ir = iy / ix with iy <= ix, and ix, iy both > 0 */
    static short iDivide(short iy, short ix) {
        short itmp; /* scratch */
        short ir; /* result = iy / ix range 0., 1. returned in range 0 to 32767 */
        short idelta; /* delta on candidate result dividing each stage by factor of 2 */
/* set result r to zero and binary search step to 16384 = 0.5 */
        ir = 0;
        idelta = 16384; /* set as 2^14 = 0.5 */
/* to reduce quantization effects, boost ix and iy to the maximum signed 16 bit value */
        while ((ix < 16384) && (iy < 16384)) {
            ix = (short) (ix + ix);
            iy = (short) (iy + iy);
        }
/* loop over binary sub-division algorithm solving for ir*ix = iy */
        do {
/* generate new candidate solution for ir and test if we are too high or too low */
            itmp = (short) (ir + idelta); /* itmp=ir+delta, the candidate solution */
            itmp = (short) ((itmp * ix) >> 15);
            if (itmp <= iy) ir += idelta;
            idelta = (short) (idelta >> 1); /* divide by 2 using right shift one bit */
        } while (idelta >= MINDELTADIV); /* last loop is performed for idelta=MINDELTADIV */
        return (ir);
    }

    public static void iecompass(result res,
                                 short iBpx, short iBpy, short iBpz,
                                 short iGpx, short iGpy, short iGpz) {
/* stack variables */
/* iBpx, iBpy, iBpz: the three components of the magnetometer sensor */
/* iGpx, iGpy, iGpz: the three components of the accelerometer sensor */
/* local variables */
        short iSin, iCos; /* sine and cosine */
        /* calculate current roll angle Phi */
        res.angles[0] = iHundredAtan2Deg(iGpy, iGpz);/* Eq 13 */
/* calculate sin and cosine of roll angle Phi */
        iSin = iTrig(iGpy, iGpz); /* Eq 13: sin = opposite / hypotenuse */
        iCos = iTrig(iGpz, iGpy); /* Eq 13: cos = adjacent / hypotenuse */
/* de-rotate by roll angle Phi */
        res.iBfy = (short) ((iBpy * iCos - iBpz * iSin) >> 15);/* Eq 19 y component */
        iBpz = (short) ((iBpy * iSin + iBpz * iCos) >> 15);/* Bpy*sin(Phi)+Bpz*cos(Phi)*/
        iGpz = (short) ((iGpy * iSin + iGpz * iCos) >> 15);/* Eq 15 denominator */
/* calculate current pitch angle Theta */
        res.angles[1] = iHundredAtan2Deg((short) -iGpx, iGpz);/* Eq 15 */
/* restrict pitch angle to range -90 to 90 degrees */
        if (res.angles[1] > 9000) res.angles[1] = (short) (18000 - res.angles[1]);
        if (res.angles[1] < -9000) res.angles[1] = (short) (-18000 - res.angles[1]);
/* calculate sin and cosine of pitch angle Theta */
        iSin = (short) -iTrig(iGpx, iGpz); /* Eq 15: sin = opposite / hypotenuse */
        iCos = iTrig(iGpz, iGpx); /* Eq 15: cos = adjacent / hypotenuse */
/* correct cosine if pitch not in range -90 to 90 degrees */
        if (iCos < 0) iCos = (short) -iCos;
/* de-rotate by pitch angle Theta */
        res.iBfx = (short) ((iBpx * iCos + iBpz * iSin) >> 15); /* Eq 19: x component */
        res.iBfz = (short) ((-iBpx * iSin + iBpz * iCos) >> 15);/* Eq 19: z component */
/* calculate current yaw = e-compass angle Psi */
        res.angles[2] = iHundredAtan2Deg((short) -res.iBfy, res.iBfx); /* Eq 22 */
    }

    public static short lowpass(short filtered, short new_sample, int nsamples) {
        int tmpAngle; /* temporary angle*100 deg: range -36000 to 36000 */
        final short ANGLE_LPF = (short) (32768 / nsamples); /* low pass filter: set to 32768 / N for N samples averaging */
/* implement a modulo arithmetic exponential low pass filter on the yaw angle */
/* compute the change in angle modulo 360 degrees */
        tmpAngle = (int) new_sample - (int) filtered;
        if (tmpAngle > 18000) tmpAngle -= 36000;
        if (tmpAngle < -18000) tmpAngle += 36000;
/* calculate the new low pass filtered angle */
        tmpAngle = (int) filtered + ((ANGLE_LPF * tmpAngle) >> 15);
/* check that the angle remains in -180 to 180 deg bounds */
        if (tmpAngle > 18000) tmpAngle -= 36000;
        if (tmpAngle < -18000) tmpAngle += 36000;
/* store the correctly bounded low pass filtered angle */
        return (short) tmpAngle;
    }
}
