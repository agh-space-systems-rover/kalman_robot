#ifndef _COMPASSCAL_LIB
#define _COMPASSCAL_LIB

#ifdef __cplusplus
	extern "C" {
#endif

/* standard calling convention under Win32 is __stdcall */
#if defined(_WIN32)
	#define COMPASSCAL_API __stdcall
#else
	#define COMPASSCAL_API
#endif

/**
 * Progress report callback funtion. Most of these variables won't concern the user.
 *
 *  @param  userPtr     The user data sent for lbfgs() function by the client.
 *  @param  x           The current values of the calibration parameters. This is a 2x2 or 3x3 matrix followed by 2 or 3 offsets.
 *  @param  g           The current gradient values of the calibration parameters.
 *  @param  fx          The current value of the objective function.
 *  @param  xnorm       The Euclidean norm of the calibration parameters.
 *  @param  gnorm       The Euclidean norm of the gradients.
 *  @param  step        The line-search step used for this iteration.
 *  @param  n           The number of variables.
 *  @param  k           The iteration count.
 *  @param  ls          The number of evaluations called for this iteration.
 *  @retval int         Zero to continue the optimization process. Returning a
 *                      non-zero value will cancel the optimization process.
 */
typedef int (COMPASSCAL_API *progress_t)(
    void *userPtr,
    const double *x,
    const double *g,
    const double fx,
    const double xnorm,
    const double gnorm,
    const double step,
    int n,
    int k,
    int ls
    );

/**
 * Calculates compass correction parameters in 3 dimensions. Input data should represenst as much of the ellipsoid as possible.
 * @param sData An array of sets of compass data in the form {axis0,axis1,axis2}
 * @param cDataCount The number of elements in cData
 * @param progress A function callback for progress updates, can be NULL
 * @param args The compass correction arguments; in order: magField, offset0-1-2, gain0-1-2, T0-1-2-3-4-5
 * @param userPtr A user defined pointer that gets returned in the progress callback, can be NULL
 */
int COMPASSCAL_API getCompassArgs_3D(double cData[][3], int cDataCount, progress_t progress, double args[13], void *userPtr);
/**
 * Calculates compass correction parameters in 2 dimensions.
 * Input data should represenst a level circle rotated about the axis of gravity in the x-y (axis 0 and 1) plane.
 * This calibration will set the magnetic field in axis 2 to 0.
 * @param sData An array of sets of compass data in the form {axis0,axis1}
 * @param cDataCount The number of elements in cData
 * @param progress A function callback for progress updates, can be NULL
 * @param args The compass correction arguments; in order: magField, offset0-1-2, gain0-1-2, T0-1-2-3-4-5
 * @param userPtr A user defined pointer that gets returned in the progress callback, can be NULL
 */
int COMPASSCAL_API getCompassArgs_2D_ver1(double cData[][2], int cDataCount, progress_t progress, double args[13], void *userPtr);
/**
 * Calculates compass correction parameters for 3 dimentions, using 2 dimentional data.
 * Input data should represenst a level circle rotated about the axis of gravity in the x-y (axis 0 and 1) plane.
 * This calibration will set the axis 2 gain to the average of axis0 and axis1, but axis 2 will still have offset errors.
 * @param sData An array of sets of compass data in the form {axis0,axis1,axis2}
 * @param cDataCount The number of elements in cData
 * @param progress A function callback for progress updates, can be NULL
 * @param args The compass correction arguments; in order: magField, offset0-1-2, gain0-1-2, T0-1-2-3-4-5
 * @param userPtr A user defined pointer that gets returned in the progress callback, can be NULL
 */
int COMPASSCAL_API getCompassArgs_2D_ver2(double cData[][3], int cDataCount, progress_t progress, double args[13], void *userPtr);

#ifdef __cplusplus
}
#endif

#endif
