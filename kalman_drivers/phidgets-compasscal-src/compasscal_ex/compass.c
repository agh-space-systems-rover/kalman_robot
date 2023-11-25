#include "compass.h"

#define MAXSAMPLES 5000
double compassData[MAXSAMPLES][3];
double compassData2[MAXSAMPLES][2];
int compassDataCnt = 0;

#define VERBOSE_PROGRESS 0

void CCONV spatialData(PhidgetSpatialHandle ch,
	void *ctx,
	const double acceleration[3],
	const double angularRate[3],
	const double magneticField[3],
	double timestamp) {
	int *sampling = ctx;
	if (magneticField[0] != PUNK_DBL && *sampling) {
		compassData[compassDataCnt][0] = magneticField[0];
		compassData[compassDataCnt][1] = magneticField[1];
		compassData[compassDataCnt][2] = magneticField[2];

		compassData2[compassDataCnt][0] = magneticField[0];
		compassData2[compassDataCnt][1] = magneticField[1];

		compassDataCnt++;

		if (compassDataCnt % 100 == 0) {
			printf("Captured %d samples\n", compassDataCnt);
		}

		if (compassDataCnt == MAXSAMPLES) {
			printf("Captured max samples - Press Enter to continue.\n");
			*sampling = 0;
		}
	}
}

int captureCompassData(PhidgetSpatialHandle spatial) {
	int sampling = 0, version;
	PhidgetSpatial_setOnSpatialDataHandler(spatial, spatialData, &sampling);
	if (Phidget_openWaitForAttachment((PhidgetHandle)spatial, 1000)) {
		printf("Couldn't find a spatial\n");
		return 1;
	}

	Phidget_getDeviceVersion((PhidgetHandle)spatial, &version);
	if (version >= 300) {
		printf("Clearing any previous calibration data...\n");
		PhidgetSpatial_resetMagnetometerCorrectionParameters(spatial);
		SLEEP(100);
	}

	compassDataCnt = 0;
	printf("Press Enter to start sampling...\n");
	getchar();
	sampling = 1;
	printf("Sampling... Press Enter to stop...\n");
	getchar();
	sampling = 0;
	SLEEP(100);

	return 0;
}

static int COMPASSCAL_API progress(
	void *instance,
	const double *x,
	const double *g,
	const double fx,
	const double xnorm,
	const double gnorm,
	const double step,
	int n,
	int k,
	int ls
) {
	if (VERBOSE_PROGRESS) {
		printf("Iteration %d:\n", k);
		printf("  fx = %f, Vars: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", fx, x[9], x[10], x[11], x[0], x[4], x[8], x[3], x[6], x[1], x[7], x[2], x[5]);
		printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
		printf("\n");
	} else {
		printf("."); fflush(stdout);
	}
	return 0;
}

static int COMPASSCAL_API progress2(
	void *instance,
	const double *x,
	const double *g,
	const double fx,
	const double xnorm,
	const double gnorm,
	const double step,
	int n,
	int k,
	int ls
) {
	if (VERBOSE_PROGRESS) {
		printf("Iteration %d:\n", k);
		printf("  fx = %f, Vars: %f, %f, 0, %f, %f, 0, %f, 0, %f, 0, 0, 0\n", fx, x[4], x[5], x[0], x[3], x[1], x[2]);
		printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
		printf("\n");
	} else {
		printf("."); fflush(stdout);
	}
	return 0;
}

int main(int argc, char **argv) {
	int ret = 0;
	int choice=0;
	double args[13];
	PhidgetSpatialHandle spatial;
	Phidget_DeviceID devID;

	PhidgetSpatial_create(&spatial);

	printf("Choose:\n");
	printf("\t2: 2-axis calibration\n");
	printf("\t3: 3-axis calibration\n\n::>");

	choice = getchar(); getchar();

	switch (choice) {
	case '2':
		if (captureCompassData(spatial)) {
			printf("Error capturing data\n");
			goto done;
		}
		ret = getCompassArgs_2D_ver2(compassData, compassDataCnt, progress2, args, NULL);
		break;
	case '3':
		if (captureCompassData(spatial)) {
			printf("Error capturing data\n");
			goto done;
		}
		ret = getCompassArgs_3D(compassData, compassDataCnt, progress, args, NULL);
		break;
	default:
		printf("Error, you entered something wrong.\n");
		goto done;
		break;
	}

	/* Report the result. */
	printf("\n\nCompass calibration finished\n");
	printf("\tVars: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", args[0], args[1], args[2], args[3], args[4], args[5], args[6], args[7], args[8], args[9], args[10], args[11], args[12]);

	printf("\nThese arguments are in the right order to be passed direcly into the PhidgetSpatial_setCompassCorrectionParameters function.\n\n");
	printf("You may wish to use a more accurate value for magnetic field strength as the number provided here is only an estimate.\n\n");

	Phidget_getDeviceID((PhidgetHandle)spatial, &devID);
	if (devID != PHIDID_1056) {
		PhidgetSpatial_setMagnetometerCorrectionParameters(spatial, args[0],
			args[1], args[2], args[3],
			args[4], args[5], args[6],
			args[7], args[8], args[9], args[10], args[11], args[12]);
		PhidgetSpatial_saveMagnetometerCorrectionParameters(spatial);
		printf("Calibration values have been written to firmware on your spatial.\n "
						"These values will be maintained from now on, across power cycles, until explicitely reset or changed.\n");
	} else {
		printf("These calibration values must be set in software EVERY TIME the spatial is opened/attached,\n "
					"as they are maintained in the PC-side library code. This is best done in the attach handler.\n");
	}

done:
	Phidget_close((PhidgetHandle)spatial);
	PhidgetSpatial_delete(&spatial);
	printf("\nEnter to exit\n");
	getchar();
	return 0;
}
