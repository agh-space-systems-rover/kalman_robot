#include "stdafx.h"
#include "matrix.h"

#define NOISE 0.002 //2mG @ unit mag
#define NOISE_PROD (2 / (NOISE * NOISE))

double compassData[MAXSAMPLES][3];
int compassDataCount = 0;

int estimateOffsetsGains2(lbfgsfloatval_t offsets[3], lbfgsfloatval_t gains[3]) {
	double *H = malloc((compassDataCount + 1) * 5 * sizeof(double)); //[compassDataCount+1][5]
	double *y = malloc((compassDataCount + 1) * 2 * sizeof(double)); //[compassDataCount+1][2]
	double *Ht = malloc((compassDataCount + 1) * 5 * sizeof(double)); //[5][compassDataCount+1]
	double HtH[5][5] = { 0 };
	double HtHi[5][5] = { 0 };
	double *HtHiHt = malloc((compassDataCount + 1) * 5 * sizeof(double)); //[5][compassDataCount+1]
	double HtHiHty[5][2] = { 0 };
	int i;
	double u1, u3, u4;

	for (i=0; i < compassDataCount; i++) {
		//H11
		*(H + (i + 1) * 5 + 1) = -2 * compassData[i][0];
		*(H + (i + 1) * 5 + 2) = -2 * compassData[i][1];
		//H12
		*(H + (i + 1) * 5 + 3) = compassData[i][1] * compassData[i][1];
		*(H + (i + 1) * 5 + 4) = 1;
		//y (z)
		*(y + (i + 1) * 2 + 1) = -(compassData[i][0] * compassData[i][0]);
	}

	M_transpose(H, Ht, compassDataCount, 4);
	M_crossProduct(Ht, H, (double *)HtH, 4, compassDataCount, compassDataCount, 4);
	M_inverse((double *)HtH, (double *)HtHi, 4, 4);
	M_crossProduct((double *)HtHi, Ht, HtHiHt, 4, 4, 4, compassDataCount);
	M_crossProduct(HtHiHt, y, (double *)HtHiHty, 4, compassDataCount, compassDataCount, 1);

	offsets[0] = HtHiHty[1][1];
	offsets[1] = HtHiHty[2][1] / HtHiHty[3][1];

	u1 = HtHiHty[3][1];
	u3 = HtHiHty[4][1];
	u4 = u3 - (offsets[0] * offsets[0] + u1 * offsets[1] * offsets[1]);

	gains[0] = 1 / sqrt(fabs(u4));
	gains[1] = 1 / sqrt(fabs(u4 / u1));

	printf("Estimates: gains(%0.3lf, %0.3lf) offsets(%0.3lf, %0.3lf)\n", gains[0], gains[1], offsets[0], offsets[1]);

	return 0;
}

int estimateOffsetsGains(lbfgsfloatval_t offsets[2], lbfgsfloatval_t gains[2]) {
	double *H = malloc((compassDataCount + 1) * 7 * sizeof(double)); //[compassDataCount+1][7]
	double *y = malloc((compassDataCount + 1) * 2 * sizeof(double)); //[compassDataCount+1][2]
	double *Ht = malloc((compassDataCount + 1) * 7 * sizeof(double)); //[7][compassDataCount+1]
	double HtH[7][7] = { 0 };
	double HtHi[7][7] = { 0 };
	double *HtHiHt = malloc((compassDataCount + 1) * 7 * sizeof(double)); //[7][compassDataCount+1]
	double HtHiHty[7][2] = { 0 };
	int i;
	double u1, u2, u3, u4;

	for (i=0; i < compassDataCount; i++) {
		//H11
		*(H + (i + 1) * 7 + 1) = -2 * compassData[i][0];
		*(H + (i + 1) * 7 + 2) = -2 * compassData[i][1];
		*(H + (i + 1) * 7 + 3) = -2 * compassData[i][2];
		//H12
		*(H + (i + 1) * 7 + 4) = compassData[i][1] * compassData[i][1];
		*(H + (i + 1) * 7 + 5) = compassData[i][2] * compassData[i][2];
		*(H + (i + 1) * 7 + 6) = 1;
		//y (z)
		*(y + (i + 1) * 2 + 1) = -(compassData[i][0] * compassData[i][0]);
	}

	M_transpose(H, Ht, compassDataCount, 6);
	M_crossProduct(Ht, H, (double *)HtH, 6, compassDataCount, compassDataCount, 6);
	M_inverse((double *)HtH, (double *)HtHi, 6, 6);
	M_crossProduct((double *)HtHi, Ht, HtHiHt, 6, 6, 6, compassDataCount);
	M_crossProduct(HtHiHt, y, (double *)HtHiHty, 6, compassDataCount, compassDataCount, 1);

	offsets[0] = HtHiHty[1][1];
	offsets[1] = HtHiHty[2][1] / HtHiHty[4][1];
	offsets[2] = HtHiHty[3][1] / HtHiHty[5][1];

	u1 = HtHiHty[4][1];
	u2 = HtHiHty[5][1];
	u3 = HtHiHty[6][1];
	u4 = u3 - (offsets[0] * offsets[0] + u1 * offsets[1] * offsets[1] + u2 * offsets[2] * offsets[2]);

	gains[0] = 1 / sqrt(fabs(u4));
	gains[1] = 1 / sqrt(fabs(u4 / u1));
	gains[2] = 1 / sqrt(fabs(u4 / u2));

	printf("Estimates: gains(%0.3lf, %0.3lf, %0.3lf) offsets(%0.3lf, %0.3lf, %0.3lf)\n", gains[0], gains[1], gains[2], offsets[0], offsets[1], offsets[2]);

	free(H);
	free(y);
	free(Ht);
	free(HtHiHt);

	return 0;
}

static lbfgsfloatval_t evaluate(
	void *instance,
	const lbfgsfloatval_t *x,
	lbfgsfloatval_t *g,
	const int n,
	const lbfgsfloatval_t step
) {
	int i;
	lbfgsfloatval_t fx = 0.0;

	for (i = 0; i < compassDataCount; i++) {
		lbfgsfloatval_t ui[3];
		lbfgsfloatval_t Tui[3];
		lbfgsfloatval_t TtTui[3];
		lbfgsfloatval_t TtT[9];
		lbfgsfloatval_t Tui_length;
		lbfgsfloatval_t Tui_length_cubed;
		lbfgsfloatval_t ct;
		lbfgsfloatval_t ui_Kronecker_Tui[9];
		lbfgsfloatval_t noiseProd;
		lbfgsfloatval_t errFactor;

		ui[0] = compassData[i][0] - x[9];
		ui[1] = compassData[i][1] - x[10];
		ui[2] = compassData[i][2] - x[11];

		Tui[0] = x[0] * ui[0] + x[3] * ui[1] + x[6] * ui[2];
		Tui[1] = x[1] * ui[0] + x[4] * ui[1] + x[7] * ui[2];
		Tui[2] = x[2] * ui[0] + x[5] * ui[1] + x[8] * ui[2];

		Tui_length = (lbfgsfloatval_t)sqrt((double)(Tui[0] * Tui[0] + Tui[1] * Tui[1] + Tui[2] * Tui[2]));
		Tui_length_cubed = Tui_length*Tui_length*Tui_length;

		ct = 1 - (1 / Tui_length);
		noiseProd = NOISE_PROD * ct;

		ui_Kronecker_Tui[0] = ui[0] * Tui[0];
		ui_Kronecker_Tui[1] = ui[0] * Tui[1];
		ui_Kronecker_Tui[2] = ui[0] * Tui[2];
		ui_Kronecker_Tui[3] = ui[1] * Tui[0];
		ui_Kronecker_Tui[4] = ui[1] * Tui[1];
		ui_Kronecker_Tui[5] = ui[1] * Tui[2];
		ui_Kronecker_Tui[6] = ui[2] * Tui[0];
		ui_Kronecker_Tui[7] = ui[2] * Tui[1];
		ui_Kronecker_Tui[8] = ui[2] * Tui[2];

		g[0] += noiseProd * ui_Kronecker_Tui[0];
		g[1] += noiseProd * ui_Kronecker_Tui[1];
		g[2] += noiseProd * ui_Kronecker_Tui[2];
		g[3] += noiseProd * ui_Kronecker_Tui[3];
		g[4] += noiseProd * ui_Kronecker_Tui[4];
		g[5] += noiseProd * ui_Kronecker_Tui[5];
		g[6] += noiseProd * ui_Kronecker_Tui[6];
		g[7] += noiseProd * ui_Kronecker_Tui[7];
		g[8] += noiseProd * ui_Kronecker_Tui[8];

		TtT[0] = x[0] * x[0] + x[3] * x[3] + x[6] * x[6];
		TtT[1] = x[1] * x[0] + x[4] * x[3] + x[7] * x[6];
		TtT[2] = x[2] * x[0] + x[5] * x[3] + x[8] * x[6];
		TtT[3] = x[0] * x[1] + x[3] * x[4] + x[6] * x[7];
		TtT[4] = x[1] * x[1] + x[4] * x[4] + x[7] * x[7];
		TtT[5] = x[2] * x[1] + x[5] * x[4] + x[8] * x[7];
		TtT[6] = x[0] * x[2] + x[3] * x[5] + x[6] * x[8];
		TtT[7] = x[1] * x[2] + x[4] * x[5] + x[7] * x[8];
		TtT[8] = x[2] * x[2] + x[5] * x[5] + x[8] * x[8];

		TtTui[0] = TtT[0] * ui[0] + TtT[3] * ui[1] + TtT[6] * ui[2];
		TtTui[1] = TtT[1] * ui[0] + TtT[4] * ui[1] + TtT[7] * ui[2];
		TtTui[2] = TtT[2] * ui[0] + TtT[5] * ui[1] + TtT[8] * ui[2];

		g[9] += -noiseProd * TtTui[0];
		g[10] += -noiseProd * TtTui[1];
		g[11] += -noiseProd * TtTui[2];

		errFactor = (Tui_length - 1) / NOISE;
		errFactor *= errFactor;

		fx+= errFactor;
	}

	for (i = 0; i < 12; i++) {
		g[i] /= compassDataCount;
	}
	fx /= compassDataCount;

	return fx;
}

static lbfgsfloatval_t evaluate2(
	void *instance,
	const lbfgsfloatval_t *x,
	lbfgsfloatval_t *g,
	const int n,
	const lbfgsfloatval_t step
) {
	int i;
	lbfgsfloatval_t fx = 0.0;

	for (i = 0; i < compassDataCount; i++) {
		lbfgsfloatval_t ui[2];
		lbfgsfloatval_t Tui[2];
		lbfgsfloatval_t TtTui[2];
		lbfgsfloatval_t TtT[4];
		lbfgsfloatval_t Tui_length;
		lbfgsfloatval_t Tui_length_cubed;
		lbfgsfloatval_t ct;
		lbfgsfloatval_t ui_Kronecker_Tui[4];
		lbfgsfloatval_t noiseProd;
		lbfgsfloatval_t errFactor;

		ui[0] = compassData[i][0] - x[4];
		ui[1] = compassData[i][1] - x[5];

		Tui[0] = x[0] * ui[0] + x[2] * ui[1];
		Tui[1] = x[1] * ui[0] + x[3] * ui[1];

		Tui_length = (lbfgsfloatval_t)sqrt((double)(Tui[0] * Tui[0] + Tui[1] * Tui[1]));
		Tui_length_cubed = Tui_length*Tui_length*Tui_length;

		ct = 1 - (1 / Tui_length);
		noiseProd = NOISE_PROD * ct;

		ui_Kronecker_Tui[0] = ui[0] * Tui[0];
		ui_Kronecker_Tui[1] = ui[0] * Tui[1];
		ui_Kronecker_Tui[2] = ui[1] * Tui[0];
		ui_Kronecker_Tui[3] = ui[1] * Tui[1];

		g[0] += noiseProd * ui_Kronecker_Tui[0];
		g[1] += noiseProd * ui_Kronecker_Tui[1];
		g[2] += noiseProd * ui_Kronecker_Tui[2];
		g[3] += noiseProd * ui_Kronecker_Tui[3];

		TtT[0] = x[0] * x[0] + x[2] * x[2];
		TtT[1] = x[1] * x[0] + x[3] * x[2];
		TtT[2] = x[0] * x[1] + x[2] * x[3];
		TtT[3] = x[1] * x[1] + x[3] * x[3];

		TtTui[0] = TtT[0] * ui[0] + TtT[2] * ui[1];
		TtTui[1] = TtT[1] * ui[0] + TtT[3] * ui[1];

		g[4] += -noiseProd * TtTui[0];
		g[5] += -noiseProd * TtTui[1];

		errFactor = (Tui_length - 1) / NOISE;
		errFactor *= errFactor;

		fx+= errFactor;
	}

	for (i = 0; i < 12; i++) {
		g[i] /= compassDataCount;
	}
	fx /= compassDataCount;

	return fx;
}

//gets calibration params for all 3 axes
//input must be as much of the sphere as possible
//make sure to check that the output is actuall spherical
int CCONV getCompassArgs_3D(double cData[][3], int cDataCount, lbfgs_progress_t progress, double args[13], void *instance) {
	int i, ret = 0;
	lbfgsfloatval_t fx;
	lbfgsfloatval_t *x = lbfgs_malloc(12);
	lbfgs_parameter_t param;
	lbfgsfloatval_t offsetEstimates[3], gainEstimates[3];

	if (x == NULL) {
		printf("ERROR: Failed to allocate a memory block for variables.\n");
		return 1;
	}

	if (cDataCount < MAXSAMPLES)
		compassDataCount = cDataCount;
	else
		compassDataCount = MAXSAMPLES;
	for (i = 0; i < compassDataCount; i++) {
		compassData[i][0] = cData[i][0];
		compassData[i][1] = cData[i][1];
		compassData[i][2] = cData[i][2];
	}

	/* Initialize the variables. */
	for (i = 0; i < 12; i++) {
		x[i] = 0;
	}

	estimateOffsetsGains(offsetEstimates, gainEstimates);

	x[0] = gainEstimates[0];
	x[4] = gainEstimates[1];
	x[8] = gainEstimates[2];

	x[9] = offsetEstimates[0];
	x[10] = offsetEstimates[1];
	x[11] = offsetEstimates[2];

	/* Initialize the parameters for the L-BFGS optimization. */
	lbfgs_parameter_init(&param);
	//param.orthantwise_c = 1;
	param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

	/*
		Start the L-BFGS optimization; this will invoke the callback functions
		evaluate() and progress() when necessary.
	 */
	ret = lbfgs(12, x, &fx, evaluate, progress, instance, &param);

	args[0] = 1 / ((x[0] + x[4] + x[8]) / 3); //estimated local field
	args[1] = x[9]; //offset0
	args[2] = x[10];//offset1
	args[3] = x[11];//offset2
	args[4] = x[0];	//gain0
	args[5] = x[4]; //gain1
	args[6] = x[8];	//gain2
	args[7] = x[3];	//T0;
	args[8] = x[6];	//T1
	args[9] = x[1];	//T2
	args[10] = x[7];	//T3
	args[11] = x[2];//T4
	args[12] = x[5];//T5

	lbfgs_free(x);

	return ret;
}

//This doesn't use z at all, and sets the z vector to 0
//useful only when the compass is level at all times - heading accuracy will degrade with pitch/roll
//requires only one rotation in the x-y plane with the compass level
int CCONV getCompassArgs_2D_ver1(double cData[][2], int cDataCount, lbfgs_progress_t progress, double args[13], void *instance) {
	int i, ret = 0;
	lbfgsfloatval_t fx;
	lbfgsfloatval_t *x = lbfgs_malloc(6);
	lbfgs_parameter_t param;
	lbfgsfloatval_t offsetEstimates[2], gainEstimates[2];

	if (x == NULL) {
		printf("ERROR: Failed to allocate a memory block for variables.\n");
		return 1;
	}

	if (cDataCount < MAXSAMPLES)
		compassDataCount = cDataCount;
	else
		compassDataCount = MAXSAMPLES;
	for (i=0; i < compassDataCount; i++) {
		compassData[i][0] = cData[i][0];
		compassData[i][1] = cData[i][1];
		compassData[i][2] = 0;
	}

	/* Initialize the variables. */
	for (i = 0; i < 6; i++) {
		x[i] = 0;
	}

	estimateOffsetsGains2(offsetEstimates, gainEstimates);

	x[0] = gainEstimates[0];
	x[3] = gainEstimates[1];

	x[4] = offsetEstimates[0];
	x[5] = offsetEstimates[1];

	/* Initialize the parameters for the L-BFGS optimization. */
	lbfgs_parameter_init(&param);
	//param.orthantwise_c = 1;
	param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

	/*
		Start the L-BFGS optimization; this will invoke the callback functions
		evaluate() and progress() when necessary.
	 */
	ret = lbfgs(6, x, &fx, evaluate2, progress, instance, &param);

	args[0] = 0.5;	//can't estimate local field, so just set to 0.5
	args[1] = x[4]; //offset0
	args[2] = x[5]; //offset1
	args[3] = 0;	//offset2
	args[4] = x[0];	//gain0
	args[5] = x[3]; //gain1
	args[6] = 0;	//gain2
	args[7] = x[1];	//T0;
	args[8] = 0;	//T1
	args[9] = x[2];	//T2
	args[10] = 0;	//T3
	args[11] = 0;	//T4
	args[12] = 0;	//T5

	lbfgs_free(x);

	return ret;
}

//This will calibrate based on a single x-y rotation
//uses z for gain determination
//sets the z gain to avg of x and y gain and z offset/factors to 0
//should be better then the x-y only alg, as long as the errors in z and not really insane
//will be able to use z with the tilt sensors to calculate heading when not flat
int CCONV getCompassArgs_2D_ver2(double cData[][3], int cDataCount, lbfgs_progress_t progress, double args[13], void *instance) {
	int i, ret = 0;
	lbfgsfloatval_t fx;
	lbfgsfloatval_t *x = lbfgs_malloc(6);
	lbfgs_parameter_t param;
	lbfgsfloatval_t offsetEstimates[2], gainEstimates[2];
	double adj;

	double zangle = 0;

	if (x == NULL) {
		printf("ERROR: Failed to allocate a memory block for variables.\n");
		return 1;
	}
	
	if (cDataCount < MAXSAMPLES)
		compassDataCount = cDataCount;
	else
		compassDataCount = MAXSAMPLES;

	for (i=0; i < compassDataCount; i++) {
		compassData[i][0] = cData[i][0];
		compassData[i][1] = cData[i][1];
		compassData[i][2] = cData[i][2];

		zangle += atan(fabs(compassData[i][2]) / sqrt(compassData[i][0] * compassData[i][0] + compassData[i][1] * compassData[i][1]));
	}

	zangle/=compassDataCount;

	printf("Z angle: %0.1lf\n", zangle * 180 / 3.14159265358979323846);

	/* Initialize the variables. */
	for (i = 0; i < 6; i++) {
		x[i] = 0;
	}

	//adjust magField to account for the z-offset of the x-y plane data
	adj = cos(zangle);

	estimateOffsetsGains2(offsetEstimates, gainEstimates);

	x[0] = gainEstimates[0];
	x[3] = gainEstimates[1];

	x[4] = offsetEstimates[0];
	x[5] = offsetEstimates[1];

	/* Initialize the parameters for the L-BFGS optimization. */
	lbfgs_parameter_init(&param);
	//param.orthantwise_c = 1;
	param.linesearch = LBFGS_LINESEARCH_BACKTRACKING;

	/*
		Start the L-BFGS optimization; this will invoke the callback functions
		evaluate() and progress() when necessary.
	 */
	ret = lbfgs(6, x, &fx, evaluate2, progress, instance, &param);

	//adjust 1st 4 factors down
	for (i=0; i < 4; i++) {
		x[i] = x[i] * adj;
	}

	args[0] = 1 / ((x[0] + x[3]) / 2); //estimated local field
	args[1] = x[4]; //offset0
	args[2] = x[5]; //offset1
	args[3] = 0;	//offset2
	args[4] = x[0];	//gain0
	args[5] = x[3]; //gain1
	args[6] = (x[0] + x[3]) / 2;	//gain2
	args[7] = x[1];	//T0;
	args[8] = 0;	//T1
	args[9] = x[2];	//T2
	args[10] = 0;	//T3
	args[11] = 0;	//T4
	args[12] = 0;	//T5

	lbfgs_free(x);

	return ret;
}