double line_center(void) {

	int i;
	double min, max, I;
	double num = 0, den = 0;
	const double x[8] = {0.0630, 0.0450, 0.0270, 0.0090, -0.0090, -0.0270, -0.0450, -0.0630};

	min = linesensor->data[0];
	max = linesensor->data[0];
	for (i = 1; i < 8; i++) {
		if linesensor->data[i] < min) {
			min = linesensor->data[i]
		}
		if linesensor->data[i] > max) {
			max = linesensor->data[i]
		}
	}

	for (i = 0; i < 8; i++) {
		I = (linesensor->data[i] - min) / (max - min);
		I = 1 - I;
		num += x[i] * I;
		den += I
	}

	return num / den;

}