#include "shell.h"

uint16_t s_strlen(const char *str) {
	uint16_t i = 0;
	for(i=0; str[i]!='\0'; i++);
	return i;
}

void reverse(char *str) {
	uint16_t i = 0;
	uint16_t length = s_strlen(str) - 1;
	char c;

	for (i = 0; i < length; i++, length--) {
		c = str[i];
		str[i] = str[length];
		str[length] = c;
	}
}

void shell_itoa(int16_t n, char *str) {
	int i = 0, sign;
	if ((sign = n) < 0)
		n = -n;
	do {
		str[i++] = n % 10 + '0';
	} while((n /= 10) > 0);
	if (sign < 0)
		str[i++] = '-';
	str[i] = '\0';
	reverse(str);
}

void shell_float2str(float f, char *str) {
	int i = 0, temp = 0, sign = 0;
	if (f < 0) {
		f = -f;
		sign = 1;
	}
	temp = f * 10000;
	for (int j = 0; j < 4; j++) {
		str[i++] = temp % 10 + '0';
		temp /= 10;
	}
	str[i++] = '.';
	temp = (int) f;
	do {
		str[i++] = temp % 10 + '0';
	} while ((temp /= 10) > 0);
	if (sign != 0)
		str[i++] = '-';
	str[i] = '\0';
	reverse(str);
}

uint16_t shell_atoi(char *str) {
	uint16_t num = 0;
	while (*str != '\0') {
		num *= 10;
		num += (*str - '0');
		str++;
	}
	return num;
}

float sqrt1(const float x) {
	union {
		int i;
		float x;
	} u;
	u.x = x;
	u.i = (1 << 29) + (u.i >> 1) - (1 << 22);

	// Two Babylonian Steps (simplified from:)
	u.x = u.x + x / u.x;
	u.x = 0.25f * u.x + x / u.x;

	return u.x;
}
