#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include "pgmhandling.h"
#pragma warning(disable: 4996)

unsigned char fontCharBlock8x12[]; // forward declaration

void PGMSet(PGMData *data, unsigned char ucColor)
{
	for (int i = 0; i < data->row; i++)
	{
		for (int j = 0; j < data->col; j++)
		{
			data->matrix[i][j] = ucColor;
		}
	}
}

unsigned char PGMGetPixel(PGMData *data, int x0, int y0)
{
	return(data->matrix[y0][x0]);
}

void PGMDrawCross(PGMData *data, int x0, int y0, unsigned char ucColor, int halfLen, int width)
{
	for (int i = x0 - halfLen; i <= (x0 + halfLen); i++)
	{
		for (int ii = -width / 2; ii <= width / 2; ii++)
		{
			for (int jj = -width / 2; jj <= width / 2; jj++)
			{
				PGMSetPixel(data, i + ii, y0 + jj, ucColor);
			}
		}
	}
	for (int j = y0 - halfLen; j <= (y0 + halfLen); j++)
	{
		for (int ii = -width / 2; ii <= width / 2; ii++)
		{
			for (int jj = -width / 2; jj <= width / 2; jj++)
			{
				PGMSetPixel(data, x0 + ii, j + jj, ucColor);
			}
		}
	}
}
void PGMSetPixel(PGMData *data, int x0, int y0, unsigned char ucColor)
{
	if ((y0 < 0) || (x0 < 0))
	{
		printf("ACHTUNG!\n");
	}
	else
	{

		if ((x0 >= data->col) || (y0 >= data->row))
	{
		printf("ACHTUNG!\n");
	}
		else
		{
	data->matrix[y0][x0] = ucColor;
		}
	}
}

void PGMLine(PGMData *data, int x0, int y0, int x1, int y1, unsigned char ucColor)
{

	int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
	int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
	int err = (dx > dy ? dx : -dy) / 2, e2;

	for (;;) {
		PGMSetPixel(data, x0, y0, ucColor);
		if (x0 == x1 && y0 == y1) break;
		e2 = err;
		if (e2 > -dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
	}
}



int **allocate_dynamic_matrix(int row, int col)
{
	int **ret_val;
	int i;

	ret_val = (int **)malloc(sizeof(int *) * row);
	if (ret_val == NULL) {
		perror("memory allocation failure");
		exit(EXIT_FAILURE);
	}

	for (i = 0; i < row; ++i) {
		ret_val[i] = (int *)malloc(sizeof(int) * col);
		if (ret_val[i] == NULL) {
			perror("memory allocation failure");
			exit(EXIT_FAILURE);
		}
	}

	return ret_val;
}

void deallocate_dynamic_matrix(int **matrix, int row)
{
	int i;

	for (i = 0; i < row; ++i) {
		free(matrix[i]);
	}
	free(matrix);
}

#define HI(num) (((num) & 0x0000FF00) << 8) 
#define LO(num) ((num) & 0x000000FF) 

void SkipComments(FILE *fp)
{
	int ch;
	char line[100];
	while ((ch = fgetc(fp)) != EOF && isspace(ch)) {
		;
	}

	if (ch == '#') {
		fgets(line, sizeof(line), fp);
		SkipComments(fp);
	}
	else {
		fseek(fp, -1, SEEK_CUR);
	}
}

/*for reading:*/
PGMData* readPGM(PGMData *data, const char *file_name)
{
	FILE *pgmFile;
	char version[3];
	int i, j;
	int lo, hi;
	pgmFile = fopen(file_name, "rb");
	if (pgmFile == NULL) {
		perror("cannot open file to read");
		exit(EXIT_FAILURE);
	}
	fgets(version, sizeof(version), pgmFile);
	if (strcmp(version, "P5")) {
		fprintf(stderr, "Wrong file type!\n");
		exit(EXIT_FAILURE);
	}
	SkipComments(pgmFile);
	fscanf(pgmFile, "%d", &data->col);
	SkipComments(pgmFile);
	fscanf(pgmFile, "%d", &data->row);
	SkipComments(pgmFile);
	fscanf(pgmFile, "%d", &data->max_gray);
	fgetc(pgmFile);

	data->matrix = allocate_dynamic_matrix(data->row, data->col);
	if (data->max_gray > 255) {
		for (i = 0; i < data->row; ++i) {
			for (j = 0; j < data->col; ++j) {
				hi = fgetc(pgmFile);
				lo = fgetc(pgmFile);
				data->matrix[i][j] = (hi << 8) + lo;
			}
		}
	}
	else {
		for (i = 0; i < data->row; ++i) {
			for (j = 0; j < data->col; ++j) {
				lo = fgetc(pgmFile);
				data->matrix[i][j] = lo;
			}
		}
	}

	fclose(pgmFile);
	return data;

}

/*and for writing*/

void writePGM(const PGMData *data, const char *filename)
{
	FILE *pgmFile;
	int i, j;
	int hi, lo;

	pgmFile = fopen(filename, "wb");
	if (pgmFile == NULL) {
		perror("cannot open file to write");
		exit(EXIT_FAILURE);
	}

	fprintf(pgmFile, "P5 ");
	fprintf(pgmFile, "%d %d ", data->col, data->row);
	fprintf(pgmFile, "%d ", data->max_gray);

	if (data->max_gray > 255) {
		for (i = 0; i < data->row; ++i) {
			for (j = 0; j < data->col; ++j) {
				hi = HI(data->matrix[i][j]);
				lo = LO(data->matrix[i][j]);
				fputc(hi, pgmFile);
				fputc(lo, pgmFile);
			}

		}
	}
	else {
		for (i = 0; i < data->row; ++i) {
			for (j = 0; j < data->col; ++j) {
				lo = LO(data->matrix[i][j]);
				fputc(lo, pgmFile);
			}
		}
	}

	fclose(pgmFile);
}

void freePGM(const PGMData *data)
{
	deallocate_dynamic_matrix(data->matrix, data->row);
}

PGMData* initPgm(int rows, int cols)
{
	PGMData* tmp = (PGMData *)malloc(sizeof(PGMData));
	tmp->col = cols;
	tmp->row = rows;
	tmp->max_gray = 255;
	tmp->matrix = allocate_dynamic_matrix(rows, cols);
	return(tmp);
}



void PGMPutText(PGMData *data, char *szText, int xPos, int yPos, unsigned char ucColor)
{
	int strLen = (int)strlen(szText);
	int xTmp = xPos;
	for (int i = 0; i < strLen; i++)
	{
		char ch = szText[i];
		if ((ch >= ' ') && (ch < 0x80))
		{
			int idx = ch - ' ';
			int offset = idx * 12;
			for (int ii = 0; ii < 12; ii++)
			{
				unsigned char ucBitMask = fontCharBlock8x12[offset + ii];
				for (int jj = 0; jj < 8; jj++)
				{
					if (ucBitMask & (0x01 << (7 - jj)))
					{
						PGMSetPixel(data, xTmp + jj, yPos + ii, ucColor);
					}
				}
			}
		}
		xTmp += 8;
	}
}


void PGMEllipse(const PGMData *data, float xc, float yc, float *vecArr, unsigned char ucColor)
{
	
	double halfmajoraxissize = vecArr[0] * vecArr[0] + vecArr[2] * vecArr[2];
	double halfminoraxissize = vecArr[1] * vecArr[1] + vecArr[3] * vecArr[3];

	halfmajoraxissize = sqrt(halfmajoraxissize);
	halfminoraxissize = sqrt(halfminoraxissize);
	double halfBiggestSize = (halfminoraxissize > halfmajoraxissize) ? halfminoraxissize : halfmajoraxissize;

	float a, b;
	a = (float)halfmajoraxissize;
	b = (float)halfminoraxissize;
	float theta = (float)atan2(vecArr[2], vecArr[0]);
	
	int numStepsEsti = (int)(halfBiggestSize * 6.28); // Kreisumfang mal grob abschaetzen

	if (numStepsEsti < 1)
	{
		numStepsEsti = 1;
	}
	float stepWidth = (float)(2.0 * M_PI / numStepsEsti);
	numStepsEsti++; // damit der Kreis geschlossen wird
	int xpi = 0;
	int ypi = 0;
	for (int i = 0; i < numStepsEsti; i++)
	{
		float t = i * stepWidth;
		int xcur = (int)(xc + a*cos(t)*cos(theta) - b*sin(t)*sin(theta));
		int ycur = (int)(yc + b*sin(t)*cos(theta) + a*cos(t)*sin(theta));

		if (i > 0)
		{
			PGMLine((PGMData *)data, xpi, ypi, xcur, ycur, ucColor);

		}
		xpi = xcur;
		ypi = ycur;
	}

}



unsigned char fontCharBlock8x12[] =
{
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00	 , //  
	 0x00,0x18,0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x00,0x00	 , // !
	 0x00,0x36,0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00	 , // "
	 0x00,0x00,0x14,0x14,0x3E,0x14,0x14,0x3E,0x14,0x14,0x00,0x00	 , // #
	 0x00,0x08,0x1C,0x32,0x30,0x1C,0x06,0x26,0x1C,0x08,0x08,0x00	 , // $
	 0x00,0x13,0x2B,0x2B,0x16,0x0C,0x1A,0x35,0x35,0x32,0x00,0x00	 , // %
	 0x00,0x1C,0x36,0x36,0x1C,0x1D,0x37,0x36,0x36,0x1B,0x00,0x00	 , // &
	 0x00,0x0C,0x0C,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00	 , // '
	 0x00,0x0C,0x18,0x30,0x30,0x30,0x30,0x30,0x18,0x0C,0x00,0x00	 , // (
	 0x00,0x30,0x18,0x0C,0x0C,0x0C,0x0C,0x0C,0x18,0x30,0x00,0x00	 , // )
	 0x00,0x00,0x00,0x36,0x1C,0x7F,0x1C,0x36,0x00,0x00,0x00,0x00	 , // *
	 0x00,0x00,0x00,0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00,0x00	 , // +
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x0C,0x18	 , // ,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x00,0x00	 , // -
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00	 , // .
	 0x00,0x06,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x60,0x00,0x00	 , // /
	 0x00,0x1E,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x3C,0x00,0x00	 , // 0
	 0x00,0x0C,0x1C,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00	 , // 1
	 0x00,0x1C,0x36,0x36,0x06,0x0C,0x0C,0x18,0x30,0x3E,0x00,0x00	 , // 2
	 0x00,0x1C,0x36,0x06,0x06,0x1C,0x06,0x06,0x36,0x1C,0x00,0x00	 , // 3
	 0x00,0x18,0x18,0x18,0x36,0x36,0x36,0x3F,0x06,0x06,0x00,0x00	 , // 4
	 0x00,0x3E,0x30,0x30,0x30,0x3C,0x06,0x06,0x36,0x1C,0x00,0x00	 , // 5
	 0x00,0x0C,0x18,0x18,0x30,0x3C,0x36,0x36,0x36,0x1C,0x00,0x00	 , // 6
	 0x00,0x3E,0x06,0x06,0x0C,0x0C,0x0C,0x18,0x18,0x18,0x00,0x00	 , // 7
	 0x00,0x1C,0x36,0x36,0x36,0x1C,0x36,0x36,0x36,0x1C,0x00,0x00	 , // 8
	 0x00,0x1C,0x36,0x36,0x36,0x1E,0x0C,0x0C,0x18,0x30,0x00,0x00	 , // 9
	 0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x1C,0x1C,0x00,0x00	 , // :
	 0x00,0x00,0x00,0x1C,0x1C,0x00,0x00,0x00,0x1C,0x1C,0x0C,0x18	 , // , //
	 0x00,0x02,0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x02,0x00,0x00	 , // <
	 0x00,0x00,0x00,0x00,0x3E,0x00,0x00,0x3E,0x00,0x00,0x00,0x00	 , // =
	 0x00,0x20,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x20,0x00,0x00	 , // >
	 0x00,0x1C,0x36,0x36,0x06,0x0C,0x18,0x00,0x18,0x18,0x00,0x00	 , // ?
	 0x00,0x0E,0x19,0x33,0x35,0x35,0x35,0x32,0x18,0x0F,0x00,0x00	 , // @
	 0x00,0x08,0x1C,0x36,0x36,0x36,0x3E,0x36,0x36,0x36,0x00,0x00	 , // A
	 0x00,0x3C,0x36,0x36,0x36,0x3C,0x36,0x36,0x36,0x3C,0x00,0x00	 , // B
	 0x00,0x1C,0x36,0x36,0x30,0x30,0x30,0x36,0x36,0x1C,0x00,0x00	 , // C
	 0x00,0x3C,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x3C,0x00,0x00	 , // D
	 0x00,0x3E,0x30,0x30,0x30,0x3C,0x30,0x30,0x30,0x3E,0x00,0x00	 , // E
	 0x00,0x3E,0x30,0x30,0x30,0x3C,0x30,0x30,0x30,0x30,0x00,0x00	 , // F
	 0x00,0x1C,0x36,0x30,0x30,0x36,0x36,0x36,0x36,0x1C,0x00,0x00	 , // G
	 0x00,0x36,0x36,0x36,0x36,0x3E,0x36,0x36,0x36,0x36,0x00,0x00	 , // H
	 0x00,0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00,0x00	 , // I
	 0x00,0x06,0x06,0x06,0x06,0x06,0x06,0x36,0x36,0x1C,0x00,0x00	 , // J
	 0x00,0x36,0x36,0x36,0x3C,0x38,0x3C,0x36,0x36,0x36,0x00,0x00	 , // K
	 0x00,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x3E,0x00,0x00	 , // L
	 0x00,0x63,0x63,0x77,0x77,0x7F,0x6B,0x6B,0x63,0x63,0x00,0x00	 , // M
	 0x00,0x33,0x33,0x3B,0x3B,0x3F,0x37,0x33,0x33,0x33,0x00,0x00	 , // N
	 0x00,0x1C,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x1C,0x00,0x00	 , // O
	 0x00,0x3C,0x36,0x36,0x36,0x3C,0x30,0x30,0x30,0x30,0x00,0x00	 , // P
	 0x00,0x1C,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x1C,0x0C,0x06	 , // Q
	 0x00,0x3C,0x36,0x36,0x36,0x3C,0x36,0x36,0x36,0x36,0x00,0x00	 , // R
	 0x00,0x1C,0x36,0x32,0x38,0x1C,0x0E,0x26,0x36,0x1C,0x00,0x00	 , // S
	 0x00,0x3F,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x00,0x00	 , // T
	 0x00,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x1C,0x00,0x00	 , // U
	 0x00,0x36,0x36,0x36,0x36,0x36,0x36,0x36,0x1C,0x08,0x00,0x00	 , // V
	 0x00,0x63,0x63,0x6B,0x6B,0x6B,0x6B,0x36,0x36,0x36,0x00,0x00	 , // W
	 0x00,0x36,0x36,0x36,0x1C,0x08,0x1C,0x36,0x36,0x36,0x00,0x00	 , // X
	 0x00,0x33,0x33,0x33,0x33,0x1E,0x0C,0x0C,0x0C,0x0C,0x00,0x00	 , // Y
	 0x00,0x3E,0x06,0x0C,0x0C,0x18,0x18,0x30,0x30,0x3E,0x00,0x00	 , // Z
	 0x00,0x1E,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x1E	 , // [
	 0x00,0x30,0x30,0x18,0x18,0x0C,0x0C,0x06,0x06,0x03,0x00,0x00	 , // Backslash
	 0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C	 , // ]
	 0x00,0x08,0x1C,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00	 , // ^
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F	 , // _
	 0x00,0x18,0x18,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00	 , // `
	 0x00,0x00,0x00,0x00,0x1C,0x06,0x1E,0x36,0x36,0x1E,0x00,0x00	 , // a
	 0x00,0x30,0x30,0x30,0x3C,0x36,0x36,0x36,0x36,0x3C,0x00,0x00	 , // b
	 0x00,0x00,0x00,0x00,0x1C,0x36,0x30,0x30,0x36,0x1C,0x00,0x00	 , // c
	 0x00,0x06,0x06,0x06,0x1E,0x36,0x36,0x36,0x36,0x1E,0x00,0x00	 , // d
	 0x00,0x00,0x00,0x00,0x1C,0x36,0x3E,0x30,0x32,0x1C,0x00,0x00	 , // e
	 0x00,0x0E,0x18,0x18,0x3E,0x18,0x18,0x18,0x18,0x18,0x00,0x00	 , // f
	 0x00,0x00,0x00,0x00,0x1E,0x36,0x36,0x36,0x1E,0x06,0x26,0x1C	 , // g
	 0x00,0x30,0x30,0x30,0x3C,0x36,0x36,0x36,0x36,0x36,0x00,0x00	 , // h
	 0x00,0x0C,0x00,0x00,0x1C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00,0x00	 , // i
	 0x00,0x0C,0x00,0x00,0x1C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x38	 , // j
	 0x00,0x30,0x30,0x30,0x36,0x3C,0x38,0x3C,0x36,0x36,0x00,0x00	 , // k
	 0x00,0x1C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00,0x00	 , // l
	 0x00,0x00,0x00,0x00,0x76,0x7F,0x6B,0x6B,0x6B,0x63,0x00,0x00	 , // m
	 0x00,0x00,0x00,0x00,0x3C,0x36,0x36,0x36,0x36,0x36,0x00,0x00	 , // n
	 0x00,0x00,0x00,0x00,0x1C,0x36,0x36,0x36,0x36,0x1C,0x00,0x00	 , // o
	 0x00,0x00,0x00,0x00,0x3C,0x36,0x36,0x36,0x36,0x3C,0x30,0x30	 , // p
	 0x00,0x00,0x00,0x00,0x1E,0x36,0x36,0x36,0x36,0x1E,0x06,0x06	 , // q
	 0x00,0x00,0x00,0x00,0x36,0x3E,0x30,0x30,0x30,0x30,0x00,0x00	 , // r
	 0x00,0x00,0x00,0x00,0x1E,0x30,0x3C,0x1E,0x06,0x3C,0x00,0x00	 , // s
	 0x00,0x00,0x18,0x18,0x3C,0x18,0x18,0x18,0x18,0x0E,0x00,0x00	 , // t
	 0x00,0x00,0x00,0x00,0x36,0x36,0x36,0x36,0x36,0x1E,0x00,0x00	 , // u
	 0x00,0x00,0x00,0x00,0x36,0x36,0x36,0x36,0x1C,0x08,0x00,0x00	 , // v
	 0x00,0x00,0x00,0x00,0x63,0x6B,0x6B,0x6B,0x3E,0x36,0x00,0x00	 , // w
	 0x00,0x00,0x00,0x00,0x36,0x36,0x1C,0x1C,0x36,0x36,0x00,0x00	 , // x
	 0x00,0x00,0x00,0x00,0x36,0x36,0x36,0x36,0x1C,0x0C,0x18,0x30	 , // y
	 0x00,0x00,0x00,0x00,0x3E,0x06,0x0C,0x18,0x30,0x3E,0x00,0x00	 , // z
	 0x00,0x0E,0x18,0x18,0x18,0x30,0x18,0x18,0x18,0x0E,0x00,0x00	 , // {
	 0x00,0x18,0x18,0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x18,0x18	 , // |
	 0x00,0x30,0x18,0x18,0x18,0x0C,0x18,0x18,0x18,0x30,0x00,0x00	 , // }
	 0x00,0x1A,0x3E,0x2C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00	 , // ~
	 0x00,0x00,0x00,0x08,0x1C,0x36,0x22,0x22,0x3E,0x3E,0x00,0x00	 , // 
};


#if 0
void PGMEllipseRotated
t = linspace(0, 2 * pi, 100);
theta = deg2rad(105);
a = 2;
b = 1;
x0 = 0.15;
y0 = 0.30;
x = x0 + a*cos(t)*cos(theta) - b*sin(t)*sin(theta);
y = y0 + b*sin(t)*cos(theta) + a*cos(t)*sin(theta);
figure;
plot(x, y);
axis equal;
#endif
#if 0
https://stackoverflow.com/questions/2163370/byte-codes-for-pixel-maps-for-ascii-characters
#endif