#include "sick_scan/sick_scan_base.h" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */
#ifndef PGMHANDLING_H
#define PGMHANDLING_H

typedef struct _PGMData {
    int row;
    int col;
    int max_gray;
    int **matrix;
} PGMData;

int **allocate_dynamic_matrix(int row, int col);
void deallocate_dynamic_matrix(int **matrix, int row);
void PGMSet(PGMData *data, unsigned char ucColor);
void PGMLine(PGMData *data, int x1, int y1, int x2, int y2, unsigned char ucColor);
unsigned char PGMGetPixel(PGMData *data, int x0, int y0);
void PGMSetPixel(PGMData *data, int x0, int y0, unsigned char ucColor);
void PGMDrawCross(PGMData *data, int x0, int y0, unsigned char ucColor, int halfLen, int width);
PGMData* readPGM(PGMData *data, const char *file_name);
void writePGM(const PGMData *data, const char *filename);
void freePGM(const PGMData *data);
PGMData* initPgm(int rows, int cols);
void PGMPutText(PGMData *data, char *szText, int xPos, int yPos, unsigned char ucColor);
void PGMEllipse(const PGMData *data, float xc, float yc, float *vecArr, unsigned char ucColor);


#endif
