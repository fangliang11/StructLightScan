#pragma once


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
	float mod;
	int x_connectivity;
	int y_connectivity;
	int no_of_edges;
} params_t;

//PIXELM information
struct PIXELM
{
	int increment;		//No. of 2*pi to add to the pixel to unwrap it
	int number_of_pixels_in_group;//No. of pixel in the pixel group
	float value;			//value of the pixel
	float reliability;
	unsigned char input_mask;	//0 pixel is masked. NOMASK pixel is not masked
	unsigned char extended_mask;	//0 pixel is masked. NOMASK pixel is not masked
	int group;			//group No.
	int new_group;
	struct PIXELM *head;		//pointer to the first pixel in the group in the linked list
	struct PIXELM *last;		//pointer to the last pixel in the group
	struct PIXELM *next;		//pointer to the next pixel in the group
};

typedef struct PIXELM PIXELM;
typedef enum { yes, no } yes_no;

//the EDGE is the line that connects two pixels.
//if we have S pixels, then we have S horizontal edges and S vertical edges
struct EDGE
{
	float reliab;			//reliabilty of the edge and it depends on the two pixels
	PIXELM *pointer_1;		//pointer to the first pixel
	PIXELM *pointer_2;		//pointer to the second pixel
	int increment;		//No. of 2*pi to add to one of the pixels to
				  //unwrap it with respect to the second
};

typedef struct EDGE EDGE;


class CUnwrap2D
{
public:
	CUnwrap2D();
	~CUnwrap2D();


public:
	void unwrap2D(float* wrapped_image, float* UnwrappedImage, unsigned char* input_mask,
		int image_width, int image_height,
		int wrap_around_x, int wrap_around_y);


private:
	yes_no find_pivot(EDGE *left, EDGE *right, float *pivot_ptr);
	EDGE *partition(EDGE *left, EDGE *right, float pivot);
	void quicker_sort(EDGE *left, EDGE *right);
	void  initialisePIXELs(float *wrapped_image, unsigned char *input_mask, unsigned char *extended_mask, PIXELM *pixel, int image_width, int image_height);
	float wrap(float pixel_value);
	int find_wrap(float pixelL_value, float pixelR_value);
	void extend_mask(unsigned char *input_mask, unsigned char *extended_mask,
		int image_width, int image_height,
		params_t *params);
	void calculate_reliability(float *wrappedImage, PIXELM *pixel,
		int image_width, int image_height,
		params_t *params);
	void  horizontalEDGEs(PIXELM *pixel, EDGE *edge,
		int image_width, int image_height,
		params_t *params);
	void  verticalEDGEs(PIXELM *pixel, EDGE *edge,
		int image_width, int image_height,
		params_t *params);
	void  gatherPIXELs(EDGE *edge, params_t *params);
	void  unwrapImage(PIXELM *pixel, int image_width, int image_height);
	void  maskImage(PIXELM *pixel, unsigned char *input_mask, int image_width, int image_height);
	void  returnImage(PIXELM *pixel, float *unwrapped_image, int image_width, int image_height);



};

