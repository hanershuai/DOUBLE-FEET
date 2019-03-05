/*
 * image.h
 *
 *  Created on: 2014年9月15日
 *      Author: xuf
 */


#ifndef IMAGE_H_
#define IMAGE_H_

#include "global.h"



int Image_Init();
void Image_Run();
void Image_Show();
void Image_Clear();
void Color_Table_field_ball(IplImage *src );
void Color_Table_goal(IplImage *src );
int Read_ColorTable();
int Read_ColorTable_Near();
void Test(IplImage *src);

bool get_ball_pos(CvMat *src);
bool get_near_ball_pos(CvMat *src);
bool get_ball_pos_1(CvMat *src);
bool get_ball_pos_3(IplImage *src);
bool Goal_find(int line_range,int white_ratio,int green_ratio,int threshold_hough,int param1,int param2);
void print(IplImage* img, int x, int y, const char* text);
bool get_ball_pos_2( CvMat* img, float dp,
                         int min_radius, int max_radius,
                          float acc_threshold,float max_count_best_threshold);
bool image_update();
void 			Image_search(void);
void 			Image_chase(void);
void 			Image_round_yaw(void);
void 			Image_search_goal(void);
void 			Image_round_goal(void);
void            Image_adjust_ball(void);
void            Image_approach_ball(void);
void 			Image_kick(void);


#endif /* IMAGE_H_ */
