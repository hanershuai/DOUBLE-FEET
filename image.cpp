/*
 * image.cpp
*/


#include "global.h"

#define hough_cmp_gt(l1,l2) (aux[l1] > aux[l2])
//static CV_IMPLEMENT_QSORT_EX( icvHoughSortDescent32s, int, hough_cmp_gt, const int* );
//找球的坐标
double t_begin;											//用于计时,找到球时间
double t_end;												//丢球时间
 //IplImage *BI,*RI,*GI;
void 			Image_search(void)
{
	Image_Run();
//	cout<<"image search"<<endl;
	robot_info.ball_find = get_ball_pos(mat);

	if(!robot_info.ball_find){
		if(robot_info.ball_info.x < 160){
			if(robot_info.ball_info.y < 120){
				robot_info.lost_ball_states = 1;	//左上
			}
			else{
				robot_info.lost_ball_states = 2;	//左下
			}
		}
		else{
			if(robot_info.ball_info.y < 120){
				robot_info.lost_ball_states = 3;	//右上
			}
			else{
				robot_info.lost_ball_states = 4;	//右下  判断球的位置
			}
		}
	}

	Image_Show();

}
void 			Image_chase(void)
{
	Image_Run();
	if(robot_info.ball_dis4robot>70)  	robot_info.ball_find = get_ball_pos(mat);
	else	robot_info.ball_find = get_near_ball_pos(mat) ;
	//robot_info.ball_find = get_near_ball_pos(mat) ;
		if(!robot_info.ball_find){
			if(robot_info.ball_info.x < 160){
				if(robot_info.ball_info.y < 120){
					robot_info.lost_ball_states = 1;	//左上
				}
				else{
					robot_info.lost_ball_states = 2;	//左下
				}
			}
			else{
				if(robot_info.ball_info.y < 120){
					robot_info.lost_ball_states = 3;	//右上
				}
				else{
					robot_info.lost_ball_states = 4;	//右下
				}
			}
		}

		Image_Show();

}
void 			Image_round_yaw(void)
{
	/*cout<<"image round_yaw"<<endl;*/

	Image_Run();
	robot_info.ball_find = get_ball_pos(mat);

	if(!robot_info.ball_find){
		if(robot_info.ball_info.x < 160){
			if(robot_info.ball_info.y < 120){
				robot_info.lost_ball_states = 1;	//左上
			}
			else{
				robot_info.lost_ball_states = 2;	//左下
			}
		}
		else{
			if(robot_info.ball_info.y < 120){
				robot_info.lost_ball_states = 3;	//右上
			}
			else{
				robot_info.lost_ball_states = 4;	//右下
			}
		}
	}

	Image_Show();

}

void 			Image_search_goal(void)
{
	//cout<<"image search goal"<<endl;
	Image_Run();
	robot_info.goal_find=Goal_find(100,0.5,0.5,threshold_houghline,param1,param2);
	Image_Show();
}
void            Image_adjust_ball(void)
{
//	cout<<"image adjust_ball"<<endl;
		Image_Run();
		robot_info.ball_find = get_ball_pos(mat);
		if(!robot_info.ball_find){
			if(robot_info.ball_info.x < 160){
				if(robot_info.ball_info.y < 120){
					robot_info.lost_ball_states = 1;	//左上
				}
				else{
					robot_info.lost_ball_states = 2;	//左下
				}
			}
			else{
				if(robot_info.ball_info.y < 120){
					robot_info.lost_ball_states = 3;	//右上
				}
				else{
					robot_info.lost_ball_states = 4;	//右下
				}
			}
		}
		Image_Show();

}


void 			Image_round_goal(void)
{
	//cout<<"image round_goal"<<endl;
	Image_Run();
	if(robot_info.ball_dis4robot>70)  	robot_info.ball_find = get_ball_pos(mat);
	else	robot_info.ball_find = get_near_ball_pos(mat) ;
	//robot_info.ball_find = get_near_ball_pos(mat) ;
	if(!robot_info.ball_find){
		if(robot_info.ball_info.x < 160){
			if(robot_info.ball_info.y < 120){
				robot_info.lost_ball_states = 1;	//左上
			}
			else{
				robot_info.lost_ball_states = 2;	//左下
			}
		}
		else{
			if(robot_info.ball_info.y < 120){
				robot_info.lost_ball_states = 3;	//右上
			}
			else{
				robot_info.lost_ball_states = 4;	//右下
			}
		}
	}
	Image_Show();
}


void 			Image_approach_ball(void)
{
	Image_Run();
	if(robot_info.ball_dis4robot>70)  	robot_info.ball_find = get_ball_pos(mat);
	else	robot_info.ball_find = get_near_ball_pos(mat) ;

		if(!robot_info.ball_find){
			if(robot_info.ball_info.x < 160){
				if(robot_info.ball_info.y < 120){
					robot_info.lost_ball_states = 1;	//左上
				}
				else{
					robot_info.lost_ball_states = 2;	//左下
				}
			}
			else{
				if(robot_info.ball_info.y < 120){
					robot_info.lost_ball_states = 3;	//右上
				}
				else{
					robot_info.lost_ball_states = 4;	//右下
				}
			}
		}

		Image_Show();

}



void Image_kick(void)
{
	//cout<<"image kick"<<endl;
	robot_info.ball_find =false;

}




//图像初始化
int Image_Init(void)
{
	if(Read_ColorTable() == 0)
			{
				return 0;
			}
	//选择摄像头
//#ifdef GONG_KONG
	//cout<<"choose camera 0"<<endl;
	//capture = cvCreateCameraCapture(0);
//#else
	cout<<"choose camera 1"<<endl;
	capture = cvCreateCameraCapture(-1);
//#endif

	Image = cvQueryFrame(capture);
#ifndef GONG_KONG
	cvNamedWindow("show_Image",1);
//	cvNamedWindow("field_Image",1);
//	cvNamedWindow("ball_Image",1);
//	cvNamedWindow("goal_Image",1);
//	cvNamedWindow("goal_color_dst",1);
//	cvNamedWindow("goal_canny_Image",1);
//	cvNamedWindow("edges",1);
#endif

			CvSize sz;
			//这里对图像改变大小
			stor = cvCreateMemStorage(0);
			stor_vertical= cvCreateMemStorage(0);
			lines = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
			lines_vertical = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), stor_vertical);
			sz.height = Image->height*0.5;
			sz.width = Image->width*0.5;
			pImage = cvCreateImage( sz, Image->depth, 3); //构造目标图象
			cvResize(Image,pImage,CV_INTER_LINEAR);
			show_Image = cvCreateImage(cvGetSize(pImage),pImage->depth,3);
			ball_Image = cvCreateImage(cvGetSize(pImage),pImage->depth,pImage->nChannels);
			field_Image=cvCreateImage(cvGetSize(pImage),pImage->depth,pImage->nChannels);
			goal_Image=cvCreateImage(cvGetSize(pImage),pImage->depth,pImage->nChannels);
			ball_Image_gray=cvCreateImage(cvGetSize(pImage),pImage->depth,1);
			field_Image_gray = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
			goal_Image_gray = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
			goal_canny_Image = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
			goal_color_dst = cvCreateImage(cvGetSize(pImage),pImage->depth,3);

			ball_Image_gray1=cvCreateImage(cvGetSize(pImage),pImage->depth,1);
            //ball_Image_gray2=cvCreateImage(cvGetSize(pImage),pImage->depth,1);

//			cvCvtColor(pImage, pImage, CV_RGB2HSV);											//这些操作在while(1)中也有,这是对第一帧的操作,因为edges需要用mat来初始化,而mat需要HSVImagegray
//			Color_Table_HSV(pImage);
//			cvCvtColor(HSVImage,HSVImagegray,CV_RGB2GRAY);
//			cvCvtColor(goal_RGB,goal_Gray,CV_RGB2GRAY);
//			cvCvtColor(white_Image,white_cvtImage,CV_RGB2GRAY);

//			cvCreateTrackbar("threshold","show_Image",&threshold_houghline,200,NULL);
//			cvCreateTrackbar("param1","show_Image",&param1,100,NULL);
//			cvCreateTrackbar("param2","show_Image",&param2,100,NULL);

//			cvCreateTrackbar("kp_low","Image",&int_low_kp,200,NULL);
//			cvCreateTrackbar("ki_low","Image",&int_low_ki,200,NULL);
//			cvCreateTrackbar("kd_low","Image",&int_low_kd,200,NULL);
//			cvCreateTrackbar("kp_up","Image",&int_up_kp,200,NULL);
//			cvCreateTrackbar("ki_up","Image",&int_up_ki,200,NULL);
//			cvCreateTrackbar("kd_up","Image",&int_up_kd,200,NULL);
//			cvCreateTrackbar("white_points_low_2","show_Image",&white_points_low_2,300,NULL);
//			cvCreateTrackbar("white_points_up_2","show_Image",&white_points_up_2,300,NULL);
//			cvCreateTrackbar("green_points_low_2","show_Image",&green_points_low_2,300,NULL);
//			cvCreateTrackbar("green_points_up_2","show_Image",&green_points_up_2,300,NULL);
//			cvCreateTrackbar("green_circle_low_2","show_Image",&green_circle_low_2,300,NULL);
//			cvCreateTrackbar("green_circle_up_2","show_Image",&green_circle_up_2,300,NULL);
//			cvCreateTrackbar("max_count_best","HSVImage",&max_count_best_2,300,NULL);
//			cvCreateTrackbar("white_points_low_1","show_Image",&white_points_low_1,300,NULL);
//			cvCreateTrackbar("white_points_up_1","show_Image",&white_points_up_1,300,NULL);
//			cvCreateTrackbar("green_points_low_1","show_Image",&green_points_low_1,300,NULL);
//			cvCreateTrackbar("green_points_up_1","show_Image",&green_points_up_1,300,NULL);
            /*BI = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
			GI = cvCreateImage(cvGetSize(pImage),pImage->depth,1);
			RI = cvCreateImage(cvGetSize(pImage),pImage->depth,1);*/
			mat = cvGetMat(field_Image_gray, &temp);  //深拷贝
			//canny算子求单像素二值化边缘，保存在edges变量中
			edges = cvCreateMat( mat->rows, mat->cols, CV_8UC1 );
			return 1;
}

void Image_Run()
{
    static double time0=0;
    static double ball_R_Last=20;
    double ball_R=robot_info.ball_info.r;
	Image = cvQueryFrame(capture);//读取视频帧
	cvResize(Image,show_Image,CV_INTER_LINEAR);//将image设置成与show_Image大小相同
	cvResize(Image,pImage,CV_INTER_LINEAR);//与上类似

    /*cvSplit(pImage,BI,GI,RI,NULL);
    cvEqualizeHist(BI,BI);
    cvEqualizeHist(GI,GI);
    cvEqualizeHist(RI,RI);
    cvMerge(BI,GI,RI,NULL,pImage);
	cvShowImage("Pimage",pImage);*/

	Color_Table_goal(pImage);
	cvCvtColor(pImage, pImage, CV_RGB2HSV);
	Color_Table_field_ball(pImage);
    cvCvtColor(field_Image,field_Image_gray,CV_RGB2GRAY);
    cvCvtColor(ball_Image,ball_Image_gray,CV_RGB2GRAY);
	cvCvtColor(goal_Image,goal_Image_gray,CV_RGB2GRAY);//图像ball/goal Image gray是输出结果


	cvCvtColor(ball_Image,ball_Image_gray1,CV_RGB2GRAY);
	//cvCvtColor(ball_Image,ball_Image_gray2,CV_RGB2GRAY);
	//cvDilate(ball_Image_gray1,ball_Image_gray1,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_RECT));
	cvMorphologyEx(ball_Image_gray1,ball_Image_gray1,NULL,cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_RECT),CV_MOP_CLOSE);
	cvMorphologyEx(ball_Image_gray1,ball_Image_gray1,NULL,cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_CROSS),CV_MOP_CLOSE);
	cvMorphologyEx(ball_Image_gray1,ball_Image_gray1,NULL,cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_ELLIPSE),CV_MOP_CLOSE);
	cvMorphologyEx(ball_Image_gray1,ball_Image_gray1,NULL,cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_RECT),CV_MOP_OPEN);
	//Test(ball_Image_gray2);
	//cvSmooth(ball_Image_gray1,ball_Image_gray1,CV_GAUSSIAN,5,5);

    {CvPoint  *line;
			//cvCanny( goal_Image_gray, goal_canny_Image, 100, 200, 3 );//边缘检测
			cvClearSeq(lines);
			lines = cvHoughLines2( ball_Image_gray, stor, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, threshold_houghline, param1, 20 ); //直接得到直线序列
			for( int i = 0; i < lines ->total; i++ )  //lines存储的是直线
			{
						line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
						cvLine( field_Image_gray, line[0], line[1], CV_RGB( 255, 255, 255 ) );  //	color_dst是三通道图像用来存直线图像将找到的直线标记为白色
			            cvLine( ball_Image_gray, line[0], line[1], CV_RGB( 0, 0, 0 ) );
			            cvLine( field_Image_gray, cvPoint(line[0].x+1,line[0].y+1), cvPoint(line[1].x+1,line[1].y+1), CV_RGB( 255, 255, 255 ) );
                        cvLine( field_Image_gray, cvPoint(line[0].x-1,line[0].y-1), cvPoint(line[1].x-1,line[1].y-1), CV_RGB( 255, 255, 255 ) );
                        cvLine( ball_Image_gray, cvPoint(line[0].x+1,line[0].y+1), cvPoint(line[1].x+1,line[1].y+1), CV_RGB( 0,0,0 ) );
                        cvLine( ball_Image_gray, cvPoint(line[0].x-1,line[0].y-1), cvPoint(line[1].x-1,line[1].y-1), CV_RGB( 0,0,0 ) );
			}
    }
	//cvErode(field_Image_gray,field_Image_gray,cvCreateStructuringElementEx(2,2,1,1,CV_SHAPE_RECT));
	//cvDilate(ball_Image_gray,ball_Image_gray,cvCreateStructuringElementEx(2,2,1,1,CV_SHAPE_RECT));
	//cvErode(field_Image_gray,field_Image_gray,cvCreateStructuringElementEx(5,5,1,1,CV_SHAPE_CROSS));
	//cvDilate(ball_Image_gray,ball_Image_gray,cvCreateStructuringElementEx(5,5,1,1,CV_SHAPE_CROSS));
	//cvErode(field_Image_gray,field_Image_gray,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_ELLIPSE));
	//cvDilate(ball_Image_gray,ball_Image_gray,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_ELLIPSE));
	//cvMorphologyEx(ball_Image_gray,ball_Image_gray,NULL,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_RECT),CV_MOP_CLOSE);
	//cvMorphologyEx(ball_Image_gray,ball_Image_gray,NULL,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_CROSS),CV_MOP_CLOSE);
	//cvMorphologyEx(ball_Image_gray,ball_Image_gray,NULL,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_ELLIPSE),CV_MOP_CLOSE);
	//cvMorphologyEx(ball_Image_gray,ball_Image_gray,NULL,cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_RECT),CV_MOP_OPEN);
	cvErode(field_Image_gray,field_Image_gray);
	cvDilate(ball_Image_gray,ball_Image_gray);
	//cvShowImage("field_image_gray",field_Image_gray);
	cvShowImage("ball_Image_gray1",ball_Image_gray1);
    cvShowImage("ball_image_gray",ball_Image_gray);
    mat = cvGetMat(field_Image_gray, &temp);  //深拷贝
    //mat = cvGetMat(field_Image_gray_mat, &temp);  //深拷贝
	//cvShowImage("mat",mat);

	//Read_near_colorTable
	if((time0==0||get_time()-time0>=10)&&ball_R>50&&ball_R_Last<=50)
	{
	time0=get_time();
	cout<<"will read near near"<<endl;
	Read_ColorTable_Near();
	cout<<"have read near near"<<endl;
	}
	else if(get_time()-time0>=10&&ball_R<50&&ball_R_Last>=50)
	{
	time0=get_time();
	cout<<"will read color"<<endl;
	Read_ColorTable();
	cout<<"have read color"<<endl;
	}
	ball_R_Last=ball_R;
}
void Image_Show()
{
//#ifndef GONG_KONG
	 cvShowImage("edges",edges);
	cvShowImage("show_Image",show_Image);
	//cvShowImage("goal_canny_Image",goal_canny_Image);
//	cvShowImage("goal_Image",goal_Image);
//	cvShowImage("ball_Image",ball_Image);
	//cvShowImage("field_Image",field_Image);
	cvShowImage("goal_Image_gray",goal_Image_gray);
//	cvShowImage("ball_Image_gray",ball_Image_gray);
//	cvShowImage("field_Image_gray",field_Image_gray);
//	cvShowImage("goal_color_dst",goal_color_dst);
//#endif
	cvSetZero(goal_canny_Image);
	cvSetZero(goal_color_dst);
	cvSetZero(edges);
//	cvSetZero(field_Image);
//	cvSetZero(goal_Image);
//	cvSetZero(ball_Image);

}

void Image_Clear()
{
	cvDestroyWindow("Image");
	cvDestroyWindow("HSVImage");
	cvDestroyWindow("edges");
	cvDestroyWindow("white_cvtimage");
	cvDestroyWindow("goal_color_dst");
	cvClearMemStorage(stor);
	cvClearMemStorage(stor_vertical);
	cvReleaseImage(&ball_Image);
	cvReleaseImage(&field_Image);
	cvReleaseImage(&goal_Image);
	cvReleaseCapture(&capture);

}

void Color_Table_field_ball(IplImage *src )
{
	CvSize  size=cvGetSize(src);
	IplImage *temp_image=cvCreateImage(cvGetSize(pImage),pImage->depth,3);
//	long i;
	CvScalar s;
	int x,y;
	int nstep=src->widthStep;
	int nchannel=src->nChannels;
    for(y=0;y<size.height;y++)
    {
    	for(x=0;x<size.width;x++)
	    {
	    	//得到HS值
		    H=src->imageData[y*nstep+x*nchannel+0];
		    S=src->imageData[y*nstep+x*nchannel+1];
		    V=src->imageData[y*nstep+x*nchannel+2];
            H=H/4; S=S/4;V=V/4;
		  // 查表

		   switch (HSV[H][S][V])
		   {
		   case  0:
		   //cout<<"0"<<endl;												//代表其他颜色
			   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(field_Image,y,x,s);//set the (i,j) pixel value
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(ball_Image,y,x,s);//set the (i,j) pixel value
			   break;
		   case  1:
		 //  cout<<"1"<<endl;														//代表绿色
			   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(field_Image,y,x,s);//set the (i,j) pixel value
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(ball_Image,y,x,s);//set the (i,j) pixel value
		       break;
	   	   case  2:
	   	 //  cout<<"2"<<endl;													//代表白色
	   	   	   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(field_Image,y,x,s);//set the (i,j) pixel value
	   	   	   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(ball_Image,y,x,s);//set the (i,j) pixel value
			       break;
	   	   case  3:															//代表白色
	   	   	   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(field_Image,y,x,s);//set the (i,j) pixel value
	   	   	   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(ball_Image,y,x,s);//set the (i,j) pixel value
			       break;
		   	  default:
		   		     //cout<<"                                default   HSV  colour="<<HSV[H][S][V]+0<<endl;
		   		      break;
		   }
	    }
    }
    cvSmooth(field_Image,field_Image,CV_MEDIAN,3);
    cvSmooth(ball_Image,ball_Image,CV_MEDIAN,3);
   // cvMorphologyEx(field_Image,field_Image,temp_image,NULL,CV_MOP_ERODE,1);
   //  cvMorphologyEx(goal_Image,goal_Image,temp_image,NULL,CV_MOP_ERODE,1);

}


void Color_Table_goal(IplImage *src )
{
	CvSize  size=cvGetSize(src);
//	long i;
	CvScalar s;
	int x,y;
	int nstep=src->widthStep;
	int nchannel=src->nChannels;
    for(y=0;y<size.height;y++)
    {
    	for(x=0;x<size.width;x++)
	    {
	    	//得到RGB值
		    B=src->imageData[y*nstep+x*nchannel+0];
		    G=src->imageData[y*nstep+x*nchannel+1];
		    R=src->imageData[y*nstep+x*nchannel+2];

            R=R/4; G=G/4;B=B/4;
		  // 查表

		   switch (RGB[B][G][R])
		   {
		   case  0:														//代表其他颜色
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(goal_Image,y,x,s);//set the (i,j) pixel value
			   break;
            case  1:														//代表绿色
			   s.val[0]=0;
			   s.val[1]=0;
			   s.val[2]=0;
			   cvSet2D(goal_Image,y,x,s);//set the (i,j) pixel value
			   break;
		   case 2:																//代表白色
			   s.val[0]=255;
			   s.val[1]=255;
			   s.val[2]=255;
			   cvSet2D(goal_Image,y,x,s);//set the (i,j) pixel value
		       break;
	   default:
		   		     //cout<<"                                default  RGB   colour="<<RGB[B][G][R]+0<<endl;
		   		      break;
		   }
	    }
    }
    cvSmooth(goal_Image,goal_Image,CV_MEDIAN,3);
    //cvShowImage("goal_image",goal_Image);
}

int Read_ColorTable()
{
	long x,y,z;
	static int flag=0;
	FILE  *fp_HSV;

		  if((fp_HSV = fopen("ColourTable_HSV.txt", "r+")) == NULL){
			  cout<<"can't open the file\n"<<endl;
		  	  return 0;
		  }
		        	 for( x=0;x<64;x++)
		        		  		{
		        				for( y=0;y<64;y++)
		        					{
		        					for( z=0;z<64;z++)
		        					{
		                            rewind(fp_HSV);
//		        					fp_position=ftell(fp_HSV);
//		        					cout<<"重置指针位置"<<fp_position<<endl;
		                            fseek(fp_HSV,4*long(x*64*64+y*64+z),0);
		        						fread(&HSV[x][y][z],sizeof(int), 1,fp_HSV) ;
	       						//cout<<"colour="<<HSV[x][y][z]+0<<endl;
		        					}
		        					}
		        		  		}

				fclose(fp_HSV);

				if(flag==0)
				{
				   FILE  *fp_RGB;

				 if((fp_RGB = fopen("ColourTable_RGB.txt", "r+")) == NULL){
					  cout<<"can't open the file\n"<<endl;
				  	  return 0;
				  }
				        	 for( x=0;x<64;x++)
				        		  		{
				        				for( y=0;y<64;y++)
				        					{
				        					for( z=0;z<64;z++)
				        					{
				                            rewind(fp_RGB);
//				        					fp_position=ftell(fp_RGB);
		//		        					cout<<"重置指针位置"<<fp_position<<endl;
				                            fseek(fp_RGB,4*long(x*64*64+y*64+z),0);
				        						fread(&RGB[x][y][z],sizeof(int), 1,fp_RGB) ;
				        					//	cout<<"colour="<<HSV[x][y][z]+0<<endl;
				        					}
				        					}
				        		  		}

						fclose(fp_RGB);
						flag++;
				}



			//	cvWaitKey();
				return 1;
}
int Read_ColorTable_Near()
{
	long x,y,z;
	FILE  *fp_HSV;
   // int fp_position=3;




//	cout<<"bein"<<endl;

		  if((fp_HSV = fopen("ColourTable_HSV_near.txt", "r+")) == NULL){
			  cout<<"can't open the file\n"<<endl;
		  	  return 0;
		  }
	//	  cout<<"have open"<<endl;
		  		                            rewind(fp_HSV);

		        	 for( x=0;x<64;x++)
		        		  		{
		        				for( y=0;y<64;y++)
		        					{
		        					for( z=0;z<64;z++)
		        					{
		        				//	fp_position=ftell(fp_HSV);
		        			//		cout<<"重置指针位置"<<fp_position<<endl;
		                            fseek(fp_HSV,4*long(x*64*64+y*64+z),0);
		        						fread(&HSV[x][y][z],sizeof(int), 1,fp_HSV) ;
	       				//		cout<<"colour="<<HSV[x][y][z]+0<<endl;
		        					}
		        					}
		        		  		}

				fclose(fp_HSV);
        //   cout<<"read over"<<endl;

	//			cvWaitKey();
				return 1;
}


bool get_ball_pos(CvMat *src){

		//if(get_ball_pos_1(mat) == true){
		   if(get_ball_pos_1(mat) == true){
		    	t_begin = get_time();
                cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(0,0,255),2,8,0);
	//	    	robot_info.calc_ball_info();
		//    	 cout<<robot_info.ball_dis4robot<<endl;
		    	 return true;
		    }
		 else if(get_ball_pos_3(ball_Image_gray)==true)
		 	 {
		    	t_begin = get_time();
		    	 cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(255,0,0),2,8,0);
		  //  	 robot_info.calc_ball_info();
		    //	 cout<<robot_info.ball_dis4robot<<endl;
		    //cout<<"firstfirstfirst"<<endl;
		    	 return true;
		 	 }
		 	 else if(get_ball_pos_3(ball_Image_gray1)==true)
		 	 {
		    	t_begin = get_time();
		    	 cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(255,255,0),2,8,0);
		  //  	 robot_info.calc_ball_info();
		    //	 cout<<robot_info.ball_dis4robot<<endl;
		    //cout<<"secondsecondsecond"<<endl;
		    	 return true;
		 	 }

		 	 else if(get_ball_pos_2(src,1.5,(ball_Image->width/40),ball_Image->width/2,1.0,((float)max_count_best_2/100.0)) == true){
		    	t_begin = get_time();
		    	 cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(0,255,255),2,8,0);
//		    	 cout<<"ball found"<<endl;
		    	 return true;
		 	 }




		else{
		    	t_end = get_time();
//				cout<<"t_gap = "<<t_end - t_begin<<endl;
		    	if((t_end-t_begin)>1){
//		    	cout<<"can't find the ball"<<endl;
		    	 return false;
		    	}
		    	else return true;
		    }
}

bool get_near_ball_pos(CvMat *src){

		      if(get_ball_pos_3(ball_Image_gray)==true)
			 {
				t_begin = get_time();
				cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(255,0,0),2,8,0);
				return true;
			 }
			  if(get_ball_pos_3(ball_Image_gray1)==true)
			 {
				t_begin = get_time();
				cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(255,255,0),2,8,0);
				return true;
			 }
			else if(get_ball_pos_2(src,1.5,(ball_Image->width/40),ball_Image->width/2,1.0,((float)max_count_best_2/100.0)) == true)
			{
				t_begin = get_time();
				 cvCircle(show_Image,cvPoint(robot_info.ball_info.x,robot_info.ball_info.y),robot_info.ball_info.r,cvScalar(0,255,255),2,8,0);
//		    	 cout<<"ball found"<<endl;
				 return true;
			}
		    else{
		    	t_end = get_time();
//				cout<<"t_gap = "<<t_end - t_begin<<endl;
		    	if((t_end-t_begin)>3){
//		    	cout<<"can't find the ball"<<endl;
		    	 return false;
		    	}
		    	else return true;
		    }
}

bool get_ball_pos_1(CvMat *src)
{

	const double area_ratio_limit = 0.60;	//可调
		CvPoint2D32f center;
		float real_radius=0;
		double cont_area;
		double circle_area;
		double area_ratio = 0;
		double max_area_ratio = 0;
		float max_radius = 0;

		cvFindContours(src, stor, &cont, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));
        //cout<<"cont_total"<<cont->total<<endl;
		for (; cont; cont = cont->h_next)
		{

			if(cvMinEnclosingCircle(cont, &center, &real_radius) <= 0){
				continue;
			}
			if(cont->total < 5 || cont->total > 200){
				continue;
			}
	//		cout<<"cont_total="<<cont->total<<endl;
			if(real_radius < 10 || real_radius > 100){
				continue;
			}

			cvDrawContours(edges,cont,cvScalar(255,255,255),cvScalar(255,255,255),0);
			cont_area = cvContourArea(cont);
				circle_area = CV_PI * real_radius * real_radius;
				area_ratio = cont_area/circle_area;

				if(area_ratio < area_ratio_limit){

					continue;
				}
				if(area_ratio < max_area_ratio || real_radius < max_radius){
					continue;

				}

				max_area_ratio = area_ratio;
							max_radius = real_radius;
							   int white_points_in_circle=0;
								int green_points_in_circle = 0;
								int nchannel = ball_Image_gray->nChannels;
								int nstep = ball_Image_gray->widthStep;
							        	for(int l = -real_radius;l  <= real_radius ;l++){
							        		int d = sqrt(real_radius*real_radius - l*l);
							        		for(int d_tmp = -d;d_tmp < d ; d_tmp++){
							        			 int cy_l = center.y+l;
							        			 int cx_d = center.x+d_tmp;
							        			 if(cy_l<1 || cy_l>ball_Image_gray->height-1) continue;
							        			if(cx_d<1 || cx_d>ball_Image_gray->width-1) continue;
							        			if(ball_Image_gray->imageData[cy_l*nstep+cx_d*nchannel]!=0){
							        				white_points_in_circle++;
				//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
							        			}
							        			if(field_Image_gray->imageData[cy_l*nstep+cx_d*nchannel] !=0){
							        				green_points_in_circle++;
				//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
							        			}
							        		}
							        	}
//							        	cout<<"white points in circle="<<white_points_in_circle<<endl;
						        	if((white_points_in_circle>real_radius*real_radius*((float)white_points_low_1/100))&&(white_points_in_circle<real_radius*real_radius*((float)white_points_up_1/100))&&
								(green_points_in_circle>real_radius*real_radius*((float)green_points_low_1/100))&&(green_points_in_circle<real_radius*real_radius*((float)green_points_up_1/100)))
							        	{
											robot_info.ball_info.x = (center.x + 0.5);
											robot_info.ball_info.y =  (center.y + 0.5);
											robot_info.ball_info.r = (real_radius+0.5);
									        //cout<<"r1="<<robot_info.ball_info.r<<endl;
											return true;

							        	}
		}

			return false;




}

bool get_ball_pos_2( CvMat* img, float dp,
                         int min_radius, int max_radius,
                          float acc_threshold,float max_count_best_threshold)
{
	//参数：
	//img: 输入图像
	//dp: 识别精度,1.0表示按照原图精度
	//min_dist: 圆心点位置识别精度
	//min_radius: 所需要找的圆的最小半径
	//max_radius：所需要找的圆的最大半径
	//canny_threshold：canny算子的高阀值
	//acc_threshold：累加器阀值，计数大于改阀值的点即被认为是可能的圆心
	//circles: 保存找到的符合条件的所有圆
	//circles_max: 最多需要的找到的圆的个数
    const int SHIFT = 10, ONE = 1 << SHIFT;
    cv::Ptr<CvMat> dx, dy;
 //   cv::Ptr<CvMat> edges, edge_initial,accum, dist_buf;
    cv::Ptr<CvMat> accum, dist_buf;
    std::vector<int> sort_buf;
    cv::Ptr<CvMemStorage> storage;

    int x, y, i, j, k, center_count, nz_count;
    float min_radius2 = (float)min_radius*min_radius;
    float max_radius2 = (float)max_radius*max_radius;
    int rows, cols, arows, acols;
    int astep, *adata;
    float* ddata;

    CvSeq *nz, *centers;
    float idp, dr;
    CvSeqReader reader;




    cvFindContours(img, stor, &cont, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

    for (; cont; cont = cont->h_next)
    {
    	if(cont->total<50){
    		continue;
    	}
    	cvDrawContours(edges,cont,CV_RGB(255,255,255),CV_RGB(255,255,255),0);
    }

    //sobel算子求水平和垂直方向的边缘，用于计算边缘点的法线方向
    dx = cvCreateMat( img->rows, img->cols, CV_16SC1 );
    dy = cvCreateMat( img->rows, img->cols, CV_16SC1 );
    cvSobel( img, dx, 1, 0, 3 );
    cvSobel( img, dy, 0, 1, 3 );

    //dp表示识别精度
    if( dp < 1.f )
        dp = 1.f;
    idp = 1.f/dp;
    //accum用作累加器，包含图像中每一个点的计数。图像中每一个点都有一个计数，点的计数表示每一个canny边缘点法线方向上，
    //到该点距离为R的边缘点的个数，初始化为0
    accum = cvCreateMat( cvCeil(img->rows*idp)+2, cvCeil(img->cols*idp)+2, CV_32SC1 );
    cvZero(accum);

    storage = cvCreateMemStorage();
    nz = cvCreateSeq( CV_32SC2, sizeof(CvSeq), sizeof(CvPoint), storage );
    //centers用于保存可能的圆心点
    centers = cvCreateSeq( CV_32SC1, sizeof(CvSeq), sizeof(int), storage );

    rows = img->rows;
    cols = img->cols;
    arows = accum->rows - 2;
    acols = accum->cols - 2;
    adata = accum->data.i;
    astep = accum->step/sizeof(adata[0]);
    // Accumulate circle evidence for each edge pixel
    //以下这个循环用于获取所有可能的圆边缘点，存储在nz中，同时设置
    //累加器中的值
    for( y = 0; y < rows; y++ )
    {
        const uchar* edges_row = edges->data.ptr + y*edges->step;
        const short* dx_row = (const short*)(dx->data.ptr + y*dx->step);
        const short* dy_row = (const short*)(dy->data.ptr + y*dy->step);

        for( x = 0; x < cols; x++ )
        {
            float vx, vy;
            int sx, sy, x0, y0, x1, y1, r;
            CvPoint pt;
     //vx,vy均为梯度
            vx = dx_row[x];
            vy = dy_row[x];

            if( !edges_row[x] || (vx == 0 && vy == 0) )
                continue;

            float mag = sqrt(vx*vx+vy*vy);
            assert( mag >= 1 );
            //sx表示cos, sy表示sin
            sx = cvRound((vx*idp)*ONE/mag);
            sy = cvRound((vy*idp)*ONE/mag);

            x0 = cvRound((x*idp)*ONE);
            y0 = cvRound((y*idp)*ONE);
            // Step from min_radius to max_radius in both directions of the gradient
            //循环两次表示需要计算两个方向，法线方向和法线的反方向
            for(int k1 = 0; k1 < 2; k1++ )
            {
                //半径方向的水平增量和垂直增量
                x1 = x0 + min_radius * sx;
                y1 = y0 + min_radius * sy;
                //在法线方向和反方向上，距离边缘点的距离为输入的最大半径和最小半径范围内找点
                //每找到一个点，该点的累加器计数就加1
                for( r = min_radius; r <= max_radius; x1 += sx, y1 += sy, r++ )
                {
                    int x2 = x1 >> SHIFT, y2 = y1 >> SHIFT;
                    if( (unsigned)x2 >= (unsigned)acols || (unsigned)y2 >= (unsigned)arows)
                        break;
                    adata[y2*astep + x2]++;
                }
                //反方向
                sx = -sx; sy = -sy;
            }
            //保存可能的圆边缘点
            pt.x = x; pt.y = y;
            cvSeqPush( nz, &pt );
        }
    }

    nz_count = nz->total;
//    cout<<"边缘点个数"<<nz_count<<endl;
    if( !nz_count )
    {
//    	cout<<"problem 1"<<endl;
        return false;
    }
        //Find possible circle centers
    //累加器中，计数大于阀值的点，被认为可能的圆心点。因为计算各点计数过程中，距离有误差，所以
   //在与阀值比较时，该点计数先要与4邻域内的各个点的计数比较，最大者才能和阀值比较。可能的圆心
   //点保存在centers中
    for( y = 1; y < arows - 1; y++ )
    {
        for( x = 1; x < acols - 1; x++ )
        {
            int base = y*(acols+2) + x;
            if( adata[base] > acc_threshold &&
                adata[base] > adata[base-1] && adata[base] > adata[base+1] &&
                adata[base] > adata[base-acols-2] && adata[base] > adata[base+acols+2])
//				&&adata[base] > adata[base-acols]&&adata[base] > adata[base-acols-2]&&
//				adata[base] > adata[base+acols+1]&&adata[base] > adata[base+acols])									//扫四临域,不扫八临域
              {
//			    cout<<"满足条件的Center的累加器"<<adata[base]<<endl;
            	if (adata[base]> 10)
                cvSeqPush(centers, &base);
              }
        }
    }
    center_count = centers->total;
//    cout<< "满足条件的center共有"<<center_count<<endl;
    if( !center_count||center_count>2000 )
    {
//    	cout<<"problem 2"<<endl;
        return false;
    }
    sort_buf.resize( MAX(center_count,nz_count) );
    //链表结构的certers转化成连续存储结构sort_buf
    cvCvtSeqToArray( centers, &sort_buf[0] );
    //经过icvHoughSortDescent32s函数后，以sort_buf中元素作为adata数组下标,
   //adata中的元素降序排列, 即adata[sort_buf[0]]是adata所有元素中最大的,
   //adata[sort_buf[center_count-1]]是所有元素中最小的
//    icvHoughSortDescent32s( &sort_buf[0], center_count, adata );
    cvClearSeq( centers );
    //经过排序后的元素，重新以链表形式存储到centers中
    cvSeqPushMulti( centers, &sort_buf[0], center_count );

    dist_buf = cvCreateMat( 1, nz_count, CV_32FC1 );
    ddata = dist_buf->data.fl;

    dr = dp;
    // For each found possible center
    // Estimate radius and check support
    //对于每一个可能的圆心点，计算所有边缘点到该圆心点的距离。由于centers中的
   //元素已经经过前面排序，所以累加器计数最大的可能圆心点最先进行下面的操作




//    if(center_count>50){												//扫的圆心点数不能太少,特别是距离球比较远时
  //  	center_count = 50;
 //  }

    int max_count_best = 0;
    for(i=0;i< center_count;i++)

    {
        if(center_count>500) i=i+1;														//如果圆心点数太多,运算量就会很大,就让它隔几个点找一次.
        if(center_count>1000) i=i+1;
        int ofs = *(int*)cvGetSeqElem( centers, i );

        y = ofs/(acols+2);
        x = ofs - (y)*(acols+2);

        //Calculate circle's center in pixels
        float cx = (float)((x + 0.5f)*dp), cy = (float)(( y + 0.5f )*dp);

        float start_dist, dist_sum;
        float r_best = 0;
        int max_count = 0;
        // Check distance with previously detected circles
        //如果该可能的圆心点和已经确认的圆心点的距离小于阀值，则表示
       //这个圆心点和已经确认的圆心点是同一个点


        // Estimate best radius
        cvStartReadSeq( nz, &reader );

        //求所有边缘点到当前圆心点的距离，符合条件的距离值保存在ddata中
        for( j = k = 0; j < nz_count; j++ )
        {
            CvPoint pt;
            float _dx, _dy, _r2;
            CV_READ_SEQ_ELEM( pt, reader );
            _dx = cx - pt.x; _dy = cy - pt.y;
            _r2 = _dx*_dx + _dy*_dy;
            if(min_radius2 <= _r2 && _r2 <= max_radius2 )
            {
                ddata[k] = _r2;
                sort_buf[k] = k;
                k++;
            }
        }

        int nz_count1 = k, start_idx = nz_count1 - 1;
        if( nz_count1 == 0 )
            continue;
        dist_buf->cols = nz_count1;
        cvPow( dist_buf, dist_buf, 0.5 );
        //经过如下处理后，以sort_buf中元素作为ddata数组下标,ddata中的元素降序排列,
       //即ddata[sort_buf[0]]是ddata所有元素中最大的, ddata[sort_buf[nz_count1-1]]
       //是所有元素中最小的
//        icvHoughSortDescent32s( &sort_buf[0], nz_count1, (int*)ddata );
        //对所有的距离值做处理，求出最可能圆半径值，max_count为到圆心的距离为最可能半径值的点的个数
        dist_sum = start_dist = ddata[sort_buf[nz_count1-1]];

        for( j = nz_count1 - 2; j >= 0; j-- )
        {
            float d = ddata[sort_buf[j]];

            if( d > max_radius )
                break;

            if( d - start_dist > dr )
            {
                float r_cur = ddata[sort_buf[(j + start_idx)/2]];
                if( (start_idx - j)*r_best >= max_count*r_cur ||
                    (r_best < FLT_EPSILON && start_idx - j >= max_count) )
                {
                    r_best = r_cur;
                    max_count = start_idx - j;
                }
                start_dist = d;
                start_idx = j;
                dist_sum = 0;
            }
            dist_sum += d;
        }
        // Check if the circle has enough support
        //max_count大于阀值，表示这几个边缘点构成一个圆
        int white_points_in_circle=0;
        int green_points_in_circle = 0;
        int green_circle_in_circle = 0;
    	int nchannel = ball_Image_gray->nChannels;
    	int nstep = ball_Image_gray->widthStep;

        if( max_count > int(acc_threshold*r_best) )
        {
//        	CvScalar s;
        	for(int l = -r_best;l  <= r_best ;l++){
        		int d = sqrt(r_best*r_best - l*l);
        		for(int d_tmp = -d;d_tmp <= d ; d_tmp++){
        			 //       			s.val[2] = 255;
        			//					s.val[1] = 0;
        			//					s.val[0] = 0;
        								 int cy_l = cy+l;
        								 int cx_d = cx+d_tmp;
        								 if(cy_l<3 || cy_l>ball_Image_gray->height-3) continue;
        								 if(cx_d<3 || cx_d>ball_Image_gray->width-3) continue;
        			        			if((d_tmp == -d)||(d_tmp == d)){
        			//        				if(HSVImagegray->imageData[cy_l*nstep+cx_d*nchannel] ==0)	green_circle_in_circle++;
        			        				if(field_Image_gray->imageData[cy_l*nstep+(cx_d-2)*nchannel] !=0) green_circle_in_circle++;
        			        				if(field_Image_gray->imageData[cy_l*nstep+(cx_d+2)*nchannel] !=0) green_circle_in_circle++;
        			        			}

        			        			if(cy_l<1 || cy_l>ball_Image_gray->height-1) continue;
        			        			if(cx_d<1 || cx_d>ball_Image_gray->width-1) continue;
        			        			if(ball_Image_gray->imageData[cy_l*nstep+cx_d*nchannel]!=0){
        			        				white_points_in_circle++;
        			 //       				cvSet2D(ball_Image,cy_l,cx_d,s);
        			        			}
        			//        			s.val[2] = 0;
        			 //       			s.val[1] = 255;
        			//					s.val[0] = 0;
        			        			if(field_Image_gray->imageData[cy_l*nstep+cx_d*nchannel] !=0){
        			        				green_points_in_circle++;
        			//        				cvSet2D(ball_Image,cy_l,cx_d,s);
        			        			}
        			        		}
        	}
	 		 if((white_points_in_circle>r_best*r_best*((float)white_points_low_2/100))&&(white_points_in_circle<r_best*r_best*((float)white_points_up_2/100))&&
	 				 (green_points_in_circle>r_best*r_best*((float)green_points_low_2/100))&&(green_points_in_circle<r_best*r_best*((float)green_points_up_2/100))&&
					 (green_circle_in_circle > r_best*CV_PI*2*((float)green_circle_low_2/100))&&(green_circle_in_circle < r_best*CV_PI*2*((float)green_circle_up_2/100)))
	 		 {
//	 			cout<<"max_count/r_best="<<(float)max_count/r_best<<endl;
	 			if(max_count > max_count_best){
	 				robot_info.ball_info.x = cx;
	 				robot_info.ball_info.y = cy;
	 				robot_info.ball_info.r = r_best;
	 				max_count_best = max_count;
//					cout<<"i="<<i<<endl;
//				  cout<<"r_best="<<r_best<<endl;
//					cout<<"max_count_best/r_best="<<(float)max_count_best/r_best<<endl;
//				  cout<<"white_points_in_circle_ratio="<<((float)white_points_in_circle)/(r_best*r_best)<<endl;
//				  cout<<"green_points_in_circle_ratio="<<((float)green_points_in_circle)/(r_best*r_best)<<endl;
//				  cout<<"green_circle_in_circle="<<((float)green_circle_in_circle)/(r_best*CV_PI*2)<<endl;

				  }
	 		 }
	 		 else continue;
            	}
        else continue;

    }
    if(max_count_best > max_count_best_threshold*(robot_info.ball_info.r))	return true;
    else	 return false;
}


bool get_ball_pos_3(IplImage *src)
{
	const double area_ratio_limit = 0.60;	//可调
		CvPoint2D32f center;
		float real_radius=0;
		double cont_area;
		double circle_area;
		double area_ratio = 0;
		double max_area_ratio = 0;
		float max_radius = 0;

		cvFindContours(src, stor, &cont, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0, 0));

		for (; cont; cont = cont->h_next)
		{

			if(cvMinEnclosingCircle(cont, &center, &real_radius) <= 0){
				continue;
			}
			if(cont->total < 10 || cont->total > 370){
				continue;
			}
	//		cout<<"cont_total="<<cont->total<<endl;
			if(real_radius < 10 || real_radius > 100){
				continue;
			}

			cvDrawContours(edges,cont,cvScalar(255,255,255),cvScalar(255,255,255),0);
			cont_area = cvContourArea(cont);
				circle_area = CV_PI * real_radius * real_radius;
				area_ratio = cont_area/circle_area;

				if(area_ratio < area_ratio_limit){

					continue;
				}
				if(area_ratio < max_area_ratio || real_radius < max_radius){
					continue;

				}

				max_area_ratio = area_ratio;
							max_radius = real_radius;
							   int white_points_in_circle=0;
								int green_points_in_circle = 0;
								int nchannel = ball_Image_gray->nChannels;
								int nstep = ball_Image_gray->widthStep;
							        	for(int l = -real_radius;l  <= real_radius ;l++){
							        		int d = sqrt(real_radius*real_radius - l*l);
							        		for(int d_tmp = -d;d_tmp < d ; d_tmp++){
							        			 int cy_l = center.y+l;
							        			 int cx_d = center.x+d_tmp;
					       			 if(cy_l<1 || cy_l>ball_Image_gray->height-1) continue;
							        			if(cx_d<1 || cx_d>ball_Image_gray->width-1) continue;
							        			if(ball_Image_gray->imageData[cy_l*nstep+cx_d*nchannel]!=0){
							        				white_points_in_circle++;
				//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
							        			}
							        			if(field_Image_gray->imageData[cy_l*nstep+cx_d*nchannel] !=0){
							        				green_points_in_circle++;
				//			        				cvSet2D(ball_Image,cy_l,cx_d,s);
							        			}
							        		}
							        	}
//							        	cout<<"white points in circle="<<white_points_in_circle<<endl;
						        	if((white_points_in_circle>real_radius*real_radius*((float)white_points_low_3/100))&&(white_points_in_circle<real_radius*real_radius*((float)white_points_up_3/100))&&
								(green_points_in_circle>real_radius*real_radius*((float)green_points_low_3/100))&&(green_points_in_circle<real_radius*real_radius*((float)green_points_up_3/100)))
							        	{
											robot_info.ball_info.x = (center.x + 0.5);
											robot_info.ball_info.y =  (center.y + 0.5);
											robot_info.ball_info.r = (real_radius+0.5);
											//cout<<"r3="<<robot_info.ball_info.r<<endl;
											return true;

							        	}
		}

			return false;
}


bool Goal_find(int line_range,int white_ratio,int green_ratio,int threshold_houghline,int param1,int param2)
//line_range 允许处理的最多直线，否则直接退出程序
// white_ratio,  green_ratio    两个比率作为两个判断条件，来判断是不是门柱
//threshold阈值参数。如果相应的累计值大于 threshold， 则函数返回的这个线段
//param1对概率 Hough 变换，它是最小线段长度.
//param2这个参数表示在同一条直线上进行碎线段连接的最大间隔值(gap), 即当同一条直线上的两条碎线段之间的间隔小于param2时，将其合二为一
{

			CvPoint  *line,*line_vertical;
			int i=0,j=0,possible_goal_point[10][3]={0},goal_point[10][2]={0};    //possible_goal_point[j][0]为x,[1]为highpoint--0,[2]为lowpoint---240
			float degree=70;
			//cvCanny( goal_Image_gray, goal_canny_Image, 100, 200, 3 );//边缘检测
			cvCanny( goal_Image_gray, goal_canny_Image, 100, 200, 3 );//边缘检测
			cvClearSeq(lines);
			lines = cvHoughLines2( goal_canny_Image, stor, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, threshold_houghline, param1, param2 ); //直接得到直线序列
//			cout<<"lines="<<2*lines ->total<<endl;
			if(lines->total>100)
				{
//				cout<<"problem1:  too much lines"<<endl;
				robot_info.goal_info.state=0;
				return false ;
				}
		//	循环直线序列，将所有直线标记出来
			for( int i = 0; i < lines ->total; i++ )  //lines存储的是直线
			{
						line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
						cvLine( goal_color_dst, line[0], line[1], CV_RGB( 255, 255, 255 ) );  //	color_dst是三通道图像用来存直线图像将找到的直线标记为白色
			}
		//找球门，对每条直线进行操作,存入门柱线至cvSeqPush
			cvClearSeq(lines_vertical);
			for(  i = 0; i < lines ->total; i++ )  //lines存储的是直线
			{
							line = ( CvPoint* )cvGetSeqElem( lines, i );  //lines序列里面存储的是像素点坐标
							if(fabs(atan2(double(line[1].y-line[0].y),double(line[1].x-line[0].x)))>CV_PI*0.5/90*degree)
							 {
											cvLine( goal_color_dst, line[0], line[1], CV_RGB( 255, 0, 0 ) );
											cvSeqPush(lines_vertical,&line[0]);
											cvSeqPush(lines_vertical,&line[1]);
								}
			}
//			cout<<"lines_vertical->total="<<lines_vertical->total<<endl;
			if(lines_vertical->total==0)
			{
//						cout<<"problem2: no possible goal"<<endl;
				robot_info.goal_info.state=0;
						return false;
			}
			int point_new_x,point_new_y,z,flag=0;
			j=0;
			//聚类,找可能是门柱的点的横坐标
			line_vertical = ( CvPoint* )cvGetSeqElem( lines, 0 );
			possible_goal_point[0][0]=line_vertical->x;
			possible_goal_point[0][1]=line_vertical->y;

			for(i = 0; i < lines_vertical ->total; i++ )
			{
						flag=0;
						line_vertical = ( CvPoint* )cvGetSeqElem( lines_vertical, i );
//						cout<<"              "<<endl;
//						cout<<"line_vertical.x="<<line->x<<"       line_vertical.y="<<line->y<<endl;
						point_new_x=line_vertical->x;
						point_new_y=line_vertical->y;
						z=0;
						for(z=0;z<j+1;z++)
						{
									if(abs(int(point_new_x-possible_goal_point[z][0]))<line_range)
									{
													possible_goal_point[z][0]=(possible_goal_point[z][0]+point_new_x)/2;
													if(point_new_y<possible_goal_point[z][1])
													{
														possible_goal_point[z][1]=point_new_y;
													}
													if(point_new_y>possible_goal_point[z][2])
													{
														possible_goal_point[z][2]=point_new_y;
													}
													flag=1;
													break;
									}
						}

						if(flag==0)
						{
										j=j+1;
//										cout<<"j is changed"<<endl;
										if(j>=10)
											{
//											cout<<"problem3:  not a good view, jump this frame"<<endl;
											robot_info.goal_info.state=0;
											return false;
											}
										possible_goal_point[j][0]=point_new_x;
										possible_goal_point[j][1]=point_new_y;
										possible_goal_point[j][2]=point_new_y;
						}
			}

			//去除伪门柱
			int d_start,d_end,h_start,h_end,x,y,goal_account_white_point,goal_account_green_point,k=0;
			int nchannel = goal_Image_gray->nChannels;
			int nstep = goal_Image_gray->widthStep;

			for(i=0;i<j+1;i++)
			{
//判断级1：检测矩形区域，累加阈值到goal_account，即图像中的白色点数
						goal_account_white_point=0;
						goal_account_green_point=0;
						if(possible_goal_point[i][0]-5<0)
									d_start=0;
						else
									d_start=possible_goal_point[i][0]-5;
						if(possible_goal_point[i][0]+5>320)
									d_end=320;
						else
									d_end=possible_goal_point[i][0]+5;
						h_start=possible_goal_point[i][1];
						h_end=possible_goal_point[i][2];

//						cout<<"d_start="<<d_start<<"        d_end="<<d_end<<"    h_start="<<h_start<<"h_end="<<h_end<<endl;

						for(y=h_start;y<h_end;y++)
						{
									for(x=d_start;x<d_end;x++)
									{
											if(goal_Image_gray->imageData[y*nstep+x*nchannel]!=0)
											{
												goal_account_white_point++;
											}
									}
						}

						if (fabs(goal_account_white_point*1.0/((d_end-d_start)*(h_end-h_start)))>white_ratio)
						{
//判断级２：检测门柱的下部是否与绿色区域相连
							goal_account_green_point=0;
							if(possible_goal_point[i][0]-5<0)
										d_start=0;
							else
										d_start=possible_goal_point[i][0]-5;
							if(possible_goal_point[i][0]+5>320)
										d_end=320;
							else
										d_end=possible_goal_point[i][0]+5;
							h_start=possible_goal_point[i][2];
							h_end=possible_goal_point[i][2]+20;
							if(h_end>240) 							h_end=240;
							for(y=h_start;y<h_end;y++)
							{
										for(x=d_start;x<d_end;x++)
										{
												if((field_Image_gray->imageData[y*nstep+x*nchannel]!=0))
												{
													goal_account_green_point++;

												}
										}
							}

							if (fabs(goal_account_green_point*1.0/((d_end-d_start)*(h_end-h_start)))>green_ratio)
							//输出最终的矩形
							{
							goal_point[k][0]=possible_goal_point[i][0];
							goal_point[k][1]=possible_goal_point[i][2];
							cvCircle(show_Image,cvPoint(goal_point[k][0],goal_point[k][1]),5,CV_RGB(50,150,255),8);

							k++;
							if (k>2)
								{
//								cout<<"problem4: too much goal found"<<endl;
								robot_info.goal_info.state=0;
								return false;
								}
							}
						}
			}
//判断左右门柱
			if(k-1==1)
			{
				headangle=head_angle[0];
				robot_info.goal_info.state=2;
				if(goal_point[0][0]<goal_point[1][0])
				{
					robot_info.goal_info.left_goal=goal_point[0][0];
						robot_info.goal_info.right_goal=goal_point[1][0];
//						cout<<"left_goal is"<<left_goal.x<<"   "<<left_goal.y<<"right_goal is"<<right_goal.x<<"   "<<right_goal.y<<endl;
				}
				else
				{
					robot_info.goal_info.left_goal=goal_point[1][0];
						robot_info.goal_info.right_goal=goal_point[0][0];
//						cout<<"left_goal is"<<left_goal.x<<"   "<<left_goal.y<<"right_goal is"<<right_goal.x<<"   "<<right_goal.y<<endl;
				}
			}
			else
			{
				robot_info.goal_info.left_goal=goal_point[0][0];
				robot_info.goal_info.state=1;
//				cout<<"one goal is found"<<endl;
			}

			return true;
}


void print(IplImage* img, int x, int y, const char* text){
	    CvFont font;
	    double hscale = 2.0;
	    double vscale = 2.0;
	    int linewidth = 2;
	    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX,hscale,vscale,0,linewidth);
	    CvScalar textColor =cvScalar(0,255,255);
	    CvPoint textPos =cvPoint(x, y);
	    cvPutText(img, text, textPos, &font,textColor);
}


//发送数据， 成功则返回未发送完的字节数， 失败则返回-1
int write_sock(int sock_fd, char *buf, int length)
{
	int byte_send;
	if((byte_send = write(sock_fd, buf, length)) == -1){
		std::cout<<"error in write sock."<<std::endl;
		return -1;
	}
	else{
		return length - byte_send;
	}
}

//发送一幅图像
bool image_send_one_frame(IplImage *img)
{
	int length = img->imageSize;
	int bytes_remain;
	char *img_data = img->imageData;
	while((bytes_remain = write_sock(client_sock, img_data, length)) != 0)
	{
		if(bytes_remain == -1)
			return false;
		img_data += length - bytes_remain;
		length = bytes_remain;
	}
	return true;
}

bool image_update()
{
	image_send_one_frame(show_Image);
//	cvShowImage("simple-demo", show_Image);
	char ch = cvWaitKey(1);
	if(ch == ' '){
		return false;
	}
	return true;
}

