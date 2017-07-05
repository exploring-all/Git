#include<stdio.h>
#include<conio.h>
#include<opencv2/opencv.hpp>
#include <iostream> 
#include <Kinect.h>
#include<cmath>

#define BODY_COUNT 6

#define string_dst0 "E://数据输出//0//"
#define string_dst1 "E://数据输出//1//"
#define string_dst2 "E://数据输出//2//"
#define string_dst3 "E://数据输出//3//"
#define string_dst4 "E://数据输出//4//"
#define string_dst5 "E://数据输出//5//"

using namespace cv;
using namespace std;

void  func_0();
void  func_1();
void  func_2();
void  func_3();
void  func_4();
void  func_5();



char filename_0[100];
char filename_1[100];
char filename_2[100];
char filename_3[100];
char filename_4[100];
char filename_5[100];
char filename_6[100];
char filename_7[100];
char filename_8[100];
char filename_9[100];
char filename1_0[100];
char filename1_1[100];
char filename1_2[100];
char filename1_3[100];
char filename1_4[100];
char filename1_5[100];
char filename1_6[100];
char filename1_7[100];
char filename1_8[100];
char filename1_9[100];
int order =0;

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

int main()
{
	//int xuhao=10;
	
	BOOL OK_0=0,OK_1=0,OK_2=0,OK_3=0,OK_4=0,OK_5=0;
	int count0,count1,count2,count3,count4,count5;
	//int qqq;
	HRESULT hResult; 
	setUseOptimized(true);

	//sensor
	IKinectSensor *pSensor;
	GetDefaultKinectSensor(&pSensor);
	pSensor->Open();

	//source
	IBodyIndexFrameSource *pBodyIndexSource;
	pSensor->get_BodyIndexFrameSource(&pBodyIndexSource);
	IColorFrameSource*pColorSource;
	pSensor->get_ColorFrameSource(&pColorSource);
	IDepthFrameSource*pDepthSource;
	pSensor->get_DepthFrameSource(&pDepthSource);
	IBodyFrameSource* pBodySource;
	pSensor->get_BodyFrameSource( &pBodySource );


	//reader
	IBodyIndexFrameReader *pBodyIndexReader;
	pBodyIndexSource->OpenReader(&pBodyIndexReader);
	IColorFrameReader*pColorReader;
	pColorSource->OpenReader(&pColorReader);
	IDepthFrameReader*pDepthReader;
	pDepthSource->OpenReader(&pDepthReader);
	IBodyFrameReader* pBodyReader;
	pBodySource->OpenReader( &pBodyReader );


	//CoordinateMapper
	ICoordinateMapper* pCoordinateMapper;
	pSensor->get_CoordinateMapper(&pCoordinateMapper);

	//Description
	IFrameDescription* pDepthDescription;
	pDepthSource->get_FrameDescription(&pDepthDescription);
	int depthWidth = 0;
	int depthHeight = 0;
	pDepthDescription->get_Width(&depthWidth);  //512
	pDepthDescription->get_Height(&depthHeight);  //424

	IFrameDescription* pColorDescription;
	pColorSource->get_FrameDescription(&pColorDescription);
	int colorWidth = 0;
	int colorHeight = 0;
	pColorDescription->get_Width(&colorWidth);//1920
	pColorDescription->get_Height(&colorHeight);//1080

	vector<DepthSpacePoint>depthspacepoints(colorWidth*colorHeight);  //vector定义了DepthSpacePoint数据类型的数组,名字是depthspacepoints，小括号里面是参数
	Mat img(colorHeight, colorWidth, CV_8UC4);  //彩色图存储矩阵容器
	Mat bodyMat( colorHeight / 2, colorWidth / 2, CV_8UC4 );	
	int a,b,c,d,e,f,g,h,q,m,n;

	//============================================================//
	char chari[10000];
	char chari1[10000];
	
	int num = 0,num1=0,num2=0,num3=0,num4=0,num5=0,num6=0,num7=0,num8=0,num9=0;
	string  dst_xml ;
	string dst_xml_no;
	//==========================================================//

	while (1)
	{  

		//int key=waitKey(1);

		IBodyIndexFrame* pBodyIndexFrame = nullptr;
		unsigned int nBodyIndexBufferSize = 0;
		unsigned char *pBodyIndexBuffer = NULL;

		hResult = pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame);//Frame是真正存储数据的类，每一次都让Reader读到的数据传到Frame中
		if (SUCCEEDED(hResult))
		{
			hResult = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);  //主要就是把Frame中的数据转存到一个数组中  pBodyIndexBuffer就是一个424*512大小的16位unsigned int数组，用来存储深度数据
		}

		if (SUCCEEDED(hResult))
		{
			IDepthFrame* pDepthFrame = nullptr;
			UINT nDepthBufferSize = 0;
			UINT16 *pDepthBuffer = NULL;

			hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);//获取到最近的深度帧
			if (SUCCEEDED(hResult))
			{
				hResult = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);  //得到一个深度帧数据的点
			}

			if (SUCCEEDED(hResult))
			{
				IColorFrame  *pColorFrame = nullptr;
				UINT nColorBufferSize = 0;
				RGBQUAD *pColorBuffer = NULL;    //RGBQUAD是一个结构体，其保存一个像素点的RGB值         
				hResult = pColorReader->AcquireLatestFrame(&pColorFrame);  //获取到最后的彩色帧
				if (SUCCEEDED(hResult))
				{
					hResult = pColorFrame->CopyConvertedFrameDataToArray(colorHeight*colorWidth * 4, reinterpret_cast<BYTE*>(img.data), ColorImageFormat::ColorImageFormat_Bgra);  //彩色帧数据转变成要求的格式。
					if (SUCCEEDED(hResult))
					{
						hResult = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));  //得到颜色帧数据的点
					}
				}

				//--------------骨骼信息向彩色图映射start--------------------------
				IBodyFrame* pBodyFrame = nullptr;
				hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
				if( SUCCEEDED( hResult ) )
				{
					IBody* pBody[BODY_COUNT] = { 0 }; //默认的是 6 个骨骼 ，初始化所有的骨骼信息
					hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );//更新骨骼数据，
					if( SUCCEEDED( hResult ) ){
						for( int count = 0; count < BODY_COUNT; count++ ){  //count数从0到5，6个人找骨骼来显示
							BOOLEAN bTracked = false;    //初始化“能追踪到人体”的值为否。
							hResult = pBody[count]->get_IsTracked( &bTracked );  //确认能着追踪到人体。
							if( SUCCEEDED( hResult ) && bTracked ){
								Joint joint[JointType::JointType_Count];   //取得人体Joint(关节)。JointType是一个枚举类型，不同位置的关节点都是不同的标号表示的。count是一个数值25。
								hResult = pBody[ count ]->GetJoints( JointType::JointType_Count, joint );  //取得人体Joint(关节)。

								if( SUCCEEDED( hResult ) ){
									CvPoint skeletonPoint[BODY_COUNT][JointType_Count] = { cvPoint(0,0) };							
									// Joint
									for( int type = 0; type < JointType::JointType_Count; type++ ){  //遍历25个关节点
										ColorSpacePoint colorSpacePoint = { 0 };   //ColorSpacePoint结构变量，后一个colorSpacePoint实例化对象。 Represents a 2D point in color space, expressed in pixels.代表了一种2d点在颜色空间中,用像素表示。
										pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );//摄像头点空间到颜色空间的转换，把关节点的坐标转换到颜色空间上，便于显示
									}
								}				
							}
						}

						//--------------骨骼信息向彩色图映射end-------------------------
						if (SUCCEEDED(hResult))
						{
							pCoordinateMapper->MapColorFrameToDepthSpace(depthWidth * depthHeight, (UINT16*)pDepthBuffer, colorWidth * colorHeight, &depthspacepoints[0]);  //从颜色空间到深度空间的坐标映射，叫做变换也好  depthspacepoints[0]是不是数组的首地址？应该是
#pragma omp parallel for  //openmp关于一个预处理的东西
							for (int i = 0; i < colorHeight; i++)
								for (int j = 0; j < colorWidth; j++)
								{
									int k = i*colorWidth + j;//循环遍历每一个像素
									DepthSpacePoint p = depthspacepoints[k];  //用匹配的深度图像代表每一个像素。  DepthSpacePoint是一个结构体 p是结构体类型的变量

									int depthX = (int)(p.X + 0.5f); //用这个结构体类型变量p调用结构体中的成员变量，若把p换位k不行,因为k定义为是一个整形的变量，所以上一句的功能就是把获取到的每一像素都传给结构体类型的变量
									int depthY = (int)(p.Y + 0.5f);

									if ((depthX >= 0 && depthX < depthWidth) && (depthY >= 0 && depthY < depthHeight))
									{
										BYTE player = pBodyIndexBuffer[depthX + (depthY * depthWidth)];  //BYTE即unsigned char,把获取到的每一个像素都交给player
										if (player == 0xff)  //深度空间if句判断每一个像素是不是属于人体，0~5分别表示可识别的6个人，0xff表示该像素不属于人（或说可能属于第7个人，但kinect识别不了，所以就不把该像素当做人体的了）
										{ 
											img.at <Vec4b>(i,j) = Vec4b(255,255,255,255); //彩色空间处理：0，0，0表示黑色，255表示alpha透明度,数字越小越透明
										}//这里并没写一个判断找到人体怎么办，默认就是不进行处理，那么就是直接显示咯
									}
									else  //这个else是和上上个if匹配的，表示如果在搜索空间（搜索人体）之外的地方也需要置为黑色
										img.at <Vec4b>(i, j) = Vec4b(255,255,255,255);
								}//彩色空间遍历双层循环在这结束
						}  // pCoordinateMapper

						cv::resize( img, bodyMat, cv::Size(), 0.5, 0.5 );
						imshow("BODY", bodyMat);
						
						
						//=====================骨骼坐标获取start=============================================

						pBodyFrame->GetAndRefreshBodyData(_countof(pBody), pBody); 
						for (int i = 0; i < BODY_COUNT; ++i)  
						{
							IBody* pBody1 = pBody[i];
							if (pBody1)   //索引到的人的序号，在这里就是判断是不是空指针。
							{
								BOOLEAN bTracked = false;   //是否已被跟踪
								hResult = pBody1->get_IsTracked(&bTracked);

								if (SUCCEEDED(hResult) && bTracked)  //如果索引到新的人体并且已能被跟踪到
								{

									cout<<"索引号： "<<i<<endl;
//================保存出现的索引标号start=============================
									switch (i)
									{
									case 0:{count0 = 0;break;}
									case 1:{count1 = 1;break;}
									case 2:{count2 = 2;break;}
									case 3:{count3 = 3;break;}
									case 4:{count4 = 4;break;}
									case 5:{count5 = 5;break;}
									default: break;
									}
//===============保存出现的索引标号end================================

									Joint joints[JointType_Count];//存储关节点类 Joint是一个结构体变量  JointType_Count=25

									//存储深度坐标系中的关节点位置
									//DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[_countof(joints)];
									ColorSpacePoint *colorSpacePosition = new ColorSpacePoint[_countof(joints)];

									//获得关节点类
									hResult = pBody1->GetJoints(_countof(joints), joints);
									if (SUCCEEDED(hResult))
									{
										for (int j = 0; j < _countof(joints); ++j)
										{
											//将关节点坐标从摄像机坐标系（-1~1）转到深度坐标系（424*512）
											//	pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthSpacePosition[j]);
											pCoordinateMapper->MapCameraPointToColorSpace(joints[j].Position, &colorSpacePosition[j]);
										}   

										int	T0=joints[0].TrackingState;
										int T1=joints[1].TrackingState;
										int T2=joints[2].TrackingState;
										int	T3=joints[3].TrackingState;
										int T4=joints[4].TrackingState;
										int T5=joints[5].TrackingState;
										int	T6=joints[6].TrackingState;
										int T7=joints[7].TrackingState;
										int T8=joints[8].TrackingState;
										int	T9=joints[9].TrackingState;
										int T10=joints[10].TrackingState;
										int T11=joints[11].TrackingState;
										int	T12=joints[12].TrackingState;
										int T13=joints[13].TrackingState;
										int T14=joints[14].TrackingState;
										int	T15=joints[15].TrackingState;
										int T16=joints[16].TrackingState;
										int T17=joints[17].TrackingState;
										int	T18=joints[18].TrackingState;
										int T19=joints[19].TrackingState;
										int T20=joints[20].TrackingState;
										int	T21=joints[21].TrackingState;
										int T22=joints[22].TrackingState;
										int T23=joints[23].TrackingState;
										int	T24=joints[24].TrackingState;
										

										//======所有的骨骼节点坐标编号start==================
										float X0=colorSpacePosition[JointType_SpineBase].X;
										float Y0=colorSpacePosition[JointType_SpineBase].Y;
										float X1=colorSpacePosition[JointType_SpineMid].X;
										float Y1=colorSpacePosition[JointType_SpineMid].Y;
										float X2=colorSpacePosition[JointType_Neck].X;
										float Y2=colorSpacePosition[JointType_Neck].Y;
										float X3=colorSpacePosition[JointType_Head].X;
										float Y3=colorSpacePosition[JointType_Head].Y;
										float X4=colorSpacePosition[JointType_ShoulderLeft].X;
										float Y4=colorSpacePosition[JointType_ShoulderLeft].Y;
										float X5=colorSpacePosition[JointType_ElbowLeft].X;
										float Y5=colorSpacePosition[JointType_ElbowLeft].Y;
										float X6=colorSpacePosition[JointType_WristLeft].X;
										float Y6=colorSpacePosition[JointType_WristLeft].Y;
										float X7=colorSpacePosition[JointType_HandLeft].X;
										float Y7=colorSpacePosition[JointType_HandLeft].Y;
										float X8=colorSpacePosition[JointType_ShoulderRight].X;
										float Y8=colorSpacePosition[JointType_ShoulderRight].Y;
										float X9=colorSpacePosition[JointType_ElbowRight].X;
										float Y9=colorSpacePosition[JointType_ElbowRight].Y;
										float X10=colorSpacePosition[JointType_WristRight].X;
										float Y10=colorSpacePosition[JointType_WristRight].Y;
										float X11=colorSpacePosition[JointType_HandRight].X;
										float Y11=colorSpacePosition[JointType_HandRight].Y;
										float X12=colorSpacePosition[JointType_HipLeft].X;
										float Y12=colorSpacePosition[JointType_HipLeft].Y;
										float X13=colorSpacePosition[JointType_KneeLeft].X;
										float Y13=colorSpacePosition[JointType_KneeLeft].Y;
										float X14=colorSpacePosition[JointType_AnkleLeft].X;
										float Y14=colorSpacePosition[JointType_AnkleLeft].Y;
										float X15=colorSpacePosition[JointType_FootLeft].X;
										float Y15=colorSpacePosition[JointType_FootLeft].Y;
										float X16=colorSpacePosition[JointType_HipRight].X;
										float Y16=colorSpacePosition[JointType_HipRight].Y;
										float X17=colorSpacePosition[JointType_KneeRight].X;
										float Y17=colorSpacePosition[JointType_KneeRight].Y;
										float X18=colorSpacePosition[JointType_AnkleRight].X;
										float Y18=colorSpacePosition[JointType_AnkleRight].Y;
										float X19=colorSpacePosition[JointType_FootRight].X;
										float Y19=colorSpacePosition[JointType_FootRight].Y;
										float X20=colorSpacePosition[JointType_SpineShoulder].X;
										float Y20=colorSpacePosition[JointType_SpineShoulder].Y;
										float X21=colorSpacePosition[JointType_HandTipLeft].X;
										float Y21=colorSpacePosition[JointType_HandTipLeft].Y;
										float X22=colorSpacePosition[JointType_ThumbLeft].X;
										float Y22=colorSpacePosition[JointType_ThumbLeft].Y;
										float X23=colorSpacePosition[JointType_HandTipRight].X;
										float Y23=colorSpacePosition[JointType_HandTipRight].Y;
										float X24=colorSpacePosition[JointType_ThumbRight].X;
										float Y24=colorSpacePosition[JointType_ThumbRight].Y;

			                           //=========所有的骨骼节点坐标编号end=================



//if(OK==1&&order!=1)   //当标志位是1且不是第一个出现时，那么进行图像筛选程序。
										//{
										//	cout<<"图像处理！！"<<endl;
										//	OK=0;
										//}

//============================图片处理start========================
//										if(OK==1&&order!=1)   //当标志位是1且不是第一个出现时，那么进行图像筛选程序。
//										{
//
//
////=================全身start================================
//											IplImage *src_0;
//											float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_0;  
//											string path_0 = "E:\\数据输出\\";  
//											string  exten_0 = "*_0.jpg";  
//
//											vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  
//
//											const int size_0 = filenames_0.size();
//
//											if(size_0!=0)
//											{
//
//											//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
//											for (int p = 0; p < size_0;p++)  
//											{  
//
//												string fileName_0 = filenames_0[p];  
//												string fileFullName_0 = path_0 + fileName_0;  
//												//cout<<"File name:"<<fileName_0<<endl;  
//												//cout<<"Full path_0:"<<fileFullName_0<<endl;  
//
//												//=============================================================
//
//												const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形
//
//												//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
//												//sprintf(windowname,"%d.jpg",p);
//												src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_0->height;i++)
//													for(int j=0;j<src_0->width;j++)
//													{
//														// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
//														if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
//													s_0[p]=per_p_0;	  //把多副图的占比值写入数组中
//
//													//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
//													//cvShowImage(windowname,src_0);
//													//waitKey(500);
//
//											}
//											//==========找出最小的占比值======================
//											float min_0=s_0[0]; //假设第一个是最小值
//											int index_0=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_0;i++)
//											{
//
//												if(s_0[i]<min_0)
//												{
//													min_0=s_0[i];
//													index_0=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_0 = filenames_0[index_0];  
//											string fileFullName_0 = path_0 + fileName_0;
//											//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;
//
//
//											const char * filename1_0=fileFullName_0.c_str();
//											char cmd_0[255] = {0};
//											//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
//											//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
//											sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-2);
//											system(cmd_0);
//
//											//=========删图=========================
//											for(int i=0;i<size_0;i++)
//											{
//												//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
//												//{
//												//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 
//
//
//												string fileName_0 = filenames_0[i];  
//												string fileFullName_0 = path_0 + fileName_0;  
//
//												const char * filename_0=fileFullName_0.c_str(); 
//
//												if( remove(filename_0) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_0);
//												else
//													perror("remove");
//												//}
//											}
//											}
////=================全身end================================
//
//
//
//
//
//
//
////=================躯干start================================
//											IplImage *src_1;
//											float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_1;  
//											string path_1 = "E:\\数据输出\\";  
//											string  exten_1 = "*_1.jpg";  
//
//											vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  
//
//											const int size_1 = filenames_1.size();
//
//											if(size_1!=0)
//											{
//
//											//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
//											for (int p = 0; p < size_1;p++)  
//											{  
//
//												string fileName_1 = filenames_1[p];  
//												string fileFullName_1 = path_1 + fileName_1;  
//												//cout<<"File name:"<<fileName_1<<endl;  
//												//cout<<"Full path_1:"<<fileFullName_1<<endl;  
//
//												//=============================================================
//
//												const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形
//
//												//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
//												//sprintf(windowname,"%d.jpg",p);
//												src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_1->height;i++)
//													for(int j=0;j<src_1->width;j++)
//													{
//														// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
//														if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
//													s_1[p]=per_p_1;	  //把多副图的占比值写入数组中
//
//													//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
//													//cvShowImage(windowname,src_1);
//													//waitKey(500);
//
//											}
//											//==========找出最小的占比值======================
//											float min_1=s_1[0]; //假设第一个是最小值
//											int index_1=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_1;i++)
//											{
//
//												if(s_1[i]<min_1)
//												{
//													min_1=s_1[i];
//													index_1=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_1 = filenames_1[index_1];  
//											string fileFullName_1 = path_1 + fileName_1;
//											//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;
//
//
//											const char * filename1_1=fileFullName_1.c_str();
//											char cmd_1[255] = {0};
//											//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
//											//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
//											sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-2);
//											system(cmd_1);
//
//											//=========删图=========================
//											for(int i=0;i<size_1;i++)
//											{
//												//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
//												//{
//												//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 
//
//
//												string fileName_1 = filenames_1[i];  
//												string fileFullName_1 = path_1 + fileName_1;  
//
//												const char * filename_1=fileFullName_1.c_str(); 
//
//												if( remove(filename_1) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_1);
//												else
//													perror("remove");
//												//}
//											}
//											}
////=================躯干end================================
//
////===========右臂上处理start============================
//
//                                            IplImage *src_2;
//											float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_2;  
//											string path_2 = "E:\\数据输出\\";  
//											string  exten_2 = "*_2.jpg";  
//
//											vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  
//
//											const int size_2 = filenames_2.size();     
//											//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
//											if(size_2!=0)
//											{
//											for (int p = 0; p < size_2;p++)  
//											{  
//
//												string fileName_2 = filenames_2[p];  
//												string fileFullName_2 = path_2 + fileName_2;  
//												//cout<<"File name:"<<fileName_2<<endl;  
//												//cout<<"Full path_2:"<<fileFullName_2<<endl;  
//
//												//=============================================================
//
//												const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形
//
//												//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
//												//sprintf(windowname,"%d.jpg",p);
//												src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_2->height;i++)
//													for(int j=0;j<src_2->width;j++)
//													{
//														// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
//														if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
//													s_2[p]=per_p_2;	  //把多副图的占比值写入数组中
//
//													//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
//													//cvShowImage(windowname,src_2);
//													//waitKey(500);
//
//											}
//											//==========找出最小的占比值======================
//											float min_2=s_2[0]; //假设第一个是最小值
//											int index_2=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_2;i++)
//											{
//
//												if(s_2[i]<min_2)
//												{
//													min_2=s_2[i];
//													index_2=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_2 = filenames_2[index_2];  
//											string fileFullName_2 = path_2 + fileName_2;
//											//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;
//
//
//											const char * filename1_2=fileFullName_2.c_str();
//											char cmd_2[255] = {0};
//											//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
//											//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
//											sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-2);
//											system(cmd_2);
//
//											//=========删图=========================
//											for(int i=0;i<size_2;i++)
//											{
//												//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
//												//{
//												//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 
//
//
//												string fileName_2 = filenames_2[i];  
//												string fileFullName_2 = path_2 + fileName_2;  
//
//												const char * filename_2=fileFullName_2.c_str(); 
//
//												if( remove(filename_2) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_2);
//												else
//													perror("remove");
//												//}
//											}
//											}
////===========右臂上处理end==============================
//
////===========右臂下处理start============================
//                                            IplImage *src_3;
//											float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_3;  
//											string path_3 = "E:\\数据输出\\";  
//											string  exten_3 = "*_3.jpg";  
//
//											vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  
//
//											const int size_3 = filenames_3.size(); 
//											if(size_3!=0)
//											{
//											for (int p = 0; p < size_3;p++)  
//											{  
//
//												string fileName_3 = filenames_3[p];  
//												string fileFullName_3 = path_3 + fileName_3;   
//
//												//=============================================================
//
//												const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形
//
//												src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_3->height;i++)
//													for(int j=0;j<src_3->width;j++)
//													{
//														
//														if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
//													s_3[p]=per_p_3;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_3=s_3[0]; //假设第一个是最小值
//											int index_3=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_3;i++)
//											{
//
//												if(s_3[i]<min_3)
//												{
//													min_3=s_3[i];
//													index_3=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_3 = filenames_3[index_3];  
//											string fileFullName_3 = path_3 + fileName_3;
//										
//											const char * filename1_3=fileFullName_3.c_str();
//											char cmd_3[255] = {0};
//											
//											sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-2);
//											system(cmd_3);
//
//											//=========删图=========================
//											for(int i=0;i<size_3;i++)
//											{
//												
//												string fileName_3 = filenames_3[i];  
//												string fileFullName_3 = path_3 + fileName_3;  
//
//												const char * filename_3=fileFullName_3.c_str(); 
//
//												if( remove(filename_3) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_3);
//												else
//													perror("remove");
//												//}
//											}
//											}
//
////===========右臂下处理end==============================
////===========左臂上处理start============================
//                                            IplImage *src_4;
//											float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_4;  
//											string path_4 = "E:\\数据输出\\";  
//											string  exten_4 = "*_4.jpg";  
//
//											vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  
//
//											const int size_4 = filenames_4.size(); 
//											if(size_4!=0)
//											{
//											for (int p = 0; p < size_4;p++)  
//											{  
//
//												string fileName_4 = filenames_4[p];  
//												string fileFullName_4 = path_4 + fileName_4;   
//
//												//=============================================================
//
//												const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形
//
//												src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_4->height;i++)
//													for(int j=0;j<src_4->width;j++)
//													{
//														
//														if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
//													s_4[p]=per_p_4;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_4=s_4[0]; //假设第一个是最小值
//											int index_4=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_4;i++)
//											{
//
//												if(s_4[i]<min_4)
//												{
//													min_4=s_4[i];
//													index_4=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_4 = filenames_4[index_4];  
//											string fileFullName_4 = path_4 + fileName_4;
//										
//											const char * filename1_4=fileFullName_4.c_str();
//											char cmd_4[255] = {0};
//											
//											sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-2);
//											system(cmd_4);
//
//											//=========删图=========================
//											for(int i=0;i<size_4;i++)
//											{
//												
//												string fileName_4 = filenames_4[i];  
//												string fileFullName_4 = path_4 + fileName_4;  
//
//												const char * filename_4=fileFullName_4.c_str(); 
//
//												if( remove(filename_4) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_4);
//												else
//													perror("remove");
//												//}
//											}
//											}
//
////===========左臂上处理end==============================
////===========左臂下处理start============================
//                                            IplImage *src_5;
//											float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_5;  
//											string path_5 = "E:\\数据输出\\";  
//											string  exten_5 = "*_5.jpg";  
//
//											vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  
//
//											const int size_5 = filenames_5.size(); 
//											if(size_5!=0)
//											{
//											for (int p = 0; p < size_5;p++)  
//											{  
//
//												string fileName_5 = filenames_5[p];  
//												string fileFullName_5 = path_5 + fileName_5;   
//
//												//=============================================================
//
//												const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形
//
//												src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_5->height;i++)
//													for(int j=0;j<src_5->width;j++)
//													{
//														
//														if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
//													s_5[p]=per_p_5;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_5=s_5[0]; //假设第一个是最小值
//											int index_5=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_5;i++)
//											{
//
//												if(s_5[i]<min_5)
//												{
//													min_5=s_5[i];
//													index_5=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_5 = filenames_5[index_5];  
//											string fileFullName_5 = path_5 + fileName_5;
//										
//											const char * filename1_5=fileFullName_5.c_str();
//											char cmd_5[255] = {0};
//											
//											sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-2);
//											system(cmd_5);
//
//											//=========删图=========================
//											for(int i=0;i<size_5;i++)
//											{
//												
//												string fileName_5 = filenames_5[i];  
//												string fileFullName_5 = path_5 + fileName_5;  
//
//												const char * filename_5=fileFullName_5.c_str(); 
//
//												if( remove(filename_5) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_5);
//												else
//													perror("remove");
//											}
//											}
//
////===========左臂下处理end==============================
//
////===========右腿上处理start============================
//                                            IplImage *src_6;
//											float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_6;  
//											string path_6 = "E:\\数据输出\\";  
//											string  exten_6 = "*_6.jpg";  
//
//											vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  
//
//											const int size_6 = filenames_6.size();
//											if(size_6!=0)
//											{
//											for (int p = 0; p < size_6;p++)  
//											{  
//
//												string fileName_6 = filenames_6[p];  
//												string fileFullName_6 = path_6 + fileName_6;   
//
//												//=============================================================
//
//												const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形
//
//												src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_6->height;i++)
//													for(int j=0;j<src_6->width;j++)
//													{
//														
//														if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
//													s_6[p]=per_p_6;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_6=s_6[0]; //假设第一个是最小值
//											int index_6=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_6;i++)
//											{
//
//												if(s_6[i]<min_6)
//												{
//													min_6=s_6[i];
//													index_6=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_6 = filenames_6[index_6];  
//											string fileFullName_6 = path_6 + fileName_6;
//										
//											const char * filename1_6=fileFullName_6.c_str();
//											char cmd_6[255] = {0};
//											
//											sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-2);
//											system(cmd_6);
//
//											//=========删图=========================
//											for(int i=0;i<size_6;i++)
//											{
//												
//												string fileName_6 = filenames_6[i];  
//												string fileFullName_6 = path_6 + fileName_6;  
//
//												const char * filename_6=fileFullName_6.c_str(); 
//
//												if( remove(filename_6) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_6);
//												else
//													perror("remove");
//											}
//											}
//
//
////===========右腿上处理end==============================
////===========右腿下处理start============================
//                                            IplImage *src_7;
//											float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_7;  
//											string path_7 = "E:\\数据输出\\";  
//											string  exten_7 = "*_7.jpg";  
//
//											vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  
//
//											const int size_7 = filenames_7.size();  
//											if(size_7!=0)
//											{
//											for (int p = 0; p < size_7;p++)  
//											{  
//
//												string fileName_7 = filenames_7[p];  
//												string fileFullName_7 = path_7 + fileName_7;   
//
//												//=============================================================
//
//												const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形
//
//												src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_7->height;i++)
//													for(int j=0;j<src_7->width;j++)
//													{
//														
//														if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
//													s_7[p]=per_p_7;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_7=s_7[0]; //假设第一个是最小值
//											int index_7=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_7;i++)
//											{
//
//												if(s_7[i]<min_7)
//												{
//													min_7=s_7[i];
//													index_7=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_7 = filenames_7[index_7];  
//											string fileFullName_7 = path_7 + fileName_7;
//										
//											const char * filename1_7=fileFullName_7.c_str();
//											char cmd_7[255] = {0};
//											
//											sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-2);
//											system(cmd_7);
//
//											//=========删图=========================
//											for(int i=0;i<size_7;i++)
//											{
//												
//												string fileName_7 = filenames_7[i];  
//												string fileFullName_7 = path_7 + fileName_7;  
//
//												const char * filename_7=fileFullName_7.c_str(); 
//
//												if( remove(filename_7) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_7);
//												else
//													perror("remove");
//											}
//											}
//
//
////===========右腿下处理end==============================
////===========左腿上处理start============================
//
//                                            IplImage *src_8;
//											float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_8;  
//											string path_8 = "E:\\数据输出\\";  
//											string  exten_8 = "*_8.jpg";  
//
//											vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  
//
//											const int size_8 = filenames_8.size();  
//											if(size_8!=0)
//											{
//											for (int p = 0; p < size_8;p++)  
//											{  
//
//												string fileName_8 = filenames_8[p];  
//												string fileFullName_8 = path_8 + fileName_8;   
//
//												//=============================================================
//
//												const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形
//
//												src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_8->height;i++)
//													for(int j=0;j<src_8->width;j++)
//													{
//														
//														if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
//													s_8[p]=per_p_8;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_8=s_8[0]; //假设第一个是最小值
//											int index_8=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_8;i++)
//											{
//
//												if(s_8[i]<min_8)
//												{
//													min_8=s_8[i];
//													index_8=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_8 = filenames_8[index_8];  
//											string fileFullName_8 = path_8 + fileName_8;
//										
//											const char * filename1_8=fileFullName_8.c_str();
//											char cmd_8[255] = {0};
//											
//											sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-2);
//											system(cmd_8);
//
//											//=========删图=========================
//											for(int i=0;i<size_8;i++)
//											{
//												
//												string fileName_8 = filenames_8[i];  
//												string fileFullName_8 = path_8 + fileName_8;  
//
//												const char * filename_8=fileFullName_8.c_str(); 
//
//												if( remove(filename_8) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_8);
//												else
//													perror("remove");
//											}
//											}
//
//
//
//
////===========左腿上处理end==============================
//
////===========左腿下处理start============================
//                                            IplImage *src_9;
//											float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧
//
//											//=================找指定路径下的某种标号的图片================
//											Directory dir_9;  
//											string path_9 = "E:\\数据输出\\";  
//											string  exten_9 = "*_9.jpg";  
//
//											vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  
//
//											const int size_9 = filenames_9.size();  
//											if(size_9!=0)
//											{
//											for (int p = 0; p < size_9;p++)  
//											{  
//
//												string fileName_9 = filenames_9[p];  
//												string fileFullName_9 = path_9 + fileName_9;   
//
//												//=============================================================
//
//												const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形
//
//												src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图
//
//
//												int n=0;  //统计白色像素的个数  
//												for(int i=0;i<src_9->height;i++)
//													for(int j=0;j<src_9->width;j++)
//													{
//														
//														if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
//															n++; 
//
//													}
//													float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
//													s_9[p]=per_p_9;	  //把多副图的占比值写入数组中
//
//											}
//											//==========找出最小的占比值======================
//											float min_9=s_9[0]; //假设第一个是最小值
//											int index_9=0;   //用于记录数组找到的最小值的下标
//
//											for(int i=0;i<size_9;i++)
//											{
//
//												if(s_9[i]<min_9)
//												{
//													min_9=s_9[i];
//													index_9=i;
//												}
//
//											}
//											cout<<endl;
//											string fileName_9 = filenames_9[index_9];  
//											string fileFullName_9 = path_9 + fileName_9;
//										
//											const char * filename1_9=fileFullName_9.c_str();
//											char cmd_9[255] = {0};
//											
//											sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-2);
//											system(cmd_9);
//
//											//=========删图=========================
//											for(int i=0;i<size_9;i++)
//											{
//												
//												string fileName_9 = filenames_9[i];  
//												string fileFullName_9 = path_9 + fileName_9;  
//
//												const char * filename_9=fileFullName_9.c_str(); 
//
//												if( remove(filename_9) == 0 )   //remove返回0表示删除成功。		
//													printf("Removed %s.\n", filename_9);
//												else
//													perror("remove");
//											}
//											}
//===========左腿下处理end==============================
										//	OK=0;
										//}
//===========================================图片处理end=========================================

//===========================================================================//

							    	   string dst_img_name;
									   switch (i)
									   {
									   case 0:{dst_img_name = string_dst0;break;}
									   case 1:{dst_img_name = string_dst1;break;}
									   case 2:{dst_img_name = string_dst2;break;}
									   case 3:{dst_img_name = string_dst3;break;}
									   case 4:{dst_img_name = string_dst4;break;}
								       case 5:{dst_img_name = string_dst5;break;}
									   default: break;
									   }

										
										sprintf_s( chari, "%04d", num);  
										dst_img_name += chari;
										dst_img_name += "_0";//全身
										dst_img_name += ".jpg";

										vector<int> compression_params;
										compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
										compression_params.push_back(9);
										
										if(T1==2)
										{
											cv::imwrite( dst_img_name, bodyMat, compression_params);
											num++;
											//waitKey(2000);
											//cout<<"当前保存图像序号： "<<num-1<<endl;
										}
										//=====================================================================================//

										//画头
										//circle(img,Point(X3,Y3),(Y2-Y3),Scalar(0,255,0),3,8);
																			
											//画上身躯干矩形框
											double result0;
											result0 = atan2 (Y2-Y0,X2-X0) * 180 / CV_PI;
											RotatedRect rRect0=RotatedRect(Point2f((X2+X0)/2,(Y2+Y0)/2),Size2f(Y0-Y2,(Y0-Y2)*2/3),result0); //定义一个旋转矩形    
											Point2f vertices0[4];
											rRect0.points(vertices0);//提取旋转矩形的四个角点    
											//  for(int i=0;i<4;i++)    
											//	  {
											//	line(img,vertices0[i],vertices0[(i+1)%4],Scalar(255,255,255),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
											//   }

											//=================提取上身躯干部分=========================================

											a=vertices0[0].x;
											b=vertices0[1].x;
											c=vertices0[2].x;
											d=vertices0[3].x;
											e=vertices0[0].y;
											f=vertices0[1].y;
											g=vertices0[2].y;
											h=vertices0[3].y;
											//比较的是x坐标,最后的排序从大到小是：a,b,c,d
											(b>a)?q=a,a=b,b=q:a;   
											(c>a)?q=a,a=c,c=q:a;
											(d>a)?q=a,a=d,d=q:a;
											(c>b)?q=b,b=c,c=q:b;
											(d>b)?q=b,b=d,d=q:b;
											(d>c)?q=c,c=d,d=q:c;
											//比较的是y坐标,最后的排序从大到小是：e,f,g,h
											(f>e)?q=e,e=f,f=q:e;   
											(g>e)?q=e,e=g,g=q:e; 
											(h>e)?q=e,e=h,h=q:e;
											(g>f)?q=f,f=g,g=q:f;
											(h>f)?q=f,f=h,h=q:f;
											(h>g)?q=g,g=h,h=q:g;
											m=a-d+1;  //自适应外接矩形的长度
											n=e-h+1;  //自适应外接举行的宽度

											Mat img1(n,m,CV_8UC4,Scalar(255,255,255));
											if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
											{
												for(int i=h;i<e;i++)  //高
													for(int j=d;j<a;j++)//宽
													{
														if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
															img1.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
													}
											}

										
											switch (i)
											{
											case 0:{dst_img_name = string_dst0;break;}
											case 1:{dst_img_name = string_dst1;break;}
											case 2:{dst_img_name = string_dst2;break;}
											case 3:{dst_img_name = string_dst3;break;}
											case 4:{dst_img_name = string_dst4;break;}
											case 5:{dst_img_name = string_dst5;break;}
											default: break;
											}
											sprintf_s( chari, "%04d", num1);  //保存4位整数形式	
											dst_img_name += chari;
											dst_img_name += "_1";//上身躯干
											dst_img_name += ".jpg";

											if(T0==2&&T2==2)
											{
												cv::imwrite( dst_img_name, img1, compression_params);
												num1++;
												//waitKey(2000);
										   	}
										//===================================================================


										//右胳膊画矩形框
										double result1;
										result1 = atan2 (Y8-Y9,X8-X9) * 180 / CV_PI;
										float q1=sqrt(pow((X8-X9),2)+pow((Y8-Y9),2));  //求两个骨骼节点距离作为矩形的长度
										RotatedRect rRect1=RotatedRect(Point2f((X8+X9)/2,(Y8+Y9)/2),Size2f(q1,q1/2),result1); //定义一个旋转矩形    
										Point2f vertices1[4];    
										rRect1.points(vertices1);//提取旋转矩形的四个角点    
										//   for(int i=0;i<4;i++)    
										//   {    
										//     line(img,vertices1[i],vertices1[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//   }    

										double result2;
										result2 = atan2 (Y9-Y11,X9-X11) * 180 / CV_PI;
										float q2=sqrt(pow((X9-X11),2)+pow((Y9-Y11),2));
										RotatedRect rRect2=RotatedRect(Point2f((X11+X9)/2,(Y11+Y9)/2),Size2f(q2,q2/2),result2); //定义一个旋转矩形    
										Point2f vertices2[4];    
										rRect2.points(vertices2);//提取旋转矩形的四个角点    
										//    for(int i=0;i<4;i++)    
										//    {    
										//      line(img,vertices2[i],vertices2[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//   }  


										//=================提取右臂上部分=========================================

										a=vertices1[0].x;
										b=vertices1[1].x;
										c=vertices1[2].x;
										d=vertices1[3].x;
										e=vertices1[0].y;
										f=vertices1[1].y;
										g=vertices1[2].y;
										h=vertices1[3].y;
										//比较的是x坐标
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img2(n,m,CV_8UC4,Scalar(255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img2.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}										
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num2);  
										dst_img_name += chari;
										dst_img_name += "_2";//右臂上
										dst_img_name += ".jpg";

										if(T8==2&&T9==2)
										{
											cv::imwrite( dst_img_name, img2, compression_params);	
											num2++;
											//waitKey(2000);
										}

										//===================================================================

										//=================提取右臂下部分=========================================
										a=vertices2[0].x;
										b=vertices2[1].x;
										c=vertices2[2].x;
										d=vertices2[3].x;
										e=vertices2[0].y;
										f=vertices2[1].y;
										g=vertices2[2].y;
										h=vertices2[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img3(n,m,CV_8UC4,Scalar(255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img3.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
											
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num3);  	
										dst_img_name += chari;
										dst_img_name += "_3";//右臂下
										dst_img_name += ".jpg";

										if(T9==2&&T11==2)
										{
											cv::imwrite( dst_img_name, img3, compression_params);
											num3++;
											//waitKey(2000);
										}

										//===================================================================

										//左胳膊画矩形框
										double result3;
										result3 = atan2 (Y4-Y5,X4-X5) * 180 / CV_PI;
										float q3=sqrt(pow((X4-X5),2)+pow((Y4-Y5),2));
										RotatedRect rRect3=RotatedRect(Point2f((X4+X5)/2,(Y4+Y5)/2),Size2f(q3,q3/2),result3); //定义一个旋转矩形    
										Point2f vertices3[4];    
										rRect3.points(vertices3);//提取旋转矩形的四个角点    
										//   for(int i=0;i<4;i++)    
										//   {    
										//     line(img,vertices3[i],vertices3[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//   } 

										double result4;
										result4 = atan2 (Y5-Y7,X5-X7) * 180 / CV_PI;
										float q4=sqrt(pow((X7-X5),2)+pow((Y7-Y5),2));
										RotatedRect rRect4=RotatedRect(Point2f((X7+X5)/2,(Y7+Y5)/2),Size2f(q4,q4/2),result4); //定义一个旋转矩形    
										Point2f vertices4[4];    
										rRect4.points(vertices4);//提取旋转矩形的四个角点    
										//     for(int i=0;i<4;i++)    
										//     {    
										//      line(img,vertices4[i],vertices4[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//     } 


										//=================提取左臂上部分=========================================
										a=vertices3[0].x;
										b=vertices3[1].x;
										c=vertices3[2].x;
										d=vertices3[3].x;
										e=vertices3[0].y;
										f=vertices3[1].y;
										g=vertices3[2].y;
										h=vertices3[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img4(n,m,CV_8UC4,Scalar(255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img4.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
										
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num4);  	
										dst_img_name += chari;
										dst_img_name += "_4";//左臂上
										dst_img_name += ".jpg";
										if(T4==2&&T5==2)
										{
											cv::imwrite( dst_img_name, img4, compression_params);
											num4++;
											//waitKey(2000);
										}
										//===================================================================

										//=================提取左臂下部分=========================================
										a=vertices4[0].x;
										b=vertices4[1].x;
										c=vertices4[2].x;
										d=vertices4[3].x;
										e=vertices4[0].y;
										f=vertices4[1].y;
										g=vertices4[2].y;
										h=vertices4[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img5(n,m,CV_8UC4,Scalar(255,255,255));   //（高，宽）
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img5.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num5);  	
										dst_img_name += chari;
										dst_img_name += "_5";//左臂下		
										dst_img_name += ".jpg";
										if(T7==2&&T9==2)
										{
											cv::imwrite( dst_img_name, img5, compression_params);	
											num5++;
											//waitKey(2000);
										}
										//===================================================================


										//右腿画矩形框			
										double result5;
										result5 = atan2 (Y16-Y17,X16-X17) * 180 / CV_PI;
										float q5=sqrt(pow((X17-X16),2)+pow((Y17-Y16),2));
										RotatedRect rRect5=RotatedRect(Point2f((X16+X17)/2,(Y16+Y17)/2),Size2f(q5,q5/2),result5); //定义一个旋转矩形    
										Point2f vertices5[4];    
										rRect5.points(vertices5);//提取旋转矩形的四个角点    
										//   for(int i=0;i<4;i++)    
										//   {    
										//     line(img,vertices5[i],vertices5[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//  } 

										double result6;
										result6 = atan2 (Y17-Y18,X17-X18) * 180 / CV_PI;
										float q6=sqrt(pow((X17-X18),2)+pow((Y17-Y18),2));
										RotatedRect rRect6=RotatedRect(Point2f((X18+X17)/2,(Y18+Y17)/2),Size2f(q6,q6/2),result6); //定义一个旋转矩形    
										Point2f vertices6[4];    
										rRect6.points(vertices6);//提取旋转矩形的四个角点    
										//    for(int i=0;i<4;i++)    
										//    {    
										//      line(img,vertices6[i],vertices6[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//    }


										//=================提取右腿上部分=========================================
										a=vertices5[0].x;
										b=vertices5[1].x;
										c=vertices5[2].x;
										d=vertices5[3].x;
										e=vertices5[0].y;
										f=vertices5[1].y;
										g=vertices5[2].y;
										h=vertices5[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img6(n,m,CV_8UC4,Scalar(255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img6.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
										
										
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num6); 	
										dst_img_name += chari;
										dst_img_name += "_6";//右腿上
										dst_img_name += ".jpg";

										if(T16==2&&T17==2)
										{
											cv::imwrite( dst_img_name, img6, compression_params);
											num6++;
											//waitKey(2000);
										}
										//===================================================================


										//=================提取右腿下部分=========================================
										a=vertices6[0].x;
										b=vertices6[1].x;
										c=vertices6[2].x;
										d=vertices6[3].x;
										e=vertices6[0].y;
										f=vertices6[1].y;
										g=vertices6[2].y;
										h=vertices6[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img7(n,m,CV_8UC4,Scalar(255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img7.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
																			
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num7);  	
										dst_img_name += chari;
										dst_img_name += "_7";//右腿下
										dst_img_name += ".jpg";
										if(T17==2&&T18==2)
										{
											cv::imwrite( dst_img_name, img7, compression_params);
											num7++;
											//waitKey(2000);
										}
										//dst_img_name = "E://数据输出//body-mapping-depth//测试//";
										//===================================================================


										//画左腿
										double result7;
										result7 = atan2 (Y12-Y13,X12-X13) * 180 / CV_PI;
										float q7=sqrt(pow((X12-X13),2)+pow((Y12-Y13),2));
										RotatedRect rRect7=RotatedRect(Point2f((X13+X12)/2,(Y13+Y12)/2),Size2f(q7,q7/2),result7); //定义一个旋转矩形    
										Point2f vertices7[4];    
										rRect7.points(vertices7);//提取旋转矩形的四个角点    
										//     for(int i=0;i<4;i++)    
										//     {    
										//       line(img,vertices7[i],vertices7[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//     }

										double result8;
										result8 = atan2 (Y13-Y14,X13-X14) * 180 / CV_PI;
										float q8=sqrt(pow((X14-X13),2)+pow((Y14-Y13),2));
										RotatedRect rRect8=RotatedRect(Point2f((X13+X14)/2,(Y13+Y14)/2),Size2f(q8,q8/2),result8); //定义一个旋转矩形    
										Point2f vertices8[4];    
										rRect8.points(vertices8);//提取旋转矩形的四个角点    
										//    for(int i=0;i<4;i++)    
										//    {    
										//      line(img,vertices8[i],vertices8[(i+1)%4],Scalar(0,255,0),3,8,0);//四个角点连成线，最终形成旋转的矩形。    
										//    }


										//=================提取左腿上部分=========================================
										a=vertices7[0].x;
										b=vertices7[1].x;
										c=vertices7[2].x;
										d=vertices7[3].x;
										e=vertices7[0].y;
										f=vertices7[1].y;
										g=vertices7[2].y;
										h=vertices7[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;

										m=a-d+1;
										n=e-h+1;

										Mat img8(n,m,CV_8UC4,Scalar(255,255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img8.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
										
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num8);  	
										dst_img_name += chari;
										dst_img_name += "_8";//左腿上
										dst_img_name += ".jpg";

										if(T12==2&&T13==2)
										{
											cv::imwrite( dst_img_name, img8, compression_params);
											num8++;
											//waitKey(2000);
										}
										//===================================================================


										//=================提取左腿下部分=========================================
										a=vertices8[0].x;
										b=vertices8[1].x;
										c=vertices8[2].x;
										d=vertices8[3].x;
										e=vertices8[0].y;
										f=vertices8[1].y;
										g=vertices8[2].y;
										h=vertices8[3].y;
										//比较的是x坐标,最后的排序从大到小是：a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//比较的是y坐标,最后的排序从大到小是：e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img9(n,m,CV_8UC4,Scalar(255,255,255,255));
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //高
												for(int j=d;j<a;j++)//宽
												{
													if(img.at<Vec4b>(i,j)!=Vec4b(255,255,255,255))
														img9.at<Vec4b>(i-h,j-d)=img.at<Vec4b>(i,j);	
												}
										}
										
										switch (i)
										{
										case 0:{dst_img_name = string_dst0;break;}
										case 1:{dst_img_name = string_dst1;break;}
										case 2:{dst_img_name = string_dst2;break;}
										case 3:{dst_img_name = string_dst3;break;}
										case 4:{dst_img_name = string_dst4;break;}
										case 5:{dst_img_name = string_dst5;break;}
										default: break;
										}
										sprintf_s( chari, "%04d", num9);  
										dst_img_name += chari;
										dst_img_name += "_9";//左腿下
										dst_img_name += ".jpg";

										if(T13==2&&T14==2)
										{
										    cv::imwrite( dst_img_name, img9, compression_params);
											num9++;
											//waitKey(2000);
											
										}
										//===================================================================	

									}  //如果成功的获取到关节点类
								}  //如果成功的追踪到人体	
							}  //pbody1指针结束处
							// SafeRelease( pBody1 );	
						}  //bodycount循环结束处          for循环的位置在这结束
					}   //如果更新骨骼信息成功

					for( int count = 0; count < BODY_COUNT; count++ )//循环安全释放6个名为body的指针
					{   
						SafeRelease( pBody[count] );
					}

					SafeRelease( pBodyFrame );   
				}  //如果body成功获取最后一帧
				/* cv::resize( img, bodyMat, cv::Size(), 0.5, 0.5 );*/
				SafeRelease(pColorFrame);
			}  //颜色Frame

			//====================骨骼坐标获取end==============================================

			SafeRelease(pDepthFrame);
		}//IDepthFrame
	    SafeRelease(pBodyIndexFrame);

		if (waitKey(10)==27)  { break; }  //按Esc键退出
		
		if (count0==10)   { OK_0=1; count0=11; order++; }
		if (count1==10)   { OK_1=1; count1=11; order++; }
	    if (count2==10)	  { OK_2=1; count2=11; order++; }
		if (count3==10)   { OK_3=1; count3=11; order++; }
		if (count4==10)   { OK_4=1; count4=11; order++; }
		if (count5==10)   { OK_5=1; count5=11; order++; }

		if (count0==0)    count0=10; 
		if (count1==1)    count1=10; 
		if (count2==2)    count2=10; 
		if (count3==3)    count3=10; 
		if (count4==4)    count4=10; 
		if (count5==5)    count5=10; 
		
		if(OK_0==1)   
		{ 
			  func_0();
			OK_0=0; 
		}
		if(OK_1==1)  
		{ 
			   func_1();
			OK_1=0; 
		}
		if(OK_2==1)  
		{
			  func_2();
			OK_2=0; 
		}
		if(OK_3==1)   
		{ 
			  func_3();
			OK_3=0;
		}
		if(OK_4==1)   
		{ 
			  func_4();
			OK_4=0; 
		}
		if(OK_5==1)   
		{ 
			   func_5();
			OK_5=0;
		}
		/*if(OK_0==1)   { cout<<"索引号为 0 图像处理  |"<<"  人体保存序号 "<<order-1<<endl; OK_0=0; }
		if(OK_1==1)   { cout<<"索引号为 1 图像处理  |"<<"  人体保存序号 "<<order-1<<endl; OK_1=0; }
		if(OK_2==1)   { cout<<"索引号为 2 图像处理  |"<<"  人体保存序号 "<<order-1<<endl; OK_2=0; }
		if(OK_3==1)   { cout<<"索引号为 3 图像处理  |"<<"  人体保存序号 "<<order-1<<endl; OK_3=0; }
		if(OK_4==1)   { cout<<"索引号为 4 图像处理  |"<<"  人体保存序号 "<<order-1<<endl; OK_4=0; }
		if(OK_5==1)   { cout<<"索引号为 5 图像处理  |"<<"  人体保存序号 "<<order-1<<endl; OK_5=0; }*/


	}  //while（1）   pBodyIndexFrame

	SafeRelease( pColorSource );
	SafeRelease( pDepthSource );
	SafeRelease(pBodyIndexSource);
	SafeRelease( pBodySource );
	SafeRelease( pColorReader );
	SafeRelease( pDepthReader );
	SafeRelease(pBodyIndexReader);
	SafeRelease( pBodyReader );

	SafeRelease(pDepthDescription);
	if (pSensor) 
	{
		pSensor->Close();
	}
	SafeRelease(pSensor);

	return 0;
}   //主函数结束

//============================图片处理start========================
void  func_0()
{
                                         cout<<"索引号为 0 筛选处理  /"<<"  保存序号 "<<order-1<<endl<<endl;
                                         //=================全身start================================
											IplImage *src_0;
											float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_0;  
											string path_0 = "E:\\数据输出\\0\\";  
											string  exten_0 = "*_0.jpg";  

											vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

											const int size_0 = filenames_0.size();

											if(size_0!=0)
											{

											//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
											for (int p = 0; p < size_0;p++)  
											{  

												string fileName_0 = filenames_0[p];  
												string fileFullName_0 = path_0 + fileName_0;  
												//cout<<"File name:"<<fileName_0<<endl;  
												//cout<<"Full path_0:"<<fileFullName_0<<endl;  

												//=============================================================

												const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形

												//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
												//sprintf(windowname,"%d.jpg",p);
												src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_0->height;i++)
													for(int j=0;j<src_0->width;j++)
													{
														// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
														if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
													s_0[p]=per_p_0;	  //把多副图的占比值写入数组中

													//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
													//cvShowImage(windowname,src_0);
													//waitKey(500);

											}
											//==========找出最小的占比值======================
											float min_0=s_0[0]; //假设第一个是最小值
											int index_0=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_0;i++)
											{

												if(s_0[i]<min_0)
												{
													min_0=s_0[i];
													index_0=i;
												}

											}
											//cout<<endl;
											string fileName_0 = filenames_0[index_0];  
											string fileFullName_0 = path_0 + fileName_0;
											//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;


											const char * filename1_0=fileFullName_0.c_str();
											char cmd_0[255] = {0};
											//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
											//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
											sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-1);
											system(cmd_0);
										

											//=========删图=========================
											for(int i=0;i<size_0;i++)
											{
												//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
												//{
												//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 


												string fileName_0 = filenames_0[i];  
												string fileFullName_0 = path_0 + fileName_0;  

												const char * filename_0=fileFullName_0.c_str(); 

												if( remove(filename_0) == 0 ) ;  //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_0);
												else
													perror("remove");
												//}
											}
											}
                                      //=================全身end================================

                                      //=================躯干start================================
											IplImage *src_1;
											float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_1;  
											string path_1 = "E:\\数据输出\\0\\";  
											string  exten_1 = "*_1.jpg";  

											vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

											const int size_1 = filenames_1.size();

											if(size_1!=0)
											{

											//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
											for (int p = 0; p < size_1;p++)  
											{  

												string fileName_1 = filenames_1[p];  
												string fileFullName_1 = path_1 + fileName_1;  
												//cout<<"File name:"<<fileName_1<<endl;  
												//cout<<"Full path_1:"<<fileFullName_1<<endl;  

												//=============================================================

												const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形

												//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
												//sprintf(windowname,"%d.jpg",p);
												src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_1->height;i++)
													for(int j=0;j<src_1->width;j++)
													{
														// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
														if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
													s_1[p]=per_p_1;	  //把多副图的占比值写入数组中

													//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
													//cvShowImage(windowname,src_1);
													//waitKey(500);

											}
											//==========找出最小的占比值======================
											float min_1=s_1[0]; //假设第一个是最小值
											int index_1=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_1;i++)
											{

												if(s_1[i]<min_1)
												{
													min_1=s_1[i];
													index_1=i;
												}

											}
											//cout<<endl;
											string fileName_1 = filenames_1[index_1];  
											string fileFullName_1 = path_1 + fileName_1;
											//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;


											const char * filename1_1=fileFullName_1.c_str();
											char cmd_1[255] = {0};
											//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
											//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
											sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-1);
											system(cmd_1);

											//=========删图=========================
											for(int i=0;i<size_1;i++)
											{
												//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
												//{
												//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 


												string fileName_1 = filenames_1[i];  
												string fileFullName_1 = path_1 + fileName_1;  

												const char * filename_1=fileFullName_1.c_str(); 

												if( remove(filename_1) == 0 ) ;  //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_1);
												else
													perror("remove");
												//}
											}
											}
                                           //=================躯干end================================

                                           //===========右臂上处理start============================

                                            IplImage *src_2;
											float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_2;  
											string path_2 = "E:\\数据输出\\0\\";  
											string  exten_2 = "*_2.jpg";  

											vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

											const int size_2 = filenames_2.size();     
											//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
											if(size_2!=0)
											{
											for (int p = 0; p < size_2;p++)  
											{  

												string fileName_2 = filenames_2[p];  
												string fileFullName_2 = path_2 + fileName_2;  
												//cout<<"File name:"<<fileName_2<<endl;  
												//cout<<"Full path_2:"<<fileFullName_2<<endl;  

												//=============================================================

												const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形

												//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
												//sprintf(windowname,"%d.jpg",p);
												src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_2->height;i++)
													for(int j=0;j<src_2->width;j++)
													{
														// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
														if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
													s_2[p]=per_p_2;	  //把多副图的占比值写入数组中

													//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
													//cvShowImage(windowname,src_2);
													//waitKey(500);

											}
											//==========找出最小的占比值======================
											float min_2=s_2[0]; //假设第一个是最小值
											int index_2=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_2;i++)
											{

												if(s_2[i]<min_2)
												{
													min_2=s_2[i];
													index_2=i;
												}

											}
											//cout<<endl;
											string fileName_2 = filenames_2[index_2];  
											string fileFullName_2 = path_2 + fileName_2;
											//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;


											const char * filename1_2=fileFullName_2.c_str();
											char cmd_2[255] = {0};
											//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
											//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
											sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-1);
											system(cmd_2);

											//=========删图=========================
											for(int i=0;i<size_2;i++)
											{
												//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
												//{
												//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 


												string fileName_2 = filenames_2[i];  
												string fileFullName_2 = path_2 + fileName_2;  

												const char * filename_2=fileFullName_2.c_str(); 

												if( remove(filename_2) == 0 );   //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_2);
												else
													perror("remove");
												//}
											}
											}
//===========右臂上处理end==============================

//===========右臂下处理start============================
                                            IplImage *src_3;
											float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_3;  
											string path_3 = "E:\\数据输出\\0\\";  
											string  exten_3 = "*_3.jpg";  

											vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  

											const int size_3 = filenames_3.size(); 
											if(size_3!=0)
											{
											for (int p = 0; p < size_3;p++)  
											{  

												string fileName_3 = filenames_3[p];  
												string fileFullName_3 = path_3 + fileName_3;   

												//=============================================================

												const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形

												src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_3->height;i++)
													for(int j=0;j<src_3->width;j++)
													{
														
														if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
													s_3[p]=per_p_3;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_3=s_3[0]; //假设第一个是最小值
											int index_3=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_3;i++)
											{

												if(s_3[i]<min_3)
												{
													min_3=s_3[i];
													index_3=i;
												}

											}
											//cout<<endl;
											string fileName_3 = filenames_3[index_3];  
											string fileFullName_3 = path_3 + fileName_3;
										
											const char * filename1_3=fileFullName_3.c_str();
											char cmd_3[255] = {0};
											
											sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-1);
											system(cmd_3);

											//=========删图=========================
											for(int i=0;i<size_3;i++)
											{
												
												string fileName_3 = filenames_3[i];  
												string fileFullName_3 = path_3 + fileName_3;  

												const char * filename_3=fileFullName_3.c_str(); 

												if( remove(filename_3) == 0 ) ;  //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_3);
												else
													perror("remove");
												//}
											}
											}

//===========右臂下处理end==============================
//===========左臂上处理start============================
                                            IplImage *src_4;
											float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_4;  
											string path_4 = "E:\\数据输出\\0\\";  
											string  exten_4 = "*_4.jpg";  

											vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  

											const int size_4 = filenames_4.size(); 
											if(size_4!=0)
											{
											for (int p = 0; p < size_4;p++)  
											{  

												string fileName_4 = filenames_4[p];  
												string fileFullName_4 = path_4 + fileName_4;   

												//=============================================================

												const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形

												src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_4->height;i++)
													for(int j=0;j<src_4->width;j++)
													{
														
														if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
													s_4[p]=per_p_4;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_4=s_4[0]; //假设第一个是最小值
											int index_4=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_4;i++)
											{

												if(s_4[i]<min_4)
												{
													min_4=s_4[i];
													index_4=i;
												}

											}
											//cout<<endl;
											string fileName_4 = filenames_4[index_4];  
											string fileFullName_4 = path_4 + fileName_4;
										
											const char * filename1_4=fileFullName_4.c_str();
											char cmd_4[255] = {0};
											
											sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-1);
											system(cmd_4);

											//=========删图=========================
											for(int i=0;i<size_4;i++)
											{
												
												string fileName_4 = filenames_4[i];  
												string fileFullName_4 = path_4 + fileName_4;  

												const char * filename_4=fileFullName_4.c_str(); 

												if( remove(filename_4) == 0 ) ;  //remove返回0表示删除成功。		
												//	printf("Removed %s.\n", filename_4);
												else
													perror("remove");
												//}
											}
											}

//===========左臂上处理end==============================
//===========左臂下处理start============================
                                            IplImage *src_5;
											float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_5;  
											string path_5 = "E:\\数据输出\\0\\";  
											string  exten_5 = "*_5.jpg";  

											vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  

											const int size_5 = filenames_5.size(); 
											if(size_5!=0)
											{
											for (int p = 0; p < size_5;p++)  
											{  

												string fileName_5 = filenames_5[p];  
												string fileFullName_5 = path_5 + fileName_5;   

												//=============================================================

												const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形

												src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_5->height;i++)
													for(int j=0;j<src_5->width;j++)
													{
														
														if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
													s_5[p]=per_p_5;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_5=s_5[0]; //假设第一个是最小值
											int index_5=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_5;i++)
											{

												if(s_5[i]<min_5)
												{
													min_5=s_5[i];
													index_5=i;
												}

											}
											//cout<<endl;
											string fileName_5 = filenames_5[index_5];  
											string fileFullName_5 = path_5 + fileName_5;
										
											const char * filename1_5=fileFullName_5.c_str();
											char cmd_5[255] = {0};
											
											sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-1);
											system(cmd_5);

											//=========删图=========================
											for(int i=0;i<size_5;i++)
											{
												
												string fileName_5 = filenames_5[i];  
												string fileFullName_5 = path_5 + fileName_5;  

												const char * filename_5=fileFullName_5.c_str(); 

												if( remove(filename_5) == 0 )  ; //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_5);
												else
													perror("remove");
											}
											}

//===========左臂下处理end==============================

//===========右腿上处理start============================
                                            IplImage *src_6;
											float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_6;  
											string path_6 = "E:\\数据输出\\0\\";  
											string  exten_6 = "*_6.jpg";  

											vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  

											const int size_6 = filenames_6.size();
											if(size_6!=0)
											{
											for (int p = 0; p < size_6;p++)  
											{  

												string fileName_6 = filenames_6[p];  
												string fileFullName_6 = path_6 + fileName_6;   

												//=============================================================

												const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形

												src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_6->height;i++)
													for(int j=0;j<src_6->width;j++)
													{
														
														if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
													s_6[p]=per_p_6;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_6=s_6[0]; //假设第一个是最小值
											int index_6=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_6;i++)
											{

												if(s_6[i]<min_6)
												{
													min_6=s_6[i];
													index_6=i;
												}

											}
											//cout<<endl;
											string fileName_6 = filenames_6[index_6];  
											string fileFullName_6 = path_6 + fileName_6;
										
											const char * filename1_6=fileFullName_6.c_str();
											char cmd_6[255] = {0};
											
											sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-1);
											system(cmd_6);

											//=========删图=========================
											for(int i=0;i<size_6;i++)
											{
												
												string fileName_6 = filenames_6[i];  
												string fileFullName_6 = path_6 + fileName_6;  

												const char * filename_6=fileFullName_6.c_str(); 

												if( remove(filename_6) == 0 ) ;  //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_6);
												else
													perror("remove");
											}
											}


//===========右腿上处理end==============================
//===========右腿下处理start============================
                                            IplImage *src_7;
											float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_7;  
											string path_7 = "E:\\数据输出\\0\\";  
											string  exten_7 = "*_7.jpg";  

											vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  

											const int size_7 = filenames_7.size();  
											if(size_7!=0)
											{
											for (int p = 0; p < size_7;p++)  
											{  

												string fileName_7 = filenames_7[p];  
												string fileFullName_7 = path_7 + fileName_7;   

												//=============================================================

												const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形

												src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_7->height;i++)
													for(int j=0;j<src_7->width;j++)
													{
														
														if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
													s_7[p]=per_p_7;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_7=s_7[0]; //假设第一个是最小值
											int index_7=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_7;i++)
											{

												if(s_7[i]<min_7)
												{
													min_7=s_7[i];
													index_7=i;
												}

											}
											//cout<<endl;
											string fileName_7 = filenames_7[index_7];  
											string fileFullName_7 = path_7 + fileName_7;
										
											const char * filename1_7=fileFullName_7.c_str();
											char cmd_7[255] = {0};
											
											sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-1);
											system(cmd_7);

											//=========删图=========================
											for(int i=0;i<size_7;i++)
											{
												
												string fileName_7 = filenames_7[i];  
												string fileFullName_7 = path_7 + fileName_7;  

												const char * filename_7=fileFullName_7.c_str(); 

												if( remove(filename_7) == 0 ) ;  //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_7);
												else
													perror("remove");
											}
											}


//===========右腿下处理end==============================
//===========左腿上处理start============================

                                            IplImage *src_8;
											float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_8;  
											string path_8 = "E:\\数据输出\\0\\";  
											string  exten_8 = "*_8.jpg";  

											vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  

											const int size_8 = filenames_8.size();  
											if(size_8!=0)
											{
											for (int p = 0; p < size_8;p++)  
											{  

												string fileName_8 = filenames_8[p];  
												string fileFullName_8 = path_8 + fileName_8;   

												//=============================================================

												const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形

												src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_8->height;i++)
													for(int j=0;j<src_8->width;j++)
													{
														
														if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
													s_8[p]=per_p_8;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_8=s_8[0]; //假设第一个是最小值
											int index_8=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_8;i++)
											{

												if(s_8[i]<min_8)
												{
													min_8=s_8[i];
													index_8=i;
												}

											}
											//cout<<endl;
											string fileName_8 = filenames_8[index_8];  
											string fileFullName_8 = path_8 + fileName_8;
										
											const char * filename1_8=fileFullName_8.c_str();
											char cmd_8[255] = {0};
											
											sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-1);
											system(cmd_8);

											//=========删图=========================
											for(int i=0;i<size_8;i++)
											{
												
												string fileName_8 = filenames_8[i];  
												string fileFullName_8 = path_8 + fileName_8;  

												const char * filename_8=fileFullName_8.c_str(); 

												if( remove(filename_8) == 0 )  ; //remove返回0表示删除成功。		
													//printf("Removed %s.\n", filename_8);
												else
													perror("remove");
											}
											}




//===========左腿上处理end==============================

//===========左腿下处理start============================
                                            IplImage *src_9;
											float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

											//=================找指定路径下的某种标号的图片================
											Directory dir_9;  
											string path_9 = "E:\\数据输出\\0\\";  
											string  exten_9 = "*_9.jpg";  

											vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  

											const int size_9 = filenames_9.size();  
											if(size_9!=0)
											{
											for (int p = 0; p < size_9;p++)  
											{  

												string fileName_9 = filenames_9[p];  
												string fileFullName_9 = path_9 + fileName_9;   

												//=============================================================

												const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形

												src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图


												int n=0;  //统计白色像素的个数  
												for(int i=0;i<src_9->height;i++)
													for(int j=0;j<src_9->width;j++)
													{
														
														if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
															n++; 

													}
													float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
													s_9[p]=per_p_9;	  //把多副图的占比值写入数组中

											}
											//==========找出最小的占比值======================
											float min_9=s_9[0]; //假设第一个是最小值
											int index_9=0;   //用于记录数组找到的最小值的下标

											for(int i=0;i<size_9;i++)
											{

												if(s_9[i]<min_9)
												{
													min_9=s_9[i];
													index_9=i;
												}

											}
											//cout<<endl;
											string fileName_9 = filenames_9[index_9];  
											string fileFullName_9 = path_9 + fileName_9;
										
											const char * filename1_9=fileFullName_9.c_str();
											char cmd_9[255] = {0};
											
											sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-1);
											system(cmd_9);

											//=========删图=========================
											for(int i=0;i<size_9;i++)
											{
												
												string fileName_9 = filenames_9[i];  
												string fileFullName_9 = path_9 + fileName_9;  

												const char * filename_9=fileFullName_9.c_str(); 

												if( remove(filename_9) == 0 ) ;  //remove返回0表示删除成功。		
												//	printf("Removed %s.\n", filename_9);
												else
													perror("remove");
											}
											}
                                      //===========左腿下处理end==============================
	
}







void func_1()
{
cout<<"索引号为 1 筛选处理  /"<<"  保存序号 "<<order-1<<endl<<endl;
//=================全身start================================
IplImage *src_0;
float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_0;  
string path_0 = "E:\\数据输出\\1\\";  
string  exten_0 = "*_0.jpg";  

vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

const int size_0 = filenames_0.size();

if(size_0!=0)
{

	//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
	for (int p = 0; p < size_0;p++)  
	{  

		string fileName_0 = filenames_0[p];  
		string fileFullName_0 = path_0 + fileName_0;  
		//cout<<"File name:"<<fileName_0<<endl;  
		//cout<<"Full path_0:"<<fileFullName_0<<endl;  

		//=============================================================

		const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形

		//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
		//sprintf(windowname,"%d.jpg",p);
		src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_0->height;i++)
			for(int j=0;j<src_0->width;j++)
			{
				// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
				if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
			s_0[p]=per_p_0;	  //把多副图的占比值写入数组中

			//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
			//cvShowImage(windowname,src_0);
			//waitKey(500);

	}
	//==========找出最小的占比值======================
	float min_0=s_0[0]; //假设第一个是最小值
	int index_0=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_0;i++)
	{

		if(s_0[i]<min_0)
		{
			min_0=s_0[i];
			index_0=i;
		}

	}
	//cout<<endl;
	string fileName_0 = filenames_0[index_0];  
	string fileFullName_0 = path_0 + fileName_0;
	//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;


	const char * filename1_0=fileFullName_0.c_str();
	char cmd_0[255] = {0};
	//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
	//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
	sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-1);
	system(cmd_0);

	//=========删图=========================
	for(int i=0;i<size_0;i++)
	{
		//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
		//{
		//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 


		string fileName_0 = filenames_0[i];  
		string fileFullName_0 = path_0 + fileName_0;  

		const char * filename_0=fileFullName_0.c_str(); 

		if( remove(filename_0) == 0 ) ;  //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_0);
		else
			perror("remove");
		//}
	}
}
//=================全身end================================

//=================躯干start================================
IplImage *src_1;
float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_1;  
string path_1 = "E:\\数据输出\\1\\";  
string  exten_1 = "*_1.jpg";  

vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

const int size_1 = filenames_1.size();

if(size_1!=0)
{

	//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
	for (int p = 0; p < size_1;p++)  
	{  

		string fileName_1 = filenames_1[p];  
		string fileFullName_1 = path_1 + fileName_1;  
		//cout<<"File name:"<<fileName_1<<endl;  
		//cout<<"Full path_1:"<<fileFullName_1<<endl;  

		//=============================================================

		const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形

		//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
		//sprintf(windowname,"%d.jpg",p);
		src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_1->height;i++)
			for(int j=0;j<src_1->width;j++)
			{
				// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
				if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
			s_1[p]=per_p_1;	  //把多副图的占比值写入数组中

			//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
			//cvShowImage(windowname,src_1);
			//waitKey(500);

	}
	//==========找出最小的占比值======================
	float min_1=s_1[0]; //假设第一个是最小值
	int index_1=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_1;i++)
	{

		if(s_1[i]<min_1)
		{
			min_1=s_1[i];
			index_1=i;
		}

	}
	//cout<<endl;
	string fileName_1 = filenames_1[index_1];  
	string fileFullName_1 = path_1 + fileName_1;
	//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;


	const char * filename1_1=fileFullName_1.c_str();
	char cmd_1[255] = {0};
	//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
	//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
	sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-1);
	system(cmd_1);

	//=========删图=========================
	for(int i=0;i<size_1;i++)
	{
		//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
		//{
		//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 


		string fileName_1 = filenames_1[i];  
		string fileFullName_1 = path_1 + fileName_1;  

		const char * filename_1=fileFullName_1.c_str(); 

		if( remove(filename_1) == 0 ) ;  //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_1);
		else
			perror("remove");
		//}
	}
}
//=================躯干end================================

//===========右臂上处理start============================

IplImage *src_2;
float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_2;  
string path_2 = "E:\\数据输出\\1\\";  
string  exten_2 = "*_2.jpg";  

vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

const int size_2 = filenames_2.size();     
//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
if(size_2!=0)
{
	for (int p = 0; p < size_2;p++)  
	{  

		string fileName_2 = filenames_2[p];  
		string fileFullName_2 = path_2 + fileName_2;  
		//cout<<"File name:"<<fileName_2<<endl;  
		//cout<<"Full path_2:"<<fileFullName_2<<endl;  

		//=============================================================

		const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形

		//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
		//sprintf(windowname,"%d.jpg",p);
		src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_2->height;i++)
			for(int j=0;j<src_2->width;j++)
			{
				// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
				if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
			s_2[p]=per_p_2;	  //把多副图的占比值写入数组中

			//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
			//cvShowImage(windowname,src_2);
			//waitKey(500);

	}
	//==========找出最小的占比值======================
	float min_2=s_2[0]; //假设第一个是最小值
	int index_2=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_2;i++)
	{

		if(s_2[i]<min_2)
		{
			min_2=s_2[i];
			index_2=i;
		}

	}
	//cout<<endl;
	string fileName_2 = filenames_2[index_2];  
	string fileFullName_2 = path_2 + fileName_2;
	//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;


	const char * filename1_2=fileFullName_2.c_str();
	char cmd_2[255] = {0};
	//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
	//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
	sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-1);
	system(cmd_2);

	//=========删图=========================
	for(int i=0;i<size_2;i++)
	{
		//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
		//{
		//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 


		string fileName_2 = filenames_2[i];  
		string fileFullName_2 = path_2 + fileName_2;  

		const char * filename_2=fileFullName_2.c_str(); 

		if( remove(filename_2) == 0 ) ;  //remove返回0表示删除成功。		
		//	printf("Removed %s.\n", filename_2);
		else
			perror("remove");
		//}
	}
}
//===========右臂上处理end==============================

//===========右臂下处理start============================
IplImage *src_3;
float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_3;  
string path_3 = "E:\\数据输出\\1\\";  
string  exten_3 = "*_3.jpg";  

vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  

const int size_3 = filenames_3.size(); 
if(size_3!=0)
{
	for (int p = 0; p < size_3;p++)  
	{  

		string fileName_3 = filenames_3[p];  
		string fileFullName_3 = path_3 + fileName_3;   

		//=============================================================

		const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形

		src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_3->height;i++)
			for(int j=0;j<src_3->width;j++)
			{

				if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
			s_3[p]=per_p_3;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_3=s_3[0]; //假设第一个是最小值
	int index_3=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_3;i++)
	{

		if(s_3[i]<min_3)
		{
			min_3=s_3[i];
			index_3=i;
		}

	}
	//cout<<endl;
	string fileName_3 = filenames_3[index_3];  
	string fileFullName_3 = path_3 + fileName_3;

	const char * filename1_3=fileFullName_3.c_str();
	char cmd_3[255] = {0};

	sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-1);
	system(cmd_3);

	//=========删图=========================
	for(int i=0;i<size_3;i++)
	{

		string fileName_3 = filenames_3[i];  
		string fileFullName_3 = path_3 + fileName_3;  

		const char * filename_3=fileFullName_3.c_str(); 

		if( remove(filename_3) == 0 ) ;  //remove返回0表示删除成功。		
		//	printf("Removed %s.\n", filename_3);
		else
			perror("remove");
		//}
	}
}

//===========右臂下处理end==============================
//===========左臂上处理start============================
IplImage *src_4;
float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_4;  
string path_4 = "E:\\数据输出\\1\\";  
string  exten_4 = "*_4.jpg";  

vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  

const int size_4 = filenames_4.size(); 
if(size_4!=0)
{
	for (int p = 0; p < size_4;p++)  
	{  

		string fileName_4 = filenames_4[p];  
		string fileFullName_4 = path_4 + fileName_4;   

		//=============================================================

		const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形

		src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_4->height;i++)
			for(int j=0;j<src_4->width;j++)
			{

				if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
			s_4[p]=per_p_4;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_4=s_4[0]; //假设第一个是最小值
	int index_4=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_4;i++)
	{

		if(s_4[i]<min_4)
		{
			min_4=s_4[i];
			index_4=i;
		}

	}
	//cout<<endl;
	string fileName_4 = filenames_4[index_4];  
	string fileFullName_4 = path_4 + fileName_4;

	const char * filename1_4=fileFullName_4.c_str();
	char cmd_4[255] = {0};

	sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-1);
	system(cmd_4);

	//=========删图=========================
	for(int i=0;i<size_4;i++)
	{

		string fileName_4 = filenames_4[i];  
		string fileFullName_4 = path_4 + fileName_4;  

		const char * filename_4=fileFullName_4.c_str(); 

		if( remove(filename_4) == 0 );   //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_4);
		else
			perror("remove");
		//}
	}
}

//===========左臂上处理end==============================
//===========左臂下处理start============================
IplImage *src_5;
float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_5;  
string path_5 = "E:\\数据输出\\1\\";  
string  exten_5 = "*_5.jpg";  

vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  

const int size_5 = filenames_5.size(); 
if(size_5!=0)
{
	for (int p = 0; p < size_5;p++)  
	{  

		string fileName_5 = filenames_5[p];  
		string fileFullName_5 = path_5 + fileName_5;   

		//=============================================================

		const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形

		src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_5->height;i++)
			for(int j=0;j<src_5->width;j++)
			{

				if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
			s_5[p]=per_p_5;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_5=s_5[0]; //假设第一个是最小值
	int index_5=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_5;i++)
	{

		if(s_5[i]<min_5)
		{
			min_5=s_5[i];
			index_5=i;
		}

	}
	//cout<<endl;
	string fileName_5 = filenames_5[index_5];  
	string fileFullName_5 = path_5 + fileName_5;

	const char * filename1_5=fileFullName_5.c_str();
	char cmd_5[255] = {0};

	sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-1);
	system(cmd_5);

	//=========删图=========================
	for(int i=0;i<size_5;i++)
	{

		string fileName_5 = filenames_5[i];  
		string fileFullName_5 = path_5 + fileName_5;  

		const char * filename_5=fileFullName_5.c_str(); 

		if( remove(filename_5) == 0 );   //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_5);
		else
			perror("remove");
	}
}

//===========左臂下处理end==============================

//===========右腿上处理start============================
IplImage *src_6;
float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_6;  
string path_6 = "E:\\数据输出\\1\\";  
string  exten_6 = "*_6.jpg";  

vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  

const int size_6 = filenames_6.size();
if(size_6!=0)
{
	for (int p = 0; p < size_6;p++)  
	{  

		string fileName_6 = filenames_6[p];  
		string fileFullName_6 = path_6 + fileName_6;   

		//=============================================================

		const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形

		src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_6->height;i++)
			for(int j=0;j<src_6->width;j++)
			{

				if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
			s_6[p]=per_p_6;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_6=s_6[0]; //假设第一个是最小值
	int index_6=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_6;i++)
	{

		if(s_6[i]<min_6)
		{
			min_6=s_6[i];
			index_6=i;
		}

	}
	//cout<<endl;
	string fileName_6 = filenames_6[index_6];  
	string fileFullName_6 = path_6 + fileName_6;

	const char * filename1_6=fileFullName_6.c_str();
	char cmd_6[255] = {0};

	sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-1);
	system(cmd_6);

	//=========删图=========================
	for(int i=0;i<size_6;i++)
	{

		string fileName_6 = filenames_6[i];  
		string fileFullName_6 = path_6 + fileName_6;  

		const char * filename_6=fileFullName_6.c_str(); 

		if( remove(filename_6) == 0 ) ;  //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_6);
		else
			perror("remove");
	}
}


//===========右腿上处理end==============================
//===========右腿下处理start============================
IplImage *src_7;
float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_7;  
string path_7 = "E:\\数据输出\\1\\";  
string  exten_7 = "*_7.jpg";  

vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  

const int size_7 = filenames_7.size();  
if(size_7!=0)
{
	for (int p = 0; p < size_7;p++)  
	{  

		string fileName_7 = filenames_7[p];  
		string fileFullName_7 = path_7 + fileName_7;   

		//=============================================================

		const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形

		src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_7->height;i++)
			for(int j=0;j<src_7->width;j++)
			{

				if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
			s_7[p]=per_p_7;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_7=s_7[0]; //假设第一个是最小值
	int index_7=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_7;i++)
	{

		if(s_7[i]<min_7)
		{
			min_7=s_7[i];
			index_7=i;
		}

	}
	//cout<<endl;
	string fileName_7 = filenames_7[index_7];  
	string fileFullName_7 = path_7 + fileName_7;

	const char * filename1_7=fileFullName_7.c_str();
	char cmd_7[255] = {0};

	sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-1);
	system(cmd_7);

	//=========删图=========================
	for(int i=0;i<size_7;i++)
	{

		string fileName_7 = filenames_7[i];  
		string fileFullName_7 = path_7 + fileName_7;  

		const char * filename_7=fileFullName_7.c_str(); 

		if( remove(filename_7) == 0 ) ;  //remove返回0表示删除成功。		
		//	printf("Removed %s.\n", filename_7);
		else
			perror("remove");
	}
}


//===========右腿下处理end==============================
//===========左腿上处理start============================

IplImage *src_8;
float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_8;  
string path_8 = "E:\\数据输出\\1\\";  
string  exten_8 = "*_8.jpg";  

vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  

const int size_8 = filenames_8.size();  
if(size_8!=0)
{
	for (int p = 0; p < size_8;p++)  
	{  

		string fileName_8 = filenames_8[p];  
		string fileFullName_8 = path_8 + fileName_8;   

		//=============================================================

		const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形

		src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_8->height;i++)
			for(int j=0;j<src_8->width;j++)
			{

				if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
			s_8[p]=per_p_8;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_8=s_8[0]; //假设第一个是最小值
	int index_8=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_8;i++)
	{

		if(s_8[i]<min_8)
		{
			min_8=s_8[i];
			index_8=i;
		}

	}
	//cout<<endl;
	string fileName_8 = filenames_8[index_8];  
	string fileFullName_8 = path_8 + fileName_8;

	const char * filename1_8=fileFullName_8.c_str();
	char cmd_8[255] = {0};

	sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-1);
	system(cmd_8);

	//=========删图=========================
	for(int i=0;i<size_8;i++)
	{

		string fileName_8 = filenames_8[i];  
		string fileFullName_8 = path_8 + fileName_8;  

		const char * filename_8=fileFullName_8.c_str(); 

		if( remove(filename_8) == 0 ) ;  //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_8);
		else
			perror("remove");
	}
}




//===========左腿上处理end==============================

//===========左腿下处理start============================
IplImage *src_9;
float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

//=================找指定路径下的某种标号的图片================
Directory dir_9;  
string path_9 = "E:\\数据输出\\1\\";  
string  exten_9 = "*_9.jpg";  

vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  

const int size_9 = filenames_9.size();  
if(size_9!=0)
{
	for (int p = 0; p < size_9;p++)  
	{  

		string fileName_9 = filenames_9[p];  
		string fileFullName_9 = path_9 + fileName_9;   

		//=============================================================

		const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形

		src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图


		int n=0;  //统计白色像素的个数  
		for(int i=0;i<src_9->height;i++)
			for(int j=0;j<src_9->width;j++)
			{

				if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
					n++; 

			}
			float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
			s_9[p]=per_p_9;	  //把多副图的占比值写入数组中

	}
	//==========找出最小的占比值======================
	float min_9=s_9[0]; //假设第一个是最小值
	int index_9=0;   //用于记录数组找到的最小值的下标

	for(int i=0;i<size_9;i++)
	{

		if(s_9[i]<min_9)
		{
			min_9=s_9[i];
			index_9=i;
		}

	}
	//cout<<endl;
	string fileName_9 = filenames_9[index_9];  
	string fileFullName_9 = path_9 + fileName_9;

	const char * filename1_9=fileFullName_9.c_str();
	char cmd_9[255] = {0};

	sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-1);
	system(cmd_9);

	//=========删图=========================
	for(int i=0;i<size_9;i++)
	{

		string fileName_9 = filenames_9[i];  
		string fileFullName_9 = path_9 + fileName_9;  

		const char * filename_9=fileFullName_9.c_str(); 

		if( remove(filename_9) == 0 ) ;  //remove返回0表示删除成功。		
			//printf("Removed %s.\n", filename_9);
		else
			perror("remove");
	}
}
//===========左腿下处理end==============================
}




void func_2()
{
	cout<<"索引号为 2 筛选处理  /"<<"  保存序号 "<<order-1<<endl<<endl;
	//=================全身start================================
	IplImage *src_0;
	float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_0;  
	string path_0 = "E:\\数据输出\\2\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
				s_0[p]=per_p_0;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_0=s_0[0]; //假设第一个是最小值
		int index_0=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_0;i++)
		{

			if(s_0[i]<min_0)
			{
				min_0=s_0[i];
				index_0=i;
			}

		}
		//cout<<endl;
		string fileName_0 = filenames_0[index_0];  
		string fileFullName_0 = path_0 + fileName_0;
		//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========删图=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================全身end================================

	//=================躯干start================================
	IplImage *src_1;
	float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_1;  
	string path_1 = "E:\\数据输出\\2\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
				s_1[p]=per_p_1;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_1=s_1[0]; //假设第一个是最小值
		int index_1=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_1;i++)
		{

			if(s_1[i]<min_1)
			{
				min_1=s_1[i];
				index_1=i;
			}

		}
		//cout<<endl;
		string fileName_1 = filenames_1[index_1];  
		string fileFullName_1 = path_1 + fileName_1;
		//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========删图=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 )  ; //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================躯干end================================

	//===========右臂上处理start============================

	IplImage *src_2;
	float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_2;  
	string path_2 = "E:\\数据输出\\2\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
				s_2[p]=per_p_2;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_2=s_2[0]; //假设第一个是最小值
		int index_2=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_2;i++)
		{

			if(s_2[i]<min_2)
			{
				min_2=s_2[i];
				index_2=i;
			}

		}
		//cout<<endl;
		string fileName_2 = filenames_2[index_2];  
		string fileFullName_2 = path_2 + fileName_2;
		//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========删图=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========右臂上处理end==============================

	//===========右臂下处理start============================
	IplImage *src_3;
	float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_3;  
	string path_3 = "E:\\数据输出\\2\\";  
	string  exten_3 = "*_3.jpg";  

	vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  

	const int size_3 = filenames_3.size(); 
	if(size_3!=0)
	{
		for (int p = 0; p < size_3;p++)  
		{  

			string fileName_3 = filenames_3[p];  
			string fileFullName_3 = path_3 + fileName_3;   

			//=============================================================

			const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形

			src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
				s_3[p]=per_p_3;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_3=s_3[0]; //假设第一个是最小值
		int index_3=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_3;i++)
		{

			if(s_3[i]<min_3)
			{
				min_3=s_3[i];
				index_3=i;
			}

		}
		//cout<<endl;
		string fileName_3 = filenames_3[index_3];  
		string fileFullName_3 = path_3 + fileName_3;

		const char * filename1_3=fileFullName_3.c_str();
		char cmd_3[255] = {0};

		sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========删图=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========右臂下处理end==============================
	//===========左臂上处理start============================
	IplImage *src_4;
	float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_4;  
	string path_4 = "E:\\数据输出\\2\\";  
	string  exten_4 = "*_4.jpg";  

	vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  

	const int size_4 = filenames_4.size(); 
	if(size_4!=0)
	{
		for (int p = 0; p < size_4;p++)  
		{  

			string fileName_4 = filenames_4[p];  
			string fileFullName_4 = path_4 + fileName_4;   

			//=============================================================

			const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形

			src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
				s_4[p]=per_p_4;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_4=s_4[0]; //假设第一个是最小值
		int index_4=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_4;i++)
		{

			if(s_4[i]<min_4)
			{
				min_4=s_4[i];
				index_4=i;
			}

		}
		//cout<<endl;
		string fileName_4 = filenames_4[index_4];  
		string fileFullName_4 = path_4 + fileName_4;

		const char * filename1_4=fileFullName_4.c_str();
		char cmd_4[255] = {0};

		sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========删图=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========左臂上处理end==============================
	//===========左臂下处理start============================
	IplImage *src_5;
	float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_5;  
	string path_5 = "E:\\数据输出\\2\\";  
	string  exten_5 = "*_5.jpg";  

	vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  

	const int size_5 = filenames_5.size(); 
	if(size_5!=0)
	{
		for (int p = 0; p < size_5;p++)  
		{  

			string fileName_5 = filenames_5[p];  
			string fileFullName_5 = path_5 + fileName_5;   

			//=============================================================

			const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形

			src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
				s_5[p]=per_p_5;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_5=s_5[0]; //假设第一个是最小值
		int index_5=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_5;i++)
		{

			if(s_5[i]<min_5)
			{
				min_5=s_5[i];
				index_5=i;
			}

		}
		//cout<<endl;
		string fileName_5 = filenames_5[index_5];  
		string fileFullName_5 = path_5 + fileName_5;

		const char * filename1_5=fileFullName_5.c_str();
		char cmd_5[255] = {0};

		sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========删图=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========左臂下处理end==============================

	//===========右腿上处理start============================
	IplImage *src_6;
	float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_6;  
	string path_6 = "E:\\数据输出\\2\\";  
	string  exten_6 = "*_6.jpg";  

	vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  

	const int size_6 = filenames_6.size();
	if(size_6!=0)
	{
		for (int p = 0; p < size_6;p++)  
		{  

			string fileName_6 = filenames_6[p];  
			string fileFullName_6 = path_6 + fileName_6;   

			//=============================================================

			const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形

			src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
				s_6[p]=per_p_6;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_6=s_6[0]; //假设第一个是最小值
		int index_6=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_6;i++)
		{

			if(s_6[i]<min_6)
			{
				min_6=s_6[i];
				index_6=i;
			}

		}
		//cout<<endl;
		string fileName_6 = filenames_6[index_6];  
		string fileFullName_6 = path_6 + fileName_6;

		const char * filename1_6=fileFullName_6.c_str();
		char cmd_6[255] = {0};

		sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========删图=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========右腿上处理end==============================
	//===========右腿下处理start============================
	IplImage *src_7;
	float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_7;  
	string path_7 = "E:\\数据输出\\2\\";  
	string  exten_7 = "*_7.jpg";  

	vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  

	const int size_7 = filenames_7.size();  
	if(size_7!=0)
	{
		for (int p = 0; p < size_7;p++)  
		{  

			string fileName_7 = filenames_7[p];  
			string fileFullName_7 = path_7 + fileName_7;   

			//=============================================================

			const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形

			src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
				s_7[p]=per_p_7;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_7=s_7[0]; //假设第一个是最小值
		int index_7=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_7;i++)
		{

			if(s_7[i]<min_7)
			{
				min_7=s_7[i];
				index_7=i;
			}

		}
		//cout<<endl;
		string fileName_7 = filenames_7[index_7];  
		string fileFullName_7 = path_7 + fileName_7;

		const char * filename1_7=fileFullName_7.c_str();
		char cmd_7[255] = {0};

		sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========删图=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========右腿下处理end==============================
	//===========左腿上处理start============================

	IplImage *src_8;
	float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_8;  
	string path_8 = "E:\\数据输出\\2\\";  
	string  exten_8 = "*_8.jpg";  

	vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  

	const int size_8 = filenames_8.size();  
	if(size_8!=0)
	{
		for (int p = 0; p < size_8;p++)  
		{  

			string fileName_8 = filenames_8[p];  
			string fileFullName_8 = path_8 + fileName_8;   

			//=============================================================

			const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形

			src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
				s_8[p]=per_p_8;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_8=s_8[0]; //假设第一个是最小值
		int index_8=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_8;i++)
		{

			if(s_8[i]<min_8)
			{
				min_8=s_8[i];
				index_8=i;
			}

		}
		//cout<<endl;
		string fileName_8 = filenames_8[index_8];  
		string fileFullName_8 = path_8 + fileName_8;

		const char * filename1_8=fileFullName_8.c_str();
		char cmd_8[255] = {0};

		sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========删图=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 ) ;  //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========左腿上处理end==============================

	//===========左腿下处理start============================
	IplImage *src_9;
	float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_9;  
	string path_9 = "E:\\数据输出\\2\\";  
	string  exten_9 = "*_9.jpg";  

	vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  

	const int size_9 = filenames_9.size();  
	if(size_9!=0)
	{
		for (int p = 0; p < size_9;p++)  
		{  

			string fileName_9 = filenames_9[p];  
			string fileFullName_9 = path_9 + fileName_9;   

			//=============================================================

			const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形

			src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
				s_9[p]=per_p_9;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_9=s_9[0]; //假设第一个是最小值
		int index_9=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_9;i++)
		{

			if(s_9[i]<min_9)
			{
				min_9=s_9[i];
				index_9=i;
			}

		}
		//cout<<endl;
		string fileName_9 = filenames_9[index_9];  
		string fileFullName_9 = path_9 + fileName_9;

		const char * filename1_9=fileFullName_9.c_str();
		char cmd_9[255] = {0};

		sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========删图=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========左腿下处理end==============================
}



void func_3()
{
	cout<<"索引号为 3 筛选处理  /"<<"  保存序号 "<<order-1<<endl<<endl;
	//=================全身start================================
	IplImage *src_0;
	float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_0;  
	string path_0 = "E:\\数据输出\\3\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
				s_0[p]=per_p_0;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_0=s_0[0]; //假设第一个是最小值
		int index_0=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_0;i++)
		{

			if(s_0[i]<min_0)
			{
				min_0=s_0[i];
				index_0=i;
			}

		}
		//cout<<endl;
		string fileName_0 = filenames_0[index_0];  
		string fileFullName_0 = path_0 + fileName_0;
		//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========删图=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================全身end================================

	//=================躯干start================================
	IplImage *src_1;
	float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_1;  
	string path_1 = "E:\\数据输出\\3\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
				s_1[p]=per_p_1;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_1=s_1[0]; //假设第一个是最小值
		int index_1=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_1;i++)
		{

			if(s_1[i]<min_1)
			{
				min_1=s_1[i];
				index_1=i;
			}

		}
		//cout<<endl;
		string fileName_1 = filenames_1[index_1];  
		string fileFullName_1 = path_1 + fileName_1;
		//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========删图=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================躯干end================================

	//===========右臂上处理start============================

	IplImage *src_2;
	float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_2;  
	string path_2 = "E:\\数据输出\\3\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
				s_2[p]=per_p_2;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_2=s_2[0]; //假设第一个是最小值
		int index_2=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_2;i++)
		{

			if(s_2[i]<min_2)
			{
				min_2=s_2[i];
				index_2=i;
			}

		}
		//cout<<endl;
		string fileName_2 = filenames_2[index_2];  
		string fileFullName_2 = path_2 + fileName_2;
		//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========删图=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========右臂上处理end==============================

	//===========右臂下处理start============================
	IplImage *src_3;
	float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_3;  
	string path_3 = "E:\\数据输出\\3\\";  
	string  exten_3 = "*_3.jpg";  

	vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  

	const int size_3 = filenames_3.size(); 
	if(size_3!=0)
	{
		for (int p = 0; p < size_3;p++)  
		{  

			string fileName_3 = filenames_3[p];  
			string fileFullName_3 = path_3 + fileName_3;   

			//=============================================================

			const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形

			src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
				s_3[p]=per_p_3;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_3=s_3[0]; //假设第一个是最小值
		int index_3=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_3;i++)
		{

			if(s_3[i]<min_3)
			{
				min_3=s_3[i];
				index_3=i;
			}

		}
		//cout<<endl;
		string fileName_3 = filenames_3[index_3];  
		string fileFullName_3 = path_3 + fileName_3;

		const char * filename1_3=fileFullName_3.c_str();
		char cmd_3[255] = {0};

		sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========删图=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 ) ;  //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========右臂下处理end==============================
	//===========左臂上处理start============================
	IplImage *src_4;
	float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_4;  
	string path_4 = "E:\\数据输出\\3\\";  
	string  exten_4 = "*_4.jpg";  

	vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  

	const int size_4 = filenames_4.size(); 
	if(size_4!=0)
	{
		for (int p = 0; p < size_4;p++)  
		{  

			string fileName_4 = filenames_4[p];  
			string fileFullName_4 = path_4 + fileName_4;   

			//=============================================================

			const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形

			src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
				s_4[p]=per_p_4;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_4=s_4[0]; //假设第一个是最小值
		int index_4=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_4;i++)
		{

			if(s_4[i]<min_4)
			{
				min_4=s_4[i];
				index_4=i;
			}

		}
		//cout<<endl;
		string fileName_4 = filenames_4[index_4];  
		string fileFullName_4 = path_4 + fileName_4;

		const char * filename1_4=fileFullName_4.c_str();
		char cmd_4[255] = {0};

		sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========删图=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========左臂上处理end==============================
	//===========左臂下处理start============================
	IplImage *src_5;
	float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_5;  
	string path_5 = "E:\\数据输出\\3\\";  
	string  exten_5 = "*_5.jpg";  

	vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  

	const int size_5 = filenames_5.size(); 
	if(size_5!=0)
	{
		for (int p = 0; p < size_5;p++)  
		{  

			string fileName_5 = filenames_5[p];  
			string fileFullName_5 = path_5 + fileName_5;   

			//=============================================================

			const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形

			src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
				s_5[p]=per_p_5;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_5=s_5[0]; //假设第一个是最小值
		int index_5=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_5;i++)
		{

			if(s_5[i]<min_5)
			{
				min_5=s_5[i];
				index_5=i;
			}

		}
		//cout<<endl;
		string fileName_5 = filenames_5[index_5];  
		string fileFullName_5 = path_5 + fileName_5;

		const char * filename1_5=fileFullName_5.c_str();
		char cmd_5[255] = {0};

		sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========删图=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========左臂下处理end==============================

	//===========右腿上处理start============================
	IplImage *src_6;
	float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_6;  
	string path_6 = "E:\\数据输出\\3\\";  
	string  exten_6 = "*_6.jpg";  

	vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  

	const int size_6 = filenames_6.size();
	if(size_6!=0)
	{
		for (int p = 0; p < size_6;p++)  
		{  

			string fileName_6 = filenames_6[p];  
			string fileFullName_6 = path_6 + fileName_6;   

			//=============================================================

			const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形

			src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
				s_6[p]=per_p_6;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_6=s_6[0]; //假设第一个是最小值
		int index_6=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_6;i++)
		{

			if(s_6[i]<min_6)
			{
				min_6=s_6[i];
				index_6=i;
			}

		}
		//cout<<endl;
		string fileName_6 = filenames_6[index_6];  
		string fileFullName_6 = path_6 + fileName_6;

		const char * filename1_6=fileFullName_6.c_str();
		char cmd_6[255] = {0};

		sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========删图=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 ) ;  //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========右腿上处理end==============================
	//===========右腿下处理start============================
	IplImage *src_7;
	float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_7;  
	string path_7 = "E:\\数据输出\\3\\";  
	string  exten_7 = "*_7.jpg";  

	vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  

	const int size_7 = filenames_7.size();  
	if(size_7!=0)
	{
		for (int p = 0; p < size_7;p++)  
		{  

			string fileName_7 = filenames_7[p];  
			string fileFullName_7 = path_7 + fileName_7;   

			//=============================================================

			const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形

			src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
				s_7[p]=per_p_7;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_7=s_7[0]; //假设第一个是最小值
		int index_7=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_7;i++)
		{

			if(s_7[i]<min_7)
			{
				min_7=s_7[i];
				index_7=i;
			}

		}
		//cout<<endl;
		string fileName_7 = filenames_7[index_7];  
		string fileFullName_7 = path_7 + fileName_7;

		const char * filename1_7=fileFullName_7.c_str();
		char cmd_7[255] = {0};

		sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========删图=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========右腿下处理end==============================
	//===========左腿上处理start============================

	IplImage *src_8;
	float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_8;  
	string path_8 = "E:\\数据输出\\3\\";  
	string  exten_8 = "*_8.jpg";  

	vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  

	const int size_8 = filenames_8.size();  
	if(size_8!=0)
	{
		for (int p = 0; p < size_8;p++)  
		{  

			string fileName_8 = filenames_8[p];  
			string fileFullName_8 = path_8 + fileName_8;   

			//=============================================================

			const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形

			src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
				s_8[p]=per_p_8;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_8=s_8[0]; //假设第一个是最小值
		int index_8=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_8;i++)
		{

			if(s_8[i]<min_8)
			{
				min_8=s_8[i];
				index_8=i;
			}

		}
	//	cout<<endl;
		string fileName_8 = filenames_8[index_8];  
		string fileFullName_8 = path_8 + fileName_8;

		const char * filename1_8=fileFullName_8.c_str();
		char cmd_8[255] = {0};

		sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========删图=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 );   //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========左腿上处理end==============================

	//===========左腿下处理start============================
	IplImage *src_9;
	float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_9;  
	string path_9 = "E:\\数据输出\\3\\";  
	string  exten_9 = "*_9.jpg";  

	vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  

	const int size_9 = filenames_9.size();  
	if(size_9!=0)
	{
		for (int p = 0; p < size_9;p++)  
		{  

			string fileName_9 = filenames_9[p];  
			string fileFullName_9 = path_9 + fileName_9;   

			//=============================================================

			const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形

			src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
				s_9[p]=per_p_9;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_9=s_9[0]; //假设第一个是最小值
		int index_9=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_9;i++)
		{

			if(s_9[i]<min_9)
			{
				min_9=s_9[i];
				index_9=i;
			}

		}
		//cout<<endl;
		string fileName_9 = filenames_9[index_9];  
		string fileFullName_9 = path_9 + fileName_9;

		const char * filename1_9=fileFullName_9.c_str();
		char cmd_9[255] = {0};

		sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========删图=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========左腿下处理end==============================
}



void func_4()
{
	cout<<"索引号为 4 筛选处理  /"<<"  保存序号 "<<order-1<<endl<<endl;
	//=================全身start================================
	IplImage *src_0;
	float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_0;  
	string path_0 = "E:\\数据输出\\4\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
				s_0[p]=per_p_0;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_0=s_0[0]; //假设第一个是最小值
		int index_0=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_0;i++)
		{

			if(s_0[i]<min_0)
			{
				min_0=s_0[i];
				index_0=i;
			}

		}
		//cout<<endl;
		string fileName_0 = filenames_0[index_0];  
		string fileFullName_0 = path_0 + fileName_0;
		//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========删图=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================全身end================================

	//=================躯干start================================
	IplImage *src_1;
	float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_1;  
	string path_1 = "E:\\数据输出\\4\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
				s_1[p]=per_p_1;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_1=s_1[0]; //假设第一个是最小值
		int index_1=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_1;i++)
		{

			if(s_1[i]<min_1)
			{
				min_1=s_1[i];
				index_1=i;
			}

		}
		//cout<<endl;
		string fileName_1 = filenames_1[index_1];  
		string fileFullName_1 = path_1 + fileName_1;
		//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========删图=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================躯干end================================

	//===========右臂上处理start============================

	IplImage *src_2;
	float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_2;  
	string path_2 = "E:\\数据输出\\4\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
				s_2[p]=per_p_2;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_2=s_2[0]; //假设第一个是最小值
		int index_2=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_2;i++)
		{

			if(s_2[i]<min_2)
			{
				min_2=s_2[i];
				index_2=i;
			}

		}
		//cout<<endl;
		string fileName_2 = filenames_2[index_2];  
		string fileFullName_2 = path_2 + fileName_2;
		//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========删图=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========右臂上处理end==============================

	//===========右臂下处理start============================
	IplImage *src_3;
	float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_3;  
	string path_3 = "E:\\数据输出\\4\\";  
	string  exten_3 = "*_3.jpg";  

	vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  

	const int size_3 = filenames_3.size(); 
	if(size_3!=0)
	{
		for (int p = 0; p < size_3;p++)  
		{  

			string fileName_3 = filenames_3[p];  
			string fileFullName_3 = path_3 + fileName_3;   

			//=============================================================

			const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形

			src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
				s_3[p]=per_p_3;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_3=s_3[0]; //假设第一个是最小值
		int index_3=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_3;i++)
		{

			if(s_3[i]<min_3)
			{
				min_3=s_3[i];
				index_3=i;
			}

		}
		//cout<<endl;
		string fileName_3 = filenames_3[index_3];  
		string fileFullName_3 = path_3 + fileName_3;

		const char * filename1_3=fileFullName_3.c_str();
		char cmd_3[255] = {0};

		sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========删图=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 );   //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========右臂下处理end==============================
	//===========左臂上处理start============================
	IplImage *src_4;
	float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_4;  
	string path_4 = "E:\\数据输出\\4\\";  
	string  exten_4 = "*_4.jpg";  

	vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  

	const int size_4 = filenames_4.size(); 
	if(size_4!=0)
	{
		for (int p = 0; p < size_4;p++)  
		{  

			string fileName_4 = filenames_4[p];  
			string fileFullName_4 = path_4 + fileName_4;   

			//=============================================================

			const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形

			src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
				s_4[p]=per_p_4;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_4=s_4[0]; //假设第一个是最小值
		int index_4=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_4;i++)
		{

			if(s_4[i]<min_4)
			{
				min_4=s_4[i];
				index_4=i;
			}

		}
		//cout<<endl;
		string fileName_4 = filenames_4[index_4];  
		string fileFullName_4 = path_4 + fileName_4;

		const char * filename1_4=fileFullName_4.c_str();
		char cmd_4[255] = {0};

		sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========删图=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 )  ; //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========左臂上处理end==============================
	//===========左臂下处理start============================
	IplImage *src_5;
	float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_5;  
	string path_5 = "E:\\数据输出\\4\\";  
	string  exten_5 = "*_5.jpg";  

	vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  

	const int size_5 = filenames_5.size(); 
	if(size_5!=0)
	{
		for (int p = 0; p < size_5;p++)  
		{  

			string fileName_5 = filenames_5[p];  
			string fileFullName_5 = path_5 + fileName_5;   

			//=============================================================

			const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形

			src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
				s_5[p]=per_p_5;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_5=s_5[0]; //假设第一个是最小值
		int index_5=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_5;i++)
		{

			if(s_5[i]<min_5)
			{
				min_5=s_5[i];
				index_5=i;
			}

		}
		//cout<<endl;
		string fileName_5 = filenames_5[index_5];  
		string fileFullName_5 = path_5 + fileName_5;

		const char * filename1_5=fileFullName_5.c_str();
		char cmd_5[255] = {0};

		sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========删图=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========左臂下处理end==============================

	//===========右腿上处理start============================
	IplImage *src_6;
	float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_6;  
	string path_6 = "E:\\数据输出\\4\\";  
	string  exten_6 = "*_6.jpg";  

	vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  

	const int size_6 = filenames_6.size();
	if(size_6!=0)
	{
		for (int p = 0; p < size_6;p++)  
		{  

			string fileName_6 = filenames_6[p];  
			string fileFullName_6 = path_6 + fileName_6;   

			//=============================================================

			const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形

			src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
				s_6[p]=per_p_6;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_6=s_6[0]; //假设第一个是最小值
		int index_6=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_6;i++)
		{

			if(s_6[i]<min_6)
			{
				min_6=s_6[i];
				index_6=i;
			}

		}
		//cout<<endl;
		string fileName_6 = filenames_6[index_6];  
		string fileFullName_6 = path_6 + fileName_6;

		const char * filename1_6=fileFullName_6.c_str();
		char cmd_6[255] = {0};

		sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========删图=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========右腿上处理end==============================
	//===========右腿下处理start============================
	IplImage *src_7;
	float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_7;  
	string path_7 = "E:\\数据输出\\4\\";  
	string  exten_7 = "*_7.jpg";  

	vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  

	const int size_7 = filenames_7.size();  
	if(size_7!=0)
	{
		for (int p = 0; p < size_7;p++)  
		{  

			string fileName_7 = filenames_7[p];  
			string fileFullName_7 = path_7 + fileName_7;   

			//=============================================================

			const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形

			src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
				s_7[p]=per_p_7;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_7=s_7[0]; //假设第一个是最小值
		int index_7=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_7;i++)
		{

			if(s_7[i]<min_7)
			{
				min_7=s_7[i];
				index_7=i;
			}

		}
		//cout<<endl;
		string fileName_7 = filenames_7[index_7];  
		string fileFullName_7 = path_7 + fileName_7;

		const char * filename1_7=fileFullName_7.c_str();
		char cmd_7[255] = {0};

		sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========删图=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========右腿下处理end==============================
	//===========左腿上处理start============================

	IplImage *src_8;
	float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_8;  
	string path_8 = "E:\\数据输出\\4\\";  
	string  exten_8 = "*_8.jpg";  

	vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  

	const int size_8 = filenames_8.size();  
	if(size_8!=0)
	{
		for (int p = 0; p < size_8;p++)  
		{  

			string fileName_8 = filenames_8[p];  
			string fileFullName_8 = path_8 + fileName_8;   

			//=============================================================

			const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形

			src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
				s_8[p]=per_p_8;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_8=s_8[0]; //假设第一个是最小值
		int index_8=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_8;i++)
		{

			if(s_8[i]<min_8)
			{
				min_8=s_8[i];
				index_8=i;
			}

		}
		//cout<<endl;
		string fileName_8 = filenames_8[index_8];  
		string fileFullName_8 = path_8 + fileName_8;

		const char * filename1_8=fileFullName_8.c_str();
		char cmd_8[255] = {0};

		sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========删图=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========左腿上处理end==============================

	//===========左腿下处理start============================
	IplImage *src_9;
	float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_9;  
	string path_9 = "E:\\数据输出\\4\\";  
	string  exten_9 = "*_9.jpg";  

	vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  

	const int size_9 = filenames_9.size();  
	if(size_9!=0)
	{
		for (int p = 0; p < size_9;p++)  
		{  

			string fileName_9 = filenames_9[p];  
			string fileFullName_9 = path_9 + fileName_9;   

			//=============================================================

			const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形

			src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
				s_9[p]=per_p_9;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_9=s_9[0]; //假设第一个是最小值
		int index_9=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_9;i++)
		{

			if(s_9[i]<min_9)
			{
				min_9=s_9[i];
				index_9=i;
			}

		}
		//cout<<endl;
		string fileName_9 = filenames_9[index_9];  
		string fileFullName_9 = path_9 + fileName_9;

		const char * filename1_9=fileFullName_9.c_str();
		char cmd_9[255] = {0};

		sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========删图=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove返回0表示删除成功。		
			//;	printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========左腿下处理end==============================
}


void func_5()
{
	cout<<"索引号为 5 筛选处理  /"<<"  保存序号 "<<order-1<<endl<<endl;
	//=================全身start================================
	IplImage *src_0;
	float s_0[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_0;  
	string path_0 = "E:\\数据输出\\5\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"找到 "<<size_0<<" 帧数据"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //计算白色区域在整幅图中的占比
				s_0[p]=per_p_0;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_0=s_0[0]; //假设第一个是最小值
		int index_0=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_0;i++)
		{

			if(s_0[i]<min_0)
			{
				min_0=s_0[i];
				index_0=i;
			}

		}
		//cout<<endl;
		string fileName_0 = filenames_0[index_0];  
		string fileFullName_0 = path_0 + fileName_0;
		//cout<<"白色像素最小占比： "<<min_0<<"  "<<"标号是： "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\处理后\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\处理后\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========删图=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_0,"E:\\数据输出\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================全身end================================

	//=================躯干start================================
	IplImage *src_1;
	float s_1[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_1;  
	string path_1 = "E:\\数据输出\\5\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"找到 "<<size_1<<" 帧数据"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //计算白色区域在整幅图中的占比
				s_1[p]=per_p_1;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_1=s_1[0]; //假设第一个是最小值
		int index_1=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_1;i++)
		{

			if(s_1[i]<min_1)
			{
				min_1=s_1[i];
				index_1=i;
			}

		}
		//cout<<endl;
		string fileName_1 = filenames_1[index_1];  
		string fileFullName_1 = path_1 + fileName_1;
		//cout<<"白色像素最小占比： "<<min_1<<"  "<<"标号是： "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\处理后\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\处理后\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========删图=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_1,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================躯干end================================

	//===========右臂上处理start============================

	IplImage *src_2;
	float s_2[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_2;  
	string path_2 = "E:\\数据输出\\5\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"找到 "<<size_2<<" 帧数据"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //把string类型的路径转换成char形

			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) 蓝色通道像素
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //计算白色区域在整幅图中的占比
				s_2[p]=per_p_2;	  //把多副图的占比值写入数组中

				//cout<<"白色像素个数： "<<n<<"	"<<"白色像素占比： "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========找出最小的占比值======================
		float min_2=s_2[0]; //假设第一个是最小值
		int index_2=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_2;i++)
		{

			if(s_2[i]<min_2)
			{
				min_2=s_2[i];
				index_2=i;
			}

		}
		//cout<<endl;
		string fileName_2 = filenames_2[index_2];  
		string fileFullName_2 = path_2 + fileName_2;
		//cout<<"白色像素最小占比： "<<min_2<<"  "<<"标号是： "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\处理后\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\处理后\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\处理后\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========删图=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //除了需要的标号的那个图以外，全部都删除
			//{
			//sprintf(filename_2,"E:\\数据输出\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========右臂上处理end==============================

	//===========右臂下处理start============================
	IplImage *src_3;
	float s_3[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_3;  
	string path_3 = "E:\\数据输出\\5\\";  
	string  exten_3 = "*_3.jpg";  

	vector<string> filenames_3 = dir_3.GetListFiles(path_3, exten_3, false);  

	const int size_3 = filenames_3.size(); 
	if(size_3!=0)
	{
		for (int p = 0; p < size_3;p++)  
		{  

			string fileName_3 = filenames_3[p];  
			string fileFullName_3 = path_3 + fileName_3;   

			//=============================================================

			const char * filename_3=fileFullName_3.c_str();  //把string类型的路径转换成char形

			src_3=cvLoadImage (filename_3, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //计算白色区域在整幅图中的占比
				s_3[p]=per_p_3;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_3=s_3[0]; //假设第一个是最小值
		int index_3=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_3;i++)
		{

			if(s_3[i]<min_3)
			{
				min_3=s_3[i];
				index_3=i;
			}

		}
		//cout<<endl;
		string fileName_3 = filenames_3[index_3];  
		string fileFullName_3 = path_3 + fileName_3;

		const char * filename1_3=fileFullName_3.c_str();
		char cmd_3[255] = {0};

		sprintf(cmd_3, "copy %s E:\\处理后\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========删图=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========右臂下处理end==============================
	//===========左臂上处理start============================
	IplImage *src_4;
	float s_4[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_4;  
	string path_4 = "E:\\数据输出\\5\\";  
	string  exten_4 = "*_4.jpg";  

	vector<string> filenames_4 = dir_4.GetListFiles(path_4, exten_4, false);  

	const int size_4 = filenames_4.size(); 
	if(size_4!=0)
	{
		for (int p = 0; p < size_4;p++)  
		{  

			string fileName_4 = filenames_4[p];  
			string fileFullName_4 = path_4 + fileName_4;   

			//=============================================================

			const char * filename_4=fileFullName_4.c_str();  //把string类型的路径转换成char形

			src_4=cvLoadImage (filename_4, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //计算白色区域在整幅图中的占比
				s_4[p]=per_p_4;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_4=s_4[0]; //假设第一个是最小值
		int index_4=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_4;i++)
		{

			if(s_4[i]<min_4)
			{
				min_4=s_4[i];
				index_4=i;
			}

		}
		//cout<<endl;
		string fileName_4 = filenames_4[index_4];  
		string fileFullName_4 = path_4 + fileName_4;

		const char * filename1_4=fileFullName_4.c_str();
		char cmd_4[255] = {0};

		sprintf(cmd_4, "copy %s E:\\处理后\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========删图=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 );   //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========左臂上处理end==============================
	//===========左臂下处理start============================
	IplImage *src_5;
	float s_5[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_5;  
	string path_5 = "E:\\数据输出\\5\\";  
	string  exten_5 = "*_5.jpg";  

	vector<string> filenames_5 = dir_5.GetListFiles(path_5, exten_5, false);  

	const int size_5 = filenames_5.size(); 
	if(size_5!=0)
	{
		for (int p = 0; p < size_5;p++)  
		{  

			string fileName_5 = filenames_5[p];  
			string fileFullName_5 = path_5 + fileName_5;   

			//=============================================================

			const char * filename_5=fileFullName_5.c_str();  //把string类型的路径转换成char形

			src_5=cvLoadImage (filename_5, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //计算白色区域在整幅图中的占比
				s_5[p]=per_p_5;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_5=s_5[0]; //假设第一个是最小值
		int index_5=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_5;i++)
		{

			if(s_5[i]<min_5)
			{
				min_5=s_5[i];
				index_5=i;
			}

		}
		//cout<<endl;
		string fileName_5 = filenames_5[index_5];  
		string fileFullName_5 = path_5 + fileName_5;

		const char * filename1_5=fileFullName_5.c_str();
		char cmd_5[255] = {0};

		sprintf(cmd_5, "copy %s E:\\处理后\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========删图=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 ) ;  //remove返回0表示删除成功。		
			//	printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========左臂下处理end==============================

	//===========右腿上处理start============================
	IplImage *src_6;
	float s_6[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_6;  
	string path_6 = "E:\\数据输出\\5\\";  
	string  exten_6 = "*_6.jpg";  

	vector<string> filenames_6 = dir_6.GetListFiles(path_6, exten_6, false);  

	const int size_6 = filenames_6.size();
	if(size_6!=0)
	{
		for (int p = 0; p < size_6;p++)  
		{  

			string fileName_6 = filenames_6[p];  
			string fileFullName_6 = path_6 + fileName_6;   

			//=============================================================

			const char * filename_6=fileFullName_6.c_str();  //把string类型的路径转换成char形

			src_6=cvLoadImage (filename_6, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //计算白色区域在整幅图中的占比
				s_6[p]=per_p_6;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_6=s_6[0]; //假设第一个是最小值
		int index_6=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_6;i++)
		{

			if(s_6[i]<min_6)
			{
				min_6=s_6[i];
				index_6=i;
			}

		}
		//cout<<endl;
		string fileName_6 = filenames_6[index_6];  
		string fileFullName_6 = path_6 + fileName_6;

		const char * filename1_6=fileFullName_6.c_str();
		char cmd_6[255] = {0};

		sprintf(cmd_6, "copy %s E:\\处理后\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========删图=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========右腿上处理end==============================
	//===========右腿下处理start============================
	IplImage *src_7;
	float s_7[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_7;  
	string path_7 = "E:\\数据输出\\5\\";  
	string  exten_7 = "*_7.jpg";  

	vector<string> filenames_7 = dir_7.GetListFiles(path_7, exten_7, false);  

	const int size_7 = filenames_7.size();  
	if(size_7!=0)
	{
		for (int p = 0; p < size_7;p++)  
		{  

			string fileName_7 = filenames_7[p];  
			string fileFullName_7 = path_7 + fileName_7;   

			//=============================================================

			const char * filename_7=fileFullName_7.c_str();  //把string类型的路径转换成char形

			src_7=cvLoadImage (filename_7, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //计算白色区域在整幅图中的占比
				s_7[p]=per_p_7;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_7=s_7[0]; //假设第一个是最小值
		int index_7=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_7;i++)
		{

			if(s_7[i]<min_7)
			{
				min_7=s_7[i];
				index_7=i;
			}

		}
		//cout<<endl;
		string fileName_7 = filenames_7[index_7];  
		string fileFullName_7 = path_7 + fileName_7;

		const char * filename1_7=fileFullName_7.c_str();
		char cmd_7[255] = {0};

		sprintf(cmd_7, "copy %s E:\\处理后\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========删图=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========右腿下处理end==============================
	//===========左腿上处理start============================

	IplImage *src_8;
	float s_8[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_8;  
	string path_8 = "E:\\数据输出\\5\\";  
	string  exten_8 = "*_8.jpg";  

	vector<string> filenames_8 = dir_8.GetListFiles(path_8, exten_8, false);  

	const int size_8 = filenames_8.size();  
	if(size_8!=0)
	{
		for (int p = 0; p < size_8;p++)  
		{  

			string fileName_8 = filenames_8[p];  
			string fileFullName_8 = path_8 + fileName_8;   

			//=============================================================

			const char * filename_8=fileFullName_8.c_str();  //把string类型的路径转换成char形

			src_8=cvLoadImage (filename_8, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //计算白色区域在整幅图中的占比
				s_8[p]=per_p_8;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_8=s_8[0]; //假设第一个是最小值
		int index_8=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_8;i++)
		{

			if(s_8[i]<min_8)
			{
				min_8=s_8[i];
				index_8=i;
			}

		}
		//cout<<endl;
		string fileName_8 = filenames_8[index_8];  
		string fileFullName_8 = path_8 + fileName_8;

		const char * filename1_8=fileFullName_8.c_str();
		char cmd_8[255] = {0};

		sprintf(cmd_8, "copy %s E:\\处理后\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========删图=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 );   //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========左腿上处理end==============================

	//===========左腿下处理start============================
	IplImage *src_9;
	float s_9[100];   //创建一个数组，用于对每一副图像的白色区域占比值保存下来,用100已结足够大了一个部位应该不会拍100帧

	//=================找指定路径下的某种标号的图片================
	Directory dir_9;  
	string path_9 = "E:\\数据输出\\5\\";  
	string  exten_9 = "*_9.jpg";  

	vector<string> filenames_9 = dir_9.GetListFiles(path_9, exten_9, false);  

	const int size_9 = filenames_9.size();  
	if(size_9!=0)
	{
		for (int p = 0; p < size_9;p++)  
		{  

			string fileName_9 = filenames_9[p];  
			string fileFullName_9 = path_9 + fileName_9;   

			//=============================================================

			const char * filename_9=fileFullName_9.c_str();  //把string类型的路径转换成char形

			src_9=cvLoadImage (filename_9, 0);   //直接转换成灰度图


			int n=0;  //统计白色像素的个数  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //计算灰度图中白色像素的个数
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //计算白色区域在整幅图中的占比
				s_9[p]=per_p_9;	  //把多副图的占比值写入数组中

		}
		//==========找出最小的占比值======================
		float min_9=s_9[0]; //假设第一个是最小值
		int index_9=0;   //用于记录数组找到的最小值的下标

		for(int i=0;i<size_9;i++)
		{

			if(s_9[i]<min_9)
			{
				min_9=s_9[i];
				index_9=i;
			}

		}
		//cout<<endl;
		string fileName_9 = filenames_9[index_9];  
		string fileFullName_9 = path_9 + fileName_9;

		const char * filename1_9=fileFullName_9.c_str();
		char cmd_9[255] = {0};

		sprintf(cmd_9, "copy %s E:\\处理后\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========删图=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove返回0表示删除成功。		
				//printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========左腿下处理end==============================
}
