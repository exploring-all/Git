#include<stdio.h>
#include<conio.h>
#include<opencv2/opencv.hpp>
#include <iostream> 
#include <Kinect.h>
#include<cmath>

#define BODY_COUNT 6


#define string_dst0 "E://�������//0//"
#define string_dst1 "E://�������//1//"
#define string_dst2 "E://�������//2//"
#define string_dst3 "E://�������//3//"
#define string_dst4 "E://�������//4//"
#define string_dst5 "E://�������//5//"

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

	vector<DepthSpacePoint>depthspacepoints(colorWidth*colorHeight);  //vector������DepthSpacePoint�������͵�����,������depthspacepoints��С���������ǲ���
	Mat img(colorHeight, colorWidth, CV_8UC4);  //��ɫͼ�洢��������
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

		hResult = pBodyIndexReader->AcquireLatestFrame(&pBodyIndexFrame);//Frame�������洢���ݵ��࣬ÿһ�ζ���Reader���������ݴ���Frame��
		if (SUCCEEDED(hResult))
		{
			hResult = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);  //��Ҫ���ǰ�Frame�е�����ת�浽һ��������  pBodyIndexBuffer����һ��424*512��С��16λunsigned int���飬�����洢�������
		}

		if (SUCCEEDED(hResult))
		{
			IDepthFrame* pDepthFrame = nullptr;
			UINT nDepthBufferSize = 0;
			UINT16 *pDepthBuffer = NULL;

			hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);//��ȡ����������֡
			if (SUCCEEDED(hResult))
			{
				hResult = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);  //�õ�һ�����֡���ݵĵ�
			}

			if (SUCCEEDED(hResult))
			{
				IColorFrame  *pColorFrame = nullptr;
				UINT nColorBufferSize = 0;
				RGBQUAD *pColorBuffer = NULL;    //RGBQUAD��һ���ṹ�壬�䱣��һ�����ص��RGBֵ         
				hResult = pColorReader->AcquireLatestFrame(&pColorFrame);  //��ȡ�����Ĳ�ɫ֡
				if (SUCCEEDED(hResult))
				{
					hResult = pColorFrame->CopyConvertedFrameDataToArray(colorHeight*colorWidth * 4, reinterpret_cast<BYTE*>(img.data), ColorImageFormat::ColorImageFormat_Bgra);  //��ɫ֡����ת���Ҫ��ĸ�ʽ��
					if (SUCCEEDED(hResult))
					{
						hResult = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));  //�õ���ɫ֡���ݵĵ�
					}
				}

				//--------------������Ϣ���ɫͼӳ��start--------------------------
				IBodyFrame* pBodyFrame = nullptr;
				hResult = pBodyReader->AcquireLatestFrame( &pBodyFrame );
				if( SUCCEEDED( hResult ) )
				{
					IBody* pBody[BODY_COUNT] = { 0 }; //Ĭ�ϵ��� 6 ������ ����ʼ�����еĹ�����Ϣ
					hResult = pBodyFrame->GetAndRefreshBodyData( BODY_COUNT, pBody );//���¹������ݣ�
					if( SUCCEEDED( hResult ) ){
						for( int count = 0; count < BODY_COUNT; count++ ){  //count����0��5��6�����ҹ�������ʾ
							BOOLEAN bTracked = false;    //��ʼ������׷�ٵ����塱��ֵΪ��
							hResult = pBody[count]->get_IsTracked( &bTracked );  //ȷ������׷�ٵ����塣
							if( SUCCEEDED( hResult ) && bTracked ){
								Joint joint[JointType::JointType_Count];   //ȡ������Joint(�ؽ�)��JointType��һ��ö�����ͣ���ͬλ�õĹؽڵ㶼�ǲ�ͬ�ı�ű�ʾ�ġ�count��һ����ֵ25��
								hResult = pBody[ count ]->GetJoints( JointType::JointType_Count, joint );  //ȡ������Joint(�ؽ�)��

								if( SUCCEEDED( hResult ) ){
									CvPoint skeletonPoint[BODY_COUNT][JointType_Count] = { cvPoint(0,0) };							
									// Joint
									for( int type = 0; type < JointType::JointType_Count; type++ ){  //����25���ؽڵ�
										ColorSpacePoint colorSpacePoint = { 0 };   //ColorSpacePoint�ṹ��������һ��colorSpacePointʵ�������� Represents a 2D point in color space, expressed in pixels.������һ��2d������ɫ�ռ���,�����ر�ʾ��
										pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );//����ͷ��ռ䵽��ɫ�ռ��ת�����ѹؽڵ������ת������ɫ�ռ��ϣ�������ʾ
									}
								}				
							}
						}

						//--------------������Ϣ���ɫͼӳ��end-------------------------
						if (SUCCEEDED(hResult))
						{
							pCoordinateMapper->MapColorFrameToDepthSpace(depthWidth * depthHeight, (UINT16*)pDepthBuffer, colorWidth * colorHeight, &depthspacepoints[0]);  //����ɫ�ռ䵽��ȿռ������ӳ�䣬�����任Ҳ��  depthspacepoints[0]�ǲ���������׵�ַ��Ӧ����
#pragma omp parallel for  //openmp����һ��Ԥ����Ķ���
							for (int i = 0; i < colorHeight; i++)
								for (int j = 0; j < colorWidth; j++)
								{
									int k = i*colorWidth + j;//ѭ������ÿһ������
									DepthSpacePoint p = depthspacepoints[k];  //��ƥ������ͼ�����ÿһ�����ء�  DepthSpacePoint��һ���ṹ�� p�ǽṹ�����͵ı���

									int depthX = (int)(p.X + 0.5f); //������ṹ�����ͱ���p���ýṹ���еĳ�Ա����������p��λk����,��Ϊk����Ϊ��һ�����εı�����������һ��Ĺ��ܾ��ǰѻ�ȡ����ÿһ���ض������ṹ�����͵ı���
									int depthY = (int)(p.Y + 0.5f);

									if ((depthX >= 0 && depthX < depthWidth) && (depthY >= 0 && depthY < depthHeight))
									{
										BYTE player = pBodyIndexBuffer[depthX + (depthY * depthWidth)];  //BYTE��unsigned char,�ѻ�ȡ����ÿһ�����ض�����player
										if (player == 0xff)  //��ȿռ�if���ж�ÿһ�������ǲ����������壬0~5�ֱ��ʾ��ʶ���6���ˣ�0xff��ʾ�����ز������ˣ���˵�������ڵ�7���ˣ���kinectʶ���ˣ����ԾͲ��Ѹ����ص���������ˣ�
										{ 
											img.at <Vec4b>(i,j) = Vec4b(255,255,255,255); //��ɫ�ռ䴦��0��0��0��ʾ��ɫ��255��ʾalpha͸����,����ԽСԽ͸��
										}//���ﲢûдһ���ж��ҵ�������ô�죬Ĭ�Ͼ��ǲ����д�����ô����ֱ����ʾ��
									}
									else  //���else�Ǻ����ϸ�ifƥ��ģ���ʾ����������ռ䣨�������壩֮��ĵط�Ҳ��Ҫ��Ϊ��ɫ
										img.at <Vec4b>(i, j) = Vec4b(255,255,255,255);
								}//��ɫ�ռ����˫��ѭ���������
						}  // pCoordinateMapper

						cv::resize( img, bodyMat, cv::Size(), 0.5, 0.5 );
						imshow("BODY", bodyMat);
						
						
						//=====================���������ȡstart=============================================

						pBodyFrame->GetAndRefreshBodyData(_countof(pBody), pBody); 
						for (int i = 0; i < BODY_COUNT; ++i)  
						{
							IBody* pBody1 = pBody[i];
							if (pBody1)   //���������˵���ţ�����������ж��ǲ��ǿ�ָ�롣
							{
								BOOLEAN bTracked = false;   //�Ƿ��ѱ�����
								hResult = pBody1->get_IsTracked(&bTracked);

								if (SUCCEEDED(hResult) && bTracked)  //����������µ����岢�����ܱ����ٵ�
								{

									cout<<"�����ţ� "<<i<<endl;
//================������ֵ��������start=============================
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
//===============������ֵ��������end================================

									Joint joints[JointType_Count];//�洢�ؽڵ��� Joint��һ���ṹ�����  JointType_Count=25

									//�洢�������ϵ�еĹؽڵ�λ��
									//DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[_countof(joints)];
									ColorSpacePoint *colorSpacePosition = new ColorSpacePoint[_countof(joints)];

									//��ùؽڵ���
									hResult = pBody1->GetJoints(_countof(joints), joints);
									if (SUCCEEDED(hResult))
									{
										for (int j = 0; j < _countof(joints); ++j)
										{
											//���ؽڵ���������������ϵ��-1~1��ת���������ϵ��424*512��
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
										

										//======���еĹ����ڵ�������start==================
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

			                           //=========���еĹ����ڵ�������end=================



//if(OK==1&&order!=1)   //����־λ��1�Ҳ��ǵ�һ������ʱ����ô����ͼ��ɸѡ����
										//{
										//	cout<<"ͼ������"<<endl;
										//	OK=0;
										//}

//============================ͼƬ����start========================
//										if(OK==1&&order!=1)   //����־λ��1�Ҳ��ǵ�һ������ʱ����ô����ͼ��ɸѡ����
//										{
//
//
////=================ȫ��start================================
//											IplImage *src_0;
//											float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_0;  
//											string path_0 = "E:\\�������\\";  
//											string  exten_0 = "*_0.jpg";  
//
//											vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  
//
//											const int size_0 = filenames_0.size();
//
//											if(size_0!=0)
//											{
//
//											//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
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
//												const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��
//
//												//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
//												//sprintf(windowname,"%d.jpg",p);
//												src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_0->height;i++)
//													for(int j=0;j<src_0->width;j++)
//													{
//														// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
//														if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
//													s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������
//
//													//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
//													//cvShowImage(windowname,src_0);
//													//waitKey(500);
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_0=s_0[0]; //�����һ������Сֵ
//											int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;
//
//
//											const char * filename1_0=fileFullName_0.c_str();
//											char cmd_0[255] = {0};
//											//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
//											//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
//											sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-2);
//											system(cmd_0);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_0;i++)
//											{
//												//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
//												//{
//												//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 
//
//
//												string fileName_0 = filenames_0[i];  
//												string fileFullName_0 = path_0 + fileName_0;  
//
//												const char * filename_0=fileFullName_0.c_str(); 
//
//												if( remove(filename_0) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_0);
//												else
//													perror("remove");
//												//}
//											}
//											}
////=================ȫ��end================================
//
//
//
//
//
//
//
////=================����start================================
//											IplImage *src_1;
//											float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_1;  
//											string path_1 = "E:\\�������\\";  
//											string  exten_1 = "*_1.jpg";  
//
//											vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  
//
//											const int size_1 = filenames_1.size();
//
//											if(size_1!=0)
//											{
//
//											//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
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
//												const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��
//
//												//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
//												//sprintf(windowname,"%d.jpg",p);
//												src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_1->height;i++)
//													for(int j=0;j<src_1->width;j++)
//													{
//														// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
//														if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
//													s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������
//
//													//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
//													//cvShowImage(windowname,src_1);
//													//waitKey(500);
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_1=s_1[0]; //�����һ������Сֵ
//											int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;
//
//
//											const char * filename1_1=fileFullName_1.c_str();
//											char cmd_1[255] = {0};
//											//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
//											//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
//											sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-2);
//											system(cmd_1);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_1;i++)
//											{
//												//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
//												//{
//												//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 
//
//
//												string fileName_1 = filenames_1[i];  
//												string fileFullName_1 = path_1 + fileName_1;  
//
//												const char * filename_1=fileFullName_1.c_str(); 
//
//												if( remove(filename_1) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_1);
//												else
//													perror("remove");
//												//}
//											}
//											}
////=================����end================================
//
////===========�ұ��ϴ���start============================
//
//                                            IplImage *src_2;
//											float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_2;  
//											string path_2 = "E:\\�������\\";  
//											string  exten_2 = "*_2.jpg";  
//
//											vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  
//
//											const int size_2 = filenames_2.size();     
//											//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
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
//												const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��
//
//												//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
//												//sprintf(windowname,"%d.jpg",p);
//												src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_2->height;i++)
//													for(int j=0;j<src_2->width;j++)
//													{
//														// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
//														if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
//													s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������
//
//													//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
//													//cvShowImage(windowname,src_2);
//													//waitKey(500);
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_2=s_2[0]; //�����һ������Сֵ
//											int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;
//
//
//											const char * filename1_2=fileFullName_2.c_str();
//											char cmd_2[255] = {0};
//											//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
//											//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
//											sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-2);
//											system(cmd_2);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_2;i++)
//											{
//												//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
//												//{
//												//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 
//
//
//												string fileName_2 = filenames_2[i];  
//												string fileFullName_2 = path_2 + fileName_2;  
//
//												const char * filename_2=fileFullName_2.c_str(); 
//
//												if( remove(filename_2) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_2);
//												else
//													perror("remove");
//												//}
//											}
//											}
////===========�ұ��ϴ���end==============================
//
////===========�ұ��´���start============================
//                                            IplImage *src_3;
//											float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_3;  
//											string path_3 = "E:\\�������\\";  
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
//												const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��
//
//												src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_3->height;i++)
//													for(int j=0;j<src_3->width;j++)
//													{
//														
//														if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
//													s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_3=s_3[0]; //�����һ������Сֵ
//											int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-2);
//											system(cmd_3);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_3;i++)
//											{
//												
//												string fileName_3 = filenames_3[i];  
//												string fileFullName_3 = path_3 + fileName_3;  
//
//												const char * filename_3=fileFullName_3.c_str(); 
//
//												if( remove(filename_3) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_3);
//												else
//													perror("remove");
//												//}
//											}
//											}
//
////===========�ұ��´���end==============================
////===========����ϴ���start============================
//                                            IplImage *src_4;
//											float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_4;  
//											string path_4 = "E:\\�������\\";  
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
//												const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��
//
//												src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_4->height;i++)
//													for(int j=0;j<src_4->width;j++)
//													{
//														
//														if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
//													s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_4=s_4[0]; //�����һ������Сֵ
//											int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-2);
//											system(cmd_4);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_4;i++)
//											{
//												
//												string fileName_4 = filenames_4[i];  
//												string fileFullName_4 = path_4 + fileName_4;  
//
//												const char * filename_4=fileFullName_4.c_str(); 
//
//												if( remove(filename_4) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_4);
//												else
//													perror("remove");
//												//}
//											}
//											}
//
////===========����ϴ���end==============================
////===========����´���start============================
//                                            IplImage *src_5;
//											float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_5;  
//											string path_5 = "E:\\�������\\";  
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
//												const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��
//
//												src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_5->height;i++)
//													for(int j=0;j<src_5->width;j++)
//													{
//														
//														if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
//													s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_5=s_5[0]; //�����һ������Сֵ
//											int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-2);
//											system(cmd_5);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_5;i++)
//											{
//												
//												string fileName_5 = filenames_5[i];  
//												string fileFullName_5 = path_5 + fileName_5;  
//
//												const char * filename_5=fileFullName_5.c_str(); 
//
//												if( remove(filename_5) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_5);
//												else
//													perror("remove");
//											}
//											}
//
////===========����´���end==============================
//
////===========�����ϴ���start============================
//                                            IplImage *src_6;
//											float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_6;  
//											string path_6 = "E:\\�������\\";  
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
//												const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��
//
//												src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_6->height;i++)
//													for(int j=0;j<src_6->width;j++)
//													{
//														
//														if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
//													s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_6=s_6[0]; //�����һ������Сֵ
//											int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-2);
//											system(cmd_6);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_6;i++)
//											{
//												
//												string fileName_6 = filenames_6[i];  
//												string fileFullName_6 = path_6 + fileName_6;  
//
//												const char * filename_6=fileFullName_6.c_str(); 
//
//												if( remove(filename_6) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_6);
//												else
//													perror("remove");
//											}
//											}
//
//
////===========�����ϴ���end==============================
////===========�����´���start============================
//                                            IplImage *src_7;
//											float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_7;  
//											string path_7 = "E:\\�������\\";  
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
//												const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��
//
//												src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_7->height;i++)
//													for(int j=0;j<src_7->width;j++)
//													{
//														
//														if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
//													s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_7=s_7[0]; //�����һ������Сֵ
//											int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-2);
//											system(cmd_7);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_7;i++)
//											{
//												
//												string fileName_7 = filenames_7[i];  
//												string fileFullName_7 = path_7 + fileName_7;  
//
//												const char * filename_7=fileFullName_7.c_str(); 
//
//												if( remove(filename_7) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_7);
//												else
//													perror("remove");
//											}
//											}
//
//
////===========�����´���end==============================
////===========�����ϴ���start============================
//
//                                            IplImage *src_8;
//											float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_8;  
//											string path_8 = "E:\\�������\\";  
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
//												const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��
//
//												src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_8->height;i++)
//													for(int j=0;j<src_8->width;j++)
//													{
//														
//														if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
//													s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_8=s_8[0]; //�����һ������Сֵ
//											int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-2);
//											system(cmd_8);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_8;i++)
//											{
//												
//												string fileName_8 = filenames_8[i];  
//												string fileFullName_8 = path_8 + fileName_8;  
//
//												const char * filename_8=fileFullName_8.c_str(); 
//
//												if( remove(filename_8) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_8);
//												else
//													perror("remove");
//											}
//											}
//
//
//
//
////===========�����ϴ���end==============================
//
////===========�����´���start============================
//                                            IplImage *src_9;
//											float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡
//
//											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
//											Directory dir_9;  
//											string path_9 = "E:\\�������\\";  
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
//												const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��
//
//												src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ
//
//
//												int n=0;  //ͳ�ư�ɫ���صĸ���  
//												for(int i=0;i<src_9->height;i++)
//													for(int j=0;j<src_9->width;j++)
//													{
//														
//														if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
//															n++; 
//
//													}
//													float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
//													s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������
//
//											}
//											//==========�ҳ���С��ռ��ֵ======================
//											float min_9=s_9[0]; //�����һ������Сֵ
//											int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�
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
//											sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-2);
//											system(cmd_9);
//
//											//=========ɾͼ=========================
//											for(int i=0;i<size_9;i++)
//											{
//												
//												string fileName_9 = filenames_9[i];  
//												string fileFullName_9 = path_9 + fileName_9;  
//
//												const char * filename_9=fileFullName_9.c_str(); 
//
//												if( remove(filename_9) == 0 )   //remove����0��ʾɾ���ɹ���		
//													printf("Removed %s.\n", filename_9);
//												else
//													perror("remove");
//											}
//											}
//===========�����´���end==============================
										//	OK=0;
										//}
//===========================================ͼƬ����end=========================================

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
										dst_img_name += "_0";//ȫ��
										dst_img_name += ".jpg";

										vector<int> compression_params;
										compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
										compression_params.push_back(9);
										
										if(T1==2)
										{
											cv::imwrite( dst_img_name, bodyMat, compression_params);
											num++;
											//waitKey(2000);
											//cout<<"��ǰ����ͼ����ţ� "<<num-1<<endl;
										}
										//=====================================================================================//

										//��ͷ
										//circle(img,Point(X3,Y3),(Y2-Y3),Scalar(0,255,0),3,8);
																			
											//���������ɾ��ο�
											double result0;
											result0 = atan2 (Y2-Y0,X2-X0) * 180 / CV_PI;
											RotatedRect rRect0=RotatedRect(Point2f((X2+X0)/2,(Y2+Y0)/2),Size2f(Y0-Y2,(Y0-Y2)*2/3),result0); //����һ����ת����    
											Point2f vertices0[4];
											rRect0.points(vertices0);//��ȡ��ת���ε��ĸ��ǵ�    
											//  for(int i=0;i<4;i++)    
											//	  {
											//	line(img,vertices0[i],vertices0[(i+1)%4],Scalar(255,255,255),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
											//   }

											//=================��ȡ�������ɲ���=========================================

											a=vertices0[0].x;
											b=vertices0[1].x;
											c=vertices0[2].x;
											d=vertices0[3].x;
											e=vertices0[0].y;
											f=vertices0[1].y;
											g=vertices0[2].y;
											h=vertices0[3].y;
											//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
											(b>a)?q=a,a=b,b=q:a;   
											(c>a)?q=a,a=c,c=q:a;
											(d>a)?q=a,a=d,d=q:a;
											(c>b)?q=b,b=c,c=q:b;
											(d>b)?q=b,b=d,d=q:b;
											(d>c)?q=c,c=d,d=q:c;
											//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
											(f>e)?q=e,e=f,f=q:e;   
											(g>e)?q=e,e=g,g=q:e; 
											(h>e)?q=e,e=h,h=q:e;
											(g>f)?q=f,f=g,g=q:f;
											(h>f)?q=f,f=h,h=q:f;
											(h>g)?q=g,g=h,h=q:g;
											m=a-d+1;  //����Ӧ��Ӿ��εĳ���
											n=e-h+1;  //����Ӧ��Ӿ��еĿ��

											Mat img1(n,m,CV_8UC4,Scalar(255,255,255));
											if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
											{
												for(int i=h;i<e;i++)  //��
													for(int j=d;j<a;j++)//��
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
											sprintf_s( chari, "%04d", num1);  //����4λ������ʽ	
											dst_img_name += chari;
											dst_img_name += "_1";//��������
											dst_img_name += ".jpg";

											if(T0==2&&T2==2)
											{
												cv::imwrite( dst_img_name, img1, compression_params);
												num1++;
												//waitKey(2000);
										   	}
										//===================================================================


										//�Ҹ첲�����ο�
										double result1;
										result1 = atan2 (Y8-Y9,X8-X9) * 180 / CV_PI;
										float q1=sqrt(pow((X8-X9),2)+pow((Y8-Y9),2));  //�����������ڵ������Ϊ���εĳ���
										RotatedRect rRect1=RotatedRect(Point2f((X8+X9)/2,(Y8+Y9)/2),Size2f(q1,q1/2),result1); //����һ����ת����    
										Point2f vertices1[4];    
										rRect1.points(vertices1);//��ȡ��ת���ε��ĸ��ǵ�    
										//   for(int i=0;i<4;i++)    
										//   {    
										//     line(img,vertices1[i],vertices1[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//   }    

										double result2;
										result2 = atan2 (Y9-Y11,X9-X11) * 180 / CV_PI;
										float q2=sqrt(pow((X9-X11),2)+pow((Y9-Y11),2));
										RotatedRect rRect2=RotatedRect(Point2f((X11+X9)/2,(Y11+Y9)/2),Size2f(q2,q2/2),result2); //����һ����ת����    
										Point2f vertices2[4];    
										rRect2.points(vertices2);//��ȡ��ת���ε��ĸ��ǵ�    
										//    for(int i=0;i<4;i++)    
										//    {    
										//      line(img,vertices2[i],vertices2[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//   }  


										//=================��ȡ�ұ��ϲ���=========================================

										a=vertices1[0].x;
										b=vertices1[1].x;
										c=vertices1[2].x;
										d=vertices1[3].x;
										e=vertices1[0].y;
										f=vertices1[1].y;
										g=vertices1[2].y;
										h=vertices1[3].y;
										//�Ƚϵ���x����
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_2";//�ұ���
										dst_img_name += ".jpg";

										if(T8==2&&T9==2)
										{
											cv::imwrite( dst_img_name, img2, compression_params);	
											num2++;
											//waitKey(2000);
										}

										//===================================================================

										//=================��ȡ�ұ��²���=========================================
										a=vertices2[0].x;
										b=vertices2[1].x;
										c=vertices2[2].x;
										d=vertices2[3].x;
										e=vertices2[0].y;
										f=vertices2[1].y;
										g=vertices2[2].y;
										h=vertices2[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_3";//�ұ���
										dst_img_name += ".jpg";

										if(T9==2&&T11==2)
										{
											cv::imwrite( dst_img_name, img3, compression_params);
											num3++;
											//waitKey(2000);
										}

										//===================================================================

										//��첲�����ο�
										double result3;
										result3 = atan2 (Y4-Y5,X4-X5) * 180 / CV_PI;
										float q3=sqrt(pow((X4-X5),2)+pow((Y4-Y5),2));
										RotatedRect rRect3=RotatedRect(Point2f((X4+X5)/2,(Y4+Y5)/2),Size2f(q3,q3/2),result3); //����һ����ת����    
										Point2f vertices3[4];    
										rRect3.points(vertices3);//��ȡ��ת���ε��ĸ��ǵ�    
										//   for(int i=0;i<4;i++)    
										//   {    
										//     line(img,vertices3[i],vertices3[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//   } 

										double result4;
										result4 = atan2 (Y5-Y7,X5-X7) * 180 / CV_PI;
										float q4=sqrt(pow((X7-X5),2)+pow((Y7-Y5),2));
										RotatedRect rRect4=RotatedRect(Point2f((X7+X5)/2,(Y7+Y5)/2),Size2f(q4,q4/2),result4); //����һ����ת����    
										Point2f vertices4[4];    
										rRect4.points(vertices4);//��ȡ��ת���ε��ĸ��ǵ�    
										//     for(int i=0;i<4;i++)    
										//     {    
										//      line(img,vertices4[i],vertices4[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//     } 


										//=================��ȡ����ϲ���=========================================
										a=vertices3[0].x;
										b=vertices3[1].x;
										c=vertices3[2].x;
										d=vertices3[3].x;
										e=vertices3[0].y;
										f=vertices3[1].y;
										g=vertices3[2].y;
										h=vertices3[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_4";//�����
										dst_img_name += ".jpg";
										if(T4==2&&T5==2)
										{
											cv::imwrite( dst_img_name, img4, compression_params);
											num4++;
											//waitKey(2000);
										}
										//===================================================================

										//=================��ȡ����²���=========================================
										a=vertices4[0].x;
										b=vertices4[1].x;
										c=vertices4[2].x;
										d=vertices4[3].x;
										e=vertices4[0].y;
										f=vertices4[1].y;
										g=vertices4[2].y;
										h=vertices4[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
										(f>e)?q=e,e=f,f=q:e;   
										(g>e)?q=e,e=g,g=q:e;
										(h>e)?q=e,e=h,h=q:e;
										(g>f)?q=f,f=g,g=q:f;
										(h>f)?q=f,f=h,h=q:f;
										(h>g)?q=g,g=h,h=q:g;
										m=a-d+1;
										n=e-h+1;

										Mat img5(n,m,CV_8UC4,Scalar(255,255,255));   //���ߣ���
										if((h>0)&&(h<1080)&&(e>0)&&(e<1080)&&(d>0)&&(d<1920)&&(a>0)&&(a<1920))
										{
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_5";//�����		
										dst_img_name += ".jpg";
										if(T7==2&&T9==2)
										{
											cv::imwrite( dst_img_name, img5, compression_params);	
											num5++;
											//waitKey(2000);
										}
										//===================================================================


										//���Ȼ����ο�			
										double result5;
										result5 = atan2 (Y16-Y17,X16-X17) * 180 / CV_PI;
										float q5=sqrt(pow((X17-X16),2)+pow((Y17-Y16),2));
										RotatedRect rRect5=RotatedRect(Point2f((X16+X17)/2,(Y16+Y17)/2),Size2f(q5,q5/2),result5); //����һ����ת����    
										Point2f vertices5[4];    
										rRect5.points(vertices5);//��ȡ��ת���ε��ĸ��ǵ�    
										//   for(int i=0;i<4;i++)    
										//   {    
										//     line(img,vertices5[i],vertices5[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//  } 

										double result6;
										result6 = atan2 (Y17-Y18,X17-X18) * 180 / CV_PI;
										float q6=sqrt(pow((X17-X18),2)+pow((Y17-Y18),2));
										RotatedRect rRect6=RotatedRect(Point2f((X18+X17)/2,(Y18+Y17)/2),Size2f(q6,q6/2),result6); //����һ����ת����    
										Point2f vertices6[4];    
										rRect6.points(vertices6);//��ȡ��ת���ε��ĸ��ǵ�    
										//    for(int i=0;i<4;i++)    
										//    {    
										//      line(img,vertices6[i],vertices6[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//    }


										//=================��ȡ�����ϲ���=========================================
										a=vertices5[0].x;
										b=vertices5[1].x;
										c=vertices5[2].x;
										d=vertices5[3].x;
										e=vertices5[0].y;
										f=vertices5[1].y;
										g=vertices5[2].y;
										h=vertices5[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_6";//������
										dst_img_name += ".jpg";

										if(T16==2&&T17==2)
										{
											cv::imwrite( dst_img_name, img6, compression_params);
											num6++;
											//waitKey(2000);
										}
										//===================================================================


										//=================��ȡ�����²���=========================================
										a=vertices6[0].x;
										b=vertices6[1].x;
										c=vertices6[2].x;
										d=vertices6[3].x;
										e=vertices6[0].y;
										f=vertices6[1].y;
										g=vertices6[2].y;
										h=vertices6[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_7";//������
										dst_img_name += ".jpg";
										if(T17==2&&T18==2)
										{
											cv::imwrite( dst_img_name, img7, compression_params);
											num7++;
											//waitKey(2000);
										}
										//dst_img_name = "E://�������//body-mapping-depth//����//";
										//===================================================================


										//������
										double result7;
										result7 = atan2 (Y12-Y13,X12-X13) * 180 / CV_PI;
										float q7=sqrt(pow((X12-X13),2)+pow((Y12-Y13),2));
										RotatedRect rRect7=RotatedRect(Point2f((X13+X12)/2,(Y13+Y12)/2),Size2f(q7,q7/2),result7); //����һ����ת����    
										Point2f vertices7[4];    
										rRect7.points(vertices7);//��ȡ��ת���ε��ĸ��ǵ�    
										//     for(int i=0;i<4;i++)    
										//     {    
										//       line(img,vertices7[i],vertices7[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//     }

										double result8;
										result8 = atan2 (Y13-Y14,X13-X14) * 180 / CV_PI;
										float q8=sqrt(pow((X14-X13),2)+pow((Y14-Y13),2));
										RotatedRect rRect8=RotatedRect(Point2f((X13+X14)/2,(Y13+Y14)/2),Size2f(q8,q8/2),result8); //����һ����ת����    
										Point2f vertices8[4];    
										rRect8.points(vertices8);//��ȡ��ת���ε��ĸ��ǵ�    
										//    for(int i=0;i<4;i++)    
										//    {    
										//      line(img,vertices8[i],vertices8[(i+1)%4],Scalar(0,255,0),3,8,0);//�ĸ��ǵ������ߣ������γ���ת�ľ��Ρ�    
										//    }


										//=================��ȡ�����ϲ���=========================================
										a=vertices7[0].x;
										b=vertices7[1].x;
										c=vertices7[2].x;
										d=vertices7[3].x;
										e=vertices7[0].y;
										f=vertices7[1].y;
										g=vertices7[2].y;
										h=vertices7[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_8";//������
										dst_img_name += ".jpg";

										if(T12==2&&T13==2)
										{
											cv::imwrite( dst_img_name, img8, compression_params);
											num8++;
											//waitKey(2000);
										}
										//===================================================================


										//=================��ȡ�����²���=========================================
										a=vertices8[0].x;
										b=vertices8[1].x;
										c=vertices8[2].x;
										d=vertices8[3].x;
										e=vertices8[0].y;
										f=vertices8[1].y;
										g=vertices8[2].y;
										h=vertices8[3].y;
										//�Ƚϵ���x����,��������Ӵ�С�ǣ�a,b,c,d
										(b>a)?q=a,a=b,b=q:a;   
										(c>a)?q=a,a=c,c=q:a;
										(d>a)?q=a,a=d,d=q:a;
										(c>b)?q=b,b=c,c=q:b;
										(d>b)?q=b,b=d,d=q:b;
										(d>c)?q=c,c=d,d=q:c;
										//�Ƚϵ���y����,��������Ӵ�С�ǣ�e,f,g,h
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
											for(int i=h;i<e;i++)  //��
												for(int j=d;j<a;j++)//��
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
										dst_img_name += "_9";//������
										dst_img_name += ".jpg";

										if(T13==2&&T14==2)
										{
										    cv::imwrite( dst_img_name, img9, compression_params);
											num9++;
											//waitKey(2000);
											
										}
										//===================================================================	

									}  //����ɹ��Ļ�ȡ���ؽڵ���
								}  //����ɹ���׷�ٵ�����	
							}  //pbody1ָ�������
							// SafeRelease( pBody1 );	
						}  //bodycountѭ��������          forѭ����λ���������
					}   //������¹�����Ϣ�ɹ�

					for( int count = 0; count < BODY_COUNT; count++ )//ѭ����ȫ�ͷ�6����Ϊbody��ָ��
					{   
						SafeRelease( pBody[count] );
					}

					SafeRelease( pBodyFrame );   
				}  //���body�ɹ���ȡ���һ֡
				/* cv::resize( img, bodyMat, cv::Size(), 0.5, 0.5 );*/
				SafeRelease(pColorFrame);
			}  //��ɫFrame

			//====================���������ȡend==============================================

			SafeRelease(pDepthFrame);
		}//IDepthFrame
	    SafeRelease(pBodyIndexFrame);

		if (waitKey(10)==27)  { break; }  //��Esc���˳�
		
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
		/*if(OK_0==1)   { cout<<"������Ϊ 0 ͼ����  |"<<"  ���屣����� "<<order-1<<endl; OK_0=0; }
		if(OK_1==1)   { cout<<"������Ϊ 1 ͼ����  |"<<"  ���屣����� "<<order-1<<endl; OK_1=0; }
		if(OK_2==1)   { cout<<"������Ϊ 2 ͼ����  |"<<"  ���屣����� "<<order-1<<endl; OK_2=0; }
		if(OK_3==1)   { cout<<"������Ϊ 3 ͼ����  |"<<"  ���屣����� "<<order-1<<endl; OK_3=0; }
		if(OK_4==1)   { cout<<"������Ϊ 4 ͼ����  |"<<"  ���屣����� "<<order-1<<endl; OK_4=0; }
		if(OK_5==1)   { cout<<"������Ϊ 5 ͼ����  |"<<"  ���屣����� "<<order-1<<endl; OK_5=0; }*/


	}  //while��1��   pBodyIndexFrame

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
}   //����������

//============================ͼƬ����start========================
void  func_0()
{
                                         cout<<"������Ϊ 0 ɸѡ����  /"<<"  ������� "<<order-1<<endl<<endl;
                                         //=================ȫ��start================================
											IplImage *src_0;
											float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_0;  
											string path_0 = "E:\\�������\\0\\";  
											string  exten_0 = "*_0.jpg";  

											vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

											const int size_0 = filenames_0.size();

											if(size_0!=0)
											{

											//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
											for (int p = 0; p < size_0;p++)  
											{  

												string fileName_0 = filenames_0[p];  
												string fileFullName_0 = path_0 + fileName_0;  
												//cout<<"File name:"<<fileName_0<<endl;  
												//cout<<"Full path_0:"<<fileFullName_0<<endl;  

												//=============================================================

												const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��

												//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
												//sprintf(windowname,"%d.jpg",p);
												src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_0->height;i++)
													for(int j=0;j<src_0->width;j++)
													{
														// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
														if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
													s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������

													//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
													//cvShowImage(windowname,src_0);
													//waitKey(500);

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_0=s_0[0]; //�����һ������Сֵ
											int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;


											const char * filename1_0=fileFullName_0.c_str();
											char cmd_0[255] = {0};
											//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
											//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
											sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-1);
											system(cmd_0);
										

											//=========ɾͼ=========================
											for(int i=0;i<size_0;i++)
											{
												//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
												//{
												//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 


												string fileName_0 = filenames_0[i];  
												string fileFullName_0 = path_0 + fileName_0;  

												const char * filename_0=fileFullName_0.c_str(); 

												if( remove(filename_0) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_0);
												else
													perror("remove");
												//}
											}
											}
                                      //=================ȫ��end================================

                                      //=================����start================================
											IplImage *src_1;
											float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_1;  
											string path_1 = "E:\\�������\\0\\";  
											string  exten_1 = "*_1.jpg";  

											vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

											const int size_1 = filenames_1.size();

											if(size_1!=0)
											{

											//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
											for (int p = 0; p < size_1;p++)  
											{  

												string fileName_1 = filenames_1[p];  
												string fileFullName_1 = path_1 + fileName_1;  
												//cout<<"File name:"<<fileName_1<<endl;  
												//cout<<"Full path_1:"<<fileFullName_1<<endl;  

												//=============================================================

												const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��

												//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
												//sprintf(windowname,"%d.jpg",p);
												src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_1->height;i++)
													for(int j=0;j<src_1->width;j++)
													{
														// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
														if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
													s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������

													//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
													//cvShowImage(windowname,src_1);
													//waitKey(500);

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_1=s_1[0]; //�����һ������Сֵ
											int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;


											const char * filename1_1=fileFullName_1.c_str();
											char cmd_1[255] = {0};
											//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
											//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
											sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-1);
											system(cmd_1);

											//=========ɾͼ=========================
											for(int i=0;i<size_1;i++)
											{
												//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
												//{
												//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 


												string fileName_1 = filenames_1[i];  
												string fileFullName_1 = path_1 + fileName_1;  

												const char * filename_1=fileFullName_1.c_str(); 

												if( remove(filename_1) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_1);
												else
													perror("remove");
												//}
											}
											}
                                           //=================����end================================

                                           //===========�ұ��ϴ���start============================

                                            IplImage *src_2;
											float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_2;  
											string path_2 = "E:\\�������\\0\\";  
											string  exten_2 = "*_2.jpg";  

											vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

											const int size_2 = filenames_2.size();     
											//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
											if(size_2!=0)
											{
											for (int p = 0; p < size_2;p++)  
											{  

												string fileName_2 = filenames_2[p];  
												string fileFullName_2 = path_2 + fileName_2;  
												//cout<<"File name:"<<fileName_2<<endl;  
												//cout<<"Full path_2:"<<fileFullName_2<<endl;  

												//=============================================================

												const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��

												//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
												//sprintf(windowname,"%d.jpg",p);
												src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_2->height;i++)
													for(int j=0;j<src_2->width;j++)
													{
														// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
														if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
													s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������

													//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
													//cvShowImage(windowname,src_2);
													//waitKey(500);

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_2=s_2[0]; //�����һ������Сֵ
											int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;


											const char * filename1_2=fileFullName_2.c_str();
											char cmd_2[255] = {0};
											//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
											//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
											sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-1);
											system(cmd_2);

											//=========ɾͼ=========================
											for(int i=0;i<size_2;i++)
											{
												//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
												//{
												//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 


												string fileName_2 = filenames_2[i];  
												string fileFullName_2 = path_2 + fileName_2;  

												const char * filename_2=fileFullName_2.c_str(); 

												if( remove(filename_2) == 0 );   //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_2);
												else
													perror("remove");
												//}
											}
											}
//===========�ұ��ϴ���end==============================

//===========�ұ��´���start============================
                                            IplImage *src_3;
											float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_3;  
											string path_3 = "E:\\�������\\0\\";  
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

												const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��

												src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_3->height;i++)
													for(int j=0;j<src_3->width;j++)
													{
														
														if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
													s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_3=s_3[0]; //�����һ������Сֵ
											int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-1);
											system(cmd_3);

											//=========ɾͼ=========================
											for(int i=0;i<size_3;i++)
											{
												
												string fileName_3 = filenames_3[i];  
												string fileFullName_3 = path_3 + fileName_3;  

												const char * filename_3=fileFullName_3.c_str(); 

												if( remove(filename_3) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_3);
												else
													perror("remove");
												//}
											}
											}

//===========�ұ��´���end==============================
//===========����ϴ���start============================
                                            IplImage *src_4;
											float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_4;  
											string path_4 = "E:\\�������\\0\\";  
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

												const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��

												src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_4->height;i++)
													for(int j=0;j<src_4->width;j++)
													{
														
														if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
													s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_4=s_4[0]; //�����һ������Сֵ
											int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-1);
											system(cmd_4);

											//=========ɾͼ=========================
											for(int i=0;i<size_4;i++)
											{
												
												string fileName_4 = filenames_4[i];  
												string fileFullName_4 = path_4 + fileName_4;  

												const char * filename_4=fileFullName_4.c_str(); 

												if( remove(filename_4) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
												//	printf("Removed %s.\n", filename_4);
												else
													perror("remove");
												//}
											}
											}

//===========����ϴ���end==============================
//===========����´���start============================
                                            IplImage *src_5;
											float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_5;  
											string path_5 = "E:\\�������\\0\\";  
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

												const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��

												src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_5->height;i++)
													for(int j=0;j<src_5->width;j++)
													{
														
														if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
													s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_5=s_5[0]; //�����һ������Сֵ
											int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-1);
											system(cmd_5);

											//=========ɾͼ=========================
											for(int i=0;i<size_5;i++)
											{
												
												string fileName_5 = filenames_5[i];  
												string fileFullName_5 = path_5 + fileName_5;  

												const char * filename_5=fileFullName_5.c_str(); 

												if( remove(filename_5) == 0 )  ; //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_5);
												else
													perror("remove");
											}
											}

//===========����´���end==============================

//===========�����ϴ���start============================
                                            IplImage *src_6;
											float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_6;  
											string path_6 = "E:\\�������\\0\\";  
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

												const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��

												src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_6->height;i++)
													for(int j=0;j<src_6->width;j++)
													{
														
														if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
													s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_6=s_6[0]; //�����һ������Сֵ
											int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-1);
											system(cmd_6);

											//=========ɾͼ=========================
											for(int i=0;i<size_6;i++)
											{
												
												string fileName_6 = filenames_6[i];  
												string fileFullName_6 = path_6 + fileName_6;  

												const char * filename_6=fileFullName_6.c_str(); 

												if( remove(filename_6) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_6);
												else
													perror("remove");
											}
											}


//===========�����ϴ���end==============================
//===========�����´���start============================
                                            IplImage *src_7;
											float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_7;  
											string path_7 = "E:\\�������\\0\\";  
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

												const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��

												src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_7->height;i++)
													for(int j=0;j<src_7->width;j++)
													{
														
														if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
													s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_7=s_7[0]; //�����һ������Сֵ
											int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-1);
											system(cmd_7);

											//=========ɾͼ=========================
											for(int i=0;i<size_7;i++)
											{
												
												string fileName_7 = filenames_7[i];  
												string fileFullName_7 = path_7 + fileName_7;  

												const char * filename_7=fileFullName_7.c_str(); 

												if( remove(filename_7) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_7);
												else
													perror("remove");
											}
											}


//===========�����´���end==============================
//===========�����ϴ���start============================

                                            IplImage *src_8;
											float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_8;  
											string path_8 = "E:\\�������\\0\\";  
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

												const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��

												src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_8->height;i++)
													for(int j=0;j<src_8->width;j++)
													{
														
														if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
													s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_8=s_8[0]; //�����һ������Сֵ
											int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-1);
											system(cmd_8);

											//=========ɾͼ=========================
											for(int i=0;i<size_8;i++)
											{
												
												string fileName_8 = filenames_8[i];  
												string fileFullName_8 = path_8 + fileName_8;  

												const char * filename_8=fileFullName_8.c_str(); 

												if( remove(filename_8) == 0 )  ; //remove����0��ʾɾ���ɹ���		
													//printf("Removed %s.\n", filename_8);
												else
													perror("remove");
											}
											}




//===========�����ϴ���end==============================

//===========�����´���start============================
                                            IplImage *src_9;
											float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

											//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
											Directory dir_9;  
											string path_9 = "E:\\�������\\0\\";  
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

												const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��

												src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ


												int n=0;  //ͳ�ư�ɫ���صĸ���  
												for(int i=0;i<src_9->height;i++)
													for(int j=0;j<src_9->width;j++)
													{
														
														if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
															n++; 

													}
													float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
													s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������

											}
											//==========�ҳ���С��ռ��ֵ======================
											float min_9=s_9[0]; //�����һ������Сֵ
											int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
											
											sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-1);
											system(cmd_9);

											//=========ɾͼ=========================
											for(int i=0;i<size_9;i++)
											{
												
												string fileName_9 = filenames_9[i];  
												string fileFullName_9 = path_9 + fileName_9;  

												const char * filename_9=fileFullName_9.c_str(); 

												if( remove(filename_9) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
												//	printf("Removed %s.\n", filename_9);
												else
													perror("remove");
											}
											}
                                      //===========�����´���end==============================
	
}







void func_1()
{
cout<<"������Ϊ 1 ɸѡ����  /"<<"  ������� "<<order-1<<endl<<endl;
//=================ȫ��start================================
IplImage *src_0;
float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_0;  
string path_0 = "E:\\�������\\1\\";  
string  exten_0 = "*_0.jpg";  

vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

const int size_0 = filenames_0.size();

if(size_0!=0)
{

	//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
	for (int p = 0; p < size_0;p++)  
	{  

		string fileName_0 = filenames_0[p];  
		string fileFullName_0 = path_0 + fileName_0;  
		//cout<<"File name:"<<fileName_0<<endl;  
		//cout<<"Full path_0:"<<fileFullName_0<<endl;  

		//=============================================================

		const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��

		//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
		//sprintf(windowname,"%d.jpg",p);
		src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_0->height;i++)
			for(int j=0;j<src_0->width;j++)
			{
				// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
				if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
			s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������

			//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
			//cvShowImage(windowname,src_0);
			//waitKey(500);

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_0=s_0[0]; //�����һ������Сֵ
	int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
	//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;


	const char * filename1_0=fileFullName_0.c_str();
	char cmd_0[255] = {0};
	//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
	//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
	sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-1);
	system(cmd_0);

	//=========ɾͼ=========================
	for(int i=0;i<size_0;i++)
	{
		//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
		//{
		//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 


		string fileName_0 = filenames_0[i];  
		string fileFullName_0 = path_0 + fileName_0;  

		const char * filename_0=fileFullName_0.c_str(); 

		if( remove(filename_0) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_0);
		else
			perror("remove");
		//}
	}
}
//=================ȫ��end================================

//=================����start================================
IplImage *src_1;
float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_1;  
string path_1 = "E:\\�������\\1\\";  
string  exten_1 = "*_1.jpg";  

vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

const int size_1 = filenames_1.size();

if(size_1!=0)
{

	//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
	for (int p = 0; p < size_1;p++)  
	{  

		string fileName_1 = filenames_1[p];  
		string fileFullName_1 = path_1 + fileName_1;  
		//cout<<"File name:"<<fileName_1<<endl;  
		//cout<<"Full path_1:"<<fileFullName_1<<endl;  

		//=============================================================

		const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��

		//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
		//sprintf(windowname,"%d.jpg",p);
		src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_1->height;i++)
			for(int j=0;j<src_1->width;j++)
			{
				// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
				if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
			s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������

			//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
			//cvShowImage(windowname,src_1);
			//waitKey(500);

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_1=s_1[0]; //�����һ������Сֵ
	int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
	//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;


	const char * filename1_1=fileFullName_1.c_str();
	char cmd_1[255] = {0};
	//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
	//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
	sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-1);
	system(cmd_1);

	//=========ɾͼ=========================
	for(int i=0;i<size_1;i++)
	{
		//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
		//{
		//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 


		string fileName_1 = filenames_1[i];  
		string fileFullName_1 = path_1 + fileName_1;  

		const char * filename_1=fileFullName_1.c_str(); 

		if( remove(filename_1) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_1);
		else
			perror("remove");
		//}
	}
}
//=================����end================================

//===========�ұ��ϴ���start============================

IplImage *src_2;
float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_2;  
string path_2 = "E:\\�������\\1\\";  
string  exten_2 = "*_2.jpg";  

vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

const int size_2 = filenames_2.size();     
//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
if(size_2!=0)
{
	for (int p = 0; p < size_2;p++)  
	{  

		string fileName_2 = filenames_2[p];  
		string fileFullName_2 = path_2 + fileName_2;  
		//cout<<"File name:"<<fileName_2<<endl;  
		//cout<<"Full path_2:"<<fileFullName_2<<endl;  

		//=============================================================

		const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��

		//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
		//sprintf(windowname,"%d.jpg",p);
		src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_2->height;i++)
			for(int j=0;j<src_2->width;j++)
			{
				// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
				if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
			s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������

			//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
			//cvShowImage(windowname,src_2);
			//waitKey(500);

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_2=s_2[0]; //�����һ������Сֵ
	int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
	//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;


	const char * filename1_2=fileFullName_2.c_str();
	char cmd_2[255] = {0};
	//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
	//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
	sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-1);
	system(cmd_2);

	//=========ɾͼ=========================
	for(int i=0;i<size_2;i++)
	{
		//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
		//{
		//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 


		string fileName_2 = filenames_2[i];  
		string fileFullName_2 = path_2 + fileName_2;  

		const char * filename_2=fileFullName_2.c_str(); 

		if( remove(filename_2) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
		//	printf("Removed %s.\n", filename_2);
		else
			perror("remove");
		//}
	}
}
//===========�ұ��ϴ���end==============================

//===========�ұ��´���start============================
IplImage *src_3;
float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_3;  
string path_3 = "E:\\�������\\1\\";  
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

		const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��

		src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_3->height;i++)
			for(int j=0;j<src_3->width;j++)
			{

				if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
			s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_3=s_3[0]; //�����һ������Сֵ
	int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-1);
	system(cmd_3);

	//=========ɾͼ=========================
	for(int i=0;i<size_3;i++)
	{

		string fileName_3 = filenames_3[i];  
		string fileFullName_3 = path_3 + fileName_3;  

		const char * filename_3=fileFullName_3.c_str(); 

		if( remove(filename_3) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
		//	printf("Removed %s.\n", filename_3);
		else
			perror("remove");
		//}
	}
}

//===========�ұ��´���end==============================
//===========����ϴ���start============================
IplImage *src_4;
float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_4;  
string path_4 = "E:\\�������\\1\\";  
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

		const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��

		src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_4->height;i++)
			for(int j=0;j<src_4->width;j++)
			{

				if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
			s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_4=s_4[0]; //�����һ������Сֵ
	int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-1);
	system(cmd_4);

	//=========ɾͼ=========================
	for(int i=0;i<size_4;i++)
	{

		string fileName_4 = filenames_4[i];  
		string fileFullName_4 = path_4 + fileName_4;  

		const char * filename_4=fileFullName_4.c_str(); 

		if( remove(filename_4) == 0 );   //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_4);
		else
			perror("remove");
		//}
	}
}

//===========����ϴ���end==============================
//===========����´���start============================
IplImage *src_5;
float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_5;  
string path_5 = "E:\\�������\\1\\";  
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

		const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��

		src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_5->height;i++)
			for(int j=0;j<src_5->width;j++)
			{

				if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
			s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_5=s_5[0]; //�����һ������Сֵ
	int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-1);
	system(cmd_5);

	//=========ɾͼ=========================
	for(int i=0;i<size_5;i++)
	{

		string fileName_5 = filenames_5[i];  
		string fileFullName_5 = path_5 + fileName_5;  

		const char * filename_5=fileFullName_5.c_str(); 

		if( remove(filename_5) == 0 );   //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_5);
		else
			perror("remove");
	}
}

//===========����´���end==============================

//===========�����ϴ���start============================
IplImage *src_6;
float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_6;  
string path_6 = "E:\\�������\\1\\";  
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

		const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��

		src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_6->height;i++)
			for(int j=0;j<src_6->width;j++)
			{

				if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
			s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_6=s_6[0]; //�����һ������Сֵ
	int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-1);
	system(cmd_6);

	//=========ɾͼ=========================
	for(int i=0;i<size_6;i++)
	{

		string fileName_6 = filenames_6[i];  
		string fileFullName_6 = path_6 + fileName_6;  

		const char * filename_6=fileFullName_6.c_str(); 

		if( remove(filename_6) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_6);
		else
			perror("remove");
	}
}


//===========�����ϴ���end==============================
//===========�����´���start============================
IplImage *src_7;
float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_7;  
string path_7 = "E:\\�������\\1\\";  
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

		const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��

		src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_7->height;i++)
			for(int j=0;j<src_7->width;j++)
			{

				if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
			s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_7=s_7[0]; //�����һ������Сֵ
	int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-1);
	system(cmd_7);

	//=========ɾͼ=========================
	for(int i=0;i<size_7;i++)
	{

		string fileName_7 = filenames_7[i];  
		string fileFullName_7 = path_7 + fileName_7;  

		const char * filename_7=fileFullName_7.c_str(); 

		if( remove(filename_7) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
		//	printf("Removed %s.\n", filename_7);
		else
			perror("remove");
	}
}


//===========�����´���end==============================
//===========�����ϴ���start============================

IplImage *src_8;
float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_8;  
string path_8 = "E:\\�������\\1\\";  
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

		const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��

		src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_8->height;i++)
			for(int j=0;j<src_8->width;j++)
			{

				if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
			s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_8=s_8[0]; //�����һ������Сֵ
	int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-1);
	system(cmd_8);

	//=========ɾͼ=========================
	for(int i=0;i<size_8;i++)
	{

		string fileName_8 = filenames_8[i];  
		string fileFullName_8 = path_8 + fileName_8;  

		const char * filename_8=fileFullName_8.c_str(); 

		if( remove(filename_8) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_8);
		else
			perror("remove");
	}
}




//===========�����ϴ���end==============================

//===========�����´���start============================
IplImage *src_9;
float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
Directory dir_9;  
string path_9 = "E:\\�������\\1\\";  
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

		const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��

		src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ


		int n=0;  //ͳ�ư�ɫ���صĸ���  
		for(int i=0;i<src_9->height;i++)
			for(int j=0;j<src_9->width;j++)
			{

				if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
					n++; 

			}
			float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
			s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������

	}
	//==========�ҳ���С��ռ��ֵ======================
	float min_9=s_9[0]; //�����һ������Сֵ
	int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

	sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-1);
	system(cmd_9);

	//=========ɾͼ=========================
	for(int i=0;i<size_9;i++)
	{

		string fileName_9 = filenames_9[i];  
		string fileFullName_9 = path_9 + fileName_9;  

		const char * filename_9=fileFullName_9.c_str(); 

		if( remove(filename_9) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//printf("Removed %s.\n", filename_9);
		else
			perror("remove");
	}
}
//===========�����´���end==============================
}




void func_2()
{
	cout<<"������Ϊ 2 ɸѡ����  /"<<"  ������� "<<order-1<<endl<<endl;
	//=================ȫ��start================================
	IplImage *src_0;
	float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_0;  
	string path_0 = "E:\\�������\\2\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
				s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_0=s_0[0]; //�����һ������Сֵ
		int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========ɾͼ=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================ȫ��end================================

	//=================����start================================
	IplImage *src_1;
	float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_1;  
	string path_1 = "E:\\�������\\2\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
				s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_1=s_1[0]; //�����һ������Сֵ
		int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========ɾͼ=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 )  ; //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================����end================================

	//===========�ұ��ϴ���start============================

	IplImage *src_2;
	float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_2;  
	string path_2 = "E:\\�������\\2\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
				s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_2=s_2[0]; //�����һ������Сֵ
		int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========ɾͼ=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========�ұ��ϴ���end==============================

	//===========�ұ��´���start============================
	IplImage *src_3;
	float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_3;  
	string path_3 = "E:\\�������\\2\\";  
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

			const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��

			src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
				s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_3=s_3[0]; //�����һ������Сֵ
		int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========ɾͼ=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========�ұ��´���end==============================
	//===========����ϴ���start============================
	IplImage *src_4;
	float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_4;  
	string path_4 = "E:\\�������\\2\\";  
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

			const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��

			src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
				s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_4=s_4[0]; //�����һ������Сֵ
		int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========ɾͼ=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========����ϴ���end==============================
	//===========����´���start============================
	IplImage *src_5;
	float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_5;  
	string path_5 = "E:\\�������\\2\\";  
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

			const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��

			src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
				s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_5=s_5[0]; //�����һ������Сֵ
		int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========ɾͼ=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========����´���end==============================

	//===========�����ϴ���start============================
	IplImage *src_6;
	float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_6;  
	string path_6 = "E:\\�������\\2\\";  
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

			const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��

			src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
				s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_6=s_6[0]; //�����һ������Сֵ
		int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========ɾͼ=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========�����ϴ���end==============================
	//===========�����´���start============================
	IplImage *src_7;
	float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_7;  
	string path_7 = "E:\\�������\\2\\";  
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

			const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��

			src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
				s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_7=s_7[0]; //�����һ������Сֵ
		int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========ɾͼ=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========�����´���end==============================
	//===========�����ϴ���start============================

	IplImage *src_8;
	float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_8;  
	string path_8 = "E:\\�������\\2\\";  
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

			const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��

			src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
				s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_8=s_8[0]; //�����һ������Сֵ
		int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========ɾͼ=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========�����ϴ���end==============================

	//===========�����´���start============================
	IplImage *src_9;
	float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_9;  
	string path_9 = "E:\\�������\\2\\";  
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

			const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��

			src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
				s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_9=s_9[0]; //�����һ������Сֵ
		int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========ɾͼ=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========�����´���end==============================
}



void func_3()
{
	cout<<"������Ϊ 3 ɸѡ����  /"<<"  ������� "<<order-1<<endl<<endl;
	//=================ȫ��start================================
	IplImage *src_0;
	float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_0;  
	string path_0 = "E:\\�������\\3\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
				s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_0=s_0[0]; //�����һ������Сֵ
		int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========ɾͼ=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================ȫ��end================================

	//=================����start================================
	IplImage *src_1;
	float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_1;  
	string path_1 = "E:\\�������\\3\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
				s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_1=s_1[0]; //�����һ������Сֵ
		int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========ɾͼ=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================����end================================

	//===========�ұ��ϴ���start============================

	IplImage *src_2;
	float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_2;  
	string path_2 = "E:\\�������\\3\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
				s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_2=s_2[0]; //�����һ������Сֵ
		int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========ɾͼ=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========�ұ��ϴ���end==============================

	//===========�ұ��´���start============================
	IplImage *src_3;
	float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_3;  
	string path_3 = "E:\\�������\\3\\";  
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

			const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��

			src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
				s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_3=s_3[0]; //�����һ������Сֵ
		int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========ɾͼ=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========�ұ��´���end==============================
	//===========����ϴ���start============================
	IplImage *src_4;
	float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_4;  
	string path_4 = "E:\\�������\\3\\";  
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

			const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��

			src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
				s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_4=s_4[0]; //�����һ������Сֵ
		int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========ɾͼ=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========����ϴ���end==============================
	//===========����´���start============================
	IplImage *src_5;
	float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_5;  
	string path_5 = "E:\\�������\\3\\";  
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

			const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��

			src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
				s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_5=s_5[0]; //�����һ������Сֵ
		int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========ɾͼ=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========����´���end==============================

	//===========�����ϴ���start============================
	IplImage *src_6;
	float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_6;  
	string path_6 = "E:\\�������\\3\\";  
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

			const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��

			src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
				s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_6=s_6[0]; //�����һ������Сֵ
		int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========ɾͼ=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========�����ϴ���end==============================
	//===========�����´���start============================
	IplImage *src_7;
	float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_7;  
	string path_7 = "E:\\�������\\3\\";  
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

			const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��

			src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
				s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_7=s_7[0]; //�����һ������Сֵ
		int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========ɾͼ=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========�����´���end==============================
	//===========�����ϴ���start============================

	IplImage *src_8;
	float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_8;  
	string path_8 = "E:\\�������\\3\\";  
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

			const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��

			src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
				s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_8=s_8[0]; //�����һ������Сֵ
		int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========ɾͼ=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 );   //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========�����ϴ���end==============================

	//===========�����´���start============================
	IplImage *src_9;
	float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_9;  
	string path_9 = "E:\\�������\\3\\";  
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

			const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��

			src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
				s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_9=s_9[0]; //�����һ������Сֵ
		int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========ɾͼ=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========�����´���end==============================
}



void func_4()
{
	cout<<"������Ϊ 4 ɸѡ����  /"<<"  ������� "<<order-1<<endl<<endl;
	//=================ȫ��start================================
	IplImage *src_0;
	float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_0;  
	string path_0 = "E:\\�������\\4\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
				s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_0=s_0[0]; //�����һ������Сֵ
		int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========ɾͼ=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================ȫ��end================================

	//=================����start================================
	IplImage *src_1;
	float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_1;  
	string path_1 = "E:\\�������\\4\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
				s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_1=s_1[0]; //�����һ������Сֵ
		int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========ɾͼ=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================����end================================

	//===========�ұ��ϴ���start============================

	IplImage *src_2;
	float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_2;  
	string path_2 = "E:\\�������\\4\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
				s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_2=s_2[0]; //�����һ������Сֵ
		int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========ɾͼ=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========�ұ��ϴ���end==============================

	//===========�ұ��´���start============================
	IplImage *src_3;
	float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_3;  
	string path_3 = "E:\\�������\\4\\";  
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

			const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��

			src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
				s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_3=s_3[0]; //�����һ������Сֵ
		int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========ɾͼ=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 );   //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========�ұ��´���end==============================
	//===========����ϴ���start============================
	IplImage *src_4;
	float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_4;  
	string path_4 = "E:\\�������\\4\\";  
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

			const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��

			src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
				s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_4=s_4[0]; //�����һ������Сֵ
		int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========ɾͼ=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 )  ; //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========����ϴ���end==============================
	//===========����´���start============================
	IplImage *src_5;
	float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_5;  
	string path_5 = "E:\\�������\\4\\";  
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

			const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��

			src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
				s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_5=s_5[0]; //�����һ������Сֵ
		int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========ɾͼ=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========����´���end==============================

	//===========�����ϴ���start============================
	IplImage *src_6;
	float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_6;  
	string path_6 = "E:\\�������\\4\\";  
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

			const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��

			src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
				s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_6=s_6[0]; //�����һ������Сֵ
		int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========ɾͼ=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========�����ϴ���end==============================
	//===========�����´���start============================
	IplImage *src_7;
	float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_7;  
	string path_7 = "E:\\�������\\4\\";  
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

			const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��

			src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
				s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_7=s_7[0]; //�����һ������Сֵ
		int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========ɾͼ=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========�����´���end==============================
	//===========�����ϴ���start============================

	IplImage *src_8;
	float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_8;  
	string path_8 = "E:\\�������\\4\\";  
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

			const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��

			src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
				s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_8=s_8[0]; //�����һ������Сֵ
		int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========ɾͼ=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========�����ϴ���end==============================

	//===========�����´���start============================
	IplImage *src_9;
	float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_9;  
	string path_9 = "E:\\�������\\4\\";  
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

			const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��

			src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
				s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_9=s_9[0]; //�����һ������Сֵ
		int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========ɾͼ=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//;	printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========�����´���end==============================
}


void func_5()
{
	cout<<"������Ϊ 5 ɸѡ����  /"<<"  ������� "<<order-1<<endl<<endl;
	//=================ȫ��start================================
	IplImage *src_0;
	float s_0[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_0;  
	string path_0 = "E:\\�������\\5\\";  
	string  exten_0 = "*_0.jpg";  

	vector<string> filenames_0 = dir_0.GetListFiles(path_0, exten_0, false);  

	const int size_0 = filenames_0.size();

	if(size_0!=0)
	{

		//cout<<"�ҵ� "<<size_0<<" ֡����"<<endl;
		for (int p = 0; p < size_0;p++)  
		{  

			string fileName_0 = filenames_0[p];  
			string fileFullName_0 = path_0 + fileName_0;  
			//cout<<"File name:"<<fileName_0<<endl;  
			//cout<<"Full path_0:"<<fileFullName_0<<endl;  

			//=============================================================

			const char * filename_0=fileFullName_0.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_0=cvLoadImage (filename_0, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_0->height;i++)
				for(int j=0;j<src_0->width;j++)
				{
					// if(((uchar*)(src_0 ->imageData + src_0->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_0->imageData + i*src_0->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_0=(float)n/(src_0->height*src_0->width);  //�����ɫ����������ͼ�е�ռ��
				s_0[p]=per_p_0;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_0[p]<<endl;
				//cvShowImage(windowname,src_0);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_0=s_0[0]; //�����һ������Сֵ
		int index_0=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_0<<"  "<<"����ǣ� "<<index_0<<"   "<<fileName_0<<endl<<endl;


		const char * filename1_0=fileFullName_0.c_str();
		char cmd_0[255] = {0};
		//sprintf(cmd_0, "copy %s %s", filename1_0, "E:\\�����\\image777.jpg");
		//sprintf(cmd_0, "copy %s E:\\�����\\image888.jpg", filename1_0);
		sprintf(cmd_0, "copy %s E:\\�����\\%04d_0.jpg", filename1_0,order-1);
		system(cmd_0);

		//=========ɾͼ=========================
		for(int i=0;i<size_0;i++)
		{
			//if(i!=index_0)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_0,"E:\\�������\\%04d_0.jpg",i); 


			string fileName_0 = filenames_0[i];  
			string fileFullName_0 = path_0 + fileName_0;  

			const char * filename_0=fileFullName_0.c_str(); 

			if( remove(filename_0) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_0);
			else
				perror("remove");
			//}
		}
	}
	//=================ȫ��end================================

	//=================����start================================
	IplImage *src_1;
	float s_1[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_1;  
	string path_1 = "E:\\�������\\5\\";  
	string  exten_1 = "*_1.jpg";  

	vector<string> filenames_1 = dir_1.GetListFiles(path_1, exten_1, false);  

	const int size_1 = filenames_1.size();

	if(size_1!=0)
	{

		//cout<<"�ҵ� "<<size_1<<" ֡����"<<endl;
		for (int p = 0; p < size_1;p++)  
		{  

			string fileName_1 = filenames_1[p];  
			string fileFullName_1 = path_1 + fileName_1;  
			//cout<<"File name:"<<fileName_1<<endl;  
			//cout<<"Full path_1:"<<fileFullName_1<<endl;  

			//=============================================================

			const char * filename_1=fileFullName_1.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_1=cvLoadImage (filename_1, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_1->height;i++)
				for(int j=0;j<src_1->width;j++)
				{
					// if(((uchar*)(src_1 ->imageData + src_1->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_1->imageData + i*src_1->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_1=(float)n/(src_1->height*src_1->width);  //�����ɫ����������ͼ�е�ռ��
				s_1[p]=per_p_1;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_1[p]<<endl;
				//cvShowImage(windowname,src_1);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_1=s_1[0]; //�����һ������Сֵ
		int index_1=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_1<<"  "<<"����ǣ� "<<index_1<<"   "<<fileName_1<<endl<<endl;


		const char * filename1_1=fileFullName_1.c_str();
		char cmd_1[255] = {0};
		//sprintf(cmd_1, "copy %s %s", filename1_1, "E:\\�����\\image777.jpg");
		//sprintf(cmd_1, "copy %s E:\\�����\\image888.jpg", filename1_1);
		sprintf(cmd_1, "copy %s E:\\�����\\%04d_1.jpg", filename1_1,order-1);
		system(cmd_1);

		//=========ɾͼ=========================
		for(int i=0;i<size_1;i++)
		{
			//if(i!=index_1)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_1,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_1 = filenames_1[i];  
			string fileFullName_1 = path_1 + fileName_1;  

			const char * filename_1=fileFullName_1.c_str(); 

			if( remove(filename_1) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_1);
			else
				perror("remove");
			//}
		}
	}
	//=================����end================================

	//===========�ұ��ϴ���start============================

	IplImage *src_2;
	float s_2[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_2;  
	string path_2 = "E:\\�������\\5\\";  
	string  exten_2 = "*_2.jpg";  

	vector<string> filenames_2 = dir_2.GetListFiles(path_2, exten_2, false);  

	const int size_2 = filenames_2.size();     
	//cout<<"�ҵ� "<<size_2<<" ֡����"<<endl;
	if(size_2!=0)
	{
		for (int p = 0; p < size_2;p++)  
		{  

			string fileName_2 = filenames_2[p];  
			string fileFullName_2 = path_2 + fileName_2;  
			//cout<<"File name:"<<fileName_2<<endl;  
			//cout<<"Full path_2:"<<fileFullName_2<<endl;  

			//=============================================================

			const char * filename_2=fileFullName_2.c_str();  //��string���͵�·��ת����char��

			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",p);
			//sprintf(windowname,"%d.jpg",p);
			src_2=cvLoadImage (filename_2, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_2->height;i++)
				for(int j=0;j<src_2->width;j++)
				{
					// if(((uchar*)(src_2 ->imageData + src_2->widthStep*j))[i*3]==0)//(0-255) ��ɫͨ������
					if(((uchar *)(src_2->imageData + i*src_2->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_2=(float)n/(src_2->height*src_2->width);  //�����ɫ����������ͼ�е�ռ��
				s_2[p]=per_p_2;	  //�Ѷัͼ��ռ��ֵд��������

				//cout<<"��ɫ���ظ����� "<<n<<"	"<<"��ɫ����ռ�ȣ� "<<s_2[p]<<endl;
				//cvShowImage(windowname,src_2);
				//waitKey(500);

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_2=s_2[0]; //�����һ������Сֵ
		int index_2=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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
		//cout<<"��ɫ������Сռ�ȣ� "<<min_2<<"  "<<"����ǣ� "<<index_2<<"   "<<fileName_2<<endl<<endl;


		const char * filename1_2=fileFullName_2.c_str();
		char cmd_2[255] = {0};
		//sprintf(cmd_2, "copy %s %s", filename1_2, "E:\\�����\\image777.jpg");
		//sprintf(cmd_2, "copy %s E:\\�����\\image888.jpg", filename1_2);
		sprintf(cmd_2, "copy %s E:\\�����\\%04d_2.jpg", filename1_2,order-1);
		system(cmd_2);

		//=========ɾͼ=========================
		for(int i=0;i<size_2;i++)
		{
			//if(i!=index_2)  //������Ҫ�ı�ŵ��Ǹ�ͼ���⣬ȫ����ɾ��
			//{
			//sprintf(filename_2,"E:\\�������\\%04d_1.jpg",i); 


			string fileName_2 = filenames_2[i];  
			string fileFullName_2 = path_2 + fileName_2;  

			const char * filename_2=fileFullName_2.c_str(); 

			if( remove(filename_2) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_2);
			else
				perror("remove");
			//}
		}
	}
	//===========�ұ��ϴ���end==============================

	//===========�ұ��´���start============================
	IplImage *src_3;
	float s_3[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_3;  
	string path_3 = "E:\\�������\\5\\";  
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

			const char * filename_3=fileFullName_3.c_str();  //��string���͵�·��ת����char��

			src_3=cvLoadImage (filename_3, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_3->height;i++)
				for(int j=0;j<src_3->width;j++)
				{

					if(((uchar *)(src_3->imageData + i*src_3->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_3=(float)n/(src_3->height*src_3->width);  //�����ɫ����������ͼ�е�ռ��
				s_3[p]=per_p_3;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_3=s_3[0]; //�����һ������Сֵ
		int index_3=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_3, "copy %s E:\\�����\\%04d_3.jpg", filename1_3,order-1);
		system(cmd_3);

		//=========ɾͼ=========================
		for(int i=0;i<size_3;i++)
		{

			string fileName_3 = filenames_3[i];  
			string fileFullName_3 = path_3 + fileName_3;  

			const char * filename_3=fileFullName_3.c_str(); 

			if( remove(filename_3) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_3);
			else
				perror("remove");
			//}
		}
	}

	//===========�ұ��´���end==============================
	//===========����ϴ���start============================
	IplImage *src_4;
	float s_4[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_4;  
	string path_4 = "E:\\�������\\5\\";  
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

			const char * filename_4=fileFullName_4.c_str();  //��string���͵�·��ת����char��

			src_4=cvLoadImage (filename_4, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_4->height;i++)
				for(int j=0;j<src_4->width;j++)
				{

					if(((uchar *)(src_4->imageData + i*src_4->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_4=(float)n/(src_4->height*src_4->width);  //�����ɫ����������ͼ�е�ռ��
				s_4[p]=per_p_4;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_4=s_4[0]; //�����һ������Сֵ
		int index_4=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_4, "copy %s E:\\�����\\%04d_4.jpg", filename1_4,order-1);
		system(cmd_4);

		//=========ɾͼ=========================
		for(int i=0;i<size_4;i++)
		{

			string fileName_4 = filenames_4[i];  
			string fileFullName_4 = path_4 + fileName_4;  

			const char * filename_4=fileFullName_4.c_str(); 

			if( remove(filename_4) == 0 );   //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_4);
			else
				perror("remove");
			//}
		}
	}

	//===========����ϴ���end==============================
	//===========����´���start============================
	IplImage *src_5;
	float s_5[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_5;  
	string path_5 = "E:\\�������\\5\\";  
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

			const char * filename_5=fileFullName_5.c_str();  //��string���͵�·��ת����char��

			src_5=cvLoadImage (filename_5, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_5->height;i++)
				for(int j=0;j<src_5->width;j++)
				{

					if(((uchar *)(src_5->imageData + i*src_5->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_5=(float)n/(src_5->height*src_5->width);  //�����ɫ����������ͼ�е�ռ��
				s_5[p]=per_p_5;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_5=s_5[0]; //�����һ������Сֵ
		int index_5=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_5, "copy %s E:\\�����\\%04d_5.jpg", filename1_5,order-1);
		system(cmd_5);

		//=========ɾͼ=========================
		for(int i=0;i<size_5;i++)
		{

			string fileName_5 = filenames_5[i];  
			string fileFullName_5 = path_5 + fileName_5;  

			const char * filename_5=fileFullName_5.c_str(); 

			if( remove(filename_5) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
			//	printf("Removed %s.\n", filename_5);
			else
				perror("remove");
		}
	}

	//===========����´���end==============================

	//===========�����ϴ���start============================
	IplImage *src_6;
	float s_6[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_6;  
	string path_6 = "E:\\�������\\5\\";  
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

			const char * filename_6=fileFullName_6.c_str();  //��string���͵�·��ת����char��

			src_6=cvLoadImage (filename_6, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_6->height;i++)
				for(int j=0;j<src_6->width;j++)
				{

					if(((uchar *)(src_6->imageData + i*src_6->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_6=(float)n/(src_6->height*src_6->width);  //�����ɫ����������ͼ�е�ռ��
				s_6[p]=per_p_6;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_6=s_6[0]; //�����һ������Сֵ
		int index_6=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_6, "copy %s E:\\�����\\%04d_6.jpg", filename1_6,order-1);
		system(cmd_6);

		//=========ɾͼ=========================
		for(int i=0;i<size_6;i++)
		{

			string fileName_6 = filenames_6[i];  
			string fileFullName_6 = path_6 + fileName_6;  

			const char * filename_6=fileFullName_6.c_str(); 

			if( remove(filename_6) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_6);
			else
				perror("remove");
		}
	}


	//===========�����ϴ���end==============================
	//===========�����´���start============================
	IplImage *src_7;
	float s_7[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_7;  
	string path_7 = "E:\\�������\\5\\";  
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

			const char * filename_7=fileFullName_7.c_str();  //��string���͵�·��ת����char��

			src_7=cvLoadImage (filename_7, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_7->height;i++)
				for(int j=0;j<src_7->width;j++)
				{

					if(((uchar *)(src_7->imageData + i*src_7->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_7=(float)n/(src_7->height*src_7->width);  //�����ɫ����������ͼ�е�ռ��
				s_7[p]=per_p_7;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_7=s_7[0]; //�����һ������Сֵ
		int index_7=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_7, "copy %s E:\\�����\\%04d_7.jpg", filename1_7,order-1);
		system(cmd_7);

		//=========ɾͼ=========================
		for(int i=0;i<size_7;i++)
		{

			string fileName_7 = filenames_7[i];  
			string fileFullName_7 = path_7 + fileName_7;  

			const char * filename_7=fileFullName_7.c_str(); 

			if( remove(filename_7) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_7);
			else
				perror("remove");
		}
	}


	//===========�����´���end==============================
	//===========�����ϴ���start============================

	IplImage *src_8;
	float s_8[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_8;  
	string path_8 = "E:\\�������\\5\\";  
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

			const char * filename_8=fileFullName_8.c_str();  //��string���͵�·��ת����char��

			src_8=cvLoadImage (filename_8, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_8->height;i++)
				for(int j=0;j<src_8->width;j++)
				{

					if(((uchar *)(src_8->imageData + i*src_8->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_8=(float)n/(src_8->height*src_8->width);  //�����ɫ����������ͼ�е�ռ��
				s_8[p]=per_p_8;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_8=s_8[0]; //�����һ������Сֵ
		int index_8=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_8, "copy %s E:\\�����\\%04d_8.jpg", filename1_8,order-1);
		system(cmd_8);

		//=========ɾͼ=========================
		for(int i=0;i<size_8;i++)
		{

			string fileName_8 = filenames_8[i];  
			string fileFullName_8 = path_8 + fileName_8;  

			const char * filename_8=fileFullName_8.c_str(); 

			if( remove(filename_8) == 0 );   //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_8);
			else
				perror("remove");
		}
	}




	//===========�����ϴ���end==============================

	//===========�����´���start============================
	IplImage *src_9;
	float s_9[100];   //����һ�����飬���ڶ�ÿһ��ͼ��İ�ɫ����ռ��ֵ��������,��100�ѽ��㹻����һ����λӦ�ò�����100֡

	//=================��ָ��·���µ�ĳ�ֱ�ŵ�ͼƬ================
	Directory dir_9;  
	string path_9 = "E:\\�������\\5\\";  
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

			const char * filename_9=fileFullName_9.c_str();  //��string���͵�·��ת����char��

			src_9=cvLoadImage (filename_9, 0);   //ֱ��ת���ɻҶ�ͼ


			int n=0;  //ͳ�ư�ɫ���صĸ���  
			for(int i=0;i<src_9->height;i++)
				for(int j=0;j<src_9->width;j++)
				{

					if(((uchar *)(src_9->imageData + i*src_9->widthStep))[j]==255)  //����Ҷ�ͼ�а�ɫ���صĸ���
						n++; 

				}
				float per_p_9=(float)n/(src_9->height*src_9->width);  //�����ɫ����������ͼ�е�ռ��
				s_9[p]=per_p_9;	  //�Ѷัͼ��ռ��ֵд��������

		}
		//==========�ҳ���С��ռ��ֵ======================
		float min_9=s_9[0]; //�����һ������Сֵ
		int index_9=0;   //���ڼ�¼�����ҵ�����Сֵ���±�

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

		sprintf(cmd_9, "copy %s E:\\�����\\%04d_9.jpg", filename1_9,order-1);
		system(cmd_9);

		//=========ɾͼ=========================
		for(int i=0;i<size_9;i++)
		{

			string fileName_9 = filenames_9[i];  
			string fileFullName_9 = path_9 + fileName_9;  

			const char * filename_9=fileFullName_9.c_str(); 

			if( remove(filename_9) == 0 ) ;  //remove����0��ʾɾ���ɹ���		
				//printf("Removed %s.\n", filename_9);
			else
				perror("remove");
		}
	}
	//===========�����´���end==============================
}
