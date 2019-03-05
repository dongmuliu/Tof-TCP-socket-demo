
#include <string>
#include<vector>
#include<stdint.h>
#include <stdio.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include<vector>
#include<stdint.h>
#include <stdio.h>
#include <winsock2.h>

using namespace cv;
using namespace std;

#define MAXLINE 253600
#define MAX_NUM_PIX	82656	//328 * 252
#define LOW_AMPLITUDE 	32500
#define MAX_PHASE        30000.0
#define MAX_DIST_VALUE 	30000
#define offsetPhaseDefault 0
#define   FX  286.6034
#define   FY  287.3857	
#define   CX  176.2039
#define   CY  126.5788	
#define   K1  -0.12346
#define   K2  0.159423
int		drnuLut[50][252][328];
#pragma comment (lib, "ws2_32.lib")  //加载 ws2_32.dll
char buf[MAXLINE];
char ptr_buf2[MAXLINE];
char sendline[] = "getDistanceSorted";
uint16_t raw_dep;
int scale = 1;
int delta = 0;
int ddepth = -1;
int realindex, realrow, realcol;
uint16_t fameDepthArray2[MAXLINE];
uint16_t depth[240][320];
unsigned char* ptr_buf_unsigned = (unsigned char*)ptr_buf2;
void imageAverageEightConnectivity(ushort *depthdata);
void calculationAddOffset(ushort *img);
int calculationCorrectDRNU(ushort * img);

int n;
void socket_com(char sendline[], int length)
{

	WSADATA wsaData;
	WSAStartup(MAKEWORD(2, 2), &wsaData);
	char* ptr_buf = buf;
	int count = 0;
	SOCKET  sockfd;
	int rec_len, Ret;
	struct sockaddr_in servaddr;

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd == INVALID_SOCKET)
	{
		cout << "Create Socket Failed::" << GetLastError() << endl;
		exit(0);
	}


	memset(&servaddr, 0, sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(50660);
	servaddr.sin_addr.s_addr = inet_addr("192.168.7.63");
	//if (inet_pton(AF_INET, "192.168.7.3", &servaddr.sin_addr) <= 0) {
	//	printf("inet_pton error for %s\n", "ip");
	//	exit(0);
	//}
	Ret = connect(sockfd, (SOCKADDR*)&servaddr, sizeof(servaddr));
	if (Ret == SOCKET_ERROR)
	{
		cout << "Connect Error::" << GetLastError() << endl;
		exit(0);
	}

	else
	{
		cout << "连接成功!" << endl;
	}
	/*
	if (connect(sockfd, (SOCKADDR*)&servaddr, sizeof(SOCKADDR))
	< 0) {
	printf("connect error: %s(errno: %d)\n", strerror(errno), errno);
	exit(0);
	}
	*/
	Ret = send(sockfd, sendline, strlen(sendline), 0);
	if (Ret == SOCKET_ERROR)
	{
		cout << "Send Info Error::" << GetLastError() << endl;
		exit(0);
	}
	else
	{
		//cout << "send成功!" << endl;
	}
	/*
	if (send(sockfd, sendline, strlen(sendline), 0) < 0) {
	printf("send msg error: %s(errno: %d)\n", strerror(errno), errno);
	exit(0);
	}
	*/

	int i2 = 0;
	while (count < length) //153600
	{
		rec_len = recv(sockfd, buf, MAXLINE, 0);
		for (int i = 0; i < rec_len; i++)
		{

			ptr_buf2[i2] = buf[i];
			i2++;

		}
		if (rec_len == SOCKET_ERROR)
		{
			cout << "接受Error::" << GetLastError() << endl;
			exit(0);
		}

		count = count + rec_len;

	}
	//关闭套接字
	closesocket(sockfd);
	//终止使用 DLL
	WSACleanup();

	//system("pause");

}
void calibrate(ushort *img)
{
	
	imageAverageEightConnectivity(img);
	
	
	calculationCorrectDRNU(img);
	
	calculationAddOffset(img);
	
}
//畸变矫正
//输入： 待矫正的图片
//输出： 校正后的图片
Mat undistimg(Mat src)
{
	
	Mat img;

	//内参矩阵
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);		//3*3单位矩阵
	cameraMatrix.at<double>(0, 0) = FX;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = CX;
	cameraMatrix.at<double>(1, 1) = FY;
	cameraMatrix.at<double>(1, 2) = CY;
	cameraMatrix.at<double>(2, 2) = 1;

	//畸变参数
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);		//5*1全0矩阵
	distCoeffs.at<double>(0, 0) = K1;
	distCoeffs.at<double>(1, 0) = K2;
	distCoeffs.at<double>(2, 0) = 0;
	distCoeffs.at<double>(3, 0) = 0;
	distCoeffs.at<double>(4, 0) = 0;

	Size imageSize = src.size();
	Mat map1, map2;
	//参数1：相机内参矩阵
	//参数2：畸变矩阵
	//参数3：可选输入，第一和第二相机坐标之间的旋转矩阵
	//参数4：校正后的3X3相机矩阵
	//参数5：无失真图像尺寸
	//参数6：map1数据类型，CV_32FC1或CV_16SC2
	//参数7、8：输出X/Y坐标重映射参数
	cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, imageSize, CV_32FC1, map1, map2);	//计算畸变映射
	//参数1：畸变原始图像
	//参数2：输出图像
	//参数3、4：X\Y坐标重映射
	//参数5：图像的插值方式
	//参数6：边界填充方式
	cv::remap(src, img, map1, map2, INTER_LINEAR);																	//畸变矫正
	return img.clone();
}
//均值滤波
void imageAverageEightConnectivity(ushort *depthdata)
{
	
		int pixelCounter;
		int nCols = 320;
		int nRowsPerHalf = 120;
		//int size = nCols * nRowsPerHalf * 2;
		int size = 320 * 240;
		ushort actualFrame[MAX_NUM_PIX];
		int i, j, index;
		int pixdata;
		memcpy(actualFrame, depthdata, size * sizeof(ushort));
		// up side
		// dowm side
		// left side and right side
		// normal part
		for (i = 1; i < 239; i++) {
			for (j = 1; j < 319; j++){
				index = i * 320 + j;
				pixelCounter = 0;
				pixdata = 0;
				if (actualFrame[index] < 30000) {
					pixelCounter++;
					pixdata += actualFrame[index];
				}
				if (actualFrame[index - 1]  < 30000) {   // left
					pixdata += actualFrame[index - 1];
					pixelCounter++;
				}
				if (actualFrame[index + 1]  < 30000) {   // right
					pixdata += actualFrame[index + 1];
					pixelCounter++;
				}
				if (actualFrame[index - 321]  < 30000) {   // left up
					pixdata += actualFrame[index - 321];
					pixelCounter++;
				}
				if (actualFrame[index - 320]  < 30000) {   // up
					pixdata += actualFrame[index - 320];
					pixelCounter++;
				}
				if (actualFrame[index - 319]  < 30000) {   // right up
					pixdata += actualFrame[index - 319];
					pixelCounter++;
				}
				if (actualFrame[index + 319]  < 30000) {   // left down
					pixdata += actualFrame[index - 321];
					pixelCounter++;
				}
				if (actualFrame[index + 320]  < 30000) {   // down
					pixdata += actualFrame[index - 320];
					pixelCounter++;
				}
				if (actualFrame[index + 321]  < 30000) {   // right down
					pixdata += actualFrame[index - 319];
					pixelCounter++;
				}

				if (pixelCounter < 6) {
					*(depthdata + index) = LOW_AMPLITUDE;
				}
				else {
					*(depthdata + index) = pixdata / pixelCounter;
				}
			}
		}
		//
		//end part

}
//温度校正
int calculationCorrectDRNU(ushort * img)
{
	//int i, x, y, l;
	//double dist, tempDiff = 0;
	////printf("gOffsetDRNU = %d\n", offset);  //w
	//uint16_t maxDistanceMM = 150000000 / 12000;
	//double stepLSB = gStepCalMM * MAX_PHASE / maxDistanceMM;
	////printf("stepLSB = %2.4f\n", stepLSB);  //w

	//uint16_t *pMem = img;
	//int width = 320;
	//int height = 240;
	//tempDiff = realTempChip - gTempCal_Temp;
	//for (l = 0, y = 0; y< height; y++){
	//	for (x = 0; x < width; x++, l++){
	//		dist = pMem[l];

	//		if (dist < LOW_AMPLITUDE){	//correct if not saturated
	//			
	//			if (dist<0)	//if phase jump
	//				dist += MAX_DIST_VALUE;

	//			i = (int)(round(dist / stepLSB));

	//			if (i<0) i = 0;
	//			else if (i >= 50) i = 49;

	//			dist = (double)pMem[l] - drnuLut[i][y][x];
	//			/*if (l == 1300){
	//			printf(" DRNULutPixel = %d,", drnuLut[i][y][x]);
	//			}
	//			if (l == 1301){
	//			printf("  %d,", drnuLut[i][y][x]);
	//			}
	//			if (l == 1302){
	//			printf(" %d \n", drnuLut[i][y][x]);
	//			}*/
	//			// temprature cali
	//			dist -= tempDiff * 3.12262;	// 0.312262 = 15.6131 / 50

	//			pMem[l] = (uint16_t)round(dist);

	//		}	//end if LOW_AMPLITUDE
	//	}	//end width
	//}	//end height
	////printf(" pMem = %d, %d, %d\n", pMem[1300], pMem[1301], pMem[1302]);
	return 0;

}
//深度补偿
void calculationAddOffset(ushort *img)
{
	int offset = 0;
	uint16_t maxDistanceCM = 0;
	int l = 0;
	//offsetPhase = MAX_PHASE / (gSpeedOfLightDiv2 / configGetModulationFrequency(deviceAddress) / 10) * value;	
	offset = offsetPhaseDefault;
	//printf("offset 1 = %d\n", offset);
	uint16_t val;
	uint16_t *pMem = img;
	int numPix = 320 * 240;

	for (l = 0; l<numPix; l++){
		val = pMem[l];
		if (val < 30000) { //if not low amplitude, not saturated and not  adc overflow
			pMem[l] = (val + offset) % MAX_DIST_VALUE;
		}
	}
}
int main()
{

	

	int count = 0;
	double hu[7];
	char filename[100];
	while (1)
	{

		socket_com(sendline, 153600);
		double time0 = static_cast<double>(getTickCount());
		for (int j = 0; j < 76800; j++)
		{
			raw_dep = ptr_buf_unsigned[j * 2 + 1] * 256 + ptr_buf_unsigned[2 * j];
			//cout << raw_dep << " ";
			realindex = 76800 - (j / 320 + 1) * 320 + j % 320;   //镜像
			realrow = 239 - j / 320;
			realcol = j % 320;
			fameDepthArray2[realindex] = raw_dep;

			//cout << fameDepthArray2[j] << " ";
		}
		//滤波
		calibrate(fameDepthArray2);
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				depth[i][j] = fameDepthArray2[i * 320 + j];
			}
		}

		Mat src_1(240, 320, CV_16UC1, Scalar(0));
		Mat imshowsrc(240, 320, CV_8UC1, Scalar(0));
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				src_1.at<ushort>(i, j) = depth[i][j];

			}

		}
		//畸变矫正
	 Mat src_2=	undistimg(src_1);
		
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				imshowsrc.at<uchar>(i, j) = 255 - src_2.at<ushort>(i, j) * 25 / 3000;
				//cout << src2.at<uchar>(i, j) << endl;
			}

		}


		imshow("test", imshowsrc);
		waitKey(1);
	}
	return 0;

}