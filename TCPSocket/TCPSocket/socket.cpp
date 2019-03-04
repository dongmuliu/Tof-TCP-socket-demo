
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
#pragma comment (lib, "ws2_32.lib")  //加载 ws2_32.dll
char buf[MAXLINE];
char ptr_buf2[MAXLINE];
char sendline[] = "getDistanceSorted";
uint16_t raw_dep;
int scale = 1;
int delta = 0;
int ddepth = -1;
int realindex, realrow, realcol;
//vector<vector<uint16_t> >fameDepthArray;
uint16_t fameDepthArray2[MAXLINE];
uint16_t depth[240][320];
//ushort raw_array[322][239];
//ushort real_array[321][239];
unsigned char* ptr_buf_unsigned = (unsigned char*)ptr_buf2;


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
int main()
{

	Mat img;
	Mat img_threshold;
	Mat img_gray;
	Mat img_roi;

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
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				depth[i][j] = fameDepthArray2[i * 320 + j];
			}
		}

		Mat src_1(240, 320, CV_16UC1, Scalar(0));
		Mat src_2(240, 320, CV_8UC1, Scalar(0));
		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				src_1.at<ushort>(i, j) = depth[i][j];

			}

		}

		for (int i = 0; i < 240; i++)
		{
			for (int j = 0; j < 320; j++)
			{
				src_2.at<uchar>(i, j) = 255 - src_1.at<ushort>(i, j) * 25 / 3000;
				//cout << src2.at<uchar>(i, j) << endl;
			}

		}


		imshow("src2", src_2);
		waitKey(1);
	}
	return 0;

}