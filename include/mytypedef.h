#include<stdio.h>
#include<string.h>
#include<sys/types.h>
#include<iostream>

typedef struct BITMAPFILEHEADER
{
    u_int16_t bfType;
    u_int32_t bfSize;
    u_int16_t bfReserved1;
    u_int16_t bfReserved2;
    u_int32_t bfOffBits;
}BITMAPFILEHEADER;

typedef struct BITMAPINFOHEADER
{
    u_int32_t biSize;
    u_int32_t biWidth;
    u_int32_t biHeight;
    u_int16_t biPlanes;
    u_int16_t biBitCount;
    u_int32_t biCompression;
    u_int32_t biSizeImage;
    u_int32_t biXPelsPerMeter;
    u_int32_t biYPelsPerMeter;
    u_int32_t biClrUsed;
    u_int32_t biClrImportant;
}BITMAPINFODEADER;

void showBmpHead(BITMAPFILEHEADER &pBmpHead)
{
    std::cout<<"位图文件头:"<<std::endl;
    std::cout<<"文件头类型:"<<pBmpHead.bfType<<std::endl;
    std::cout<<"文件大小:"<<pBmpHead.bfSize<<std::endl;
    std::cout<<"保留字_1:"<<pBmpHead.bfReserved1<<std::endl;
    std::cout<<"保留字_2:"<<pBmpHead.bfReserved2<<std::endl;
    std::cout<<"实际位图数据的偏移字节数:"<<pBmpHead.bfOffBits<<std::endl<<std::endl;
}

void showBmpInforHead(BITMAPINFODEADER &pBmpInforHead)
{
    std::cout<<"位图信息头:"<<std::endl;
    std::cout<<"结构体的长度:"<<pBmpInforHead.biSize<<std::endl;
    std::cout<<"位图宽:"<<pBmpInforHead.biWidth<<std::endl;
    std::cout<<"位图高:"<<pBmpInforHead.biHeight<<std::endl;
    std::cout<<"biPlanes平面数:"<<pBmpInforHead.biPlanes<<std::endl;
    std::cout<<"biBitCount采用颜色位数:"<<pBmpInforHead.biBitCount<<std::endl;
    std::cout<<"压缩方式:"<<pBmpInforHead.biCompression<<std::endl;
    std::cout<<"biSizeImage实际位图数据占用的字节数:"<<pBmpInforHead.biSizeImage<<std::endl;
    std::cout<<"X方向分辨率:"<<pBmpInforHead.biXPelsPerMeter<<std::endl;
    std::cout<<"Y方向分辨率:"<<pBmpInforHead.biYPelsPerMeter<<std::endl;
    std::cout<<"使用的颜色数:"<<pBmpInforHead.biClrUsed<<std::endl;
    std::cout<<"重要颜色数:"<<pBmpInforHead.biClrImportant<<std::endl;
}