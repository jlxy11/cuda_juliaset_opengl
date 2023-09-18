/*CPU*/
//#include <GL/glut.h>
//#include <cmath>
//#include <chrono>
//#include<iostream>
//
//std::chrono::high_resolution_clock::time_point previousTime;
//int frameCount = 0;
//float frameRate = 0.0;
//
//
//const int n = 800;
//const int WIDTH = n;
//const int HEIGHT = n;
//
//const int MAX_ITER = 100;
//const float BOUNDARY = 2.0;
//
//const float MIN_REAL = -2.0;
//const float MAX_REAL = 2.0;
//const float MIN_IMAG = -2.0;
//const float MAX_IMAG = 2.0;
//
//void display()
//{
//    glClear(GL_COLOR_BUFFER_BIT);
//    glLoadIdentity();
//
//    /*double dx = 2.0 / WIDTH;
//    double dy = 2.0 / HEIGHT;*/
//
//    float dx = (MAX_REAL - MIN_REAL) / WIDTH;
//    float dy = (MAX_IMAG - MIN_IMAG) / HEIGHT;
//
//    float t = glutGet(GLUT_ELAPSED_TIME) * 0.0002; // 获取时间，单位为秒
//    //double c_real = -0.8;
//    //double c_imag = 0.2 * cos(t);
//    float c_real = 0.7885 * sin(t);
//    float c_imag = 0.7885 * cos(t);
//    glBegin(GL_POINTS);
//    for (int i = 0; i < WIDTH; ++i)
//    {
//        for (int j = 0; j < HEIGHT; ++j)
//        {
//
//            //double x = (-1.0 + i * dx);
//            //double y = (-1.0 + j * dy);
//            float x = MIN_REAL + i * dx;
//            float y = MIN_IMAG + j * dy;
//
//            float zx = x;
//            float zy = y;
//
//            int iter = 0;
//            while (iter < MAX_ITER && zx * zx + zy * zy < BOUNDARY * BOUNDARY)
//            {
//                float tmp = zx * zx - zy * zy + c_real;
//                zy = 2.0 * zx * zy + c_imag;
//                zx = tmp;
//                ++iter;
//            }
//
//            // 根据迭代次数绘制不同颜色的点
//            //float color = 1.0f - static_cast<float>(iter) / MAX_ITER;
//            float colorRED = (-130.0 * sinf(0.045 * (iter + 13.3)) + 160.0) / 255.0;
//            float colorGREEN = (-80.0 * sinf(0.045 * (iter + 0.0)) + 40.0) / 225.0;
//            float colorBLUE = (60.0 * sinf(0.1 * (iter - 29.3)) + 80.0) / 225.0;
//
//            if (colorRED > 1.0)
//            {
//                colorRED = 1.0;
//            }
//            else if (colorRED < 0.0)
//            {
//                colorRED = 0.0;
//            }
//
//            if (colorGREEN > 1.0)
//            {
//                colorGREEN = 1.0;
//            }
//            else if (colorGREEN < 0.0)
//            {
//                colorGREEN = 0.0;
//            }
//
//            if (colorBLUE > 1.0)
//            {
//                colorBLUE = 1.0;
//            }
//            else if (colorBLUE < 0.0)
//            {
//                colorBLUE = 0.0;
//            }
//
//            //if (iter == MAX_ITER)
//            //{
//            //    colorRED = 0.0;
//            //    colorGREEN = 0.0;
//            //    colorBLUE = 0.0;
//
//            //}
//            //
//            glColor3f(colorRED, colorGREEN, colorBLUE);
//            //glColor3f(color, color, color);
//            glVertex2d(x, y);
//            //A*SIN((I-D)*K)+C
//
//        }
//    }
//
//    glEnd();
//
//    glFlush();
//    glutSwapBuffers();
//    
//    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
//    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousTime).count() / 1000.0;
//    frameCount++;
//    if (elapsedTime >= 1.0) {
//        frameRate = frameCount / elapsedTime;
//        frameCount = 0;
//        previousTime = currentTime;
//    }
//
//    // 在控制台输出帧率
//    std::cout << "Frame rate: " << frameRate << " fps" << std::endl;
//}
//
//void reshape(int w, int h)
//{
//    glViewport(0, 0, w, h);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    //gluOrtho2D(-1, 1, -1, 1);
//    gluOrtho2D(MIN_REAL, MAX_REAL, MIN_IMAG, MAX_IMAG);
//    glMatrixMode(GL_MODELVIEW);
//}
//
//int main(int argc, char** argv)
//{
//    previousTime = std::chrono::high_resolution_clock::now();
//    glutInit(&argc, argv);
//    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
//    glutInitWindowSize(WIDTH, HEIGHT);
//    glutCreateWindow("Julia Set Visualization");
//    glutDisplayFunc(display);
//    glutReshapeFunc(reshape);
//    glutIdleFunc(display); // 设置空闲时刷新画面
//    glutMainLoop();
//    return 0;
//}

/*CUDA_slow*/
//#include <GL/glut.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <cuda_runtime.h>
//#include "device_launch_parameters.h"
//#include<chrono>
//#include<iostream>
//
//std::chrono::high_resolution_clock::time_point previousTime;
//int frameCount = 0;
//double frameRate = 0.0;
//
//
//const int n = 800;
//const int WIDTH = n;
//const int HEIGHT = n;
//
//const int MAX_ITER = 100;
//const double BOUNDARY = 2.0;
//
//const double MIN_REAL = -2.0;
//const double MAX_REAL = 2.0;
//const double MIN_IMAG = -2.0;
//const double MAX_IMAG = 2.0;
//
//__global__ void juliaSetKernel(float* output, float* xpix, float* ypix, float t)
//{
//    int idx = blockIdx.x * blockDim.x + threadIdx.x;
//    int idy = blockIdx.y * blockDim.y + threadIdx.y;
//    int offset = idy * WIDTH + idx;
//    if (offset < WIDTH * HEIGHT)
//    {
//        float x = (idx - WIDTH / 2) / (float)WIDTH * 4;
//        float y = (idy - HEIGHT / 2) / (float)HEIGHT * 4;
//        //float c_real = 0.0;
//        //float c_imag = 0.67;
//        double c_real = 0.7885 * sin(t);
//        double c_imag = 0.7885 * cos(t);
//        float z_real = x;
//        float z_imag = y;
//
//        int iter;
//        for (iter = 0; iter < MAX_ITER; iter++) {
//            float z_real2 = z_real * z_real;
//            float z_imag2 = z_imag * z_imag;
//            if (z_real2 + z_imag2 > 4.0)
//                break;
//
//            float tmp = z_real2 - z_imag2 + c_real;
//            z_imag = 2.0 * z_real * z_imag + c_imag;
//            z_real = tmp;
//        }
//
//        output[offset] = iter;
//        xpix[offset] = x;
//        ypix[offset] = y;
//    }
//
//    
//}
//
//void juliaSet(float* output, float* xpix, float* ypix, float t)
//{
//    float* dev_output;
//    float* dev_xpix;
//    float* dev_ypix;
//
//    cudaMalloc((void**)&dev_output, WIDTH * HEIGHT * sizeof(float));
//    cudaMalloc((void**)&dev_xpix, WIDTH * HEIGHT * sizeof(float));
//    cudaMalloc((void**)&dev_ypix, WIDTH * HEIGHT * sizeof(float));
//
//    dim3 threadsPerBlock(16, 16);
//    dim3 numBlocks(WIDTH / threadsPerBlock.x, HEIGHT / threadsPerBlock.y);
//
//    juliaSetKernel << <numBlocks, threadsPerBlock >> > (dev_output,dev_xpix, dev_ypix, t);
//
//    cudaMemcpy(output, dev_output, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(xpix, dev_xpix, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(ypix, dev_ypix, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
//
//    cudaFree(dev_xpix);
//    cudaFree(dev_ypix);
//    cudaFree(dev_output);
//}
//
//void reshape(int w, int h)
//{
//    glViewport(0, 0, w, h);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    //gluOrtho2D(-1, 1, -1, 1);
//    gluOrtho2D(MIN_REAL, MAX_REAL, MIN_IMAG, MAX_IMAG);
//    glMatrixMode(GL_MODELVIEW);
//}
//
//void display()
//{
//    glClear(GL_COLOR_BUFFER_BIT);
//    glLoadIdentity();
//    double t = glutGet(GLUT_ELAPSED_TIME) * 0.0002; // 获取时间，单位为秒
//    float* output = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
//    float* x = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
//    float* y = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
//
//    juliaSet(output, x, y, t);
//
//    glBegin(GL_POINTS); // 开始绘制点
//
//    for (int i = 0; i < WIDTH * HEIGHT; i++)
//    {
//        float iter = output[i];
//
//        //float color = 1.0f - static_cast<float>(iter) / MAX_ITER;
//        //float color = 1.0f - static_cast<float>(iter) / MAX_ITER;
//        float colorRED = (-130.0 * sinf(0.045 * (iter + 13.3)) + 160.0) / 255.0;
//        float colorGREEN = (-80.0 * sinf(0.045 * (iter + 0.0)) + 40.0) / 225.0;
//        float colorBLUE = (60.0 * sinf(0.1 * (iter - 29.3)) + 80.0) / 225.0;
//
//        if (colorRED > 1.0)
//        {
//            colorRED = 1.0;
//        }
//        else if (colorRED < 0.0)
//        {
//            colorRED = 0.0;
//        }
//
//        if (colorGREEN > 1.0)
//        {
//            colorGREEN = 1.0;
//        }
//        else if (colorGREEN < 0.0)
//        {
//            colorGREEN = 0.0;
//        }
//
//        if (colorBLUE > 1.0)
//        {
//            colorBLUE = 1.0;
//        }
//        else if (colorBLUE < 0.0)
//        {
//            colorBLUE = 0.0;
//        }
//
//        glColor3f(colorRED, colorGREEN, colorBLUE);
//        //glColor3f(color, color, color);
//        glVertex2d(x[i], y[i]);
//    }
//    free(output);
//    free(x);
//    free(y);
//
//    glEnd(); // 结束绘制点
//
//    glFlush();
//    glutSwapBuffers();
//    // 计算帧率
//    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
//    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousTime).count() / 1000.0;
//    frameCount++;
//    if (elapsedTime >= 1.0) {
//        frameRate = frameCount / elapsedTime;
//        frameCount = 0;
//        previousTime = currentTime;
//    }
//
//    // 在控制台输出帧率
//    std::cout << "Frame rate: " << frameRate << " fps" << std::endl;
//
//}
//
//int main(int argc, char** argv)
//{
//    previousTime = std::chrono::high_resolution_clock::now();
//    glutInit(&argc, argv);
//    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
//    glutInitWindowSize(WIDTH, HEIGHT);
//    glutCreateWindow("Julia Set Visualization");
//    glutDisplayFunc(display);
//    glutReshapeFunc(reshape);
//    glutIdleFunc(display); // 设置空闲时刷新画面
//    glutMainLoop();
//    return 0;
//}

/*CUDA_static*/
//#include <stdio.h>
//#include <stdlib.h>
//#include <math.h>
//#include <cuda_runtime.h>
//#include "device_launch_parameters.h"
//
//#define WIDTH 800
//#define HEIGHT 800
//#define MAX_ITER 100
//
//
//__global__ void juliaSetKernel(float* output, float* xpix, float* ypix, float t)
//{
//    int idx = blockIdx.x * blockDim.x + threadIdx.x;
//    int idy = blockIdx.y * blockDim.y + threadIdx.y;
//    int offset = idy * WIDTH + idx;
//
//    float x = (idx - WIDTH / 2) / (float)WIDTH * 4;
//    float y = (idy - HEIGHT / 2) / (float)HEIGHT * 4;
//    float c_real = 0.7885 * sin(t);
//    float c_imag = 0.7885 * cos(t);
//    float z_real = x;
//    float z_imag = y;
//
//    int iter;
//    for (iter = 0; iter < MAX_ITER; iter++) {
//        float z_real2 = z_real * z_real;
//        float z_imag2 = z_imag * z_imag;
//        if (z_real2 + z_imag2 > 4.0)
//            break;
//
//        float tmp = z_real2 - z_imag2 + c_real;
//        z_imag = 2.0 * z_real * z_imag + c_imag;
//        z_real = tmp;
//    }
//
//    output[offset] = iter;
//    xpix[offset] = x;
//    ypix[offset] = y;
//}
//
//void juliaSet(float* output, float* xpix, float* ypix, float t)
//{
//    float* dev_output;
//    float* dev_xpix;
//    float* dev_ypix;
//
//    cudaMalloc((void**)&dev_output, WIDTH * HEIGHT * sizeof(float));
//    cudaMalloc((void**)&dev_xpix, WIDTH * HEIGHT * sizeof(float));
//    cudaMalloc((void**)&dev_ypix, WIDTH * HEIGHT * sizeof(float));
//
//
//    dim3 threadsPerBlock(16, 16);
//    dim3 numBlocks(WIDTH / threadsPerBlock.x, HEIGHT / threadsPerBlock.y);
//
//    juliaSetKernel << <numBlocks, threadsPerBlock >> > (dev_output, dev_xpix, dev_ypix,t);
//
//    cudaMemcpy(output, dev_output, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(xpix, dev_xpix, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
//    cudaMemcpy(ypix, dev_ypix, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
//
//    cudaFree(dev_output);
//    cudaFree(dev_xpix);
//    cudaFree(dev_ypix);
//
//}
//
//int main()
//{
//    float* output = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
//    float* x = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
//    float* y = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
//
//    float t = 1.0; // 当前时间
//    juliaSet(output, x, y, t);
//
//    // 输出存储迭代次数的数组
//    for (int i = 0; i < WIDTH * HEIGHT; i++) {
//        //printf("%f ", output[i]);
//        //printf("%f ", x[i],y[i]);
//        if (output[i] > 20.0)
//        {
//            printf("%f ", output[i]);
//        }
//    }
//    
//
//    free(output);
//    printf("done");
//    return 0;
//}

/*CUDA_last_version*/
#include <GL/glut.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cuda_runtime.h>
#include "device_launch_parameters.h"
#include<chrono>
#include<iostream>

std::chrono::high_resolution_clock::time_point previousTime;
int frameCount = 0;
float frameRate = 0.0;


const int n = 800;
const int WIDTH = n;
const int HEIGHT = n;

const int MAX_ITER = 100;
const float BOUNDARY = 2.0;

const float MIN_REAL = -2.0;
const float MAX_REAL = 2.0;
const float MIN_IMAG = -2.0;
const float MAX_IMAG = 2.0;


float* x = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
float* y = (float*)malloc(WIDTH * HEIGHT * sizeof(float));

void initdraw()
{
    for (int i = 0; i < WIDTH; ++i)
    {
        for (int j = 0; j < HEIGHT; ++j)
        {
            int offset = j * WIDTH + i;
            x[offset] = (i - WIDTH / 2) / (float)WIDTH * 4;
            y[offset] = (j - HEIGHT / 2) / (float)HEIGHT * 4;

        }
    }
}

__device__ int iterateJuliaSet(float z_real, float z_imag, float c_real, float c_imag)
{
    int iter;
    for (iter = 0; iter < MAX_ITER; iter++) {
        float z_real2 = z_real * z_real;
        float z_imag2 = z_imag * z_imag;
        if (z_real2 + z_imag2 > 4.0)
            break;

        float tmp = z_real2 - z_imag2 + c_real;
        z_imag = 2.0 * z_real * z_imag + c_imag;
        z_real = tmp;
    }
    return iter;
}

__global__ void juliaSetKernel(float* colorRED, float* colorGREEN, float* colorBLUE, float t)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    int offset = idy * WIDTH + idx;
    if (offset < WIDTH * HEIGHT)
    {
        float x = (idx - WIDTH / 2) / (float)WIDTH * 4;
        float y = (idy - HEIGHT / 2) / (float)HEIGHT * 4;
        float c_real = 0.7885 * sin(t);
        float c_imag = 0.7885 * cos(t);
        float z_real = x;
        float z_imag = y;

        int iter = iterateJuliaSet(z_real, z_imag, c_real, c_imag);
        //float color = 1.0f - static_cast<float>(iter) / MAX_ITER;
        //float color = 1.0f - static_cast<float>(iter) / MAX_ITER;
        colorRED[offset] = (-130.0 * sinf(0.045 * (iter + 13.3)) + 160.0) / 255.0;
        if (colorRED[offset] > 1.0)
        {
            colorRED[offset] = 1.0;
        }
        else if (colorRED[offset] < 0.0)
        {
            colorRED[offset] = 0.0;
        }

        colorGREEN[offset] = (-80.0 * sinf(0.045 * (iter + 0.0)) + 40.0) / 225.0;
        if (colorGREEN[offset] > 1.0)
        {
            colorGREEN[offset] = 1.0;
        }
        else if (colorGREEN[offset] < 0.0)
        {
            colorGREEN[offset] = 0.0;
        }

        colorBLUE[offset] = (60.0 * sinf(0.1 * (iter - 29.3)) + 80.0) / 225.0;
        if (colorBLUE[offset] > 1.0)
        {
            colorBLUE[offset] = 1.0;
        }
        else if (colorBLUE[offset] < 0.0)
        {
            colorBLUE[offset] = 0.0;
        }
    }
}

void juliaSet(float* colorRED, float* colorGREEN, float* colorBLUE, float t)
{
    float* dev_colorRED;
    float* dev_colorGREEN;
    float* dev_colorBLUE;

    cudaMalloc((void**)&dev_colorRED, WIDTH * HEIGHT * sizeof(float));
    cudaMalloc((void**)&dev_colorGREEN, WIDTH * HEIGHT * sizeof(float));
    cudaMalloc((void**)&dev_colorBLUE, WIDTH * HEIGHT * sizeof(float));

    dim3 threadsPerBlock(16, 16);
    dim3 numBlocks(WIDTH / threadsPerBlock.x, HEIGHT / threadsPerBlock.y);

    juliaSetKernel << <numBlocks, threadsPerBlock >> > (dev_colorRED, dev_colorGREEN, dev_colorBLUE, t);

    cudaMemcpy(colorRED, dev_colorRED, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(colorGREEN, dev_colorGREEN, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
    cudaMemcpy(colorBLUE, dev_colorBLUE, WIDTH * HEIGHT * sizeof(float), cudaMemcpyDeviceToHost);
    
    cudaFree(dev_colorRED);
    cudaFree(dev_colorGREEN);
    cudaFree(dev_colorBLUE);
}

void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(MIN_REAL, MAX_REAL, MIN_IMAG, MAX_IMAG);
    glMatrixMode(GL_MODELVIEW);
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();
    double t = glutGet(GLUT_ELAPSED_TIME) * 0.0002; // 获取时间，单位为秒
    float* colorRED = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
    float* colorGREEN = (float*)malloc(WIDTH * HEIGHT * sizeof(float));
    float* colorBLUE = (float*)malloc(WIDTH * HEIGHT * sizeof(float));

    juliaSet(colorRED, colorGREEN, colorBLUE, t);
    glBegin(GL_POINTS); // 开始绘制点
    
    for (int i = 0; i < WIDTH * HEIGHT; i++)
    {
        glColor3f(colorRED[i], colorGREEN[i], colorBLUE[i]);
        glVertex2d(x[i], y[i]);
    }
    free(colorRED);
    free(colorGREEN);
    free(colorBLUE);

    glEnd(); // 结束绘制点

    glFlush();
    glutSwapBuffers();
    // 计算帧率
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    float elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousTime).count() / 1000.0;
    frameCount++;
    if (elapsedTime >= 1.0) {
        frameRate = frameCount / elapsedTime;
        frameCount = 0;
        previousTime = currentTime;
    }

    // 在控制台输出帧率
    std::cout << "Frame rate: " << frameRate << " fps" << std::endl;

}

int main(int argc, char** argv)
{
    initdraw();
    previousTime = std::chrono::high_resolution_clock::now();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WIDTH, HEIGHT);
    glutCreateWindow("Julia Set Visualization");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutIdleFunc(display); // 设置空闲时刷新画面
    glutMainLoop();
    return 0;
}