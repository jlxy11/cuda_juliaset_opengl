/*CPU*/
#include <GL/glut.h>
#include <cmath>
#include <chrono>
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

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glLoadIdentity();

    /*double dx = 2.0 / WIDTH;
    double dy = 2.0 / HEIGHT;*/

    float dx = (MAX_REAL - MIN_REAL) / WIDTH;
    float dy = (MAX_IMAG - MIN_IMAG) / HEIGHT;

    float t = glutGet(GLUT_ELAPSED_TIME) * 0.0002; // 获取时间，单位为秒
    //double c_real = -0.8;
    //double c_imag = 0.2 * cos(t);
    float c_real = 0.7885 * sin(t);
    float c_imag = 0.7885 * cos(t);
    glBegin(GL_POINTS);
    for (int i = 0; i < WIDTH; ++i)
    {
        for (int j = 0; j < HEIGHT; ++j)
        {

            //double x = (-1.0 + i * dx);
            //double y = (-1.0 + j * dy);
            float x = MIN_REAL + i * dx;
            float y = MIN_IMAG + j * dy;

            float zx = x;
            float zy = y;

            int iter = 0;
            while (iter < MAX_ITER && zx * zx + zy * zy < BOUNDARY * BOUNDARY)
            {
                float tmp = zx * zx - zy * zy + c_real;
                zy = 2.0 * zx * zy + c_imag;
                zx = tmp;
                ++iter;
            }

            // 根据迭代次数绘制不同颜色的点
            //float color = 1.0f - static_cast<float>(iter) / MAX_ITER;
            float colorRED = (-130.0 * sinf(0.045 * (iter + 13.3)) + 160.0) / 255.0;
            float colorGREEN = (-80.0 * sinf(0.045 * (iter + 0.0)) + 40.0) / 225.0;
            float colorBLUE = (60.0 * sinf(0.1 * (iter - 29.3)) + 80.0) / 225.0;

            if (colorRED > 1.0)
            {
                colorRED = 1.0;
            }
            else if (colorRED < 0.0)
            {
                colorRED = 0.0;
            }

            if (colorGREEN > 1.0)
            {
                colorGREEN = 1.0;
            }
            else if (colorGREEN < 0.0)
            {
                colorGREEN = 0.0;
            }

            if (colorBLUE > 1.0)
            {
                colorBLUE = 1.0;
            }
            else if (colorBLUE < 0.0)
            {
                colorBLUE = 0.0;
            }

            //if (iter == MAX_ITER)
            //{
            //    colorRED = 0.0;
            //    colorGREEN = 0.0;
            //    colorBLUE = 0.0;

            //}
            //
            glColor3f(colorRED, colorGREEN, colorBLUE);
            //glColor3f(color, color, color);
            glVertex2d(x, y);
            //A*SIN((I-D)*K)+C

        }
    }

    glEnd();

    glFlush();
    glutSwapBuffers();
    
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    double elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - previousTime).count() / 1000.0;
    frameCount++;
    if (elapsedTime >= 1.0) {
        frameRate = frameCount / elapsedTime;
        frameCount = 0;
        previousTime = currentTime;
    }

    // 在控制台输出帧率
    std::cout << "Frame rate: " << frameRate << " fps" << std::endl;
}

void reshape(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //gluOrtho2D(-1, 1, -1, 1);
    gluOrtho2D(MIN_REAL, MAX_REAL, MIN_IMAG, MAX_IMAG);
    glMatrixMode(GL_MODELVIEW);
}

int main(int argc, char** argv)
{
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