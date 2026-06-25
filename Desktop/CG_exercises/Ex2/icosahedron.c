#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define _PI 3.14159f

#define INIT_TRANS 0.0f
#define INIT_ANGLE 0.0f
#define INIT_SCALE 1.0f
#define INIT_LEVEL 0

enum
{
    TRANS,
    ROT,
    SCALE
};
static int g_op_mode = TRANS;

enum
{
    PERSP,
    ORTHO
};
static int g_proj_mode = PERSP;

// window dimension
static int g_width = 800;
static int g_height = 800;

// angle (in degree) to rotate around x, y, z
static GLfloat g_angle[3] = {0.0f, 0.0f, 0.0f};

// amount to translate along x, y, z
static GLfloat g_trans[3] = {0.0f, 0.0f, 0.0f};

// scaling factor along x, y, z
static GLfloat g_scale[3] = {1.0f, 1.0f, 1.0f};

// フラクタルの階層レベル（初期値0、最大5）
static int g_level = 0;

static int g_use_custom = 0;

// Task3関数3つを追加
// 1. ベクトルの長さを計算する関数
static GLfloat Norm(GLfloat v[3])
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 2. ベクトルを正規化（長さ１にする）関数
static void Normalize(GLfloat v[3])
{
    GLfloat norm = Norm(v);
    if (fabs(norm) < 1e-5f)
        return;
    v[0] = v[0] / norm;
    v[1] = v[1] / norm;
    v[2] = v[2] / norm;
}

// vout = v1 x v2 (cross-product)
static void Cross(GLfloat v1[3], GLfloat v2[3],
                  GLfloat vout[3])
{
    vout[0] = v1[1] * v2[2] - v1[2] * v2[1];
    vout[1] = v1[2] * v2[0] - v1[0] * v2[2];
    vout[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

static void myTranslatef(GLfloat tx, GLfloat ty, GLfloat tz);

static void myLookAt(GLfloat eyex, GLfloat eyey, GLfloat eyez,
                     GLfloat lookatx, GLfloat lookaty, GLfloat lookatz,
                     GLfloat upx, GLfloat upy, GLfloat upz)
{
    GLfloat look_dir[3];
    GLfloat right[3];
    GLfloat up[3];
    GLfloat m[16];

    // 1.視線ベクトル
    look_dir[0] = lookatx - eyex;
    look_dir[1] = lookaty - eyey;
    look_dir[2] = lookatz - eyez;
    Normalize(look_dir);

    // 2.右方向ベクトル
    up[0] = upx;
    up[1] = upy;
    up[2] = upz;
    Normalize(up);
    Cross(look_dir, up, right);
    Normalize(right);

    // 3.上方向
    Cross(right, look_dir, up);
    Normalize(up);

    // 4. 回転行列 m を作成
    m[0] = right[0];
    m[4] = right[1];
    m[8] = right[2];
    m[12] = 0.0f;
    m[1] = up[0];
    m[5] = up[1];
    m[9] = up[2];
    m[13] = 0.0f;
    m[2] = -look_dir[0];
    m[6] = -look_dir[1];
    m[10] = -look_dir[2];
    m[14] = 0.0f;
    m[3] = 0.0f;
    m[7] = 0.0f;
    m[11] = 0.0f;
    m[15] = 1.0f;

    glMultMatrixf(m);

    myTranslatef(-eyex, -eyey, -eyez);
}

// Model-View transforms
static void myTranslatef(GLfloat tx, GLfloat ty, GLfloat tz)
{
    // 列優先の並行移動行列
    GLfloat m[16] = {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        tx, ty, tz, 1.0f};
    glMultMatrixf(m);
}

static void myScalef(GLfloat sx, GLfloat sy, GLfloat sz)
{
    // Complete
    // 列優先の拡大縮小行列
    GLfloat m[16] = {
        sx, 0.0f, 0.0f, 0.0f,
        0.0f, sy, 0.0f, 0.0f,
        0.0f, 0.0f, sz, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f};
    glMultMatrixf(m);
}

static void myRotatef(GLfloat theta, GLfloat kx, GLfloat ky, GLfloat kz)
{
    // 角度をラジアンに変換
    GLfloat rad = theta * _PI / 180.0f;
    GLfloat c = cosf(rad);
    GLfloat s = sinf(rad);
    GLfloat t = 1.0f - c;

    GLfloat len = sqrtf(kx*kx + ky*ky + kz*kz);
    if (len < 1e-5f) return;
    kx /= len;
    ky /= len;
    kz /= len;

    GLfloat m[16] = {0.0f};
    m[0] = t*kx*kx + c;
    m[1] = t*kx*ky + s+kz;
    m[2] = t*kx*kz - s*ky;
    m[3] = 0.0f;
    m[4] = t*kx*ky - s*kz;
    m[5] = t*ky*ky + c;
    m[6] = t*ky*kz + s*kx;
    m[7] = 0.0f;
    m[8] = t*kx*kz + s*ky;
    m[9] = t*ky*kz - s*kx;
    m[10] = t*kz*kz + c;
    m[11] = 0.0f;
    m[12] = 0.0f;
    m[13] = 0.0f;
    m[14] = 0.0f;
    m[15] = 0.0f;

    glMultMatrixf(m);
}

// Projection transforms
static void myOrtho(GLfloat left, GLfloat right, GLfloat bottom, GLfloat top, GLfloat n, GLfloat f)
{
    GLfloat m[16] = {0.0f};

    m[0] = 2.0f / (right - left);
    m[5] = 2.0f / (top - bottom);
    m[10] = -2.0f / (f - n);
    m[12] = -(right + left) / (right - left);
    m[13] = -(top + bottom) / (top - bottom);
    m[14] = -(f + n) / (f - n);
    m[15] = 1.0f;

    glMultMatrixf(m);
}

static void myPerspective(GLfloat fovy, GLfloat aspect, GLfloat n, GLfloat f)
{
    GLfloat m[16] = {0.0f};

    // 視野角をラジアンに変換
    GLfloat rad = fovy * _PI / 180.0f;
    GLfloat d = 1.0f / tanf(rad / 2.0f);

    m[0] = d / aspect;
    m[5] = d;
    m[10] = -(f + n) / (f - n);
    m[11] = -1.0f;
    m[14] = -2.0f * f * n / (f - n);

    glMultMatrixf(m);
}

static void drawIcosahedron(void)
{
    GLfloat phi = (1.f + sqrtf(5.f)) * .5f;
    GLfloat a = 1.f;
    GLfloat b = 1.f / phi;

    GLfloat vertices[12][3] = {{0.f, b, -a}, {b, a, 0.f}, {-b, a, 0.f}, {0.f, b, a}, {0.f, -b, a}, {-a, 0.f, b}, {0.f, -b, -a}, {a, 0.f, -b}, {a, 0.f, b}, {-a, 0.f, -b}, {b, -a, 0.f}, {-b, -a, 0.f}};

    GLfloat color[20][3] = {{0.0f, 0.0f, 0.6f}, {0.0f, 0.0f, 0.8f}, {0.0f, 0.0f, 1.0f}, {0.f, 0.2f, 1.f}, {0.f, 0.4f, 1.f}, {0.f, 0.6f, 1.f}, {0.f, 0.8f, 1.f}, {0.f, 1.f, 1.f}, {0.2f, 1.f, 0.8f}, {0.4f, 1.f, 0.6f}, {0.6f, 1.f, 0.4f}, {0.8f, 1.f, 0.2f}, {1.f, 1.f, 0.f}, {1.f, 0.8f, 0.f}, {1.f, 0.6f, 0.f}, {1.f, 0.4f, 0.f}, {1.f, 0.2f, 0.f}, {1.f, 0.f, 0.f}, {0.8f, 0.f, 0.f}, {0.6f, 0.f, 0.f}};

    int faces[20][3] = {{2, 1, 0}, {1, 2, 3}, {5, 4, 3}, {4, 8, 3}, {7, 6, 0}, {6, 9, 0}, {11, 10, 4}, {10, 11, 6}, {9, 5, 2}, {5, 9, 11}, {8, 7, 1}, {7, 8, 10}, {2, 5, 3}, {8, 1, 3}, {9, 2, 0}, {1, 7, 0}, {11, 9, 6}, {7, 10, 6}, {5, 11, 4}, {10, 8, 4}};

    int i;

    for (i = 0; i < 20; ++i)
    {
        GLfloat *c = color[i];
        int *f = faces[i];
        int v0 = f[0], v1 = f[1], v2 = f[2];
        glColor3f(c[0], c[1], c[2]);
        glBegin(GL_TRIANGLES);
        glVertex3f(vertices[v0][0], vertices[v0][1], vertices[v0][2]);
        glVertex3f(vertices[v1][0], vertices[v1][1], vertices[v1][2]);
        glVertex3f(vertices[v2][0], vertices[v2][1], vertices[v2][2]);
        glEnd();
    }
}

// 追加
static void drawFractalIcosahedron(int level)
{
    drawIcosahedron();

    if (level > 0)
    {
        // 左下の子
        glPushMatrix();
        glTranslatef(-1.0f, -1.5f, 0.0f); // 左下へ
        glScalef(0.5f, 0.5f, 0.5f); // 半分に縮小
        drawFractalIcosahedron(level - 1);
        glPopMatrix();

        // 右下の子
        glPushMatrix();
        glTranslatef(1.0f, -1.5f, 0.0); // 右下へ
        glScalef(0.5f, 0.5f, 0.5f);
        drawFractalIcosahedron(level - 1);
        glPopMatrix();
    }
}
static void display(void)
{

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Projection transformation
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    if (g_proj_mode == PERSP)
    {
        // Complete:
        // After completing the code of myPerspective() above,
        // replace the call to gluPerspective with a call to myPerspective
        // or gluPerspective depending on the transformation mode

        if (g_use_custom == 1)
        {
            // 自作した透視投影を使う
            myPerspective(45.0, (GLdouble)g_width / (GLdouble)g_height, 0.1, 20.0);
        }
        else
        {
            gluPerspective(45.0, (GLdouble)g_width / (GLdouble)g_height, 0.1, 20.0);
        }
    }
    else
    {
        // Complete:
        // After completing the code of myOrtho() above,
        // replace the call to glOrtho with a call to myOrtho
        // or glOrtho depending on the transformation mode
        if (g_use_custom == 1)
        {
            // 自作した平行投影を使う
            myOrtho(-2.0, 2.0, -2.0, 2.0, -10.0, 10.0);
        }
        else
        {
            glOrtho(-2.0, 2.0, -2.0, 2.0, -10.0, 10.0);
        }
    }

    // Modelview transformation
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    //修正
    if (g_use_custom == 1) {
        myLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    } else {
         gluLookAt(0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    }

    glPushMatrix();

    // Complete:
    // After completing the code of myTranslatef, myScalef,
    // and myRotatef,
    // replace the call to glTranslatef, glRotatef, glScalef by calls to
    // myTranslatef, myScalef, and myRotatef or
    // glTranslatef, glRotatef, glScalef based on the transformation mode.

    if (g_use_custom == 1)
    {
        // 自作した関数を呼び出す
        myTranslatef(g_trans[0], g_trans[1], g_trans[2]);
        myRotatef(g_angle[0], 1.f, 0.f, 0.f);
        myRotatef(g_angle[1], 0.f, 1.f, 0.f);
        myRotatef(g_angle[2], 0.f, 0.f, 1.f);
        myScalef(g_scale[0], g_scale[1], g_scale[2]);
    }
    else
    {
        glTranslatef(g_trans[0], g_trans[1], g_trans[2]);
        glRotatef(g_angle[0], 1.f, 0.f, 0.f);
        glRotatef(g_angle[1], 0.f, 1.f, 0.f);
        glRotatef(g_angle[2], 0.f, 0.f, 1.f);
        glScalef(g_scale[0], g_scale[1], g_scale[2]);
    }

    drawFractalIcosahedron(g_level); // 変更前：　drawIcosahedron();

    glPopMatrix();

    glutSwapBuffers();
}

static void reshape(int w, int h)
{
    glViewport(0, 0, w, h);

    g_width = w;
    g_height = h;
}

// Increase the rotation angle by amt around ax
static void rotate(int ax, GLfloat amt)
{
    g_angle[ax] += amt;
}

// Increase the translation by amt along ax
static void translate(int ax, GLfloat amt)
{
    g_trans[ax] += amt;
}

// Multiply the scaling factor by amt along ax
static void scale(int ax, GLfloat amt)
{
    g_scale[ax] *= amt;
}

static void keyboard(unsigned char k, int x, int y)
{
    switch (k)
    {
    case 27:
        exit(EXIT_SUCCESS);
        break;

    // Complete:
    // Allow to switch between OpenGL transformations and your implementations
    // 追加
    case 'c': // 標準と自作を切り替える
    case 'C':
        g_use_custom = 1 - g_use_custom; // 0と1を反転させる
        if (g_use_custom == 1)
        {
            printf("Using Custom Matrix\n");
        }
        else
        {
            printf("Using OpenGL Matrix\n");
        }
        break;

    case 'd': // dキーでリセット
    case 'D': // 定数を定義し修正
        g_trans[0] = INIT_TRANS;
        g_trans[1] = INIT_TRANS;
        g_trans[2] = INIT_TRANS;
        g_angle[0] = INIT_ANGLE;
        g_angle[1] = INIT_ANGLE;
        g_angle[2] = INIT_ANGLE;
        g_scale[0] = INIT_SCALE;
        g_scale[1] = INIT_SCALE;
        g_scale[2] = INIT_SCALE;
        g_level = INIT_LEVEL;
        break;
        // ここまで追加

    case 'p':
        g_proj_mode = (1 - g_proj_mode);
        break;

    case 't':
    case 'T':
        g_op_mode = TRANS;
        break;

    case 'r':
    case 'R':
        g_op_mode = ROT;
        break;

    case 's':
    case 'S':
        g_op_mode = SCALE;
        break;

    // 追加
    case 'N':
        if (g_level < 5)
        {
            g_level++;
        }
        break;

    case 'n':
        if (g_level > 0)
        {
            g_level--;
        }
        break;
        // ここまで追加

    case 'x':
        if (g_op_mode == TRANS)
            translate(0, -0.5f);
        if (g_op_mode == ROT)
            rotate(0, -5.0f);
        if (g_op_mode == SCALE)
            scale(0, 0.9f);
        break;

    case 'X':
        if (g_op_mode == TRANS)
            translate(0, 0.5f);
        if (g_op_mode == ROT)
            rotate(0, 5.0f);
        if (g_op_mode == SCALE)
            scale(0, 1.1f);
        break;

    case 'y':
        if (g_op_mode == TRANS)
            translate(1, -0.5f);
        if (g_op_mode == ROT)
            rotate(1, -5.0f);
        if (g_op_mode == SCALE)
            scale(1, 0.9f);
        break;

    case 'Y':
        if (g_op_mode == TRANS)
            translate(1, 0.5f);
        if (g_op_mode == ROT)
            rotate(1, 5.0f);
        if (g_op_mode == SCALE)
            scale(1, 1.1f);
        break;

    case 'z':
        if (g_op_mode == TRANS)
            translate(2, -0.5f);
        if (g_op_mode == ROT)
            rotate(2, -5.0f);
        if (g_op_mode == SCALE)
            scale(2, 0.9f);
        break;

    case 'Z':
        if (g_op_mode == TRANS)
            translate(2, 0.5f);
        if (g_op_mode == ROT)
            rotate(2, 5.0f);
        if (g_op_mode == SCALE)
            scale(2, 1.1f);
        break;

    default:
        break;
    }

    glutPostRedisplay();
}

static void init(void)
{
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glEnable(GL_DEPTH_TEST);
}

int main(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 800);
    glutCreateWindow("Icosahedron");

    init();

    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);

    glutMainLoop();
    return 0;
}
