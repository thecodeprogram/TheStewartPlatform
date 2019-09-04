#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QtMath>
#include "math.h"

const double PI  =3.141592653589793238463;
const float  PI_F=3.14159265358979f;

static double platform_height = 111.31;


//angular values as degrees not radians.(we will enter them as degrees)
static double roll_angle = 0;
static double pitch_angle = 0;
static double yaw_angle = 0;
static double x_angle = 0;
static double y_angle = 0;
static double z_angle = 0;

static double TXrad = 0;
static double TYrad = 0;
static double TZrad = 0;

//coordinates of joint locations
static double xsi[6];
static double ysi[6];
static double xmi[6];
static double ymi[6];

//NONMoving plate joint points
static double a1[4];
static double a2[4];
static double a3[4];
static double a4[4];
static double a5[4];
static double a6[4];

//Moving platform joint points
static double b1[4];
static double b2[4];
static double b3[4];
static double b4[4];
static double b5[4];
static double b6[4];

static double t_matrix[4][4];
static double ta_matrix[4][4];

static double leg1 = 0;
static double leg2 = 0;
static double leg3 = 0;
static double leg4 = 0;
static double leg5 = 0;
static double leg6 = 0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //NONMoving plate joint points
    xsi[0] = 1.0;
    xsi[1] = 1.0;
    xsi[2] = 1.0;
    xsi[3] = 1.0;
    xsi[4] = 1.0;
    xsi[5] = 1.0;
    ysi[0] = 1.0;
    ysi[1] = 1.0;
    ysi[2] = 1.0;
    ysi[3] = 1.0;
    ysi[4] = 1.0;
    ysi[5] = 1.0;

    //Moving platform joint points
    xmi[0] = 1.0;
    xmi[1] = 1.0;
    xmi[2] = 1.0;
    xmi[3] = 1.0;
    xmi[4] = 1.0;
    xmi[5] = 1.0;
    ymi[0] = 1.0;
    ymi[1] = 1.0;
    ymi[2] = 1.0;
    ymi[3] = 1.0;
    ymi[4] = 1.0;
    ymi[5] = 1.0;




}

void MainWindow::load_t_matrix()
{
    t_matrix[0][0] = qCos(TYrad) * qCos(TZrad);
    t_matrix[0][1] = (-1) * qCos(TYrad) * qSin(TZrad);
    t_matrix[0][2] = qSin(TYrad);
    t_matrix[0][3] = x_angle;

    t_matrix[1][0] = qSin(TXrad) * qSin(TYrad) * qCos(TZrad) + (qCos(TXrad)*qSin(TZrad));
    t_matrix[1][1] = (-1) * qSin(TXrad) * qSin(TYrad) * qSin(TZrad) + (qCos(TXrad)*qCos(TZrad));
    t_matrix[1][2] = (-1) * qSin(TXrad) * qCos(TYrad);
    t_matrix[1][3] = y_angle;

    t_matrix[2][0] = (-1) * qCos(TXrad) * qSin(TYrad) * cos(TZrad) + (qSin(TXrad)*qSin(TZrad));
    t_matrix[2][1] = qCos(TXrad) * qSin(TYrad) * qSin(TZrad) + (qSin(TXrad)*qCos(TZrad));
    t_matrix[2][2] = qCos(TXrad) * qCos(TYrad);
    t_matrix[2][3] = z_angle - platform_height;

    t_matrix[3][0] = 0;
    t_matrix[3][1] = 0;
    t_matrix[3][2] = 0;
    t_matrix[3][3] = 1;
}

void MainWindow::load_ta_matrix()
{
    ta_matrix[0][0] = qCos(TYrad) * qCos(TZrad);
    ta_matrix[0][1] = (-1) * qCos(TYrad) * qSin(TZrad);
    ta_matrix[0][2] = qSin(TYrad);
    ta_matrix[0][3] = x_angle;

    ta_matrix[1][0] = qSin(TXrad) * qSin(TYrad) * qCos(TZrad) + qCos(TXrad)*qSin(TZrad);
    ta_matrix[1][1] = (-1) * qSin(TXrad) * qSin(TYrad) * qSin(TZrad) + qCos(TXrad)*qCos(TZrad);
    ta_matrix[1][2] = (-1) * qSin(TXrad) * qCos(TYrad);
    ta_matrix[1][3] = y_angle;

    ta_matrix[2][0] = (-1) * qCos(TXrad) * qSin(TYrad) * cos(TZrad) + qSin(TXrad)*qSin(TZrad);
    ta_matrix[2][1] = qCos(TXrad) * qSin(TYrad) * qSin(TZrad) + qSin(TXrad)*qCos(TZrad);
    ta_matrix[2][2] = qCos(TXrad) * qCos(TYrad);
    ta_matrix[2][3] = platform_height - z_angle;

    ta_matrix[3][0] = 0;
    ta_matrix[3][1] = 0;
    ta_matrix[3][2] = 0;
    ta_matrix[3][3] = 1;
}

void MainWindow::load_moving_platform_joint_matrix()
{
    b1[0] = (t_matrix[0][0] * xmi[0]) + (t_matrix[0][1] * ymi[0]) + (t_matrix[0][2] * 0) + (t_matrix[0][3] * 1);
    b1[1] = (t_matrix[1][0] * xmi[0]) + (t_matrix[1][1] * ymi[0]) + (t_matrix[1][2] * 0) + (t_matrix[1][3] * 1);
    b1[2] = (t_matrix[2][0] * xmi[0]) + (t_matrix[2][1] * ymi[0]) + (t_matrix[2][2] * 0) + (t_matrix[2][3] * 1);
    b1[3] = (t_matrix[3][0] * xmi[0]) + (t_matrix[3][1] * ymi[0]) + (t_matrix[3][2] * 0) + (t_matrix[3][3] * 1);

    b2[0] = (t_matrix[0][0] * xmi[1]) + (t_matrix[0][1] * ymi[1]) + (t_matrix[0][2] * 0) + (t_matrix[0][3] * 1);
    b2[1] = (t_matrix[1][0] * xmi[1]) + (t_matrix[1][1] * ymi[1]) + (t_matrix[1][2] * 0) + (t_matrix[1][3] * 1);
    b2[2] = (t_matrix[2][0] * xmi[1]) + (t_matrix[2][1] * ymi[1]) + (t_matrix[2][2] * 0) + (t_matrix[2][3] * 1);
    b2[3] = (t_matrix[3][0] * xmi[1]) + (t_matrix[3][1] * ymi[1]) + (t_matrix[3][2] * 0) + (t_matrix[3][3] * 1);

    b3[0] = (t_matrix[0][0] * xmi[2]) + (t_matrix[0][1] * ymi[2]) + (t_matrix[0][2] * 0) + (t_matrix[0][3] * 1);
    b3[1] = (t_matrix[1][0] * xmi[2]) + (t_matrix[1][1] * ymi[2]) + (t_matrix[1][2] * 0) + (t_matrix[1][3] * 1);
    b3[2] = (t_matrix[2][0] * xmi[2]) + (t_matrix[2][1] * ymi[2]) + (t_matrix[2][2] * 0) + (t_matrix[2][3] * 1);
    b3[3] = (t_matrix[3][0] * xmi[2]) + (t_matrix[3][1] * ymi[2]) + (t_matrix[3][2] * 0) + (t_matrix[3][3] * 1);

    b4[0] = (t_matrix[0][0] * xmi[3]) + (t_matrix[0][1] * ymi[3]) + (t_matrix[0][2] * 0) + (t_matrix[0][3] * 1);
    b4[1] = (t_matrix[1][0] * xmi[3]) + (t_matrix[1][1] * ymi[3]) + (t_matrix[1][2] * 0) + (t_matrix[1][3] * 1);
    b4[2] = (t_matrix[2][0] * xmi[3]) + (t_matrix[2][1] * ymi[3]) + (t_matrix[2][2] * 0) + (t_matrix[2][3] * 1);
    b4[3] = (t_matrix[3][0] * xmi[3]) + (t_matrix[3][1] * ymi[3]) + (t_matrix[3][2] * 0) + (t_matrix[3][3] * 1);

    b5[0] = (t_matrix[0][0] * xmi[4]) + (t_matrix[0][1] * ymi[4]) + (t_matrix[0][2] * 0) + (t_matrix[0][3] * 1);
    b5[1] = (t_matrix[1][0] * xmi[4]) + (t_matrix[1][1] * ymi[4]) + (t_matrix[1][2] * 0) + (t_matrix[1][3] * 1);
    b5[2] = (t_matrix[2][0] * xmi[4]) + (t_matrix[2][1] * ymi[4]) + (t_matrix[2][2] * 0) + (t_matrix[2][3] * 1);
    b5[3] = (t_matrix[3][0] * xmi[4]) + (t_matrix[3][1] * ymi[4]) + (t_matrix[3][2] * 0) + (t_matrix[3][3] * 1);

    b6[0] = (t_matrix[0][0] * xmi[5]) + (t_matrix[0][1] * ymi[5]) + (t_matrix[0][2] * 0) + (t_matrix[0][3] * 1);
    b6[1] = (t_matrix[1][0] * xmi[5]) + (t_matrix[1][1] * ymi[5]) + (t_matrix[1][2] * 0) + (t_matrix[1][3] * 1);
    b6[2] = (t_matrix[2][0] * xmi[5]) + (t_matrix[2][1] * ymi[5]) + (t_matrix[2][2] * 0) + (t_matrix[2][3] * 1);
    b6[3] = (t_matrix[3][0] * xmi[5]) + (t_matrix[3][1] * ymi[5]) + (t_matrix[3][2] * 0) + (t_matrix[3][3] * 1);

}

void MainWindow::load_moving_animated_platform_matrix()
{
    b1[0] = (ta_matrix[0][0] * xmi[0]) + (ta_matrix[0][1] * ymi[0]) + (ta_matrix[0][2] * 0) + (ta_matrix[0][3] * 1);
    b1[1] = (ta_matrix[1][0] * xmi[0]) + (ta_matrix[1][1] * ymi[0]) + (ta_matrix[1][2] * 0) + (ta_matrix[1][3] * 1);
    b1[2] = (ta_matrix[2][0] * xmi[0]) + (ta_matrix[2][1] * ymi[0]) + (ta_matrix[2][2] * 0) + (ta_matrix[2][3] * 1);
    b1[3] = (ta_matrix[3][0] * xmi[0]) + (ta_matrix[3][1] * ymi[0]) + (ta_matrix[3][2] * 0) + (ta_matrix[3][3] * 1);

    b2[0] = (ta_matrix[0][0] * xmi[1]) + (ta_matrix[0][1] * ymi[1]) + (ta_matrix[0][2] * 0) + (ta_matrix[0][3] * 1);
    b2[1] = (ta_matrix[1][0] * xmi[1]) + (ta_matrix[1][1] * ymi[1]) + (ta_matrix[1][2] * 0) + (ta_matrix[1][3] * 1);
    b2[2] = (ta_matrix[2][0] * xmi[1]) + (ta_matrix[2][1] * ymi[1]) + (ta_matrix[2][2] * 0) + (ta_matrix[2][3] * 1);
    b2[3] = (ta_matrix[3][0] * xmi[1]) + (ta_matrix[3][1] * ymi[1]) + (ta_matrix[3][2] * 0) + (ta_matrix[3][3] * 1);

    b3[0] = (ta_matrix[0][0] * xmi[2]) + (ta_matrix[0][1] * ymi[2]) + (ta_matrix[0][2] * 0) + (ta_matrix[0][3] * 1);
    b3[1] = (ta_matrix[1][0] * xmi[2]) + (ta_matrix[1][1] * ymi[2]) + (ta_matrix[1][2] * 0) + (ta_matrix[1][3] * 1);
    b3[2] = (ta_matrix[2][0] * xmi[2]) + (ta_matrix[2][1] * ymi[2]) + (ta_matrix[2][2] * 0) + (ta_matrix[2][3] * 1);
    b3[3] = (ta_matrix[3][0] * xmi[2]) + (ta_matrix[3][1] * ymi[2]) + (ta_matrix[3][2] * 0) + (ta_matrix[3][3] * 1);

    b4[0] = (ta_matrix[0][0] * xmi[3]) + (ta_matrix[0][1] * ymi[3]) + (ta_matrix[0][2] * 0) + (ta_matrix[0][3] * 1);
    b4[1] = (ta_matrix[1][0] * xmi[3]) + (ta_matrix[1][1] * ymi[3]) + (ta_matrix[1][2] * 0) + (ta_matrix[1][3] * 1);
    b4[2] = (ta_matrix[2][0] * xmi[3]) + (ta_matrix[2][1] * ymi[3]) + (ta_matrix[2][2] * 0) + (ta_matrix[2][3] * 1);
    b4[3] = (ta_matrix[3][0] * xmi[3]) + (ta_matrix[3][1] * ymi[3]) + (ta_matrix[3][2] * 0) + (ta_matrix[3][3] * 1);

    b5[0] = (ta_matrix[0][0] * xmi[4]) + (ta_matrix[0][1] * ymi[4]) + (ta_matrix[0][2] * 0) + (ta_matrix[0][3] * 1);
    b5[1] = (ta_matrix[1][0] * xmi[4]) + (ta_matrix[1][1] * ymi[4]) + (ta_matrix[1][2] * 0) + (ta_matrix[1][3] * 1);
    b5[2] = (ta_matrix[2][0] * xmi[4]) + (ta_matrix[2][1] * ymi[4]) + (ta_matrix[2][2] * 0) + (ta_matrix[2][3] * 1);
    b5[3] = (ta_matrix[3][0] * xmi[4]) + (ta_matrix[3][1] * ymi[4]) + (ta_matrix[3][2] * 0) + (ta_matrix[3][3] * 1);

    b6[0] = (ta_matrix[0][0] * xmi[5]) + (ta_matrix[0][1] * ymi[5]) + (ta_matrix[0][2] * 0) + (ta_matrix[0][3] * 1);
    b6[1] = (ta_matrix[1][0] * xmi[5]) + (ta_matrix[1][1] * ymi[5]) + (ta_matrix[1][2] * 0) + (ta_matrix[1][3] * 1);
    b6[2] = (ta_matrix[2][0] * xmi[5]) + (ta_matrix[2][1] * ymi[5]) + (ta_matrix[2][2] * 0) + (ta_matrix[2][3] * 1);
    b6[3] = (ta_matrix[3][0] * xmi[5]) + (ta_matrix[3][1] * ymi[5]) + (ta_matrix[3][2] * 0) + (ta_matrix[3][3] * 1);
}

void MainWindow::calculate_legs()
{
    leg1 = qSqrt(((qAbs(a1[0] - b1[0]))*(qAbs(a1[0] - b1[0])))+((qAbs(a1[1] - b1[1]))*(qAbs(a1[1] - b1[1])))+((qAbs(a1[2] - b1[2]))*(qAbs(a1[2] - b1[2]))));
    leg2 = qSqrt(((qAbs(a2[0] - b2[0]))*(qAbs(a2[0] - b2[0])))+((qAbs(a2[1] - b2[1]))*(qAbs(a2[1] - b2[1])))+((qAbs(a2[2] - b2[2]))*(qAbs(a2[2] - b2[2]))));
    leg3 = qSqrt(((qAbs(a3[0] - b3[0]))*(qAbs(a3[0] - b3[0])))+((qAbs(a3[1] - b3[1]))*(qAbs(a3[1] - b3[1])))+((qAbs(a3[2] - b3[2]))*(qAbs(a3[2] - b3[2]))));
    leg4 = qSqrt(((qAbs(a4[0] - b4[0]))*(qAbs(a4[0] - b4[0])))+((qAbs(a4[1] - b4[1]))*(qAbs(a4[1] - b4[1])))+((qAbs(a4[2] - b4[2]))*(qAbs(a4[2] - b4[2]))));
    leg5 = qSqrt(((qAbs(a5[0] - b5[0]))*(qAbs(a5[0] - b5[0])))+((qAbs(a5[1] - b5[1]))*(qAbs(a5[1] - b5[1])))+((qAbs(a5[2] - b5[2]))*(qAbs(a5[2] - b5[2]))));
    leg6 = qSqrt(((qAbs(a6[0] - b6[0]))*(qAbs(a6[0] - b6[0])))+((qAbs(a6[1] - b6[1]))*(qAbs(a6[1] - b6[1])))+((qAbs(a6[2] - b6[2]))*(qAbs(a6[2] - b6[2]))));
}

void MainWindow::fnc_showValues()
{
    ui->txtLeg1->setText( QString::number(leg1) );
    ui->txtLeg2->setText( QString::number(leg2) );
    ui->txtLeg3->setText( QString::number(leg3) );
    ui->txtLeg4->setText( QString::number(leg4) );
    ui->txtLeg5->setText( QString::number(leg5) );
    ui->txtLeg6->setText( QString::number(leg6) );
}

void MainWindow::fnc_getValuesFromInputs()
{
    xsi[0] = ui->txtb1x->text().toDouble();
    xsi[1] = ui->txtb2x->text().toDouble();
    xsi[2] = ui->txtb3x->text().toDouble();
    xsi[3] = ui->txtb4x->text().toDouble();
    xsi[4] = ui->txtb5x->text().toDouble();
    xsi[5] = ui->txtb6x->text().toDouble();

    ysi[0] = ui->txtb1y->text().toDouble();
    ysi[1] = ui->txtb2y->text().toDouble();
    ysi[2] = ui->txtb3y->text().toDouble();
    ysi[3] = ui->txtb4y->text().toDouble();
    ysi[4] = ui->txtb5y->text().toDouble();
    ysi[5] = ui->txtb6y->text().toDouble();

    xmi[0] = ui->txtm1x->text().toDouble();
    xmi[1] = ui->txtm2x->text().toDouble();
    xmi[2] = ui->txtm3x->text().toDouble();
    xmi[3] = ui->txtm4x->text().toDouble();
    xmi[4] = ui->txtm5x->text().toDouble();
    xmi[5] = ui->txtm6x->text().toDouble();

    ymi[0] = ui->txtm1y->text().toDouble();
    ymi[1] = ui->txtm2y->text().toDouble();
    ymi[2] = ui->txtm3y->text().toDouble();
    ymi[3] = ui->txtm4y->text().toDouble();
    ymi[4] = ui->txtm5y->text().toDouble();
    ymi[5] = ui->txtm6y->text().toDouble();


    a1[0] = xsi[0];
    a1[1] = ysi[0];
    a1[2] = 0;
    a1[3] = 1;

    a2[0] = xsi[1];
    a2[1] = ysi[1];
    a2[2] = 0;
    a2[3] = 1;

    a3[0] = xsi[2];
    a3[1] = ysi[2];
    a3[2] = 0;
    a3[3] = 1;

    a4[0] = xsi[3];
    a4[1] = ysi[3];
    a4[2] = 0;
    a4[3] = 1;

    a5[0] = xsi[4];
    a5[1] = ysi[4];
    a5[2] = 0;
    a5[3] = 1;

    a6[0] = xsi[5];
    a6[1] = ysi[5];
    a6[2] = 0;
    a6[3] = 1;

    roll_angle = ui->txtAngleRoll->text().toDouble();
    pitch_angle = ui->txtAnglePitch->text().toDouble();
    yaw_angle = ui->txtAngleYaw->text().toDouble();
    x_angle = ui->txtAngleX->text().toDouble();
    y_angle = ui->txtAngleY->text().toDouble();
    z_angle = ui->txtAngleZ->text().toDouble();


    TXrad = roll_angle  * M_PI / 180;
    TYrad = pitch_angle * M_PI / 180;
    TZrad = yaw_angle   * M_PI / 180;

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_btnCalculateInverseKinematics_clicked()
{
    fnc_getValuesFromInputs();
    load_t_matrix();
//    load_ta_matrix();
    load_moving_platform_joint_matrix();
//    load_moving_animated_platform_matrix();
    calculate_legs();
    fnc_showValues();
    bool b = true;
}
