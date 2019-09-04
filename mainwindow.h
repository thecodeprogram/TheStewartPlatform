#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public slots:
    void load_t_matrix();
    void load_ta_matrix();
    void load_moving_platform_joint_matrix();
    void load_moving_animated_platform_matrix();
    void calculate_legs();
    void fnc_showValues();
    void fnc_getValuesFromInputs();

private slots:
    void on_btnCalculateInverseKinematics_clicked();

private:
    Ui::MainWindow *ui;


};

#endif // MAINWINDOW_H
