#ifndef GENERATENETWORKDIALOG_H
#define GENERATENETWORKDIALOG_H

#include <QObject>
#include <QtWidgets>
#include <QMap>

class GenerateNetworkDialog : public QDialog
{
    Q_OBJECT
public:

    GenerateNetworkDialog();


    const QMap<QString, int> &flags() const;

private:
    QSpinBox* m_vertexCountSB;
    QGroupBox* m_parametrsOfGraphGB;

    QCheckBox* radio1,*radio2,*radio3,*radio4,*radio5;
    QVBoxLayout* GBl; //layout for checkboxes "ratioI"

    QLabel* m_vertexCountLabel;
    QPushButton* m_generateButton;
    QPushButton* m_cancelButton;
    QMap<QString,int>m_flags;



private slots:
    void checkData();
    void showForRandom(bool checked);
    void showForOriented(bool checked);
};

#endif // GenerateNetworkDialog_H
